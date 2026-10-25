// license:BSD-3-Clause
// copyright-holders:Riley
/***************************************************************************

    Zevio 1020 media DSP (ZSP400), high-level emulation

    The ARM loads a 16-bit program into the DSP (descriptor DMA to
    0xDC020000), points two 128 KB windows at shared SDRAM (0x900A0024; the
    low 16 bits << 17 give the shared base W), releases it from reset
    (0x900A0020 = 1) and talks through a mailbox:
      0x900A0104  DSP->ARM message flag, IRQ 17 (ARM acknowledges with 0)
      0x900A0108  ARM->DSP doorbell (command block at W + 0x1C000)
      0x900A0110  DSP->ARM reply flag, IRQ 18 (ARM writes to clear)
      0x900A0114  reads 3 once the DSP program is running
    Instead of running the DSP program, the protocol of the known programs is
    implemented directly and JPEG data is decoded on the host.

    MJP movie program (0x1ED0 program words), 32-bit fields, C = W + 0x1C000:
      0x80 start -> reply C+64 = 0x80
      0x81 configure: C+8/C+12 CD ring start/end (MJP file, MIAV header),
           C+24/28/32 output Y/U/V, C+36/40 left/right audio rings,
           C+44 audio ring size -> reply C+64 = 0x8081 with C+72 width,
           C+76 height, C+80 fps, C+88 frame count
      0x82 stop, 0x83 pause, 0x84 resume
    Frames are posted at M = W + 0x1C080 with code 0x92; the ARM supplies the
    next Y/U/V set at M+76/80/84 and acknowledges with 0x104 = 0. Video is
    planar YCbCr 4:2:0 shown by display layer 5; audio chunks are copied raw
    into the SPU rings (IMA4 decoded by the sound block).

    TODO:
    - JPEG program (0x11F8 words) for the boot ROM photo viewer
    - frame status codes for corrupt data, seek requests (M+88)

***************************************************************************/

#include "emu.h"
#include "zevio_dsp.h"

#include "jpeglib.h"

#include <csetjmp>

#define LOG_CMD (1U << 1)

#define VERBOSE (LOG_CMD)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ZEVIO_DSP_HLE, zevio_dsp_hle_device, "zevio_dsp_hle", "Zevio 1020 media DSP (HLE)")

namespace {

// builds of the MJP program in the games' FA libraries, all with the same protocol
// (Princess's differs from the boot ROM's mostly in relocated branch targets)
constexpr u32 MJP_PROGRAM_WORDS = 0x1ed0;
constexpr u32 MJP_PROGRAM_WORDS_B = 0x1ea0; // Wacky Race
constexpr u32 MJP_PROGRAM_WORDS_C = 0x1eb0; // Disney Princess
constexpr u32 CMD_BLOCK = 0x1c000;
constexpr u32 MSG_BLOCK = 0x1c080;

struct jpeg_error_jmp
{
	jpeg_error_mgr pub;
	std::jmp_buf jump;
};

void jpeg_error_exit_jmp(j_common_ptr cinfo)
{
	std::longjmp(reinterpret_cast<jpeg_error_jmp *>(cinfo->err)->jump, 1);
}

void jpeg_output_nothing(j_common_ptr cinfo)
{
}

// the MJP encoder writes the entropy data without 0x00 stuffing after 0xFF
std::vector<u8> fix_byte_stuffing(const std::vector<u8> &in)
{
	size_t sos = 0;
	for (size_t i = 0; i + 3 < in.size(); i++)
		if (in[i] == 0xff && in[i + 1] == 0xda)
		{
			sos = i + 2 + ((in[i + 2] << 8) | in[i + 3]);
			break;
		}
	if (!sos || sos > in.size())
		return in;
	std::vector<u8> out(in.begin(), in.begin() + sos);
	for (size_t j = sos; j < in.size(); )
	{
		// the entropy data is never stuffed and has no markers (and no EOI):
		// every 0xFF is data, including one followed by a data 0x00, so each
		// gets its stuffing byte (taking FF 00 as stuffed drops the 00 and
		// garbles the rest of the frame)
		const u8 b = in[j++];
		out.push_back(b);
		if (b == 0xff)
			out.push_back(0x00);
	}
	return out;
}

} // anonymous namespace


zevio_dsp_hle_device::zevio_dsp_hle_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, ZEVIO_DSP_HLE, tag, owner, clock),
	m_msg_irq_cb(*this),
	m_reply_irq_cb(*this),
	m_ram(nullptr),
	m_ram_base(0),
	m_ram_size(0)
{
}

void zevio_dsp_hle_device::device_start()
{
	m_frame_timer = timer_alloc(FUNC(zevio_dsp_hle_device::frame_cb), this);
	std::fill(std::begin(m_jpeg_header_save), std::end(m_jpeg_header_save), 0);
	m_jpeg_header_len = 0;
	save_item(NAME(m_mbox));
	save_item(NAME(m_prog_words));
	save_item(NAME(m_running));
	save_item(NAME(m_w));
	save_item(NAME(m_playing));
	save_item(NAME(m_paused));
	save_item(NAME(m_awaiting_ack));
	save_item(NAME(m_ring_start));
	save_item(NAME(m_ring_end));
	save_item(NAME(m_rd));
	save_item(NAME(m_out));
	save_item(NAME(m_aud));
	save_item(NAME(m_aud_wp));
	save_item(NAME(m_aud_size));
	save_item(NAME(m_width));
	save_item(NAME(m_height));
	save_item(NAME(m_frames));
	save_item(NAME(m_frame));
	save_item(NAME(m_sched));
	save_item(NAME(m_jpeg_header_save));
	save_item(NAME(m_jpeg_header_len));
}

void zevio_dsp_hle_device::device_pre_save()
{
	m_jpeg_header_len = std::min<u32>(m_jpeg_header.size(), sizeof(m_jpeg_header_save));
	std::copy_n(m_jpeg_header.begin(), m_jpeg_header_len, m_jpeg_header_save);
}

void zevio_dsp_hle_device::device_post_load()
{
	m_jpeg_header.assign(m_jpeg_header_save, m_jpeg_header_save + m_jpeg_header_len);
}

void zevio_dsp_hle_device::device_reset()
{
	std::fill(std::begin(m_mbox), std::end(m_mbox), 0);
	m_prog_words = 0;
	m_running = false;
	m_w = 0;
	m_playing = m_paused = m_awaiting_ack = false;
	m_ring_start = m_ring_end = m_rd = 0;
	std::fill(std::begin(m_out), std::end(m_out), 0);
	m_aud[0] = m_aud[1] = m_aud_wp[0] = m_aud_wp[1] = m_aud_size = 0;
	m_width = m_height = m_frames = m_frame = m_sched = 0;
	m_jpeg_header.clear();
	m_frame_timer->adjust(attotime::never);
	m_msg_irq_cb(0);
	m_reply_irq_cb(0);
}

u32 zevio_dsp_hle_device::rd32(u32 addr) const
{
	const u32 off = addr - m_ram_base;
	return (m_ram && off < m_ram_size) ? m_ram[off >> 2] : 0;
}

void zevio_dsp_hle_device::wr32(u32 addr, u32 data)
{
	const u32 off = addr - m_ram_base;
	if (m_ram && off < m_ram_size)
		m_ram[off >> 2] = data;
}

void zevio_dsp_hle_device::wr8(u32 addr, u8 data)
{
	const u32 off = addr - m_ram_base;
	if (!m_ram || off >= m_ram_size)
		return;
	const int shift = (addr & 3) * 8;
	u32 &w = m_ram[off >> 2];
	w = (w & ~(0xffU << shift)) | (u32(data) << shift);
}

u32 zevio_dsp_hle_device::ring32(u32 addr) const
{
	return ring8(addr) | (ring8(addr + 1) << 8) | (ring8(addr + 2) << 16) | (ring8(addr + 3) << 24);
}


/***************************************************************************
    Control and mailbox
***************************************************************************/

void zevio_dsp_hle_device::run_w(u32 data)
{
	const bool run = data & 1;
	if (run && !m_running)
	{
		LOGMASKED(LOG_CMD, "DSP run, program %x words, shared base %08x\n", m_prog_words, m_w);
		m_mbox[STATUS] = 3;
	}
	else if (!run && m_running)
	{
		LOGMASKED(LOG_CMD, "DSP reset\n");
		m_mbox[STATUS] = 0;
		m_playing = false;
		m_frame_timer->adjust(attotime::never);
	}
	m_running = run;
}

void zevio_dsp_hle_device::windows_w(u32 data)
{
	m_w = (data & 0xffff) << 17;
}

u32 zevio_dsp_hle_device::mbox_r(offs_t offset)
{
	if (offset == 0)
		return 0; // semaphore: always free
	return m_mbox[offset & 7];
}

void zevio_dsp_hle_device::mbox_w(offs_t offset, u32 data, u32 mem_mask)
{
	offset &= 7;
	switch (offset)
	{
	case MSG:
		m_mbox[MSG] = data;
		if (!data)
		{
			m_msg_irq_cb(0);
			if (m_awaiting_ack)
			{
				m_awaiting_ack = false;
				// FA answers each frame with the Y/U/V set for the next one (its 3-buffer rotation)
				const u32 m = m_w + MSG_BLOCK;
				m_out[0] = rd32(m + 76);
				m_out[1] = rd32(m + 80);
				m_out[2] = rd32(m + 84);
				if (m_playing && !m_paused)
					schedule_next();
			}
		}
		break;
	case DOORBELL:
		m_mbox[DOORBELL] = data;
		if (m_running)
			command(rd32(m_w + CMD_BLOCK) & 0xff);
		break;
	case REPLY:
		m_mbox[REPLY] = 0; // any write clears the reply flag
		m_reply_irq_cb(0);
		break;
	default:
		COMBINE_DATA(&m_mbox[offset]);
		break;
	}
}

void zevio_dsp_hle_device::post_reply()
{
	m_mbox[REPLY] = 1;
	m_reply_irq_cb(1);
}

void zevio_dsp_hle_device::post_message()
{
	m_mbox[MSG] = 1;
	m_awaiting_ack = true;
	m_msg_irq_cb(1);
}


/***************************************************************************
    MJP program
***************************************************************************/

void zevio_dsp_hle_device::command(u32 cmd)
{
	const u32 c = m_w + CMD_BLOCK;
	if (m_prog_words != MJP_PROGRAM_WORDS && m_prog_words != MJP_PROGRAM_WORDS_B && m_prog_words != MJP_PROGRAM_WORDS_C)
	{
		LOGMASKED(LOG_CMD, "DSP command %02x for unsupported program (%x words)\n", cmd, m_prog_words);
		return;
	}
	LOGMASKED(LOG_CMD, "MJP command %02x\n", cmd);
	switch (cmd)
	{
	case 0x80:
		// the program answers with its input buffer (DSP data word 0x7000), as both builds do (main, 0x101B)
		wr32(c + 72, 0x7000);
		wr32(c + 64, 0x80);
		post_reply();
		break;
	case 0x81:
	{
		if (getenv("ZDSP_DEBUG"))
		{
			std::string s;
			for (int i = 0; i < 16; i++)
				s += util::string_format(" %08x", rd32(c + i * 4));
			logerror("MJP 0x81 block at %08x:%s\n", c, s);
		}
		m_ring_start = rd32(c + 8);
		m_ring_end = rd32(c + 12);
		m_out[0] = rd32(c + 24);
		m_out[1] = rd32(c + 28);
		m_out[2] = rd32(c + 32);
		m_aud[0] = m_aud_wp[0] = rd32(c + 36);
		m_aud[1] = m_aud_wp[1] = rd32(c + 40);
		m_aud_size = rd32(c + 44);
		// MIAV header at the start of the ring
		const u32 h = m_ring_start;
		const bool ok = ring32(h) == 0x5641494d; // "MIAV"
		m_frames = ring32(h + 0x10);
		const u32 fps = ring32(h + 0x14) & 0xffff;
		m_width = ring32(h + 0x18) & 0xffff;
		m_height = ring32(h + 0x18) >> 16;
		m_rd = h + ring32(h + 4);
		m_frame = 0;
		m_jpeg_header.clear();
		LOGMASKED(LOG_CMD, "MJP %s %dx%d %d fps %d frames, ring %08x-%08x\n", ok ? "MIAV" : "(bad header)", m_width, m_height, fps, m_frames, m_ring_start, m_ring_end);
		wr32(c + 72, m_width);
		wr32(c + 76, m_height);
		wr32(c + 80, fps);
		// the audio format code from the header (0-2 IMA at 44.1/22.05/11.025 kHz, 3-5 16-bit, 6-8 8-bit):
		// FA picks the movie voices' sample format and rate divider from it
		wr32(c + 84, ring32(h + 0x14) >> 16);
		wr32(c + 88, m_frames);
		wr32(c + 64, ok ? 0x8081 : 0x0881);
		post_reply();
		if (ok)
		{
			m_playing = true;
			m_paused = false;
			m_awaiting_ack = false;
			// The audio clock paces the movie, not the DSP: FA acknowledges a frame
			// only once the SPU has played the audio ring down to 1.5 frames ahead
			// of the DSP's write position (Mjp_engine's A/V wait), and the DSP
			// decodes the next frame as soon as it has the acknowledge. (A fixed
			// 15 fps schedule drifts against the SPU until the play position passes
			// the writes; FA then sees a nearly full 17000-byte ring and waits a
			// whole lap, about 1.4 s.)
			m_sched = 0;
			schedule_next();
		}
		break;
	}
	case 0x82:
		m_playing = false;
		m_frame_timer->adjust(attotime::never);
		break;
	case 0x83:
		m_paused = true;
		break;
	case 0x84:
		if (m_paused)
		{
			m_paused = false;
			if (m_playing)
				schedule_next();
		}
		break;
	default:
		LOGMASKED(LOG_CMD, "unknown MJP command %02x\n", cmd);
		break;
	}
}

TIMER_CALLBACK_MEMBER(zevio_dsp_hle_device::frame_cb)
{
	if (!m_playing || m_paused || m_frame >= m_frames)
		return;
	if (m_awaiting_ack || m_mbox[MSG])
		return;   // the acknowledge reschedules
	if (!decode_frame())
		return;
	m_sched++;
	// No separate plane request (0x92 with flags 2) is needed here: FA answers every frame message
	// with the next Y/U/V set at +76/+80/+84 from its 3-buffer rotation (Mjp_engine, bratz_fa
	// 0x109D0FC0), and an extra request would advance that rotation twice per frame.
	// ZDSP_DEBUG: the emulated time of every 15th frame (should advance 1 s each at 15 fps)
	static const bool dbg = getenv("ZDSP_DEBUG") != nullptr;
	static const bool every = dbg && atoi(getenv("ZDSP_DEBUG")) >= 2;
	if (dbg && (every || m_sched % 15 == 0))
		logerror("MJP frame %u (file frame %u/%u) at %.3f s, rd %08x\n", m_sched, m_frame, m_frames, machine().time().as_double(), m_rd);
}

// the next frame goes out after the DSP's decode time
void zevio_dsp_hle_device::schedule_next()
{
	m_frame_timer->adjust(attotime::from_msec(10));
}

// parse the chunks of one frame (00dc, 01wb, 02wb) from the ring and post it
bool zevio_dsp_hle_device::decode_frame()
{
	u32 frame_no = m_frame, vsize = 0, asize = 0;
	bool have_video = false;
	for (int n = 0; n < 3; n++)
	{
		const u32 fourcc = ring32(m_rd);
		const u32 num = ring32(m_rd + 4), size = ring32(m_rd + 8);
		static const bool cdbg = getenv("ZDSP_DEBUG") && atoi(getenv("ZDSP_DEBUG")) >= 4;
		if (cdbg && m_sched < 12)
			logerror("MJP chunk %c%c%c%c num %u size %u at %08x\n", fourcc & 0xff, (fourcc >> 8) & 0xff, (fourcc >> 16) & 0xff, fourcc >> 24, num, size, m_rd);
		if (size > 0x100000)
			break;
		if (fourcc == 0x63643030) // "00dc"
		{
			if (have_video)
				break; // next frame
			have_video = true;
			frame_no = num;
			vsize = size;
			std::vector<u8> data(size);
			for (u32 i = 0; i < size; i++)
			{
				// undo the 16-bit byte swap (a trailing odd byte stays put)
				const u32 j = ((i ^ 1) < size) ? (i ^ 1) : i;
				data[i] = ring8(m_rd + 12 + j);
			}
			std::vector<u8> jpeg = fix_byte_stuffing(data);
			// a frame that brings its own tables (SOI, then DQT or DHT before SOS) is
			// complete, e.g. the first frame after a cut to another clip: use it as it is
			// and keep its tables; the others get the last tables put in front
			bool complete = jpeg.size() >= 2 && jpeg[0] == 0xff && jpeg[1] == 0xd8;
			if (complete)
			{
				complete = false;
				for (size_t i = 2; i + 1 < jpeg.size(); i++)
					if (jpeg[i] == 0xff && (jpeg[i + 1] == 0xdb || jpeg[i + 1] == 0xc4))
					{
						complete = true;
						break;
					}
					else if (jpeg[i] == 0xff && jpeg[i + 1] == 0xda)
						break;
			}
			if (num == 0 || m_jpeg_header.empty() || complete)
			{
				// keyframe: keep everything before SOS for the following frames
				for (size_t i = 0; i + 1 < jpeg.size(); i++)
					if (jpeg[i] == 0xff && jpeg[i + 1] == 0xda)
					{
						m_jpeg_header.assign(jpeg.begin(), jpeg.begin() + i);
						break;
					}
			}
			else
				jpeg.insert(jpeg.begin(), m_jpeg_header.begin(), m_jpeg_header.end());
			if (jpeg.size() < 2 || jpeg[jpeg.size() - 2] != 0xff || jpeg[jpeg.size() - 1] != 0xd9)
			{
				jpeg.push_back(0xff);
				jpeg.push_back(0xd9);
			}
			if (const char *dump = getenv("ZDSP_DUMP"))   // ZDSP_DUMP=dir: each frame's JPEG and its raw chunk
			{
				if (FILE *f = fopen(util::string_format("%s/f%04u_%s.jpg", dump, num, complete ? "own" : "hdr").c_str(), "wb"))
				{
					fwrite(jpeg.data(), 1, jpeg.size(), f);
					fclose(f);
				}
				if (FILE *f = fopen(util::string_format("%s/f%04u.raw", dump, num).c_str(), "wb"))
				{
					fwrite(data.data(), 1, data.size(), f);
					fclose(f);
				}
			}
			decode_jpeg(jpeg, m_out[0], m_out[1], m_out[2]);
			static const bool l5_log = getenv("ZL5_LOG") != nullptr;
			if (l5_log)
				logerror("DSP frame %u decoded into %08x %08x %08x at %.4f s\n", num, m_out[0], m_out[1], m_out[2], machine().time().as_double());
		}
		else if (fourcc == 0x62773130 || fourcc == 0x62773230) // "01wb" / "02wb"
		{
			const int ch = (fourcc == 0x62773230) ? 1 : 0;
			if (!ch)
				asize = size;
			for (u32 i = 0; i < size; i++)
			{
				wr8(m_aud_wp[ch], ring8(m_rd + 12 + i));
				if (++m_aud_wp[ch] >= m_aud[ch] + m_aud_size)
					m_aud_wp[ch] = m_aud[ch];
			}
		}
		else
			break;
		m_rd = ring_wrap(m_rd + 12 + size);
	}
	if (!have_video)
	{
		LOGMASKED(LOG_CMD, "MJP: no video chunk at %08x\n", m_rd);
		m_playing = false;
		return false;
	}

	const u32 m = m_w + MSG_BLOCK;
	wr32(m + 12, 0);
	wr32(m + 16, m_aud_wp[0]);
	wr32(m + 20, asize);
	wr32(m + 24, 0);
	wr32(m + 28, frame_no + 1); // frames count from 1: FA pauses when this reaches its target (0 = none)
	wr32(m + 32, 0);
	wr32(m + 36, vsize);
	wr32(m + 40, 0);
	wr32(m + 44, m_rd);
	wr32(m + 64, 0);
	wr32(m + 0, 0x92);
	m_frame = frame_no + 1;
	post_message();
	return true;
}

// decode a baseline JPEG into planar YCbCr 4:2:0 at the given addresses
bool zevio_dsp_hle_device::decode_jpeg(const std::vector<u8> &jpeg, u32 ybase, u32 ubase, u32 vbase)
{
	jpeg_decompress_struct cinfo;
	jpeg_error_jmp jerr;
	cinfo.err = jpeg_std_error(&jerr.pub);
	jerr.pub.error_exit = jpeg_error_exit_jmp;
	jerr.pub.output_message = jpeg_output_nothing;
	jpeg_create_decompress(&cinfo);
	if (setjmp(jerr.jump))
	{
		jpeg_destroy_decompress(&cinfo);
		return false;
	}
	jpeg_mem_src(&cinfo, const_cast<u8 *>(jpeg.data()), jpeg.size());
	jpeg_read_header(&cinfo, TRUE);
	cinfo.out_color_space = JCS_YCbCr;
	jpeg_start_decompress(&cinfo);
	const u32 w = cinfo.output_width;
	std::vector<u8> line(w * cinfo.output_components);
	while (cinfo.output_scanline < cinfo.output_height)
	{
		const u32 y = cinfo.output_scanline;
		JSAMPROW row = line.data();
		jpeg_read_scanlines(&cinfo, &row, 1);
		for (u32 x = 0; x < w; x++)
		{
			wr8(ybase + y * w + x, line[x * 3]);
			if (!(x & 1) && !(y & 1))
			{
				wr8(ubase + (y / 2) * (w / 2) + x / 2, line[x * 3 + 1]);
				wr8(vbase + (y / 2) * (w / 2) + x / 2, line[x * 3 + 2]);
			}
		}
	}
	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);
	return true;
}
