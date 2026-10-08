// license:BSD-3-Clause
// copyright-holders:Riley
/***************************************************************************

    LSI Zevio 1020 "Hanabi" GPU (0xA8000000) and number converters
    (0xA4000000), as used by the VTech V.Flash / V.Smile Pro.

    The command processor executes display lists from SDRAM and draws into
    the tiled 16-bit VRAM surface: 1024 pixels (2048 bytes) wide, 32x8-pixel
    tiles of 512 bytes, origin at register +0x98. All drawing coordinates are
    pixels relative to that origin.

    Knowledge comes from the boot ROM and the V2GE engine in the games' FA.VFF
    (Bratz). Display lists are executed instantly when kicked.

    Registers:
    +0x00 command (1 = run list at +0x28, 3 = reset), upper half engine state
    +0x10 IRQ enable (6 bits), +0x14 IRQ status (bits 0-5, tag in 8-15)
    +0x28 list address, +0x5C fence value, +0x98 VRAM origin

    Display list words (top byte = opcode):
    04 return from 09 call       08,a jump          09,a call
    0A,a vertex base (indexed)   0C,a transfer/transform destination
    0E|n,a data feed for D0      10,v fence         12 (unknown, no-op)
    14 end of list (IRQ, tag)    1C/1E mode words   20RRNNNN register block
    40/44/48/4C/50 3D strip/list/line strip/line list/points (fmt in 16-23)
    6Fx transform vertices to memory
    81 projection centre  82/83 clip  84 draw mode  85 2D offset
    86 texture mode  87 texture origin  88 mode  89 texture cache flush
    8A-8C fog  8D blend factor  8E lookup tables  8F-94 CLUT origins
    C0-C4 2D primitives  C8 sprite  D0/D4 load image  D1 store  D3 fill
    95/96 block terminators

    TODO:
    - lighting (directional lights 0x60-0x6E), fog, lookup tables (0x8E)
    - exact meaning of draw-mode bits (depth test/write, culling)
    - 0x12, the 0x02 vertex extension, 0x6x transform variants
    - timing (lists complete instantly)

***************************************************************************/

#include "emu.h"
#include "zevio_gpu.h"

#include <map>
#include <thread>

#define LOG_UNKNOWN (1U << 1)
#define LOG_LIST    (1U << 2)

#define VERBOSE (LOG_UNKNOWN)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ZEVIO_GPU, zevio_gpu_device, "zevio_gpu", "Zevio 1020 GPU")

namespace {

constexpr int ZW = 1024, ZH = 2048; // depth buffer in absolute GPU coordinates

inline void bgr555_to_f(u16 c, float &r, float &g, float &b)
{
	constexpr float k = 1.0f / 31.0f;
	r = (c & 0x1f) * k;
	g = ((c >> 5) & 0x1f) * k;
	b = ((c >> 10) & 0x1f) * k;
}

inline u16 f_to_bgr555(float r, float g, float b)
{
	const int ri = std::clamp(int(r * 31.0f + 0.5f), 0, 31);
	const int gi = std::clamp(int(g * 31.0f + 0.5f), 0, 31);
	const int bi = std::clamp(int(b * 31.0f + 0.5f), 0, 31);
	return 0x8000 | (bi << 10) | (gi << 5) | ri;
}

} // anonymous namespace


zevio_gpu_device::zevio_gpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, ZEVIO_GPU, tag, owner, clock),
	m_irq_cb(*this),
	m_ram(nullptr),
	m_ram_base(0),
	m_ram_size(0)
{
}

void zevio_gpu_device::device_start()
{
	m_zbuf = std::make_unique<float[]>(ZW * ZH);
	m_bands = std::clamp<int>(std::thread::hardware_concurrency(), 1, MAX_BANDS);
	m_queue = (m_bands > 1) ? osd_work_queue_alloc(WORK_QUEUE_FLAG_MULTI | WORK_QUEUE_FLAG_HIGH_FREQ) : nullptr;
	m_async = !getenv("ZGPU_SYNC");
	if (m_async)
		m_thread = std::thread([this] { render_thread(); });
	save_item(NAME(m_regs));
	save_item(NAME(m_cvt));
	save_item(NAME(m_r));
	save_item(NAME(m_w1c));
	save_item(NAME(m_w1e));
	save_item(NAME(m_w84));
	save_item(NAME(m_w88));
	save_item(NAME(m_w81));
	save_item(NAME(m_w85));
	save_item(NAME(m_w86));
	save_item(NAME(m_w87));
	save_item(NAME(m_w8d));
	save_item(NAME(m_clip0));
	save_item(NAME(m_clip1));
	save_item(NAME(m_clut));
	save_item(NAME(m_vtxbase));
	save_item(NAME(m_xfer));
}

void zevio_gpu_device::device_stop()
{
	if (m_thread.joinable())
	{
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_quit = true;
		}
		m_cv.notify_all();
		m_thread.join();
	}
	if (m_queue)
		osd_work_queue_free(m_queue);
	m_queue = nullptr;
}

void zevio_gpu_device::device_pre_save()
{
	sync();
}

void zevio_gpu_device::device_reset()
{
	sync();
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	std::fill(std::begin(m_cvt), std::end(m_cvt), 0);
	std::fill(std::begin(m_r), std::end(m_r), 0);
	m_r[0x3c] = m_r[0x3d] = m_r[0x3e] = 0x3e00;
	m_r[0x40] = m_r[0x44] = m_r[0x48] = 0x3e00;
	m_r[0x70] = 0x7fff;
	m_w1c = 0x6e60;
	m_w1e = 0x8030;
	m_w84 = 0x17;
	m_w88 = 0;
	m_w81 = m_w85 = m_w86 = m_w87 = 0;
	m_w8d = 0xffffff;
	m_clip0 = 0;
	m_clip1 = 0xffffff;
	std::fill(std::begin(m_clut), std::end(m_clut), 0);
	m_vtxbase = m_xfer = 0;
	std::fill_n(&m_zbuf[0], ZW * ZH, std::numeric_limits<float>::max());
	m_zdirty_y0 = ZH;
	m_zdirty_y1 = -1;
	update_irq();
}


/***************************************************************************
    GPU 16-bit float: sign, 6-bit exponent (bias 31), 9-bit mantissa
***************************************************************************/

float zevio_gpu_device::f16_to_float(u16 v)
{
	static const auto table = [] ()
	{
		auto t = std::make_unique<float[]>(0x10000);
		for (int i = 0; i < 0x10000; i++)
		{
			const float f = (i & 0x7fff) ? ldexpf(1.0f + (i & 0x1ff) / 512.0f, int((i >> 9) & 0x3f) - 31) : 0.0f;
			t[i] = (i & 0x8000) ? -f : f;
		}
		return t;
	}();
	return table[v];
}

u16 zevio_gpu_device::float_to_f16(float f)
{
	if (f == 0.0f || std::isnan(f))
		return 0;
	const u16 sign = (f < 0) ? 0x8000 : 0;
	int e;
	const float m = frexpf(std::fabs(f), &e); // |f| = m * 2^e, m in [0.5, 1)
	int exp = e - 1 + 31;
	if (exp <= 0)
		return 0;
	if (exp > 63)
		return sign | 0x7fff;
	const u16 mant = u16((m * 2.0f - 1.0f) * 512.0f) & 0x1ff; // truncate like the engine
	return sign | (exp << 9) | mant;
}


/***************************************************************************
    Memory
***************************************************************************/

u32 zevio_gpu_device::rd32(u32 addr) const
{
	const u32 off = addr - m_ram_base;
	if (!m_ram || off >= m_ram_size)
		return 0;
	return m_ram[off >> 2];
}

void zevio_gpu_device::wr32(u32 addr, u32 data)
{
	const u32 off = addr - m_ram_base;
	if (m_ram && off < m_ram_size)
		m_ram[off >> 2] = data;
}

void zevio_gpu_device::wr16(u32 addr, u16 data)
{
	const u32 off = addr - m_ram_base;
	if (!m_ram || off >= m_ram_size)
		return;
	const int shift = BIT(addr, 1) * 16;
	u32 &w = m_ram[off >> 2];
	w = (w & ~(0xffffU << shift)) | (u32(data) << shift);
}

// GPU coordinates are absolute in the tiled surface whose origin is 0x0FFD0000:
// FA's clip rectangles and its three display buffers agree on this. +0x98 is
// not an origin; FA points it at the only 512x240 area that is not a display
// buffer (with +0x90/+0x94 = 240 << 16 | 512), so it is probably the depth
// buffer. TODO: confirm, and whether rows 0-95 (below SDRAM) are memory.


void zevio_gpu_device::vram_w(int x, int y, u16 data)
{
	wr16(vram_addr(x, y), data); // rows below SDRAM are dropped by wr16
}


/***************************************************************************
    Registers
***************************************************************************/

void zevio_gpu_device::update_irq()
{
	m_irq_cb((m_regs[0x14 / 4] & m_regs[0x10 / 4] & 0x3f) ? ASSERT_LINE : CLEAR_LINE);
}

/*
    FA's GPU driver queues hundreds of small display lists per frame and
    starts each from the end-of-list interrupt (tag 0xFD): mark the finished
    entry done (status word 3), count--, read index++, then status 1 and kick
    for the next entry. Doing that bookkeeping here and running the next list
    directly saves an interrupt round trip per list, which was most of the
    emulated CPU time. The handler is found by its code and its literal pool;
    if it is not there, or a hook is installed, the real interrupt is used.
*/
static const u32 fa_gpu_irq_sig[] = {
	0xe52de004, 0xe24dd010, 0xe58d000c, 0xe59f30f4, 0xe5933000, 0xe58d3008, 0xe59d2008, 0xe59f30e8,
	0xe0023003, 0xe58d3008, 0xe59f20d8, 0xe3a0303f, 0xe5823000, 0xe59f20d4, 0xe3a03c01, 0xe5823000,
	0xe59d3008, 0xe2033008, 0xe3530000, 0x0a000022, 0xe59d3008, 0xe2033cff, 0xe3530cfd, 0x1a00001e
};

u32 zevio_gpu_device::fa_find_irq_handler()
{
	// FA's code lives in the upper part of SDRAM
	for (u32 a = 0x10900000; a < 0x10c00000; a += 4)
	{
		if (rd32(a) != fa_gpu_irq_sig[0])
			continue;
		bool match = true;
		for (int i = 1; i < std::size(fa_gpu_irq_sig) && match; i++)
			match = rd32(a + i * 4) == fa_gpu_irq_sig[i];
		if (match && rd32(a + 0x108) == 0xa8000014 && rd32(a + 0x110) == 0xdc000004)
			return a;
	}
	return 0;
}

bool zevio_gpu_device::fa_queue_hle()
{
	static const bool off = getenv("ZGPU_NO_QUEUE_HLE") != nullptr;
	if (off || m_regs[0x14 / 4] != 0xfd08 || !(m_regs[0x10 / 4] & 0x08))
		return false;
	// look for the handler again at most once per emulated second
	if (!m_fa_handler || rd32(m_fa_handler) != fa_gpu_irq_sig[0])
	{
		const double now = machine().time().as_double();
		if (now < m_fa_next_scan)
			return false;
		m_fa_next_scan = now + 1.0;
		m_fa_handler = fa_find_irq_handler();
		if (!m_fa_handler)
			return false;
		logerror("FA GPU queue handler at %08x\n", m_fa_handler);
	}
	const u32 q = rd32(m_fa_handler + 0x114), busy = rd32(m_fa_handler + 0x118), hook = rd32(m_fa_handler + 0x11c);
	if (rd32(hook) != 0 || rd32(q + 4) != 1)
		return false;
	u32 idx = rd32(q + 0x10);
	const u32 list = rd32(rd32(q + 0x0c) + idx * 12 + 4);
	if (list != m_regs[0x28 / 4])
		return false;

	wr32(list, 3);
	s32 count = s32(rd32(q + 0x18)) - 1;
	if (count < 0)
		count = 0;
	wr32(q + 0x18, count);
	if (s32(++idx) >= s32(rd32(q + 0x08)))
		idx = 0;
	wr32(q + 0x10, idx);
	m_regs[0x14 / 4] = 0; // acknowledged
	if (count <= 0)
	{
		wr32(busy, 0);
		return false;
	}
	const u32 next = rd32(rd32(q + 0x0c) + idx * 12 + 4);
	wr32(next, 1);
	m_regs[0x28 / 4] = next;
	return true;
}

u32 zevio_gpu_device::regs_r(offs_t offset)
{
	if (offset == 0)
		return m_regs[0] & 0xffff; // engine idle
	return m_regs[offset];
}

void zevio_gpu_device::regs_w(offs_t offset, u32 data, u32 mem_mask)
{
	static int irqlog = getenv("ZGPU_IRQLOG") ? 400 : 0;
	if (irqlog && offset <= 0x30 / 4)
	{
		irqlog--;
		logerror("%s: gpu w %02x = %08x (14 %08x 10 %08x)\n", machine().describe_context(), offset * 4, data, m_regs[0x14 / 4], m_regs[0x10 / 4]);
	}
	switch (offset)
	{
	case 0x00 / 4:
		if (data == 1)
		{
			m_defer_irq = true;
			run_list(m_regs[0x28 / 4]);
			while (fa_queue_hle())
				run_list(m_regs[0x28 / 4]);
			submit();
			m_defer_irq = false;
			update_irq();
		}
		else if (data == 3)
		{
			m_regs[0x14 / 4] = 0;
			update_irq();
		}
		return;
	case 0x14 / 4:
		if (data & 0x3f)
			m_regs[offset] &= ~(data & 0x3f);
		if (!(m_regs[offset] & 0x3f))
			m_regs[offset] = 0;
		update_irq();
		return;
	case 0x10 / 4:
		COMBINE_DATA(&m_regs[offset]);
		update_irq();
		return;
	default:
		COMBINE_DATA(&m_regs[offset]);
		return;
	}
}

u32 zevio_gpu_device::cvt_r(offs_t offset)
{
	const u32 v = m_cvt[offset];
	switch (offset)
	{
	case 0x00 / 4: // IEEE single -> f16
		return float_to_f16(*reinterpret_cast<const float *>(&v));
	case 0x08 / 4: // 20.12 fixed (27 bits) -> f16
		return float_to_f16(util::sext(v, 27) / 4096.0f);
	case 0x18 / 4: // 20.12 fixed -> f16
		return float_to_f16(s32(v) / 4096.0f);
	case 0x0c / 4: case 0x1c / 4: // f16 -> 20.12 fixed
		return u32(s32(f16_to_float(v) * 4096.0f));
	default:
		return v;
	}
}

void zevio_gpu_device::cvt_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_cvt[offset]);
}


/***************************************************************************
    Pixels
***************************************************************************/

bool zevio_gpu_device::in_clip(const rstate &s, int x, int y) const
{
	return x >= int(s.clip0 & 0xfff) && x <= int(s.clip1 & 0xfff)
		&& y >= int((s.clip0 >> 12) & 0xfff) && y <= int((s.clip1 >> 12) & 0xfff)
		&& x >= 0 && x < 1024 && y >= 0;
}

// sample the texture; u/v normalized (3D) or in texels (2D)
bool zevio_gpu_device::texel(const rstate &s, rcache &pc, float u, float v, bool normalized, float &r, float &g, float &b, float &a) const
{
	const u32 mode = s.w86;
	const int w = 8 << (mode & 7), h = 8 << ((mode >> 3) & 7);
	if (!normalized || !(mode & 0xc0))
		return texel_point(s, pc, normalized ? int(std::floor(u * w)) : int(u), normalized ? int(std::floor(v * h)) : int(v), r, g, b, a);

	// bilinear filtering (texture mode bits 6-7, set by the 3D engine)
	// TODO: confirm what the two bits select
	const float fu = u * w - 0.5f, fv = v * h - 0.5f;
	const int iu = int(std::floor(fu)), iv = int(std::floor(fv));
	const float du = fu - iu, dv = fv - iv;
	if (!BIT(mode, 8))
	{
		// BGR555 texels: 8-bit fixed point weights, transparent texels drop out
		const int wu = int(du * 256.0f), wv = int(dv * 256.0f);
		const int wt[4] = { (256 - wu) * (256 - wv), wu * (256 - wv), (256 - wu) * wv, wu * wv };
		int sr = 0, sg = 0, sb = 0, wsum = 0;
		for (int k = 0; k < 4; k++)
		{
			const int t = texel_raw(s, pc, iu + (k & 1), iv + (k >> 1));
			if (t >= 0)
			{
				sr += (t & 0x1f) * wt[k];
				sg += ((t >> 5) & 0x1f) * wt[k];
				sb += ((t >> 10) & 0x1f) * wt[k];
				wsum += wt[k];
			}
		}
		if (wsum < 32768)
			return false; // mostly transparent here
		const float k = 1.0f / (float(wsum) * 31.0f);
		r = sr * k; g = sg * k; b = sb * k; a = 1.0f;
		return true;
	}
	float sr = 0, sg = 0, sb = 0, sa = 0, wsum = 0;
	for (int k = 0; k < 4; k++)
	{
		const int ou = k & 1, ov = k >> 1;
		const float wt = (ou ? du : 1.0f - du) * (ov ? dv : 1.0f - dv);
		float cr, cg, cb, ca;
		if (texel_point(s, pc, iu + ou, iv + ov, cr, cg, cb, ca))
		{
			sr += cr * wt; sg += cg * wt; sb += cb * wt; sa += ca * wt;
			wsum += wt;
		}
	}
	if (wsum < 0.5f)
		return false; // mostly transparent here
	r = sr / wsum; g = sg / wsum; b = sb / wsum; a = sa / wsum;
	return true;
}

// raw texel after the CLUT: -1 if transparent (palette index 0; for
// non-ARGB4444 textures also 0x0000 and bit 15 set: Bratz panel backgrounds
// are 0x0000, Cars uses 0x8000 billboards, 0xFC1F text boxes, 0xFFFF icons)
int zevio_gpu_device::texel_raw(const rstate &s, rcache &pc, int tu, int tv) const
{
	const u32 mode = s.w86;
	tu &= (8 << (mode & 7)) - 1;
	tv &= (8 << ((mode >> 3) & 7)) - 1;
	const int x0 = s.w87 & 0xfff, y0 = (s.w87 >> 12) & 0xfff;
	u16 c;
	switch ((mode >> 9) & 3)
	{
	case 0:
		c = vram_r(x0 + tu, y0 + tv);
		break;
	case 1:
	{
		const u16 hw = vram_r(x0 + (tu >> 1), y0 + tv);
		const u8 idx = BIT(tu, 0) ? (hw >> 8) : (hw & 0xff);
		if (!idx)
			return -1;
		if (pc.key != s.clut0)
		{
			for (int i = 0; i < 256; i++)
				pc.pal[i] = vram_r((s.clut0 & 0xfff) + i, (s.clut0 >> 12) & 0xfff);
			pc.key = s.clut0;
		}
		c = pc.pal[idx];
		break;
	}
	default:
	{
		const u16 hw = vram_r(x0 + (tu >> 2), y0 + tv);
		const u8 idx = (hw >> ((tu & 3) * 4)) & 0x0f;
		if (!idx)
			return -1;
		if (pc.key != (s.clut3 | 0x80000000U))
		{
			for (int i = 0; i < 16; i++)
				pc.pal[i] = vram_r((s.clut3 & 0xfff) + i, (s.clut3 >> 12) & 0xfff);
			pc.key = s.clut3 | 0x80000000U;
		}
		c = pc.pal[idx];
		break;
	}
	}
	if (!BIT(mode, 8) && (c == 0 || BIT(c, 15)))
		return -1;
	return c;
}

bool zevio_gpu_device::texel_point(const rstate &s, rcache &pc, int tu, int tv, float &r, float &g, float &b, float &a) const
{
	const int t = texel_raw(s, pc, tu, tv);
	if (t < 0)
		return false;
	const u16 c = t;
	if (BIT(s.w86, 8)) // ARGB4444
	{
		r = (c & 15) / 15.0f;
		g = ((c >> 4) & 15) / 15.0f;
		b = ((c >> 8) & 15) / 15.0f;
		a = ((c >> 12) & 15) / 15.0f;
		return a > 0.0f;
	}
	bgr555_to_f(c, r, g, b);
	a = 1.0f;
	return true;
}

void zevio_gpu_device::plot(const rstate &s, int x, int y, float z, float r, float g, float b, float a, bool depth)
{
	if (!in_clip(s, x, y))
		return;
	if (depth && y < ZH)
	{
		// 0x84 bit 0 = depth test, bit 1 = depth write (Cars draws its sky and
		// ground with both clear, models with both set)
		float &zb = m_zbuf[y * ZW + x];
		if (BIT(s.w84, 0) && z > zb)
			return;
		if (BIT(s.w84, 1) && !BIT(s.w88, 1)) // translucent pass tests but does not write
			zb = z;
	}
	if (BIT(s.w88, 1) || a < 1.0f)
	{
		// blend with the destination; 0x8D gives per-channel factors
		float dr, dg, db;
		bgr555_to_f(vram_r(x, y), dr, dg, db);
		const float kr = BIT(s.w88, 1) ? ((s.w8d & 0xff) / 255.0f) * a : a;
		const float kg = BIT(s.w88, 1) ? (((s.w8d >> 8) & 0xff) / 255.0f) * a : a;
		const float kb = BIT(s.w88, 1) ? (((s.w8d >> 16) & 0xff) / 255.0f) * a : a;
		r = r * kr + dr * (1.0f - kr);
		g = g * kg + dg * (1.0f - kg);
		b = b * kb + db * (1.0f - kb);
	}
	if (m_count_stats)
	{
		m_stat_pixels++;
		m_total_pixels++;
	}
	vram_w(x, y, f_to_bgr555(r, g, b));
}


/***************************************************************************
    3D
***************************************************************************/

int zevio_gpu_device::vertex_words(u8 fmt) const
{
	return 2 + (BIT(fmt, 5) ? 1 : 0) + (BIT(fmt, 4) ? 1 : 0) + ((fmt & 0x0c) ? 1 : 0) + (BIT(fmt, 1) ? 2 : 0);
}

zevio_gpu_device::vertex_t zevio_gpu_device::fetch_vertex(u32 addr, u8 fmt) const
{
	vertex_t v{};
	const u32 w0 = rd32(addr), w1 = rd32(addr + 4);
	v.x = f16_to_float(w0 & 0xffff);
	v.y = f16_to_float(w0 >> 16);
	v.z = f16_to_float(w1 & 0xffff);
	int i = 2;
	if (BIT(fmt, 5))
	{
		v.lit = true;
		const u32 w2 = rd32(addr + 8);
		v.nx = f16_to_float(w1 >> 16);
		v.ny = f16_to_float(w2 & 0xffff);
		v.nz = f16_to_float(w2 >> 16);
		i++;
	}
	if (BIT(fmt, 4))
	{
		const u32 uv = rd32(addr + i * 4);
		v.u = f16_to_float(uv & 0xffff);
		v.v = f16_to_float(uv >> 16);
		i++;
	}
	v.r = v.g = v.b = 1.0f;
	v.hi = 0xffff;
	if (fmt & 0x0c)
	{
		const u32 c = rd32(addr + i * 4);
		if (BIT(fmt, 3))
			bgr555_to_f(c & 0xffff, v.r, v.g, v.b);
		v.hi = c >> 16;
	}
	return v;
}

// modelview: P = R40 (S3C * p) + T4C
void zevio_gpu_device::transform(vertex_t &v) const
{
	const float sx = v.x * reg_f(0x3c), sy = v.y * reg_f(0x3d), sz = v.z * reg_f(0x3e);
	const float x = reg_f(0x40) * sx + reg_f(0x41) * sy + reg_f(0x42) * sz + reg_f(0x4c);
	const float y = reg_f(0x43) * sx + reg_f(0x44) * sy + reg_f(0x45) * sz + reg_f(0x4d);
	const float z = reg_f(0x46) * sx + reg_f(0x47) * sy + reg_f(0x48) * sz + reg_f(0x4e);
	v.x = x;
	v.y = y;
	v.z = -z; // eye space looks down -Z (Bratz menu: character z -1.3..-0.6); keep positive depth

	// lighting only for vertices with normals: ambient (0x70) plus three
	// directional lights (directions 0x60/0x64/0x68, colours 0x6C-0x6E)
	if (!v.lit)
		return;
	float nx = reg_f(0x40) * v.nx + reg_f(0x41) * v.ny + reg_f(0x42) * v.nz;
	float ny = reg_f(0x43) * v.nx + reg_f(0x44) * v.ny + reg_f(0x45) * v.nz;
	float nz = reg_f(0x46) * v.nx + reg_f(0x47) * v.ny + reg_f(0x48) * v.nz;
	const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
	if (len > 0.0f)
	{
		nx /= len; ny /= len; nz /= len;
	}
	float lr, lg, lb;
	bgr555_to_f(m_r[0x70], lr, lg, lb);
	for (int i = 0; i < 3; i++)
	{
		const int d = 0x60 + i * 4;
		const float dot = -(nx * reg_f(d) + ny * reg_f(d + 1) + nz * reg_f(d + 2));
		if (dot <= 0.0f)
			continue;
		float cr, cg, cb;
		bgr555_to_f(m_r[0x6c + i], cr, cg, cb);
		lr += cr * dot;
		lg += cg * dot;
		lb += cb * dot;
	}
	v.r *= std::min(lr, 1.0f);
	v.g *= std::min(lg, 1.0f);
	v.b *= std::min(lb, 1.0f);
}

// perspective projection to screen; returns false if behind the near plane
bool zevio_gpu_device::project(vertex_t &v) const
{
	const float near_z = 1.0f / 4096.0f; // TODO: which register is the near plane (0x75 reads 17.3)
	if (v.z < near_z)
		return false;
	const float fx = reg_f(0x78), fy = reg_f(0x79);
	const float cx = m_w81 & 0xfff, cy = (m_w81 >> 12) & 0xfff;
	v.w = 1.0f / v.z;
	v.x = cx + fx * v.x * v.w;
	v.y = cy - fy * v.y * v.w; // TODO: confirm the Y direction
	return true;
}

void zevio_gpu_device::draw_triangle(const rstate &s, rcache &pc, const vertex_t &a, const vertex_t &b, const vertex_t &c, bool textured, int band_y0, int band_y1)
{
	// screen-space edge functions over the bounding box, perspective-correct UVs
	const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	if (m_count_stats)
	{
		m_stat_tris++;
		m_total_tris++;
	}
	if (std::fabs(area) < 1e-6f)
		return;
	const int cx0 = std::max<int>(s.clip0 & 0xfff, 0), cy0 = std::max<int>((s.clip0 >> 12) & 0xfff, 0);
	const int cx1 = std::min<int>(s.clip1 & 0xfff, 1023), cy1 = std::min<int>((s.clip1 >> 12) & 0xfff, ZH - 1);
	const int minx = std::max(cx0, int(std::floor(std::min({ a.x, b.x, c.x }))));
	const int maxx = std::min(cx1, int(std::ceil(std::max({ a.x, b.x, c.x }))));
	const int miny = std::max({ cy0, band_y0, int(std::floor(std::min({ a.y, b.y, c.y }))) });
	const int maxy = std::min({ cy1, band_y1 - 1, int(std::ceil(std::max({ a.y, b.y, c.y }))) });
	if (minx > maxx || miny > maxy)
		return;
	const float inv_area = 1.0f / area;
	// barycentrics step linearly along a row
	const float d0 = (b.y - c.y) * inv_area, d1 = (c.y - a.y) * inv_area;
	for (int y = miny; y <= maxy; y++)
	{
		const float py = y + 0.5f;
		const float px0 = minx + 0.5f;
		float w0 = ((b.x - px0) * (c.y - py) - (b.y - py) * (c.x - px0)) * inv_area - d0;
		float w1 = ((c.x - px0) * (a.y - py) - (c.y - py) * (a.x - px0)) * inv_area - d1;
		bool inside = false;
		for (int x = minx; x <= maxx; x++)
		{
			w0 += d0;
			w1 += d1;
			const float w2 = 1.0f - w0 - w1;
			if (w0 < 0 || w1 < 0 || w2 < 0)
			{
				if (inside)
					break; // left the (convex) triangle on this row
				continue;
			}
			inside = true;
			const float iw = w0 * a.w + w1 * b.w + w2 * c.w;
			const float z = 1.0f / iw;
			float r = w0 * a.r + w1 * b.r + w2 * c.r;
			float g = w0 * a.g + w1 * b.g + w2 * c.g;
			float bl = w0 * a.b + w1 * b.b + w2 * c.b;
			float alpha = 1.0f;
			if (textured)
			{
				const float u = (w0 * a.u * a.w + w1 * b.u * b.w + w2 * c.u * c.w) * z;
				const float v = (w0 * a.v * a.w + w1 * b.v * b.w + w2 * c.v * c.w) * z;
				float tr, tg, tb, ta;
				if (!texel(s, pc, u, v, true, tr, tg, tb, ta))
					continue;
				r *= tr;
				g *= tg;
				bl *= tb;
				alpha = ta;
			}
			plot(s, x, y, z, r, g, bl, alpha, true);
		}
	}
}

void zevio_gpu_device::draw_3d_tri(vertex_t a, vertex_t b, vertex_t c)
{
	if (m_no3d) return;
	// clip against the near plane in eye space, then project
	const float near_z = 1.0f / 4096.0f;
	vertex_t in[3] = { a, b, c };
	vertex_t out[4];
	int n = 0;
	for (int i = 0; i < 3; i++)
	{
		const vertex_t &p = in[i], &q = in[(i + 1) % 3];
		const bool pin = p.z >= near_z, qin = q.z >= near_z;
		if (pin)
			out[n++] = p;
		if (pin != qin)
		{
			const float t = (near_z - p.z) / (q.z - p.z);
			vertex_t m;
			m.x = p.x + (q.x - p.x) * t; m.y = p.y + (q.y - p.y) * t; m.z = near_z;
			m.u = p.u + (q.u - p.u) * t; m.v = p.v + (q.v - p.v) * t;
			m.r = p.r + (q.r - p.r) * t; m.g = p.g + (q.g - p.g) * t; m.b = p.b + (q.b - p.b) * t;
			m.nx = m.ny = m.nz = 0; m.w = 0; m.hi = p.hi;
			out[n++] = m;
		}
	}
	if ((VERBOSE & LOG_LIST) && (m_stat_dbg2++ % 20000) == 0)
		LOGMASKED(LOG_LIST, "tri3d n=%d z %f %f %f total tris %d pixels %d xy %f,%f clip %06x-%06x\n", n, a.z, b.z, c.z, m_total_tris, m_total_pixels, a.x, a.y, m_clip0, m_clip1);
	if (n < 3)
		return;
	for (int i = 0; i < n; i++)
		project(out[i]);
	rcmd cmd{};
	cmd.type = CMD_TRI3D;
	cmd.textured = BIT(m_w1c, 6);
	cmd.st = cur_state();
	cmd.v[0] = out[0]; cmd.v[1] = out[1]; cmd.v[2] = out[2];
	if (n == 4)
	{
		add_cmd(rcmd(cmd));
		cmd.v[1] = out[2]; cmd.v[2] = out[3];
	}
	add_cmd(std::move(cmd));
}

void zevio_gpu_device::draw_line(const rstate &s, const vertex_t &a, const vertex_t &b, int y0, int y1)
{
	const int steps = std::max(1, int(std::max(std::fabs(b.x - a.x), std::fabs(b.y - a.y))));
	for (int i = 0; i <= steps; i++)
	{
		const float t = float(i) / steps;
		const int y = int(a.y + (b.y - a.y) * t);
		if (y >= y0 && y < y1)
			plot(s, int(a.x + (b.x - a.x) * t), y, 0.0f,
					a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, 1.0f, false);
	}
}

void zevio_gpu_device::prim3d(u32 header, u32 &addr)
{
	static const bool no3d = getenv("ZGPU_NO_3D") != nullptr;
	m_no3d = no3d;
	// debug: ZGPU_ONLY=<hex upper half> draws only primitives with that header
	static const char *only = getenv("ZGPU_ONLY");
	if (only && (header >> 16) != strtoul(only, nullptr, 16))
		m_no3d = true;
	const u8 kind = header >> 24;
	const u8 fmt = (header >> 16) & 0xff;
	const int n = header & 0xffff;
	const int wpv = vertex_words(fmt & 0x3f);

	std::vector<vertex_t> verts(n);
	if (BIT(fmt, 6))
	{
		// indexed: packed u16 word offsets from the 0x0A vertex base
		for (int i = 0; i < n; i++)
		{
			const u16 idx = rd16(addr + i * 2);
			verts[i] = fetch_vertex(m_vtxbase + idx * 4, fmt & 0x3f);
		}
		addr += ((n + 1) / 2) * 4;
	}
	else
	{
		for (int i = 0; i < n; i++)
			verts[i] = fetch_vertex(addr + i * wpv * 4, fmt & 0x3f);
		addr += n * wpv * 4;
	}
	for (auto &v : verts)
		transform(v);
	static const bool zlog = getenv("ZGPU_ZLOG") != nullptr;
	if (zlog && n)
	{
		static std::map<u32, std::pair<float, float>> zr;
		static int cnt = 0;
		auto &e = zr.try_emplace(header >> 16, std::make_pair(1e9f, -1e9f)).first->second;
		for (auto &v : verts)
		{
			e.first = std::min(e.first, v.z);
			e.second = std::max(e.second, v.z);
		}
		if (++cnt % 50000 == 0)
			for (auto &k : zr)
				LOGMASKED(LOG_UNKNOWN, "ZRANGE %04x z %f .. %f d84 %06x d88 %06x\n", k.first, k.second.first, k.second.second, m_w84, m_w88);
	}
	if ((VERBOSE & LOG_LIST) && n && (m_stat_dbg++ % 5000) == 0)
	{
		m_stat_dbg++;
		vertex_t raw = fetch_vertex(BIT(fmt, 6) ? m_vtxbase + rd16(addr - ((n + 1) / 2) * 4) * 4 : addr - n * wpv * 4, fmt & 0x3f);
		LOGMASKED(LOG_LIST, "prim %08x v0 raw (%f %f %f) eye (%f %f %f) S %f %f %f R %f %f %f / %f %f %f / %f %f %f T %f %f %f f %f %f c81 %06x\n", header,
				raw.x, raw.y, raw.z, verts[0].x, verts[0].y, verts[0].z,
				reg_f(0x3c), reg_f(0x3d), reg_f(0x3e),
				reg_f(0x40), reg_f(0x41), reg_f(0x42), reg_f(0x43), reg_f(0x44), reg_f(0x45), reg_f(0x46), reg_f(0x47), reg_f(0x48),
				reg_f(0x4c), reg_f(0x4d), reg_f(0x4e), reg_f(0x78), reg_f(0x79), m_w81);
	}

	// debug: ZGPU_PRIMLOG=<time> logs every 3D primitive drawn in the 0.05 s after it
	static double primlog_at = getenv("ZGPU_PRIMLOG") ? atof(getenv("ZGPU_PRIMLOG")) : -1.0;
	const double now = machine().time().as_double();
	const bool primlog = primlog_at >= 0 && now >= primlog_at && now < primlog_at + 0.05;
	m_count_stats = primlog || (VERBOSE & LOG_LIST);
	const int pix0 = m_stat_pixels, tri0 = m_stat_tris;

	switch (kind)
	{
	case 0x40: // triangle strip
		for (int i = 0; i + 2 < n; i++)
			draw_3d_tri(verts[i], verts[i + 1], verts[i + 2]);
		break;
	case 0x44: // triangle list
		for (int i = 0; i + 2 < n; i += 3)
			draw_3d_tri(verts[i], verts[i + 1], verts[i + 2]);
		break;
	case 0x48: case 0x4c: // line strip / list
		for (int i = 0; i + 1 < n; i += (kind == 0x48) ? 1 : 2)
		{
			vertex_t a = verts[i], b = verts[i + 1];
			if (project(a) && project(b))
			{
				rcmd c{};
				c.type = CMD_LINE;
				c.st = cur_state();
				c.v[0] = a; c.v[1] = b;
				add_cmd(std::move(c));
			}
		}
		break;
	case 0x50: // points
		for (auto v : verts)
			if (project(v))
			{
				rcmd c{};
				c.type = CMD_POINT;
				c.st = cur_state();
				c.v[0] = v;
				c.flags = 1; // depth
				add_cmd(std::move(c));
			}
		break;
	default:
		LOGMASKED(LOG_UNKNOWN, "unknown 3D primitive %08x\n", header);
		break;
	}
	if (primlog && n)
	{
		float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f, z0 = 1e9f, z1 = -1e9f;
		for (auto v : verts)
		{
			z0 = std::min(z0, v.z); z1 = std::max(z1, v.z);
			if (project(v))
			{
				x0 = std::min(x0, v.x); x1 = std::max(x1, v.x);
				y0 = std::min(y0, v.y); y1 = std::max(y1, v.y);
			}
		}
		logerror("PRIM %08x tris %d pix %d x %.0f..%.0f y %.0f..%.0f z %.3f..%.3f 1c %06x 1e %06x 84 %06x 88 %06x 86 %06x 87 %06x 8d %06x clip %06x-%06x\n",
				header, m_stat_tris - tri0, m_stat_pixels - pix0, x0, x1, y0, y1, z0, z1, m_w1c, m_w1e, m_w84, m_w88, m_w86, m_w87, m_w8d, m_clip0, m_clip1);
		if (BIT(m_w1c, 6))
			log_texture();
	}
}

// debug: histogram of the current texture's 16-bit colours
void zevio_gpu_device::log_texture()
{
	sync();
	// texture colour histogram (raw 16-bit values after the CLUT)
	std::map<u16, int> hist;
	const int w = 8 << (m_w86 & 7), h = 8 << ((m_w86 >> 3) & 7);
	const int tx = m_w87 & 0xfff, ty = (m_w87 >> 12) & 0xfff;
	for (int v = 0; v < h; v++)
		for (int u = 0; u < w; u++)
		{
			u16 c;
			switch ((m_w86 >> 9) & 3)
			{
			case 0: c = vram_r(tx + u, ty + v); break;
			case 1: { const u16 hw = vram_r(tx + u / 2, ty + v); c = vram_r((m_clut[0] & 0xfff) + (BIT(u, 0) ? (hw >> 8) : (hw & 0xff)), (m_clut[0] >> 12) & 0xfff); break; }
			default: { const u16 hw = vram_r(tx + u / 4, ty + v); c = vram_r((m_clut[3] & 0xfff) + ((hw >> ((u & 3) * 4)) & 15), (m_clut[3] >> 12) & 0xfff); break; }
			}
			hist[c]++;
		}
	std::vector<std::pair<int, u16>> top;
	for (auto &e : hist)
		top.emplace_back(e.second, e.first);
	std::sort(top.rbegin(), top.rend());
	std::string s;
	for (int i = 0; i < std::min<int>(6, top.size()); i++)
		s += util::string_format(" %04x:%d", top[i].second, top[i].first);
	logerror("  TEX %dx%d clut %06x %06x:%s\n", w, h, m_clut[0], m_clut[3], s);
}

// 0x6F8: transform n inline vertices into RAM at the 0x0C address, same layout
void zevio_gpu_device::transform_to_memory(u32 header, u32 &addr)
{
	static const bool no_xform = getenv("ZGPU_NO_XFORM") != nullptr;
	if (no_xform) { addr += (header & 0xffff) * vertex_words((header >> 16) & 0x3f) * 4; return; }
	const u8 fmt = (header >> 16) & 0x3f;
	const int n = header & 0xffff;
	const int wpv = vertex_words(fmt);
	for (int i = 0; i < n; i++)
	{
		const u32 src = addr + i * wpv * 4, dst = m_xfer + i * wpv * 4;
		vertex_t v = fetch_vertex(src, fmt);
		const float sx = v.x * reg_f(0x3c), sy = v.y * reg_f(0x3d), sz = v.z * reg_f(0x3e);
		const float x = reg_f(0x40) * sx + reg_f(0x41) * sy + reg_f(0x42) * sz + reg_f(0x4c);
		const float y = reg_f(0x43) * sx + reg_f(0x44) * sy + reg_f(0x45) * sz + reg_f(0x4d);
		const float z = reg_f(0x46) * sx + reg_f(0x47) * sy + reg_f(0x48) * sz + reg_f(0x4e);
		for (int k = 0; k < wpv; k++)
			wr32(dst + k * 4, rd32(src + k * 4));
		wr32(dst, float_to_f16(x) | (u32(float_to_f16(y)) << 16));
		u32 w1 = (rd32(src + 4) & 0xffff0000) | float_to_f16(z);
		if (BIT(fmt, 5))
		{
			const float nx = reg_f(0x50) * v.nx + reg_f(0x51) * v.ny + reg_f(0x52) * v.nz;
			const float ny = reg_f(0x53) * v.nx + reg_f(0x54) * v.ny + reg_f(0x55) * v.nz;
			const float nz = reg_f(0x56) * v.nx + reg_f(0x57) * v.ny + reg_f(0x58) * v.nz;
			w1 = float_to_f16(z) | (u32(float_to_f16(nx)) << 16);
			wr32(dst + 8, float_to_f16(ny) | (u32(float_to_f16(nz)) << 16));
		}
		wr32(dst + 4, w1);
	}
	addr += n * wpv * 4;
}


/***************************************************************************
    2D
***************************************************************************/

void zevio_gpu_device::prim2d(u32 header, u32 &addr)
{
	const u8 op = header >> 24, flags = (header >> 16) & 0xff;
	const int count = header & 0xff;
	const int wpv = 2 + (BIT(flags, 0) ? 1 + (BIT(flags, 2) ? 1 : 0) : 1) + (BIT(flags, 3) ? 1 : 0);
	int n;
	switch (op)
	{
	case 0xc0: case 0xc2: n = (count - 1) / wpv; break;
	case 0xc1: n = 3; break;
	case 0xc3: n = 2; break;
	default: n = 1; break;
	}
	// 2D positions: surface origin (clip top-left) + 0x85 offset + signed x/y
	const int ox = (m_clip0 & 0xfff) + (m_w85 & 0xfff), oy = ((m_clip0 >> 12) & 0xfff) + ((m_w85 >> 12) & 0xfff);
	std::vector<vertex_t> v(n);
	for (int i = 0; i < n; i++)
	{
		const u32 base = addr + i * wpv * 4;
		const u32 xy = rd32(base + 4);
		v[i] = vertex_t{};
		v[i].x = ox + s16(xy & 0xffff);
		v[i].y = oy + s16(xy >> 16);
		v[i].w = 1.0f;
		v[i].z = 0.0f;
		int k = 2;
		u32 col = 0xffffff;
		if (BIT(flags, 0))
		{
			const u32 uv = rd32(base + k++ * 4);
			v[i].u = uv & 0xffff;
			v[i].v = uv >> 16;
			if (BIT(flags, 2))
				col = rd32(base + k++ * 4);
		}
		else
			col = rd32(base + k++ * 4);
		v[i].r = (col & 0xff) / 255.0f;
		v[i].g = ((col >> 8) & 0xff) / 255.0f;
		v[i].b = ((col >> 16) & 0xff) / 255.0f;
	}
	addr += count * 4;

	const bool textured = BIT(flags, 0);
	const rstate st = cur_state();
	auto emit = [&] (u8 type, const vertex_t &a, const vertex_t &b, const vertex_t &c)
	{
		rcmd r{};
		r.type = type;
		r.textured = textured;
		r.st = st;
		r.v[0] = a; r.v[1] = b; r.v[2] = c;
		add_cmd(std::move(r));
	};

	switch (op)
	{
	case 0xc0:
		for (int i = 0; i + 2 < n; i++)
			emit(CMD_TRI2D, v[i], v[i + 1], v[i + 2]);
		break;
	case 0xc1:
		emit(CMD_TRI2D, v[0], v[1], v[2]);
		break;
	case 0xc2:
		for (int i = 0; i + 1 < n; i++)
			emit(CMD_LINE, v[i], v[i + 1], v[i + 1]);
		break;
	case 0xc3:
		emit(CMD_LINE, v[0], v[1], v[1]);
		break;
	case 0xc4:
		emit(CMD_POINT, v[0], v[0], v[0]);
		break;
	}
}

void zevio_gpu_device::draw_tri2d(const rstate &s, rcache &pc, const vertex_t &a, const vertex_t &b, const vertex_t &c, bool textured, int y0, int y1)
{
	// 2D: affine UVs in texels, no depth
	const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	if (std::fabs(area) < 1e-6f)
		return;
	const int minx = int(std::floor(std::min({ a.x, b.x, c.x }))), maxx = int(std::ceil(std::max({ a.x, b.x, c.x })));
	const int miny = int(std::floor(std::min({ a.y, b.y, c.y }))), maxy = int(std::ceil(std::max({ a.y, b.y, c.y })));
	for (int y = std::max({ miny, 0, y0 }); y <= std::min({ maxy, ZH - 1, y1 - 1 }); y++)
		for (int x = std::max(minx, 0); x <= std::min(maxx, 1023); x++)
		{
			const float px = x + 0.5f, py = y + 0.5f;
			const float w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) / area;
			const float w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) / area;
			const float w2 = 1.0f - w0 - w1;
			if (w0 < 0 || w1 < 0 || w2 < 0)
				continue;
			float r = w0 * a.r + w1 * b.r + w2 * c.r, g = w0 * a.g + w1 * b.g + w2 * c.g, bl = w0 * a.b + w1 * b.b + w2 * c.b;
			float alpha = 1.0f;
			if (textured)
			{
				float tr, tg, tb;
				if (!texel(s, pc, w0 * a.u + w1 * b.u + w2 * c.u, w0 * a.v + w1 * b.v + w2 * c.v, false, tr, tg, tb, alpha))
					continue;
				r *= tr; g *= tg; bl *= tb;
			}
			plot(s, x, y, 0.0f, r, g, bl, alpha, false);
		}
}

void zevio_gpu_device::sprite(u32 header, u32 &addr)
{
	const u8 flags = (header >> 16) & 0xff;
	const int count = header & 0xff;
	const u32 xy = rd32(addr + 4), wh = rd32(addr + 8);
	int k = 3;
	u32 uv = 0, col = 0xffffff;
	if (BIT(flags, 0))
		uv = rd32(addr + k++ * 4);
	if (BIT(flags, 2))
		col = rd32(addr + k++ * 4);
	addr += count * 4;

	// 2D positions: surface origin (clip top-left) + 0x85 offset + signed x/y
	const int ox = (m_clip0 & 0xfff) + (m_w85 & 0xfff), oy = ((m_clip0 >> 12) & 0xfff) + ((m_w85 >> 12) & 0xfff);
	const int x0 = ox + s16(xy & 0xffff), y0 = oy + s16(xy >> 16);
	const int w = std::min<int>(wh & 0xffff, 1024), h = std::min<int>(wh >> 16, 1024);
	static double primlog_at = getenv("ZGPU_PRIMLOG") ? atof(getenv("ZGPU_PRIMLOG")) : -1.0;
	const double now = machine().time().as_double();
	if (primlog_at >= 0 && now >= primlog_at && now < primlog_at + 0.05)
	{
		logerror("SPR %08x xy %d,%d wh %dx%d uv %08x col %06x 84 %06x 88 %06x 86 %06x 87 %06x 8d %06x 1c %06x 1e %06x\n",
				header, x0, y0, w, h, uv, col, m_w84, m_w88, m_w86, m_w87, m_w8d, m_w1c, m_w1e);
		if (BIT(flags, 0))
			log_texture();
	}

	rcmd c{};
	c.type = CMD_SPRITE;
	c.st = cur_state();
	c.a0 = u32(x0 & 0xffff) | (u32(y0) << 16);
	c.a1 = u32(w) | (u32(h) << 16);
	c.a2 = uv;
	c.a3 = col;
	c.flags = flags;
	add_cmd(std::move(c));
}

void zevio_gpu_device::draw_sprite(const rstate &s, rcache &pc, const rcmd &c, int band_y0, int band_y1)
{
	const int x0 = s16(c.a0 & 0xffff), y0 = s16(c.a0 >> 16);
	const int w = c.a1 & 0xffff, h = c.a1 >> 16;
	const u32 uv = c.a2, col = c.a3;
	const u8 flags = c.flags;

	// fast path: opaque (no 0x88 blend), non-ARGB4444 texels, integer colour
	// modulation, clipped once per sprite
	static const bool nofast = getenv("ZGPU_NOFAST") != nullptr;
	if (!nofast && !BIT(s.w88, 1) && !BIT(s.w86, 8))
	{
		const int cx0 = std::max({ int(s.clip0 & 0xfff), x0, 0 });
		const int cy0 = std::max({ int((s.clip0 >> 12) & 0xfff), y0, 0, band_y0 });
		const int cx1 = std::min({ int(s.clip1 & 0xfff), x0 + w - 1, 1023 });
		const int cy1 = std::min({ int((s.clip1 >> 12) & 0xfff), y0 + h - 1, band_y1 - 1 });
		const int mr = (col & 0xff) + 1, mg = ((col >> 8) & 0xff) + 1, mb = ((col >> 16) & 0xff) + 1;
		const bool modulate = (col & 0xffffff) != 0xffffff;
		const u16 flat = 0x8000 | (((col >> 19) & 0x1f) << 10) | (((col >> 11) & 0x1f) << 5) | ((col >> 3) & 0x1f);
		for (int y = cy0; y <= cy1; y++)
			for (int x = cx0; x <= cx1; x++)
			{
				u16 t;
				if (BIT(flags, 0))
				{
					const int r = texel_raw(s, pc, int(uv & 0xffff) + (x - x0), int(uv >> 16) + (y - y0));
					if (r < 0)
						continue;
					t = r;
					if (modulate)
						t = (((((t >> 10) & 0x1f) * mb) >> 8) << 10) | (((((t >> 5) & 0x1f) * mg) >> 8) << 5) | (((t & 0x1f) * mr) >> 8);
					t |= 0x8000;
				}
				else
					t = flat;
				vram_w(x, y, t);
			}
		return;
	}

	const float cr = (col & 0xff) / 255.0f, cg = ((col >> 8) & 0xff) / 255.0f, cb = ((col >> 16) & 0xff) / 255.0f;
	for (int j = std::max(0, band_y0 - y0); j < std::min(h, band_y1 - y0); j++)
		for (int i = 0; i < w; i++)
		{
			float r = cr, g = cg, b = cb, a = 1.0f;
			if (BIT(flags, 0))
			{
				float tr, tg, tb;
				if (!texel(s, pc, float((uv & 0xffff) + i), float((uv >> 16) + j), false, tr, tg, tb, a))
					continue;
				r *= tr; g *= tg; b *= tb;
			}
			plot(s, x0 + i, y0 + j, 0.0f, r, g, b, a, false);
		}
}

void zevio_gpu_device::draw_fill(const rcmd &c, int band_y0, int band_y1)
{
	const int x = c.a0 & 0xffff, y = c.a0 >> 16, w = c.a1 & 0xffff, h = c.a1 >> 16;
	const u16 col = c.a3;
	for (int j = std::max(0, band_y0 - y); j < std::min(h, band_y1 - y); j++)
	{
		// runs of up to 32 pixels are contiguous in the tiled layout
		for (int i = 0; i < w; )
		{
			const int xx = (x + i) & 1023;
			const int run = std::min(w - i, 32 - (xx & 31));
			const u32 off = vram_addr(xx, y + j) - m_ram_base;
			if (off + run * 2 <= m_ram_size)
			{
#ifdef LSB_FIRST
				std::fill_n(reinterpret_cast<u16 *>(m_ram) + (off >> 1), run, col);
#else
				for (int k = 0; k < run; k++)
					vram_w(xx + k, y + j, col);
#endif
			}
			i += run;
		}
	}
}

void zevio_gpu_device::draw_zclear(const rcmd &c, int band_y0, int band_y1)
{
	const int y0 = std::max<int>(c.a0, band_y0), y1 = std::min<int>(c.a1 + 1, band_y1);
	if (y1 > y0)
		std::fill_n(&m_zbuf[y0 * ZW], (y1 - y0) * ZW, std::numeric_limits<float>::max());
}


/***************************************************************************
    Render thread
***************************************************************************/

zevio_gpu_device::rstate zevio_gpu_device::cur_state() const
{
	return rstate{ m_w84, m_w86, m_w87, m_w88, m_w8d, m_clip0, m_clip1, m_clut[0], m_clut[3] };
}

void zevio_gpu_device::add_cmd(rcmd &&c)
{
	// rows touched, for culling commands per band
	switch (c.type)
	{
	case CMD_TRI3D: case CMD_TRI2D:
		c.ymin = int(std::floor(std::min({ c.v[0].y, c.v[1].y, c.v[2].y })));
		c.ymax = int(std::ceil(std::max({ c.v[0].y, c.v[1].y, c.v[2].y })));
		break;
	case CMD_LINE:
		c.ymin = int(std::floor(std::min(c.v[0].y, c.v[1].y)));
		c.ymax = int(std::ceil(std::max(c.v[0].y, c.v[1].y)));
		break;
	case CMD_POINT:
		c.ymin = c.ymax = int(c.v[0].y);
		break;
	case CMD_SPRITE:
		c.ymin = s16(c.a0 >> 16);
		c.ymax = c.ymin + int(c.a1 >> 16) - 1;
		break;
	case CMD_FILL:
		c.ymin = c.a0 >> 16;
		c.ymax = c.ymin + int(c.a1 >> 16) - 1;
		break;
	case CMD_ZCLEAR:
		c.ymin = c.a0;
		c.ymax = c.a1;
		break;
	}
	c.ymin = std::max({ c.ymin, int((c.st.clip0 >> 12) & 0xfff), 0 });
	c.ymax = std::min({ c.ymax, int((c.st.clip1 >> 12) & 0xfff), ZH - 1 });
	if (c.type == CMD_FILL || c.type == CMD_ZCLEAR)
	{
		// not clipped
		c.ymin = (c.type == CMD_FILL) ? int(c.a0 >> 16) : int(c.a0);
		c.ymax = (c.type == CMD_FILL) ? int(c.a0 >> 16) + int(c.a1 >> 16) - 1 : int(c.a1);
	}
	if (c.ymin > c.ymax)
		return;
	if ((c.type == CMD_TRI3D || c.type == CMD_POINT) && BIT(c.st.w84, 1))
	{
		m_zdirty_y0 = std::min(m_zdirty_y0, c.ymin);
		m_zdirty_y1 = std::max(m_zdirty_y1, c.ymax);
	}
	if (m_count_stats)
	{
		// debug statistics are counted in order on this thread
		sync();
		std::vector<rcmd> one{ c };
		exec_band(one, 0, 1 << 20);
		return;
	}
	m_cmds.push_back(std::move(c));
	if (m_cmds.size() >= 4096)
		submit();
}

void zevio_gpu_device::submit()
{
	if (m_cmds.empty())
		return;
	if (!m_async)
	{
		exec_batch(m_cmds);
		m_cmds.clear();
		return;
	}
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_jobs.emplace_back(std::move(m_cmds));
	}
	m_cmds.clear();
	m_cv.notify_all();
}

void zevio_gpu_device::sync()
{
	submit();
	if (!m_async)
		return;
	std::unique_lock<std::mutex> lock(m_mutex);
	m_cv.wait(lock, [this] { return m_jobs.empty() && !m_busy; });
}

void zevio_gpu_device::render_thread()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	while (true)
	{
		m_cv.wait(lock, [this] { return m_quit || !m_jobs.empty(); });
		if (m_jobs.empty())
			break; // quitting
		std::vector<rcmd> cmds = std::move(m_jobs.front());
		m_jobs.pop_front();
		m_busy = true;
		lock.unlock();
		exec_batch(cmds);
		lock.lock();
		m_busy = false;
		m_cv.notify_all();
	}
}

void *zevio_gpu_device::band_cb(void *param, int threadid)
{
	auto *const job = reinterpret_cast<band_job *>(param);
	job->gpu->exec_band(*job->cmds, job->y0, job->y1);
	return nullptr;
}

void zevio_gpu_device::exec_batch(const std::vector<rcmd> &cmds)
{
	int ymin = ZH, ymax = -1;
	u64 work = 0;
	for (const auto &c : cmds)
	{
		ymin = std::min(ymin, c.ymin);
		ymax = std::max(ymax, c.ymax);
		work += u64(c.ymax - c.ymin + 1) * ((c.type == CMD_SPRITE || c.type == CMD_FILL) ? (c.a1 & 0xffff) : 64);
	}
	const int rows = ymax - ymin + 1;
	if (!m_queue || rows < 32 || work < 20000)
	{
		exec_band(cmds, 0, 1 << 20);
		return;
	}
	const int bands = std::min(m_bands, rows / 16);
	band_job jobs[MAX_BANDS];
	for (int i = 0; i < bands; i++)
		jobs[i] = { this, &cmds, ymin + rows * i / bands, ymin + rows * (i + 1) / bands };
	osd_work_item_queue_multiple(m_queue, band_cb, bands, jobs, sizeof(band_job), WORK_ITEM_FLAG_AUTO_RELEASE);
	osd_work_queue_wait(m_queue, osd_ticks_per_second() * 10);
}

void zevio_gpu_device::exec_band(const std::vector<rcmd> &cmds, int y0, int y1)
{
	rcache pc;
	for (const auto &c : cmds)
	{
		if (c.ymax < y0 || c.ymin >= y1)
			continue;
		switch (c.type)
		{
		case CMD_TRI3D:
			draw_triangle(c.st, pc, c.v[0], c.v[1], c.v[2], c.textured, y0, y1);
			break;
		case CMD_TRI2D:
			draw_tri2d(c.st, pc, c.v[0], c.v[1], c.v[2], c.textured, y0, y1);
			break;
		case CMD_LINE:
			draw_line(c.st, c.v[0], c.v[1], y0, y1);
			break;
		case CMD_POINT:
			if (int(c.v[0].y) >= y0 && int(c.v[0].y) < y1)
				plot(c.st, int(c.v[0].x), int(c.v[0].y), c.v[0].z, c.v[0].r, c.v[0].g, c.v[0].b, 1.0f, c.flags & 1);
			break;
		case CMD_SPRITE:
			draw_sprite(c.st, pc, c, y0, y1);
			break;
		case CMD_FILL:
			draw_fill(c, y0, y1);
			break;
		case CMD_ZCLEAR:
			draw_zclear(c, y0, y1);
			break;
		}
	}
}


/***************************************************************************
    Display list executor
***************************************************************************/

void zevio_gpu_device::run_list(u32 addr)
{
	LOGMASKED(LOG_LIST, "list %08x\n", addr);
	// debug: ZGPU_TRACE=<seconds> writes every command executed in the 0.05 s
	// after that emulated time to gputrace.txt
	static FILE *trace = nullptr;
	static double trace_at = getenv("ZGPU_TRACE") ? atof(getenv("ZGPU_TRACE")) : -1.0;
	const double now = machine().time().as_double();
	const bool tracing = trace_at >= 0 && now >= trace_at && now < trace_at + 0.05;
	if (tracing && !trace)
		trace = fopen("gputrace.txt", "w");
	if (tracing)
	{
		fprintf(trace, "LIST %08x t=%f base98 %08x r90 %08x r94 %08x\n", addr, now, m_regs[0x98 / 4], m_regs[0x90 / 4], m_regs[0x94 / 4]);
		fflush(trace);
	}
	std::vector<u32> stack;
	const u32 start = addr;
	for (int guard = 0; guard < 0x400000; guard++)
	{
		const u32 cmd = rd32(addr);
		if (tracing)
			fprintf(trace, "%08x: %08x %08x %08x %08x %08x %08x\n", addr, cmd, rd32(addr + 4), rd32(addr + 8), rd32(addr + 12), rd32(addr + 16), rd32(addr + 20));
		addr += 4;
		const u8 op = cmd >> 24;
		switch (op)
		{
		case 0x04: // return from a 0x09 call (nested calls are allowed)
			if (!stack.empty())
			{
				addr = stack.back();
				stack.pop_back();
			}
			break;
		case 0x08: addr = rd32(addr); break;
		case 0x09:
			if (stack.size() < 16)
				stack.push_back(addr + 4);
			addr = rd32(addr);
			break;
		case 0x0a: m_vtxbase = rd32(addr); addr += 4; break;
		case 0x0c: m_xfer = rd32(addr); addr += 4; break;
		case 0x0e: addr += 4; break;
		case 0x10: m_regs[0x5c / 4] = rd32(addr); addr += 4; break;
		case 0x00: case 0x12: case 0x89: case 0x95: case 0x96: case 0x8e: break; // 00: padding after sub-list calls
		case 0x14:
			m_regs[0x14 / 4] = (cmd & 0xff00) | 0x08;
			if (!m_defer_irq)
				update_irq();
			if (VERBOSE & LOG_LIST)
			{
				LOGMASKED(LOG_LIST, "list done: tris %d pixels %d fills %d clip %06x-%06x base %08x r81 %06x f %f %f near %f\n",
						m_stat_tris, m_stat_pixels, m_stat_fills, m_clip0, m_clip1, m_regs[0x98 / 4], m_w81, reg_f(0x78), reg_f(0x79), reg_f(0x75));
				m_stat_tris = m_stat_pixels = m_stat_fills = 0;
			}
			return;
		case 0x1c: m_w1c = cmd & 0xffffff; break;
		case 0x1e: m_w1e = cmd & 0xffffff; break;
		case 0x20:
		{
			const u32 reg = (cmd >> 16) & 0xff, n = cmd & 0xffff;
			for (u32 i = 0; i < n; i++)
				m_r[(reg + i) & 0xff] = rd16(addr + i * 2);
			addr += (n * 2 + 3) & ~3;
			break;
		}
		case 0x81: m_w81 = cmd & 0xffffff; break;
		case 0x82: m_clip0 = cmd & 0xffffff; break;
		case 0x83: m_clip1 = cmd & 0xffffff; break;
		case 0x84: m_w84 = cmd & 0xffffff; break;
		case 0x85: m_w85 = cmd & 0xffffff; break;
		case 0x86: m_w86 = cmd & 0xffffff; break;
		case 0x87: m_w87 = cmd & 0xffffff; break;
		case 0x88: m_w88 = cmd & 0xffffff; break;
		case 0x8a: case 0x8b: case 0x8c: break; // fog: TODO
		case 0x8d: m_w8d = cmd & 0xffffff; break;
		case 0x8f: case 0x90: case 0x91: case 0x92: case 0x93: case 0x94:
			m_clut[op - 0x8f] = cmd & 0xffffff;
			break;
		case 0x40: case 0x44: case 0x48: case 0x4c: case 0x50:
			prim3d(cmd, addr);
			break;
		case 0x60: case 0x61: case 0x62: case 0x63: case 0x64: case 0x65: case 0x66: case 0x67:
		case 0x68: case 0x69: case 0x6a: case 0x6b: case 0x6c: case 0x6d: case 0x6e: case 0x6f:
			transform_to_memory(cmd, addr);
			break;
		case 0xc0: case 0xc1: case 0xc2: case 0xc3: case 0xc4:
			prim2d(cmd, addr);
			break;
		case 0xc8:
			sprite(cmd, addr);
			break;
		case 0xd0: case 0xd4: // load image: pos, size, 0E|n, src
		{
			const u32 pos = rd32(addr), size = rd32(addr + 4), src = rd32(addr + 12);
			addr += 16;
			sync();
			const int x = pos & 0xffff, y = pos >> 16, w = std::min<int>(size & 0xffff, 1024), h = std::min<int>(size >> 16, 8192);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					vram_w(x + i, y + j, rd16(src + (j * w + i) * 2));
			break;
		}
		case 0xd1: // store image to the 0x0C address
		{
			const u32 pos = rd32(addr), size = rd32(addr + 4);
			addr += 8;
			sync();
			const int x = pos & 0xffff, y = pos >> 16, w = std::min<int>(size & 0xffff, 1024), h = std::min<int>(size >> 16, 8192);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					wr16(m_xfer + (j * w + i) * 2, vram_r(x + i, y + j));
			break;
		}
		case 0xd3: // fill
		{
			// Absolute coordinates, except a rectangle inside the +0x90/+0x94
			// window (512x240 in FA), which addresses the +0x98 area: FA clears
			// its frame with (surface, colour 0) and its +0x98 area (probably
			// depth) with (0,0) 512x240, 0x3732
			const u32 pos = rd32(addr), size = rd32(addr + 4), col = rd32(addr + 8);
			addr += 12;
			m_stat_fills++;
			const int x = pos & 0xffff, y = pos >> 16, w = std::min<int>(size & 0xffff, 1024), h = std::min<int>(size >> 16, 8192);
			const int ww = m_regs[0x94 / 4] & 0xffff, wh = m_regs[0x94 / 4] >> 16;
			const bool windowed = (x + w <= ww) && (y + h <= wh);
			const u32 woff = m_regs[0x98 / 4] - 0x0ffd0000;
			if (!windowed || !(woff & 0x3fff))
			{
				// the +0x98 area starts on an 8-row boundary: absolute rows
				rcmd c{};
				c.type = CMD_FILL;
				c.st = cur_state();
				c.a0 = windowed ? ((x & 0xffff) | (u32(y + (woff >> 14) * 8) << 16)) : pos;
				c.a1 = u32(w) | (u32(h) << 16);
				c.a3 = col;
				add_cmd(std::move(c));
				if (windowed && m_zdirty_y1 >= m_zdirty_y0)
				{
					// FA clears depth this way: reset the rows drawn with depth since
					rcmd z{};
					z.type = CMD_ZCLEAR;
					z.st = cur_state();
					z.a0 = m_zdirty_y0;
					z.a1 = m_zdirty_y1;
					add_cmd(std::move(z));
					m_zdirty_y0 = ZH;
					m_zdirty_y1 = -1;
				}
				break;
			}
			sync(); // the +0x98 area and the depth buffer
			const u32 base = windowed ? m_regs[0x98 / 4] : 0x0ffd0000;
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
				{
					const u32 xb = ((x + i) & 1023) * 2, yy = y + j;
					wr16(base + (yy >> 3) * 16384 + (xb >> 6) * 512 + (yy & 7) * 64 + (xb & 63), col);
				}
			if (windowed)
			{
				std::fill_n(&m_zbuf[0], ZW * ZH, std::numeric_limits<float>::max());
				m_zdirty_y0 = ZH;
				m_zdirty_y1 = -1;
			}
			break;
		}
		case 0xd2: // VRAM to VRAM copy: destination, size, source
		{
			const u32 dst = rd32(addr), size = rd32(addr + 4), src = rd32(addr + 8);
			addr += 12;
			const int dx = dst & 0xffff, dy = dst >> 16, sx = src & 0xffff, sy = src >> 16;
			const int w = std::min<int>(size & 0xffff, 1024), h = std::min<int>(size >> 16, 4096);
			sync();
			std::vector<u16> tmp(w * h);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					tmp[j * w + i] = vram_r(sx + i, sy + j);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					vram_w(dx + i, dy + j, tmp[j * w + i]);
			break;
		}
		default:
			LOGMASKED(LOG_UNKNOWN, "unknown command %08x at %08x\n", cmd, addr - 4);
			break;
		}
	}
	// never leave the engine waiting: finish as if the end command was reached
	LOGMASKED(LOG_UNKNOWN, "list runaway: started %08x, stopped at %08x (%08x %08x %08x)\n", start, addr, rd32(addr), rd32(addr + 4), rd32(addr + 8));
	m_regs[0x14 / 4] = 0xfd08;
	update_irq();
}
