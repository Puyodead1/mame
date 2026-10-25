// license:BSD-3-Clause
// copyright-holders:Riley
/***************************************************************************

    LSI Zevio 1020 "Hanabi" GPU (0xA8000000) and number converters
    (0xA4000000), as used by the VTech V.Flash / V.Smile Pro.

    The command processor executes display lists from SDRAM and draws into
    the tiled 16-bit VRAM surface: 1024 pixels (2048 bytes) wide, 32x8-pixel
    tiles of 512 bytes. GPU rows from HIWORD(+0x94) down are RAM at +0x98;
    rows above are a separate depth/aux area (see vram_map_update).

    Knowledge comes from the boot ROM and the V2GE engine in the games' FA.VFF
    (Bratz). Display lists are executed instantly when kicked.

    Registers:
    +0x00 command (1 = run list at +0x28, 3 = reset), upper half engine state
    +0x10 IRQ enable (6 bits), +0x14 IRQ status (bits 0-5, tag in 8-15)
    +0x28 list address, +0x5C capture write pointer, +0x94/+0x98 RAM mapping

    Display list words (top byte = opcode):
    04 return from 09 call       08,a jump          09,a call
    0A,a vertex base (indexed)   0C,a transfer/transform destination
    0E|n,a data feed for D0      10,a capture buf   12 (unknown, no-op)
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

#include <mutex>
#include <set>
#include <tuple>

#include <map>
#include <thread>

#define LOG_UNKNOWN (1U << 1)
#define LOG_LIST    (1U << 2)

#define VERBOSE (LOG_UNKNOWN)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ZEVIO_GPU, zevio_gpu_device, "zevio_gpu", "Zevio 1020 GPU")

namespace {

constexpr int ZW = 1024, ZH = 4096; // depth buffer, depth-area coordinates (see zptr)
// A 16-bit depth value h (a sprite's depth word, anything written into the depth area) stands for eye
// distance 1 / h: the firmware computes sprite depth as f16(K / z) with K = FX_ONE, its fixed-point 1.0
// (Sprite_setDepth, fw/fa_28.md). [EMU: assumed the same for 3D; games rule out large scales (Wacky Races
// draws menu sprites, h >= 128, over a 3D background at d >= 1 with no clear between), not small ones]
constexpr float ZSCALE = 1.0f;

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
	return (bi << 10) | (gi << 5) | ri; // bit 15 clear: the display treats a set bit 15 as transparent
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
	m_abuf = std::make_unique<u8[]>(1024 * 4096);
	m_aux = std::make_unique<u16[]>(1024 * AUX_H);
	// more bands than threads so uneven rows balance out; a HIGH_FREQ queue
	// would be capped at four threads
	const int hw = std::max<int>(std::thread::hardware_concurrency(), 1);
	m_bands = std::clamp<int>(hw * 3, 1, MAX_BANDS);
	m_queue = (hw > 1) ? osd_work_queue_alloc(WORK_QUEUE_FLAG_MULTI) : nullptr;
	// lists are drawn on a render thread; ZGPU_SYNC draws them on the emulation thread instead
	// (about 25% slower in Cars' menu; tested as a fix for corrupted menu text, which it was not)
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
	save_item(NAME(m_vram_y0));
	save_item(NAME(m_vram_base));
	save_pointer(NAME(m_aux), 1024 * AUX_H);
	m_done_timer = timer_alloc(FUNC(zevio_gpu_device::done_cb), this);
	save_item(NAME(m_busy_until));
	save_item(NAME(m_done_status));
	save_item(NAME(m_done_time));
	save_item(NAME(m_done_count));
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

void zevio_gpu_device::device_post_load()
{
	for (int r = 0; r < 256; r++)
		m_rf[r] = f16_to_float(m_r[r]);
}

void zevio_gpu_device::device_reset()
{
	sync();
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	std::fill(std::begin(m_cvt), std::end(m_cvt), 0);
	std::fill(std::begin(m_r), std::end(m_r), 0);
	for (int r : { 0x3c, 0x3d, 0x3e, 0x40, 0x44, 0x48 })
		m_r[r] = 0x3e00;
	m_r[0x70] = 0x7fff;
	device_post_load();
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
	m_vram_y0 = 0;
	m_vram_base = 0;
	std::fill_n(&m_aux[0], 1024 * AUX_H, 0);
	std::fill_n(&m_zbuf[0], ZW * ZH, std::numeric_limits<float>::max());
	m_done_count = 0;
	m_busy_until = attotime::zero;
	m_done_timer->adjust(attotime::never);
	update_irq();
}


/***************************************************************************
    GPU 16-bit float: sign, 6-bit exponent (bias 31), 9-bit mantissa
***************************************************************************/

namespace {
// built at static initialisation, so lookups need no guard
struct f16_table
{
	float t[0x10000];
	f16_table()
	{
		for (int i = 0; i < 0x10000; i++)
		{
			const float f = (i & 0x7fff) ? ldexpf(1.0f + (i & 0x1ff) / 512.0f, int((i >> 9) & 0x3f) - 31) : 0.0f;
			t[i] = (i & 0x8000) ? -f : f;
		}
	}
};
const f16_table s_f16;
}

float zevio_gpu_device::f16_to_float(u16 v)
{
	return s_f16.t[v];
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

// GPU coordinates (analysis/re/reports/gpu.md section 4.1): a row y at or
// below HIWORD(+0x94) is RAM at +0x98 + tiled(x, y - HIWORD(+0x94)). [FW] The
// boot ROM sets +0x94 = 240 << 16 | 512, +0x98 = 0x10800000, and draws into
// GPU (0,240) / (512,240) while showing 0x10802000 / 0x10800000, which only
// works with this mapping; FA sets +0x98 = 0x0FFD0000 + tiled(0, y0) (row 240,
// or row 128 in MJPEG mode), so for FA it equals the absolute layout.
// Rows above y0 hold FA's depth/aux area (cleared with depth values at GPU
// (0,0)); firmware never gives them a CPU address (Gpu_vramPtr returns NULL).
// [INT, guess] they are separate (on-chip?) memory: kept in m_aux here.
void zevio_gpu_device::vram_map_update()
{
	const int y0 = std::min<int>(m_regs[0x94 / 4] >> 16, AUX_H);
	const u32 base = m_regs[0x98 / 4];
	if (y0 != m_vram_y0 || base != m_vram_base)
	{
		sync(); // queued work was recorded with the old mapping
		m_vram_y0 = y0;
		m_vram_base = base;
	}
}

int zevio_gpu_device::abs_row(int y) const
{
	return int(s32(m_vram_base - 0x0ffd0000) >> 14) * 8 + (y - m_vram_y0);
}

// Depth lives in the depth area (GPU rows 0-239 at (0,0), see in_zarea) at colour position - 0x81 + 0x85,
// where the firmware puts and clears it (gpu.md 4.2/4.3), as 16-bit reciprocal depth (see ZSCALE). MAME keeps
// the eye distance d as a float per depth pixel; anything written into the area (FA's depth clears, and masks
// drawn there, as Bratz's menu draws to hide its side panels) sets d = ZSCALE / value there.
// [EMU: 3D and sprite depth writes do not write the 16-bit pixel back]
// debug: ZGPU_AUXLOG=<time> logs, for 0.05 s after that time, every command that writes into the depth area
// Depth compare [EMU, each rule from a console comparison]:
// - 3D pixels pass when not farther at the depth buffer's 16-bit precision (f16, 9-bit mantissa): Bratz's
//   title draws its letters as quads on the background's plane; compared as 32-bit floats they z-fought
//   into dots, the console shows them solid. (z_fails3d)
// - sprites pass only when strictly nearer: Bratz's Confirm bar is drawn right after its label at the
//   same depth (w0 0x4D33) and the console keeps the label on top. (z_fails_sprite)
// - partly transparent texels (ARGB alpha < 15) write no depth: the corner swirls (alpha 1-9) are
//   recoloured by a second sprite at the same depth, which the rule above would otherwise block.
// ZGPU_ZOLD restores the plain float compare (equal passes) and alpha-blind depth writes.
static const bool s_zold = getenv("ZGPU_ZOLD") != nullptr;
static const bool s_zalpha = !s_zold;
static const bool s_dsta = getenv("ZGPU_DSTA") != nullptr;
static inline bool z_fails3d(float z, float zb) { return s_zold ? z > zb : z > zb * (1.0f + 1.0f / 512.0f); }
static inline bool z_fails_sprite(float z, float zb) { return s_zold ? z > zb : z >= zb; }
static inline bool depth_fails(float z, float zb) { return z_fails3d(z, zb); }

static double primlog_len()
{
	static const double len = getenv("ZGPU_PRIMLOG_LEN") ? atof(getenv("ZGPU_PRIMLOG_LEN")) : 0.05;
	return len;
}

static bool auxlog_now(running_machine &m)
{
	static const double at = getenv("ZGPU_AUXLOG") ? atof(getenv("ZGPU_AUXLOG")) : -1.0;
	const double t = m.time().as_double();
	return at >= 0 && t >= at && t < at + primlog_len();
}

// ZGPU_PRIMLOG also logs image loads/stores (D0/D1) in its window
static bool xferlog_now(running_machine &m)
{
	static const double at = getenv("ZGPU_PRIMLOG") ? atof(getenv("ZGPU_PRIMLOG")) : -1.0;
	const double t = m.time().as_double();
	return at >= 0 && t >= at && t < at + primlog_len();
}

static int s_zfail; static float s_zfail_lo, s_zfail_hi, s_zfail_zlo; // ZGPU_PRIMLOG: depth-test failures of the last primitive

float *zevio_gpu_device::zptr(const rstate &s, int x, int y) const
{
	const int zx = x - zoff_x(s), zy = y - zoff_y(s);
	return (zx >= 0 && zx < ZW && zy >= 0 && zy < ZH) ? &m_zbuf[zy * ZW + zx] : nullptr;
}

void zevio_gpu_device::zset(int x, int y, u16 v)
{
	if (x < 0 || x >= ZW || y < 0 || y >= ZH)
		return;
	// ZGPU_EXP63_FAR (experiment): values with the maximum exponent (0x7E00..0x7FFF) mean infinitely far
	static const bool exp63_far = getenv("ZGPU_EXP63_FAR") != nullptr;
	const float r = (v && !(exp63_far && ((v >> 9) & 0x3f) == 0x3f)) ? f16_to_float(v) : 0.0f;
	m_zbuf[y * ZW + x] = (r > 0.0f) ? ZSCALE / r : std::numeric_limits<float>::max();
}

void zevio_gpu_device::vram_w(int x, int y, u16 data)
{
	if (vram_in_aux(y))
		aux_px(x, y) = data;
	else
		wr16(vram_addr(x, y), data);
	if (in_zarea(y))
		zset(x & 1023, y, data);
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
	// Off unless ZGPU_QUEUE_HLE is set. It completed FA's queued lists here
	// instead of through FA's GPU interrupt handler, but it writes status 3
	// into the list's first word, where the firmware's Gpu_isrRetire writes it
	// to the queue entry (analysis/re/reports/gpu.md section 6). It never
	// engaged for Bratz (kick address = entry.list + 4); Cars, Spider-Man and
	// Wacky Races run the same or better through their own handler
	static const bool off = getenv("ZGPU_QUEUE_HLE") == nullptr;
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
	m_hle_handled = true;
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
	// bits 31-16: engine state, 3 while it is still drawing (Gpu_trySubmit waits for anything else)
	if (offset == 0)
		return (m_regs[0] & 0xffff) | ((machine().time() < m_busy_until) ? 0x30000 : 0);
	return m_regs[offset];
}

/*
    Engine time. The engine draws at a finite rate, and FA starts the next queued
    list only from the end-of-list interrupt, so a scene costs real time: Disney
    Princess' castle levels update every third vblank on the console (20 fps) and
    ran at 60 in MAME, three times too fast. Each kick's lists are recorded at once
    (drawing itself still happens on the render thread) but their end status
    (+0x14, and with it the interrupt) is delivered when the engine would have
    finished, after the time for the pixels and primitives they drew.
    [EMU] the rates are fitted to console footage, not measured: 12 Mpx/s (with the
    rough pixel counts of add_cmd) gives Princess' castle 20 fps; Bratz's character
    select comes out at 17.5 fps (console 15), CPU-bound there. A per-list cost was
    tried and does not fit: FA chains queued lists into fewer kicks while the engine
    is busy. ZGPU_PXRATE (pixels/s), ZGPU_PRIMUS (us per primitive), ZGPU_XFERRATE
    (pixels/s loaded, stored or copied); ZGPU_NOTIMING turns it off
*/
double zevio_gpu_device::list_time(u64 px, u64 prims, u64 xfer) const
{
	static const double px_rate = getenv("ZGPU_PXRATE") ? atof(getenv("ZGPU_PXRATE")) : 12e6;
	static const double prim_s = (getenv("ZGPU_PRIMUS") ? atof(getenv("ZGPU_PRIMUS")) : 1.0) * 1e-6;
	static const double xfer_rate = getenv("ZGPU_XFERRATE") ? atof(getenv("ZGPU_XFERRATE")) : 20e6;
	return double(px) / px_rate + double(prims) * prim_s + double(xfer) / xfer_rate;
}

void zevio_gpu_device::done_schedule()
{
	if (m_done_count)
		m_done_timer->adjust(std::max(m_done_time[0] - machine().time(), attotime::zero));
	else
		m_done_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(zevio_gpu_device::done_cb)
{
	if (!m_done_count)
		return;
	m_regs[0x14 / 4] = m_done_status[0];
	m_done_count--;
	std::copy(&m_done_status[1], &m_done_status[1 + m_done_count], &m_done_status[0]);
	std::copy(&m_done_time[1], &m_done_time[1 + m_done_count], &m_done_time[0]);
	update_irq();
	done_schedule();
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
			const u64 px0 = m_work_px, prims0 = m_work_prims, xfer0 = m_work_xfer;
			const u32 status0 = m_regs[0x14 / 4];
			m_hle_handled = false;
			m_defer_irq = true;
			run_list(m_regs[0x28 / 4]);
			while (fa_queue_hle())
				run_list(m_regs[0x28 / 4]);
			static const bool notiming = getenv("ZGPU_NOTIMING") != nullptr;
			if (!notiming && !m_hle_handled)
			{
				// the engine starts this list when it has finished the previous one
				const attotime now = machine().time();
				m_busy_until = std::max(now, m_busy_until) + attotime::from_double(list_time(m_work_px - px0, m_work_prims - prims0, m_work_xfer - xfer0));
				if (m_regs[0x14 / 4] != status0)
				{
					if (m_done_count == DONE_MAX)
						done_cb(0);
					m_done_status[m_done_count] = m_regs[0x14 / 4];
					m_done_time[m_done_count++] = m_busy_until;
					m_regs[0x14 / 4] = status0;
					done_schedule();
				}
				// ZGPU_WORKLOG: kicks, pixels, primitives and engine busy time per emulated second
				static const bool worklog = getenv("ZGPU_WORKLOG") != nullptr;
				static u64 wl_kicks, wl_px, wl_prims, wl_xfer; static double wl_busy, wl_sec = -1;
				if (worklog)
				{
					const double t = now.as_double();
					if (wl_sec < 0)
						wl_sec = std::floor(t);
					if (t >= wl_sec + 1)
					{
						logerror("WORK %.0f s: %llu kicks, %llu px, %llu prims, %llu xfer, engine busy %.0f%%\n", wl_sec,
								(unsigned long long)wl_kicks, (unsigned long long)wl_px, (unsigned long long)wl_prims, (unsigned long long)wl_xfer, wl_busy * 100);
						wl_kicks = wl_px = wl_prims = wl_xfer = 0; wl_busy = 0; wl_sec = std::floor(t);
					}
					wl_kicks++; wl_px += m_work_px - px0; wl_prims += m_work_prims - prims0; wl_xfer += m_work_xfer - xfer0;
					wl_busy += list_time(m_work_px - px0, m_work_prims - prims0, m_work_xfer - xfer0);
				}
			}
			// hand work to the render thread in batches: Cars kicks ~800 lists a frame, most of
			// them a single state word or sprite, and waking the thread for each cost more than
			// drawing them. Everything recorded is submitted by the next sync point at the latest
			// (frame fence, VRAM load or read, mapping change). ZGPU_KICK_SUBMIT: every kick
			static const bool kick_submit = getenv("ZGPU_KICK_SUBMIT") != nullptr;
			if (!m_async || kick_submit || m_cmds.size() >= 256)
				submit();
			m_defer_irq = false;
			update_irq();

			// pixels drawn beyond what the engine could have done by now stall
			// the CPU (as a busy bus would)
			constexpr double RATE = 100e6; // pixels per second; TODO: measure
			constexpr double FREE = 20000; // up to 0.2 ms ahead is free
			const double now = machine().time().as_double();
			m_px_debt = std::max(0.0, m_px_debt - (now - m_px_time) * RATE) + double(m_work_px - px0);
			m_px_time = now;
			// only when FA's queue was handled here: games whose own interrupt
			// handler runs the engine (Bratz) must not be stalled in it
			if (m_px_debt > FREE && m_stall && !notiming && m_hle_handled)
			{
				m_stall->spin_until_time(attotime::from_double(m_px_debt / RATE));
				m_px_time = now + m_px_debt / RATE;
				m_px_debt = 0;
			}
		}
		else if (data == 3)
		{
			m_done_count = 0;
			m_busy_until = machine().time();
			done_schedule();
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
	case 0x94 / 4: case 0x98 / 4:
		COMBINE_DATA(&m_regs[offset]);
		vram_map_update();
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
ATTR_FORCE_INLINE bool zevio_gpu_device::texel(const rstate &s, rcache &pc, float u, float v, bool normalized, float &r, float &g, float &b, float &a) const
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
	// Coverage comes from the nearest texel, colour from the opaque ones among the four: Bratz's
	// character shadow is a dither of single grey texels on transparent ones (bit 15), drawn as dots on
	// the console; dropping pixels that were less than half opaque lost it entirely. [EMU]
	// ZGPU_FILTER_HALF restores the old rule (drawn when at least half the weight is opaque).
	static const bool filter_half = getenv("ZGPU_FILTER_HALF") != nullptr;
	const int nearest = (du >= 0.5f ? 1 : 0) + (dv >= 0.5f ? 2 : 0);
	if (!BIT(mode, 8))
	{
		// BGR555 texels: 8-bit fixed point weights, transparent texels drop out
		const int wu = int(du * 256.0f), wv = int(dv * 256.0f);
		const int wt[4] = { (256 - wu) * (256 - wv), wu * (256 - wv), (256 - wu) * wv, wu * wv };
		int sr = 0, sg = 0, sb = 0, wsum = 0;
		bool near_opaque = false;
		for (int k = 0; k < 4; k++)
		{
			const int t = texel_raw(s, pc, iu + (k & 1), iv + (k >> 1));
			if (t >= 0)
			{
				sr += (t & 0x1f) * wt[k];
				sg += ((t >> 5) & 0x1f) * wt[k];
				sb += ((t >> 10) & 0x1f) * wt[k];
				wsum += wt[k];
				near_opaque |= k == nearest;
			}
		}
		if (filter_half ? wsum < 32768 : (!near_opaque || !wsum))
			return false;
		const float k = 1.0f / (float(wsum) * 31.0f);
		r = sr * k; g = sg * k; b = sb * k; a = 1.0f;
		return true;
	}
	float sr = 0, sg = 0, sb = 0, sa = 0, wsum = 0;
	bool near_opaque = false;
	for (int k = 0; k < 4; k++)
	{
		const int ou = k & 1, ov = k >> 1;
		const float wt = (ou ? du : 1.0f - du) * (ov ? dv : 1.0f - dv);
		float cr, cg, cb, ca;
		if (texel_point(s, pc, iu + ou, iv + ov, cr, cg, cb, ca))
		{
			sr += cr * wt; sg += cg * wt; sb += cb * wt; sa += ca * wt;
			wsum += wt;
			near_opaque |= k == nearest;
		}
	}
	if (filter_half ? wsum < 0.5f : (!near_opaque || wsum <= 0.0f))
		return false;
	r = sr / wsum; g = sg / wsum; b = sb / wsum; a = sa / wsum;
	return true;
}

// raw texel after the CLUT: -1 if transparent (palette index 0; for
// non-ARGB4444 textures also bit 15 set, and 0x0000 in direct-colour ones: Bratz
// panel backgrounds are 0x0000, Cars uses 0x8000 billboards, 0xFC1F text boxes, 0xFFFF icons)
template <typename F>
ATTR_FORCE_INLINE int zevio_gpu_device::texel_fetch(const rstate &s, rcache &pc, int tu, int tv, F &&rd) const
{
	const u32 mode = s.w86;
	tu &= (8 << (mode & 7)) - 1;
	tv &= (8 << ((mode >> 3) & 7)) - 1;
	const int x0 = s.w87 & 0xfff, y0 = (s.w87 >> 12) & 0xfff;
	u16 c;
	switch ((mode >> 9) & 3)
	{
	case 0:
		c = rd(x0 + tu, y0 + tv);
		break;
	case 1:
	{
		const u16 hw = rd(x0 + (tu >> 1), y0 + tv);
		const u8 idx = BIT(tu, 0) ? (hw >> 8) : (hw & 0xff);
		static const bool idx0_opaque8 = getenv("ZGPU_IDX0_OPAQUE") && (atoi(getenv("ZGPU_IDX0_OPAQUE")) & 1);
		static const bool idx0_log = getenv("ZGPU_IDX0_LOG") != nullptr;
		if (!idx && idx0_log)
		{
			// print each new (w84, w86, w88, w8d) combination that samples index 0 of an 8-bit texture
			static std::mutex lock;
			static std::set<std::tuple<u32, u32, u32, u32>> seen;
			std::lock_guard<std::mutex> guard(lock);
			if (seen.emplace(s.w84, s.w86, s.w88, s.w8d).second)
				fprintf(stderr, "IDX0 w84 %06x w86 %06x w88 %06x w8d %06x clut %06x tex %06x\n", s.w84, s.w86, s.w88, s.w8d, s.clut0, s.w87);
		}
		// index 0 of an 8-bit texture is transparent only with colour keying, 0x88 bit 6 (see
		// colour_key below)
		// (ZGPU_IDX0_OPAQUE=1 makes it always opaque, =4 always transparent)
		static const bool idx0_always = getenv("ZGPU_IDX0_OPAQUE") && (atoi(getenv("ZGPU_IDX0_OPAQUE")) & 4);
		if (pc.key != s.clut0)
		{
			for (int i = 0; i < 256; i++)
				pc.pal[i] = vram_r((s.clut0 & 0xfff) + i, (s.clut0 >> 12) & 0xfff);
			pc.key = s.clut0;
		}
		// and only when its palette entry is 0x0000: Bratz draws everything with keying on, its store
		// walls and floor use index 0 as a colour (0x3D53, 0x3E1D: the floor was full of holes showing
		// stale pixels), while the textures it cuts out (the corner mask, decals) have black there
		static const bool idx0_any = getenv("ZGPU_IDX0_ANY") != nullptr; // old rule: index 0 whatever its colour
		if (!idx && !idx0_opaque8 && ((colour_key(s) && (pc.pal[0] == 0 || idx0_any)) || idx0_always))
			return -1;
		c = pc.pal[idx];
		break;
	}
	default:
	{
		const u16 hw = rd(x0 + (tu >> 2), y0 + tv);
		const u8 idx = (hw >> ((tu & 3) * 4)) & 0x0f;
		static const bool idx0_opaque4 = getenv("ZGPU_IDX0_OPAQUE") && (atoi(getenv("ZGPU_IDX0_OPAQUE")) & 2);
		if (pc.key != (s.clut3 | 0x80000000U))
		{
			for (int i = 0; i < 16; i++)
				pc.pal[i] = vram_r((s.clut3 & 0xfff) + i, (s.clut3 >> 12) & 0xfff);
			pc.key = s.clut3 | 0x80000000U;
		}
		// index 0 is see-through when its entry is 0x0000; Bratz's character shadow is a dither whose
		// dots are index 0 = 0x5294 grey (its background is bit 15), and was lost entirely
		static const bool idx0_any4 = getenv("ZGPU_IDX0_ANY") != nullptr; // old rule: index 0 whatever its colour
		if (!idx && !idx0_opaque4 && (pc.pal[0] == 0 || idx0_any4))
			return -1;
		c = pc.pal[idx];
		break;
	}
	}
	// black (0x0000) is see-through only in direct-colour textures, and like palette index 0 only with
	// colour keying (0x88 bit 6): Bratz's panel backgrounds need it see-through, Cars' Knowledge World
	// photos (0x88 = 0) draw it black (they were full of holes). A palette entry of 0x0000 at a non-zero
	// index is opaque black (Wacky Races' menu text: "Options" lost its fill and "SELECT" its outline);
	// bit 15 is transparent in both
	static const bool direct_key = getenv("ZGPU_DIRECT_KEY") != nullptr; // old rule: also direct 0x0000 when keying
	if (!BIT(mode, 8) && ((direct_key && c == 0 && !((mode >> 9) & 3) && colour_key(s)) || BIT(c, 15)))
		return -1;
	return c;
}

ATTR_FORCE_INLINE int zevio_gpu_device::texel_raw(const rstate &s, rcache &pc, int tu, int tv) const
{
	return texel_fetch(s, pc, tu, tv, [this] (int x, int y) { return vram_r(x, y); });
}

ATTR_FORCE_INLINE bool zevio_gpu_device::texel_point(const rstate &s, rcache &pc, int tu, int tv, float &r, float &g, float &b, float &a) const
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
	float *const zp = depth ? zptr(s, x, y) : nullptr;
	if (zp)
	{
		// 0x84 bit 0 = depth test, bit 1 = depth write (Cars draws its sky and
		// ground with both clear, models with both set)
		float &zb = *zp;
		if (depth_test(s) && depth_fails(z, zb))
			return;
		// depth write follows 0x84 bit 1 alone, translucent (0x88 bit 1) draws included: Wacky Races'
		// Animals panel is drawn translucent and must hide the character behind it
		if (BIT(s.w84, 1))
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

// Near plane in eye space. [EMU] fixed: FA writes 0x74 = camera +176 (near?)
// and 0x75 = camera +180 (far, also picks its depth-clear value) [FW]; which is
// used how is unknown. ZGPU_NEAR74 (experiment, after flashem): use 0x74.
float zevio_gpu_device::near_plane() const
{
	static const bool near74 = getenv("ZGPU_NEAR74") != nullptr;
	return near74 ? std::max(reg_f(0x74), 1.0f / 4096.0f) : 1.0f / 4096.0f;
}

// perspective projection to screen; returns false if behind the near plane
bool zevio_gpu_device::project(vertex_t &v) const
{
	const float near_z = near_plane();
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
	if (m_count_stats)
	{
		m_stat_tris++;
		m_total_tris++;
	}
	// Coverage with exact integer edge functions on vertices snapped to 1/16 pixel, sampled at pixel
	// centres. A pixel exactly on an edge belongs to the triangle on one fixed side of it (the edge's
	// inward normal points right, or straight down), so triangles sharing an edge never both skip a
	// pixel (cracks: Bratz's store floor kept stale pixels along its seams) nor both draw it.
	// Interpolation uses barycentrics from the same edge values. [EMU: the hardware's sub-pixel
	// precision and tie rule are not known]
	auto snap = [] (float v) -> s64 { return s64(std::lround(std::clamp(v, -1.0e7f, 1.0e7f) * 16.0f)); };
	const s64 ax = snap(a.x), ay = snap(a.y), bx = snap(b.x), by = snap(b.y), cx = snap(c.x), cy = snap(c.y);
	const s64 area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
	if (area2 == 0)
		return;
	const float area = float(area2) / 256.0f; // in pixels (the trace prints it)
	const int cx0 = std::max<int>(s.clip0 & 0xfff, 0), cy0 = std::max<int>((s.clip0 >> 12) & 0xfff, 0);
	const int cx1 = std::min<int>(s.clip1 & 0xfff, 1023), cy1 = std::min<int>((s.clip1 >> 12) & 0xfff, ZH - 1);
	// pixels whose centre (16x + 8) lies within the snapped bounding box
	auto lo = [] (s64 v) { return int(std::clamp<s64>((v - 8 + 15) >> 4, -100000, 100000)); };
	auto hi = [] (s64 v) { return int(std::clamp<s64>((v - 8) >> 4, -100000, 100000)); };
	const int minx = std::max(cx0, lo(std::min({ ax, bx, cx })));
	const int maxx = std::min(cx1, hi(std::max({ ax, bx, cx })));
	const int miny = std::max({ cy0, band_y0, lo(std::min({ ay, by, cy })) });
	const int maxy = std::min({ cy1, band_y1 - 1, hi(std::max({ ay, by, cy })) });
	if (minx > maxx || miny > maxy)
		return;
	// edge functions, positive inside: e0 opposite a (b->c), e1 opposite b (c->a), e2 opposite c (a->b)
	const s64 sgn = (area2 > 0) ? 1 : -1;
	struct edge { s64 dx, dy, e; };
	auto make = [&] (s64 px, s64 py, s64 qx, s64 qy) -> edge
	{
		edge E;
		E.dx = sgn * (py - qy) * 16; // per pixel step in x
		E.dy = sgn * (qx - px) * 16; // per pixel step in y
		const s64 sx = s64(minx) * 16 + 8, sy = s64(miny) * 16 + 8;
		E.e = sgn * ((qx - px) * (sy - py) - (qy - py) * (sx - px));
		// tie rule: an edge pixel is kept only when the inward normal points right, or straight down
		if (!(E.dx > 0 || (E.dx == 0 && E.dy > 0)))
			E.e -= 1;
		return E;
	};
	edge e0 = make(bx, by, cx, cy), e1 = make(cx, cy, ax, ay), e2 = make(ax, ay, bx, by);
	const float inv_area = 1.0f / float(area2 * sgn);
	// 0x84: bit 0 depth test, bit 1 depth write, bit 4 colour write
	const bool ztest = depth_test(s), zwrite = BIT(s.w84, 1), blend = BIT(s.w88, 1), cwrite = BIT(s.w84, 4) || BIT(s.w84, 0);
	for (int y = miny; y <= maxy; y++, e0.e += e0.dy, e1.e += e1.dy, e2.e += e2.dy)
	{
		s64 r0 = e0.e, r1 = e1.e, r2 = e2.e;
		bool inside = false;
		for (int x = minx; x <= maxx; x++, r0 += e0.dx, r1 += e1.dx, r2 += e2.dx)
		{
			if (r0 < 0 || r1 < 0 || r2 < 0)
			{
				if (inside)
					break; // left the (convex) triangle on this row
				continue;
			}
			inside = true;
			// barycentrics (the tie bias is one 1/256 pixel unit: negligible here)
			const float w0 = float(r0) * inv_area, w1 = float(r1) * inv_area, w2 = float(r2) * inv_area;
			const float iw = w0 * a.w + w1 * b.w + w2 * c.w;
			const float z = 1.0f / iw;
			// depth test first: occluded pixels need no texels (same result as plot)
			static const bool noztest = getenv("ZGPU_NOZTEST") != nullptr; // test: no 3D depth test
			float *const zp = zptr(s, x, y);
			if (zp && ztest && !noztest && depth_fails(z, *zp))
			{
				if (m_count_stats)
				{
					s_zfail++;
					s_zfail_lo = std::min(s_zfail_lo, *zp);
					s_zfail_hi = std::max(s_zfail_hi, *zp);
					s_zfail_zlo = std::min(s_zfail_zlo, z);
				}
				continue;
			}
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
			if (zp && zwrite && (alpha >= 1.0f || !s_zalpha))
			{
				*zp = z;
			}
			// ZGPU_PIXLOG=x,y (with ZGPU_ONEBAND): print what every 3D draw computes at that pixel
			static const auto pixlog = [] { int px = -1, py = -1; if (const char *e = getenv("ZGPU_PIXLOG")) sscanf(e, "%d,%d", &px, &py); return std::make_pair(px, py); }();
			if (x == pixlog.first && y == pixlog.second)
			{
				const u16 dst = vram_r(x, y);
				fprintf(stderr, "PIX %d,%d t %.4f clip %06x-%06x 81 %06x 85 %06x verts (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) 84 %06x 88 %06x 86 %06x 87 %06x 8d %06x area %.1f vtx rgb %.3f %.3f %.3f alpha %.3f z %.4f zbuf %.4g dst %04x cwrite %d\n",
						x, y, machine().time().as_double(), s.clip0, s.clip1, s.w81, s.w85, a.x, a.y, b.x, b.y, c.x, c.y, s.w84, s.w88, s.w86, s.w87, s.w8d, area, r, g, bl, alpha, z, zp ? *zp : -1.0f, dst, cwrite);
			}
			if (!cwrite)
				continue;
			if (blend || alpha < 1.0f || m_count_stats)
				plot(s, x, y, z, r, g, bl, alpha, false);
			else
				vram_w(x, y, f_to_bgr555(r, g, bl));
		}
	}
}

void zevio_gpu_device::draw_3d_tri(vertex_t a, vertex_t b, vertex_t c)
{
	if (m_no3d) return;
	// Colour is drawn when 0x84 has bit 4 or bit 0 (depth test) set; with neither, and no depth write,
	// the primitive is skipped. [EMU, from the games: Wacky Races draws untextured quads with 0x84 = 0x04
	// that must stay invisible; Bratz draws its Bratz World menu as textured quads with 0x84 = 0x05; the bits'
	// real meaning is not known (the firmware sets bits 2 and 4 from two flags of each object)]
	if (!BIT(m_w84, 4) && !BIT(m_w84, 1) && !BIT(m_w84, 0))
	{
		// ZGPU_SKIPLOG: each new state combination of a skipped primitive, with the time
		static const bool skiplog = getenv("ZGPU_SKIPLOG") != nullptr;
		if (skiplog)
		{
			static std::set<std::tuple<u32, u32, u32, u32, u32>> seen;
			if (seen.emplace(m_w84, m_w88, m_w86, m_w87, m_w1c).second)
				logerror("SKIP3D t=%.3f 84 %06x 88 %06x 86 %06x 87 %06x 1c %06x 1e %06x 8d %06x\n", machine().time().as_double(), m_w84, m_w88, m_w86, m_w87, m_w1c, m_w1e, m_w8d);
		}
		return; // writes neither colour nor depth (Wacky Race's menu draws such a quad)
	}
	if (!BIT(m_w84, 4) && !BIT(m_w84, 0))
	{
		static const bool skiplog = getenv("ZGPU_SKIPLOG") != nullptr;
		if (skiplog)
		{
			static std::set<std::tuple<u32, u32, u32, u32, u32>> seen;
			if (seen.emplace(m_w84, m_w88, m_w86, m_w87, m_w1c).second)
				logerror("DEPTHONLY3D t=%.3f 84 %06x 88 %06x 86 %06x 87 %06x 1c %06x 1e %06x 8d %06x\n", machine().time().as_double(), m_w84, m_w88, m_w86, m_w87, m_w1c, m_w1e, m_w8d);
		}
	}
	// clip against the near plane in eye space, then project
	const float near_z = near_plane();
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
			// always from the vertex in front of the plane, so the two triangles sharing this edge get
			// exactly the same point (no crack along it)
			const vertex_t &f = pin ? p : q, &k = pin ? q : p;
			const float t = (near_z - f.z) / (k.z - f.z);
			vertex_t m;
			m.x = f.x + (k.x - f.x) * t; m.y = f.y + (k.y - f.y) * t; m.z = near_z;
			m.u = f.u + (k.u - f.u) * t; m.v = f.v + (k.v - f.v) * t;
			m.r = f.r + (k.r - f.r) * t; m.g = f.g + (k.g - f.g) * t; m.b = f.b + (k.b - f.b) * t;
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
	// back-face culling: with 0x1E bit 13 set, triangles whose screen-space area is positive (y down) are
	// dropped. FA draws Bratz's two-sided menu card as two strips, the second with its vertices in the
	// other order; the console shows only the lit front (the unlit back, drawn on top, turned the card
	// opaque and bright). Wacky Races' sky dome, seen from inside, is drawn with bit 13 clear (and must
	// stay). [EMU: bits 13 and 15 (engine flags 0x0001 / 0x0020) were set and clear together in every
	// draw seen; bit 13 is assumed. ZGPU_NOCULL turns culling off]
	static const bool nocull = getenv("ZGPU_NOCULL") != nullptr;
	if (BIT(m_w1e, 13) && !nocull)
	{
		const float area = (out[1].x - out[0].x) * (out[2].y - out[0].y) - (out[1].y - out[0].y) * (out[2].x - out[0].x);
		if (area > 0.0f)
			return;
	}
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

	// debug: ZGPU_PRIMLOG=<time> logs every 3D primitive drawn in the ZGPU_PRIMLOG_LEN (default 0.05) s after it
	static double primlog_at = getenv("ZGPU_PRIMLOG") ? atof(getenv("ZGPU_PRIMLOG")) : -1.0;
	const double now = machine().time().as_double();
	const bool primlog = primlog_at >= 0 && now >= primlog_at && now < primlog_at + primlog_len();
	m_count_stats = primlog || (VERBOSE & LOG_LIST);
	const int pix0 = m_stat_pixels, tri0 = m_stat_tris;
	s_zfail = 0; s_zfail_lo = s_zfail_zlo = 1e30f; s_zfail_hi = 0.0f;

	switch (kind)
	{
	case 0x40: // triangle strip: every second triangle's vertices are in the opposite order
		for (int i = 0; i + 2 < n; i++)
		{
			if (BIT(i, 0))
				draw_3d_tri(verts[i + 1], verts[i], verts[i + 2]);
			else
				draw_3d_tri(verts[i], verts[i + 1], verts[i + 2]);
		}
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
		logerror("PRIM %08x tris %d pix %d x %.0f..%.0f y %.0f..%.0f z %.3f..%.3f 1c %06x 1e %06x 84 %06x 88 %06x 86 %06x 87 %06x 8d %06x clip %06x-%06x 81 %06x 85 %06x zfail %d stored %.3f..%.3f own %.3f\n",
				header, m_stat_tris - tri0, m_stat_pixels - pix0, x0, x1, y0, y1, z0, z1, m_w1c, m_w1e, m_w84, m_w88, m_w86, m_w87, m_w8d, m_clip0, m_clip1, m_w81, m_w85, s_zfail, s_zfail_lo, s_zfail_hi, s_zfail_zlo);
		for (int i = 0; i < std::min(n, 8); i++)
			logerror("  VTX %d eye %.3f %.3f %.3f hi %04x rgb %.2f %.2f %.2f uv %.3f %.3f lit %d\n", i, verts[i].x, verts[i].y, verts[i].z, verts[i].hi, verts[i].r, verts[i].g, verts[i].b, verts[i].u, verts[i].v, verts[i].lit);
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
	// ZGPU_TEXDUMP: also write each texture (as BGR555 after the CLUT, bit 15 shown magenta) to tex_<87>_<86>.ppm
	static const bool texdump = getenv("ZGPU_TEXDUMP") != nullptr;
	std::vector<u8> ppm;
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
			if (texdump)
			{
				const bool t = BIT(c, 15);
				ppm.push_back(t ? 255 : pal5bit(c & 0x1f));
				ppm.push_back(t ? 0 : pal5bit((c >> 5) & 0x1f));
				ppm.push_back(t ? 255 : pal5bit((c >> 10) & 0x1f));
			}
		}
	if (texdump)
	{
		if (FILE *const f = fopen(util::string_format("tex_%06x_%06x.ppm", m_w87, m_w86).c_str(), "wb"))
		{
			fprintf(f, "P6\n%d %d\n255\n", w, h);
			fwrite(ppm.data(), 1, ppm.size(), f);
			fclose(f);
		}
	}
	std::vector<std::pair<int, u16>> top;
	for (auto &e : hist)
		top.emplace_back(e.second, e.first);
	std::sort(top.rbegin(), top.rend());
	std::string s;
	for (int i = 0; i < std::min<int>(6, top.size()); i++)
		s += util::string_format(" %04x:%d", top[i].second, top[i].first);
	int zero = 0, b15 = 0;
	for (auto &e : hist)
	{
		zero += (e.first == 0) ? e.second : 0;
		b15 += BIT(e.first, 15) ? e.second : 0;
	}
	s += util::string_format("  (0000:%d bit15:%d)", zero, b15);
	if (((m_w86 >> 9) & 3) == 1)
	{
		int idx0 = 0;
		for (int v = 0; v < h; v++)
			for (int u = 0; u < w; u++)
			{
				const u16 hw = vram_r(tx + u / 2, ty + v);
				idx0 += ((BIT(u, 0) ? (hw >> 8) : hw) & 0xff) == 0;
			}
		s += util::string_format(" idx0:%d pal0:%04x", idx0, vram_r(m_clut[0] & 0xfff, (m_clut[0] >> 12) & 0xfff));
	}
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
	static double primlog_at = getenv("ZGPU_PRIMLOG") ? atof(getenv("ZGPU_PRIMLOG")) : -1.0;
	const double now = machine().time().as_double();
	if (primlog_at >= 0 && now >= primlog_at && now < primlog_at + primlog_len())
		logerror("XFORM %08x n %d fmt %02x -> %08x\n", header, n, fmt, m_xfer);
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
	// Vertex x/y (and u/v) are GPU-f16: the only firmware code that builds
	// C0-C4 lists, FA's immediate-mode builders (Render_drawArraysFloat, e.g.
	// 0x10A642F8), writes them through converter +0x00 [FW, gpu.md 3.6]. No
	// tested game draws C0-C4. [EMU] the origin (clip top-left + 0x85) and u/v
	// in texels are unchanged guesses from when these were read as s16
	const int ox = (m_clip0 & 0xfff) + (m_w85 & 0xfff), oy = ((m_clip0 >> 12) & 0xfff) + ((m_w85 >> 12) & 0xfff);
	std::vector<vertex_t> v(n);
	for (int i = 0; i < n; i++)
	{
		const u32 base = addr + i * wpv * 4;
		const u32 xy = rd32(base + 4);
		v[i] = vertex_t{};
		v[i].x = ox + f16_to_float(xy & 0xffff);
		v[i].y = oy + f16_to_float(xy >> 16);
		v[i].w = 1.0f;
		v[i].z = 0.0f;
		int k = 2;
		u32 col = 0xffffff;
		if (BIT(flags, 0))
		{
			const u32 uv = rd32(base + k++ * 4);
			v[i].u = f16_to_float(uv & 0xffff);
			v[i].v = f16_to_float(uv >> 16);
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
	{
		static double primlog_at = getenv("ZGPU_PRIMLOG") ? atof(getenv("ZGPU_PRIMLOG")) : -1.0;
		const double now = machine().time().as_double();
		if (primlog_at >= 0 && now >= primlog_at && now < primlog_at + primlog_len() && n)
			logerror("PRIM2D %08x n %d v0 %.1f,%.1f v1 %.1f,%.1f 84 %06x 88 %06x 86 %06x 87 %06x 8d %06x clip %06x-%06x 85 %06x\n", header, n,
					v[0].x, v[0].y, v[std::min(n - 1, 1)].x, v[std::min(n - 1, 1)].y, m_w84, m_w88, m_w86, m_w87, m_w8d, m_clip0, m_clip1, m_w85);
	}

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
	const u32 w0 = rd32(addr), xy = rd32(addr + 4), wh = rd32(addr + 8);
	int k = 3;
	u32 uv = 0, col = 0xffffff;
	if (BIT(flags, 0))
		uv = rd32(addr + k++ * 4);
	if (BIT(flags, 2))
		col = rd32(addr + k++ * 4);
	addr += count * 4;

	// Vertex x/y (and u/v) are GPU-f16: the only firmware code that builds
	// C0-C4 lists, FA's immediate-mode builders (Render_drawArraysFloat, e.g.
	// 0x10A642F8), writes them through converter +0x00 [FW, gpu.md 3.6]. No
	// tested game draws C0-C4. [EMU] the origin (clip top-left + 0x85) and u/v
	// in texels are unchanged guesses from when these were read as s16
	const int ox = (m_clip0 & 0xfff) + (m_w85 & 0xfff), oy = ((m_clip0 >> 12) & 0xfff) + ((m_w85 >> 12) & 0xfff);
	const int x0 = ox + s16(xy & 0xffff), y0 = oy + s16(xy >> 16);
	const int w = std::min<int>(wh & 0xffff, 1024), h = std::min<int>(wh >> 16, 1024);
	static double primlog_at = getenv("ZGPU_PRIMLOG") ? atof(getenv("ZGPU_PRIMLOG")) : -1.0;
	const double now = machine().time().as_double();
	static const int primlog_ymin = getenv("ZGPU_PRIMLOG_YMIN") ? atoi(getenv("ZGPU_PRIMLOG_YMIN")) : INT_MIN; // only sprites from this row down
	if (primlog_at >= 0 && now >= primlog_at && now < primlog_at + primlog_len() && y0 + h > primlog_ymin)
	{
		logerror("SPR %08x w0 %08x xy %d,%d wh %dx%d uv %08x col %06x 84 %06x 88 %06x 86 %06x 87 %06x 8d %06x 1c %06x 1e %06x clip %06x-%06x 85 %06x raw %08x\n",
				header, w0, x0, y0, w, h, uv, col, m_w84, m_w88, m_w86, m_w87, m_w8d, m_w1c, m_w1e, m_clip0, m_clip1, m_w85, xy, m_w81);
		if (BIT(flags, 0))
			log_texture();
	}

	if (in_zarea(y0) && auxlog_now(machine()))
		logerror("AUX sprite %d,%d %dx%d tex %06x 84 %06x\n", x0, y0, w, h, m_w87, m_w84);
	rcmd c{};
	c.type = CMD_SPRITE;
	c.st = cur_state();
	c.a0 = u32(x0 & 0xffff) | (u32(y0) << 16);
	c.a1 = u32(w) | (u32(h) << 16);
	c.a2 = uv;
	c.a3 = col;
	// ZGPU_SPRCOL_RGB (experiment): the sprite colour dword as 0x00RRGGBB
	static const bool sprcol_rgb = getenv("ZGPU_SPRCOL_RGB") != nullptr;
	if (sprcol_rgb)
		c.a3 = (col & 0xff00ff00) | ((col & 0xff) << 16) | ((col >> 16) & 0xff);
	c.flags = flags;
	// word 0, low half: reciprocal depth (see ZSCALE); 0 is infinitely far. Cars' menu draws its text at 0x4651
	// and the orange lines over it at
	// 0x463C (farther): the depth test keeps the lines behind the letters
	const float rz = f16_to_float(w0 & 0xffff);
	c.z = (rz > 0.0f) ? ZSCALE / rz : std::numeric_limits<float>::max();
	add_cmd(std::move(c));
}

void zevio_gpu_device::draw_sprite(const rstate &s, rcache &pc, const rcmd &c, int band_y0, int band_y1)
{
	const int x0 = s16(c.a0 & 0xffff), y0 = s16(c.a0 >> 16);
	const int w = c.a1 & 0xffff, h = c.a1 >> 16;
	const u32 uv = c.a2, col = c.a3;
	const u8 flags = c.flags;

	// the integer paths below fetch each destination row and texture row once (vram_row) instead of
	// working out the tiled address of every pixel; the arithmetic is unchanged
	const int cx0 = std::max({ int(s.clip0 & 0xfff), x0, 0 });
	const int cy0 = std::max({ int((s.clip0 >> 12) & 0xfff), y0, 0, band_y0 });
	const int cx1 = std::min({ int(s.clip1 & 0xfff), x0 + w - 1, 1023 });
	const int cy1 = std::min({ int((s.clip1 >> 12) & 0xfff), y0 + h - 1, band_y1 - 1 });
	const int mr = (col & 0xff) + 1, mg = ((col >> 8) & 0xff) + 1, mb = ((col >> 16) & 0xff) + 1;
	const bool textured = BIT(flags, 0);
	const int u0 = int(uv & 0xffff), v0 = int(uv >> 16);
	const int tvmask = (8 << ((s.w86 >> 3) & 7)) - 1, ty0 = (s.w87 >> 12) & 0xfff;

	// depth (0x84 bit 0 test, bit 1 write) against the 3D depth buffer, per drawn texel
	const bool ztest = depth_test(s), zwrite = BIT(s.w84, 1);
	const float d = c.z;
	auto depth_ok = [&] (int x, int y, bool opaque = true) -> bool
	{
		float *const zp = (ztest || zwrite) ? zptr(s, x, y) : nullptr;
		if (!zp)
			return true;
		float &zb = *zp;
		if (ztest && z_fails_sprite(d, zb))
			return false;
		if (zwrite && (opaque || !s_zalpha))
			zb = d;
		return true;
	};

	// fast path: opaque (no 0x88 blend), non-ARGB4444 texels, integer colour
	// modulation, clipped once per sprite
	static const bool nofast = getenv("ZGPU_NOFAST") != nullptr;
	if (!nofast && !BIT(s.w88, 1) && !BIT(s.w86, 8))
	{
		const bool modulate = (col & 0xffffff) != 0xffffff;
		const u16 flat = (((col >> 19) & 0x1f) << 10) | (((col >> 11) & 0x1f) << 5) | ((col >> 3) & 0x1f);
		for (int y = cy0; y <= cy1; y++)
		{
			const vrow drow = vram_row(y);
			const bool zrow = in_zarea(y);
			const int tv = v0 + (y - y0);
			const vrow trow = textured ? vram_row(ty0 + (tv & tvmask)) : vrow{ nullptr, true };
			auto rd = [&trow] (int x, int) -> u16 { return trow[x]; };
			for (int x = cx0; x <= cx1; x++)
			{
				u16 t;
				if (textured)
				{
					const int tu = u0 + (x - x0);
					const int r = trow.p ? texel_fetch(s, pc, tu, tv, rd) : texel_raw(s, pc, tu, tv);
					if (r < 0)
						continue;
					t = r;
					if (modulate)
						t = (((((t >> 10) & 0x1f) * mb) >> 8) << 10) | (((((t >> 5) & 0x1f) * mg) >> 8) << 5) | (((t & 0x1f) * mr) >> 8);
				}
				else
					t = flat;
				if (!depth_ok(x, y))
					continue;
				if (drow.p)
				{
					drow[x] = t;
					if (zrow)
						zset(x, y, t);
				}
				else
					vram_w(x, y, t);
				if (s_dsta)
					abuf(x, y) = 0;
			}
		}
		return;
	}

	// blended and ARGB4444 sprites: the same in integers (8-bit weights)
	{
		// c5to8[c] = c * 255 / 31, c4to8[c] = c * 255 / 15, a4[c] = c * 256 / 15, to5[v] = (v * 31 + 127) / 255
		static const struct tables
		{
			u8 c5to8[32], c4to8[16], to5[256];
			u16 a4[16];
			tables()
			{
				for (int i = 0; i < 32; i++) c5to8[i] = i * 255 / 31;
				for (int i = 0; i < 16; i++) { c4to8[i] = i * 255 / 15; a4[i] = i * 256 / 15; }
				for (int i = 0; i < 256; i++) to5[i] = (i * 31 + 127) / 255;
			}
		} tb;
		const bool blend = BIT(s.w88, 1), argb4444 = BIT(s.w86, 8);
		const int fr = (s.w8d & 0xff) + 1, fg = ((s.w8d >> 8) & 0xff) + 1, fb = ((s.w8d >> 16) & 0xff) + 1;
		// ZGPU_DSTA (experiment, after flashem): under 0x88 = 0x42 a sprite with a 16-colour-bit
		// texture takes its alpha from what an ARGB4444 sprite left there (Bratz's corner "glint")
		const bool from_dst = s_dsta && textured && !argb4444 && (s.w88 & 0x43) == 0x42;
		for (int y = cy0; y <= cy1; y++)
		{
			const vrow drow = vram_row(y);
			const bool zrow = in_zarea(y);
			const int tv = v0 + (y - y0);
			const vrow trow = textured ? vram_row(ty0 + (tv & tvmask)) : vrow{ nullptr, true };
			auto rd = [&trow] (int x, int) -> u16 { return trow[x]; };
			for (int x = cx0; x <= cx1; x++)
			{
				int r, g, b, a = 256; // 8-bit colour channels, alpha 0-256
				if (textured)
				{
					const int tu = u0 + (x - x0);
					const int t = trow.p ? texel_fetch(s, pc, tu, tv, rd) : texel_raw(s, pc, tu, tv);
					if (t < 0)
						continue;
					if (argb4444)
					{
						a = tb.a4[(t >> 12) & 15];
						if (!a)
							continue;
						r = tb.c4to8[t & 15]; g = tb.c4to8[(t >> 4) & 15]; b = tb.c4to8[(t >> 8) & 15];
					}
					else
					{
						r = tb.c5to8[t & 0x1f]; g = tb.c5to8[(t >> 5) & 0x1f]; b = tb.c5to8[(t >> 10) & 0x1f];
						if (from_dst)
						{
							a = abuf(x, y);
							if (!a)
								continue;
							a += a >> 7; // 0-255 -> 0-256
						}
					}
					r = (r * mr) >> 8; g = (g * mg) >> 8; b = (b * mb) >> 8;
				}
				else
				{
					r = col & 0xff; g = (col >> 8) & 0xff; b = (col >> 16) & 0xff;
				}
				if (!depth_ok(x, y, a >= 256))
					continue;
				if (blend || a < 256)
				{
					const u16 d = drow.p ? drow[x] : vram_r(x, y);
					const int dr = tb.c5to8[d & 0x1f], dg = tb.c5to8[(d >> 5) & 0x1f], db = tb.c5to8[(d >> 10) & 0x1f];
					const int kr = blend ? (fr * a) >> 8 : a, kg = blend ? (fg * a) >> 8 : a, kb = blend ? (fb * a) >> 8 : a;
					r = (r * kr + dr * (256 - kr)) >> 8;
					g = (g * kg + dg * (256 - kg)) >> 8;
					b = (b * kb + db * (256 - kb)) >> 8;
				}
				const u16 out = (tb.to5[b] << 10) | (tb.to5[g] << 5) | tb.to5[r];
				if (drow.p)
				{
					drow[x] = out;
					if (zrow)
						zset(x, y, out);
				}
				else
					vram_w(x, y, out);
				if (s_dsta)
					abuf(x, y) = argb4444 ? std::min(a, 255) : 0;
			}
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
			if (s_dsta)
				std::fill_n(&abuf(xx, y + j), run, 0);
			if (vram_in_aux(y + j))
			{
				std::fill_n(&aux_px(xx, y + j), run, col);
				for (int k = 0; k < run; k++)
					zset(xx + k, y + j, col);
			}
			else if (off + run * 2 <= m_ram_size)
			{
#ifdef LSB_FIRST
				std::fill_n(reinterpret_cast<u16 *>(m_ram) + (off >> 1), run, col);
				if (in_zarea(y + j))
					for (int k = 0; k < run; k++)
						zset(xx + k, y + j, col);
#else
				for (int k = 0; k < run; k++)
					vram_w(xx + k, y + j, col);
#endif
			}
			i += run;
		}
	}
}


/***************************************************************************
    Render thread
***************************************************************************/

zevio_gpu_device::rstate zevio_gpu_device::cur_state() const
{
	// 0x86 bits 16-15 are the texture's slot; FA (10A5B42C) loads the slot's
	// CLUT position with 0x8F + slot (8 bpp) / 0x92 + slot (4 bpp)
	const int slot = std::min<int>((m_w86 >> 15) & 3, 2);
	return rstate{ m_w84, m_w86, m_w87, m_w88, m_w8d, m_clip0, m_clip1, m_clut[slot], m_clut[3 + slot], m_w81, m_w85 };
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
	}
	c.ymin = std::max({ c.ymin, int((c.st.clip0 >> 12) & 0xfff), 0 });
	c.ymax = std::min({ c.ymax, int((c.st.clip1 >> 12) & 0xfff), ZH - 1 });
	if (c.type == CMD_FILL)
	{
		// not clipped
		c.ymin = int(c.a0 >> 16);
		c.ymax = int(c.a0 >> 16) + int(c.a1 >> 16) - 1;
	}
	if (c.ymin > c.ymax)
		return;
	{
		// rough pixel count for the engine's drawing time
		int w = 1;
		if (c.type == CMD_TRI3D || c.type == CMD_TRI2D)
		{
			const float x0 = std::max<float>(std::min({ c.v[0].x, c.v[1].x, c.v[2].x }), c.st.clip0 & 0xfff);
			const float x1 = std::min<float>(std::max({ c.v[0].x, c.v[1].x, c.v[2].x }), c.st.clip1 & 0xfff);
			w = std::max(0, int(x1 - x0) + 1) / 2;
		}
		else if (c.type == CMD_SPRITE || c.type == CMD_FILL)
			w = c.a1 & 0xffff;
		m_work_px += u64(c.ymax - c.ymin + 1) * w;
		m_work_prims++;
	}
	{
		// areas drawn since the last frame fence, as rows of the RAM surface
		// at 0x0FFD0000 (what the display compares); the aux area is not shown
		const int x0 = (c.type == CMD_FILL) ? int(c.a0 & 0xffff) : int(c.st.clip0 & 0xfff);
		const int x1 = (c.type == CMD_FILL) ? x0 + int(c.a1 & 0xffff) - 1 : int(c.st.clip1 & 0xfff);
		const int xo = ((m_vram_base - 0x0ffd0000) & 0x3fff) / 512 * 32;
		const std::array<int, 4> r{ x0 + xo, abs_row(std::max(c.ymin, m_vram_y0)), x1 + xo, abs_row(c.ymax) };
		if (c.ymax < m_vram_y0)
			;
		else if (m_drawn.empty() || m_drawn.back() != r)
		{
			if (m_drawn.size() >= 64)
			{
				for (auto &d : m_drawn)
					m_drawn[0] = { std::min(m_drawn[0][0], d[0]), std::min(m_drawn[0][1], d[1]), std::max(m_drawn[0][2], d[2]), std::max(m_drawn[0][3], d[3]) };
				m_drawn.resize(1);
			}
			m_drawn.push_back(r);
		}
	}
	// bands are cut in depth-area rows, so that draws meeting in the depth area (a buffer's draws, the
	// depth clears and masks drawn into the area itself) stay in order on one thread
	c.koff = in_zarea(c.ymax) ? 0 : zoff_y(c.st);
	c.ymin -= c.koff;
	c.ymax -= c.koff;
	if (m_count_stats)
	{
		// debug statistics are counted in order on this thread
		sync();
		std::vector<rcmd> one{ c };
		exec_band(one, -(1 << 20), 1 << 20);
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
		m_seq_submitted++;
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

// once per displayed frame: the display shows the previous frame, so only the
// work queued before then has to be finished and the CPU keeps running while
// the current frame renders
bool zevio_gpu_device::frame_fence(int x0, int y0, int x1, int y1)
{
	// debug: ZGPU_ZDUMP=<time> writes the depth area (512 x 240) at the first frame after that time to zdump.pgm
	static double zdump_at = getenv("ZGPU_ZDUMP") ? atof(getenv("ZGPU_ZDUMP")) : -1.0;
	if (zdump_at >= 0 && machine().time().as_double() >= zdump_at)
	{
		zdump_at = -1.0;
		sync();
		if (FILE *f = fopen("zdump.pgm", "wb"))
		{
			fprintf(f, "P5 512 240 255\n");
			for (int y = 0; y < 240; y++)
				for (int x = 0; x < 512; x++)
				{
					const float d = m_zbuf[y * ZW + x];
					fputc(d >= 1e30f ? 255 : std::clamp(int(d * 20.0f), 0, 254), f);
				}
			fclose(f);
		}
	}
	submit();
	bool overlap = false;
	for (const auto &r : m_drawn)
		if (r[0] <= x1 && r[2] >= x0 && r[1] <= y1 && r[3] >= y0)
			overlap = true;
	m_drawn.clear();
	if (!m_async)
		return false;
	std::unique_lock<std::mutex> lock(m_mutex);
	const u64 target = overlap ? m_seq_submitted : m_fence;
	m_cv.wait(lock, [this, target] { return m_seq_done >= target; });
	m_fence = m_seq_submitted;
	return !overlap; // after a full wait the current frame is complete
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
		m_seq_done++;
		m_busy = false;
		m_cv.notify_all();
	}
}

void *zevio_gpu_device::band_cb(void *param, int threadid)
{
	auto *const job = reinterpret_cast<band_job *>(param);
	job->gpu->exec_band(*job->cmds, job->y0, job->y1);
	job->left->fetch_sub(1, std::memory_order_release);
	return nullptr;
}

void zevio_gpu_device::exec_batch(const std::vector<rcmd> &cmds)
{
	int ymin = std::numeric_limits<int>::max(), ymax = std::numeric_limits<int>::min();
	u64 work = 0;
	// bands are cut in depth-area rows (add_cmd). That keeps every pixel on one thread only while all
	// commands drawing the same colour rows share one offset; a batch where commands with different
	// offsets cover the same colour rows (a fill recorded under another pass's 0x81) runs in order
	std::vector<std::array<int, 3>> spans; // offset, first and last colour row
	bool mixed = false;
	for (const auto &c : cmds)
	{
		ymin = std::min(ymin, c.ymin);
		ymax = std::max(ymax, c.ymax);
		work += u64(c.ymax - c.ymin + 1) * ((c.type == CMD_SPRITE || c.type == CMD_FILL) ? (c.a1 & 0xffff) : 64);
		const int r0 = c.ymin + c.koff, r1 = c.ymax + c.koff;
		auto it = std::find_if(spans.begin(), spans.end(), [&c] (const auto &s) { return s[0] == c.koff; });
		if (it == spans.end())
			spans.push_back({ c.koff, r0, r1 });
		else
		{
			(*it)[1] = std::min((*it)[1], r0);
			(*it)[2] = std::max((*it)[2], r1);
		}
	}
	for (size_t i = 0; i < spans.size() && !mixed; i++)
		for (size_t j = i + 1; j < spans.size() && !mixed; j++)
			mixed = spans[i][1] <= spans[j][2] && spans[j][1] <= spans[i][2];
	static const bool oneband = getenv("ZGPU_ONEBAND") != nullptr; // test: no band threads
	if (cmds.empty() || mixed || oneband)
	{
		exec_band(cmds, -(1 << 20), 1 << 20);
		return;
	}
	const int rows = ymax - ymin + 1;
	if (!m_queue || rows < 32 || work < 20000)
	{
		exec_band(cmds, -(1 << 20), 1 << 20);
		return;
	}
	const int bands = std::min(m_bands, rows / 8);
	band_job jobs[MAX_BANDS];
	std::atomic<int> left(bands);
	for (int i = 0; i < bands; i++)
		jobs[i] = { this, &cmds, ymin + rows * i / bands, ymin + rows * (i + 1) / bands, &left };
	osd_work_item_queue_multiple(m_queue, band_cb, bands, jobs, sizeof(band_job), WORK_ITEM_FLAG_AUTO_RELEASE);
	// the queue wait can return before the last item finishes: count them ourselves
	while (left.load(std::memory_order_acquire))
	{
		osd_work_queue_wait(m_queue, osd_ticks_per_second() / 1000);
		if (left.load(std::memory_order_acquire))
			std::this_thread::yield();
	}
}

void zevio_gpu_device::exec_band(const std::vector<rcmd> &cmds, int y0, int y1)
{
	rcache pc;
	for (const auto &c : cmds)
	{
		if (c.ymax < y0 || c.ymin >= y1)
			continue;
		const int cy0 = y0 + c.koff, cy1 = y1 + c.koff; // the band in colour rows
		switch (c.type)
		{
		case CMD_TRI3D:
			draw_triangle(c.st, pc, c.v[0], c.v[1], c.v[2], c.textured, cy0, cy1);
			break;
		case CMD_TRI2D:
			draw_tri2d(c.st, pc, c.v[0], c.v[1], c.v[2], c.textured, cy0, cy1);
			break;
		case CMD_LINE:
			draw_line(c.st, c.v[0], c.v[1], cy0, cy1);
			break;
		case CMD_POINT:
			if (int(c.v[0].y) >= cy0 && int(c.v[0].y) < cy1)
				plot(c.st, int(c.v[0].x), int(c.v[0].y), c.v[0].z, c.v[0].r, c.v[0].g, c.v[0].b, 1.0f, c.flags & 1);
			break;
		case CMD_SPRITE:
			draw_sprite(c.st, pc, c, cy0, cy1);
			break;
		case CMD_FILL:
			draw_fill(c, cy0, cy1);
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
		case 0x10:
			// capture buffer [FW]: FA reads +0x5C back as the write pointer just
			// past the C1 triangles the GPU stored there (FA 0x10AB66B0, 0x10A5D360).
			// TODO: nothing is captured; +0x5C = start means "empty"
			m_regs[0x5c / 4] = rd32(addr);
			addr += 4;
			break;
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
				set_r(reg + i, rd16(addr + i * 2));
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
			if (in_zarea(y) && auxlog_now(machine()))
				logerror("AUX load %d,%d %dx%d first %04x\n", x, y, w, h, rd16(src));
			if (xferlog_now(machine()))
				logerror("XFER load %d,%d %dx%d from %08x\n", x, y, w, h, src);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					vram_w(x + i, y + j, rd16(src + (j * w + i) * 2));
			m_work_xfer += u64(w) * h;
			break;
		}
		case 0xd1: // store image to the 0x0C address
		{
			const u32 pos = rd32(addr), size = rd32(addr + 4);
			addr += 8;
			sync();
			const int x = pos & 0xffff, y = pos >> 16, w = std::min<int>(size & 0xffff, 1024), h = std::min<int>(size >> 16, 8192);
			if (xferlog_now(machine()))
				logerror("XFER store %d,%d %dx%d to %08x\n", x, y, w, h, m_xfer);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					wr16(m_xfer + (j * w + i) * 2, vram_r(x + i, y + j));
			m_work_xfer += u64(w) * h;
			break;
		}
		case 0xd3: // fill
		{
			const u32 pos = rd32(addr), size = rd32(addr + 4), col = rd32(addr + 8);
			addr += 12;
			m_stat_fills++;
			const int h = std::min<int>(size >> 16, 8192);
			if (in_zarea(int(pos >> 16)) && auxlog_now(machine()))
				logerror("AUX fill %d,%d %dx%d value %04x (aux rows above %d, RAM base %08x)\n", int(pos & 0xffff), int(pos >> 16), int(size & 0xffff), h, col & 0xffff, m_vram_y0, m_vram_base);
			rcmd c{};
			c.type = CMD_FILL;
			c.st = cur_state();
			c.a0 = pos;
			c.a1 = std::min<u32>(size & 0xffff, 1024) | (u32(h) << 16);
			c.a3 = col;
			add_cmd(std::move(c));
			break;
		}
		case 0xd2: // VRAM to VRAM copy: source, size, destination
		{
			// FA's GpuImg_copyRect (Bratz 10AB99EC) emits D2000003, sy<<16|sx, h<<16|w,
			// dy<<16|dx (D6000003 instead under a flag: not handled); no known game calls it
			const u32 src = rd32(addr), size = rd32(addr + 4), dst = rd32(addr + 8);
			addr += 12;
			const int dx = dst & 0xffff, dy = dst >> 16, sx = src & 0xffff, sy = src >> 16;
			const int w = std::min<int>(size & 0xffff, 1024), h = std::min<int>(size >> 16, 4096);
			if (in_zarea(dy) && auxlog_now(machine()))
				logerror("AUX copy to %d,%d %dx%d from %d,%d\n", dx, dy, w, h, sx, sy);
			static const bool d2log = getenv("ZGPU_D2LOG") != nullptr;
			if (d2log)
				logerror("D2 t=%.3f words %08x %08x %08x (list at %08x)\n", machine().time().as_double(), dst, size, src, addr - 16);
			sync();
			std::vector<u16> tmp(w * h);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					tmp[j * w + i] = vram_r(sx + i, sy + j);
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					vram_w(dx + i, dy + j, tmp[j * w + i]);
			m_work_xfer += u64(w) * h;
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
