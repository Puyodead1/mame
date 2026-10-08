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

#define LOG_UNKNOWN (1U << 1)
#define LOG_LIST    (1U << 2)

#define VERBOSE (LOG_UNKNOWN)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ZEVIO_GPU, zevio_gpu_device, "zevio_gpu", "Zevio 1020 GPU")

namespace {

constexpr int ZW = 1024, ZH = 1024;

inline void bgr555_to_f(u16 c, float &r, float &g, float &b)
{
	r = (c & 0x1f) / 31.0f;
	g = ((c >> 5) & 0x1f) / 31.0f;
	b = ((c >> 10) & 0x1f) / 31.0f;
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

void zevio_gpu_device::device_reset()
{
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
	update_irq();
}


/***************************************************************************
    GPU 16-bit float: sign, 6-bit exponent (bias 31), 9-bit mantissa
***************************************************************************/

float zevio_gpu_device::f16_to_float(u16 v)
{
	if (!(v & 0x7fff))
		return 0.0f;
	const float f = ldexpf(1.0f + (v & 0x1ff) / 512.0f, int((v >> 9) & 0x3f) - 31);
	return (v & 0x8000) ? -f : f;
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
u32 zevio_gpu_device::vram_addr(int x, int y) const
{
	const u32 xb = (x & 1023) * 2;
	return 0x0ffd0000 + (y >> 3) * 16384 + (xb >> 6) * 512 + (y & 7) * 64 + (xb & 63);
}


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

u32 zevio_gpu_device::regs_r(offs_t offset)
{
	if (offset == 0)
		return m_regs[0] & 0xffff; // engine idle
	return m_regs[offset];
}

void zevio_gpu_device::regs_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x00 / 4:
		if (data == 1)
			run_list(m_regs[0x28 / 4]);
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

bool zevio_gpu_device::in_clip(int x, int y) const
{
	return x >= int(m_clip0 & 0xfff) && x <= int(m_clip1 & 0xfff)
		&& y >= int((m_clip0 >> 12) & 0xfff) && y <= int((m_clip1 >> 12) & 0xfff)
		&& x >= 0 && x < 1024 && y >= 0;
}

// sample the current texture; u/v normalized (3D) or in texels (2D)
bool zevio_gpu_device::texel(float u, float v, bool normalized, float &r, float &g, float &b, float &a) const
{
	const u32 mode = m_w86;
	const int w = 8 << (mode & 7), h = 8 << ((mode >> 3) & 7);
	int tu = normalized ? int(std::floor(u * w)) : int(u);
	int tv = normalized ? int(std::floor(v * h)) : int(v);
	tu &= w - 1;
	tv &= h - 1;
	const int x0 = m_w87 & 0xfff, y0 = (m_w87 >> 12) & 0xfff;
	const bool argb4444 = BIT(mode, 8);
	u16 c;
	switch ((mode >> 9) & 3)
	{
	case 0:
		c = vram_r(x0 + tu, y0 + tv);
		break;
	case 1:
	{
		const u16 hw = vram_r(x0 + tu / 2, y0 + tv);
		const u8 idx = BIT(tu, 0) ? (hw >> 8) : (hw & 0xff);
		c = vram_r((m_clut[0] & 0xfff) + idx, (m_clut[0] >> 12) & 0xfff);
		break;
	}
	default:
	{
		const u16 hw = vram_r(x0 + tu / 4, y0 + tv);
		const u8 idx = (hw >> ((tu & 3) * 4)) & 0x0f;
		c = vram_r((m_clut[3] & 0xfff) + idx, (m_clut[3] >> 12) & 0xfff);
		break;
	}
	}
	if (argb4444)
	{
		r = (c & 15) / 15.0f;
		g = ((c >> 4) & 15) / 15.0f;
		b = ((c >> 8) & 15) / 15.0f;
		a = ((c >> 12) & 15) / 15.0f;
		return a > 0.0f;
	}
	if (c == 0)
		return false; // transparent texel
	bgr555_to_f(c, r, g, b);
	a = 1.0f;
	return true;
}

void zevio_gpu_device::plot(int x, int y, float z, float r, float g, float b, float a, bool depth)
{
	if (!in_clip(x, y))
		return;
	const int zx = x - int(m_clip0 & 0xfff), zy = y - int((m_clip0 >> 12) & 0xfff);
	if (depth && zx >= 0 && zy >= 0 && zx < ZW && zy < ZH)
	{
		float &zb = m_zbuf[zy * ZW + zx];
		if (z > zb)
			return;
		if (!BIT(m_w88, 1)) // translucent pass tests but does not write
			zb = z;
	}
	if (BIT(m_w88, 1) || a < 1.0f)
	{
		// blend with the destination; 0x8D gives per-channel factors
		float dr, dg, db;
		bgr555_to_f(vram_r(x, y), dr, dg, db);
		const float kr = BIT(m_w88, 1) ? ((m_w8d & 0xff) / 255.0f) * a : a;
		const float kg = BIT(m_w88, 1) ? (((m_w8d >> 8) & 0xff) / 255.0f) * a : a;
		const float kb = BIT(m_w88, 1) ? (((m_w8d >> 16) & 0xff) / 255.0f) * a : a;
		r = r * kr + dr * (1.0f - kr);
		g = g * kg + dg * (1.0f - kg);
		b = b * kb + db * (1.0f - kb);
	}
	m_stat_pixels++;
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
	v.z = -z; // eye space looks down -Z; keep positive depth from here on

	// ambient light (0x70); directional lights are TODO
	float ar, ag, ab;
	bgr555_to_f(m_r[0x70], ar, ag, ab);
	v.r *= ar;
	v.g *= ag;
	v.b *= ab;
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

void zevio_gpu_device::draw_triangle(const vertex_t &a, const vertex_t &b, const vertex_t &c, bool textured)
{
	// screen-space edge functions over the bounding box, perspective-correct UVs
	const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	m_stat_tris++;
	if (std::fabs(area) < 1e-6f)
		return;
	const int cx0 = std::max<int>(m_clip0 & 0xfff, 0), cy0 = std::max<int>((m_clip0 >> 12) & 0xfff, 0);
	const int cx1 = std::min<int>(m_clip1 & 0xfff, 1023), cy1 = std::min<int>((m_clip1 >> 12) & 0xfff, ZH - 1);
	const int minx = std::max(cx0, int(std::floor(std::min({ a.x, b.x, c.x }))));
	const int maxx = std::min(cx1, int(std::ceil(std::max({ a.x, b.x, c.x }))));
	const int miny = std::max(cy0, int(std::floor(std::min({ a.y, b.y, c.y }))));
	const int maxy = std::min(cy1, int(std::ceil(std::max({ a.y, b.y, c.y }))));
	if (minx > maxx || miny > maxy)
		return;
	const bool depth = BIT(m_w84, 0) || true; // TODO: confirm the depth-test bit
	const float inv_area = 1.0f / area;
	for (int y = miny; y <= maxy; y++)
	{
		const float py = y + 0.5f;
		for (int x = minx; x <= maxx; x++)
		{
			const float px = x + 0.5f;
			float w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * inv_area;
			float w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * inv_area;
			float w2 = 1.0f - w0 - w1;
			if (w0 < 0 || w1 < 0 || w2 < 0)
				continue;
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
				if (!texel(u, v, true, tr, tg, tb, ta))
					continue;
				r *= tr;
				g *= tg;
				bl *= tb;
				alpha = ta;
			}
			plot(x, y, z, r, g, bl, alpha, depth);
		}
	}
}

void zevio_gpu_device::draw_3d_tri(vertex_t a, vertex_t b, vertex_t c)
{
	if (m_no3d) return;
	const bool textured = BIT(m_w1c, 6);
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
	if (n < 3)
		return;
	for (int i = 0; i < n; i++)
		project(out[i]);
	draw_triangle(out[0], out[1], out[2], textured);
	if (n == 4)
		draw_triangle(out[0], out[2], out[3], textured);
}

void zevio_gpu_device::draw_line(const vertex_t &a, const vertex_t &b)
{
	const int steps = std::max(1, int(std::max(std::fabs(b.x - a.x), std::fabs(b.y - a.y))));
	for (int i = 0; i <= steps; i++)
	{
		const float t = float(i) / steps;
		plot(int(a.x + (b.x - a.x) * t), int(a.y + (b.y - a.y) * t), 0.0f,
				a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, 1.0f, false);
	}
}

void zevio_gpu_device::prim3d(u32 header, u32 &addr)
{
	m_no3d = getenv("ZGPU_NO_3D") != nullptr;
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
	if ((VERBOSE & LOG_LIST) && n && m_stat_dbg < 40)
	{
		m_stat_dbg++;
		LOGMASKED(LOG_LIST, "prim %08x v0 eye (%f %f %f) uv %f %f col %f %f %f\n", header, verts[0].x, verts[0].y, verts[0].z, verts[0].u, verts[0].v, verts[0].r, verts[0].g, verts[0].b);
	}

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
				draw_line(a, b);
		}
		break;
	case 0x50: // points
		for (auto v : verts)
			if (project(v))
				plot(int(v.x), int(v.y), v.z, v.r, v.g, v.b, 1.0f, true);
		break;
	default:
		LOGMASKED(LOG_UNKNOWN, "unknown 3D primitive %08x\n", header);
		break;
	}
}

// 0x6F8: transform n inline vertices into RAM at the 0x0C address, same layout
void zevio_gpu_device::transform_to_memory(u32 header, u32 &addr)
{
	if (getenv("ZGPU_NO_XFORM")) { addr += (header & 0xffff) * vertex_words((header >> 16) & 0x3f) * 4; return; }
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
	const int ox = m_w85 & 0xfff, oy = (m_w85 >> 12) & 0xfff;
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
	auto tri2d = [this, textured] (const vertex_t &a, const vertex_t &b, const vertex_t &c)
	{
		// 2D: affine UVs in texels, no depth
		const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
		if (std::fabs(area) < 1e-6f)
			return;
		const int minx = int(std::floor(std::min({ a.x, b.x, c.x }))), maxx = int(std::ceil(std::max({ a.x, b.x, c.x })));
		const int miny = int(std::floor(std::min({ a.y, b.y, c.y }))), maxy = int(std::ceil(std::max({ a.y, b.y, c.y })));
		for (int y = std::max(miny, 0); y <= std::min(maxy, ZH - 1); y++)
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
					if (!texel(w0 * a.u + w1 * b.u + w2 * c.u, w0 * a.v + w1 * b.v + w2 * c.v, false, tr, tg, tb, alpha))
						continue;
					r *= tr; g *= tg; bl *= tb;
				}
				plot(x, y, 0.0f, r, g, bl, alpha, false);
			}
	};

	switch (op)
	{
	case 0xc0:
		for (int i = 0; i + 2 < n; i++)
			tri2d(v[i], v[i + 1], v[i + 2]);
		break;
	case 0xc1:
		tri2d(v[0], v[1], v[2]);
		break;
	case 0xc2:
		for (int i = 0; i + 1 < n; i++)
			draw_line(v[i], v[i + 1]);
		break;
	case 0xc3:
		draw_line(v[0], v[1]);
		break;
	case 0xc4:
		plot(int(v[0].x), int(v[0].y), 0.0f, v[0].r, v[0].g, v[0].b, 1.0f, false);
		break;
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

	const int ox = m_w85 & 0xfff, oy = (m_w85 >> 12) & 0xfff;
	const int x0 = ox + s16(xy & 0xffff), y0 = oy + s16(xy >> 16);
	const int w = wh & 0xffff, h = wh >> 16;
	const float cr = (col & 0xff) / 255.0f, cg = ((col >> 8) & 0xff) / 255.0f, cb = ((col >> 16) & 0xff) / 255.0f;
	for (int j = 0; j < h; j++)
		for (int i = 0; i < w; i++)
		{
			float r = cr, g = cg, b = cb, a = 1.0f;
			if (BIT(flags, 0))
			{
				float tr, tg, tb;
				if (!texel(float((uv & 0xffff) + i), float((uv >> 16) + j), false, tr, tg, tb, a))
					continue;
				r *= tr; g *= tg; b *= tb;
			}
			plot(x0 + i, y0 + j, 0.0f, r, g, b, a, false);
		}
}


/***************************************************************************
    Display list executor
***************************************************************************/

void zevio_gpu_device::run_list(u32 addr)
{
	LOGMASKED(LOG_LIST, "list %08x\n", addr);
	u32 ret = 0;
	bool in_call = false;
	for (int guard = 0; guard < 0x400000; guard++)
	{
		const u32 cmd = rd32(addr);
		addr += 4;
		const u8 op = cmd >> 24;
		switch (op)
		{
		case 0x04:
			if (in_call)
			{
				addr = ret;
				in_call = false;
			}
			break;
		case 0x08: addr = rd32(addr); break;
		case 0x09:
			ret = addr + 4;
			in_call = true;
			addr = rd32(addr);
			break;
		case 0x0a: m_vtxbase = rd32(addr); addr += 4; break;
		case 0x0c: m_xfer = rd32(addr); addr += 4; break;
		case 0x0e: addr += 4; break;
		case 0x10: m_regs[0x5c / 4] = rd32(addr); addr += 4; break;
		case 0x12: case 0x89: case 0x95: case 0x96: case 0x8e: break;
		case 0x14:
			m_regs[0x14 / 4] = (cmd & 0xff00) | 0x08;
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
			const int x = pos & 0xffff, y = pos >> 16, w = size & 0xffff, h = size >> 16;
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					vram_w(x + i, y + j, rd16(src + (j * w + i) * 2));
			break;
		}
		case 0xd1: // store image to the 0x0C address
		{
			const u32 pos = rd32(addr), size = rd32(addr + 4);
			addr += 8;
			const int x = pos & 0xffff, y = pos >> 16, w = size & 0xffff, h = size >> 16;
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
					wr16(m_xfer + (j * w + i) * 2, vram_r(x + i, y + j));
			break;
		}
		case 0xd3: // fill, relative to the +0x98 base (FA clears its depth area with it)
		{
			const u32 pos = rd32(addr), size = rd32(addr + 4), col = rd32(addr + 8);
			addr += 12;
			m_stat_fills++;
			const int x = pos & 0xffff, y = pos >> 16, w = size & 0xffff, h = size >> 16;
			const u32 base = m_regs[0x98 / 4];
			for (int j = 0; j < h; j++)
				for (int i = 0; i < w; i++)
				{
					const u32 xb = ((x + i) & 1023) * 2, yy = y + j;
					wr16(base + (yy >> 3) * 16384 + (xb >> 6) * 512 + (yy & 7) * 64 + (xb & 63), col);
					if (yy < ZH)
						m_zbuf[yy * ZW + ((x + i) & 1023)] = std::numeric_limits<float>::max();
				}
			break;
		}
		default:
			LOGMASKED(LOG_UNKNOWN, "unknown command %08x at %08x\n", cmd, addr - 4);
			break;
		}
	}
	LOGMASKED(LOG_UNKNOWN, "list runaway\n");
}
