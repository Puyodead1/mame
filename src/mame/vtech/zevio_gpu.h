// license:BSD-3-Clause
// copyright-holders:Riley
#ifndef MAME_VTECH_ZEVIO_GPU_H
#define MAME_VTECH_ZEVIO_GPU_H

#pragma once

class zevio_gpu_device : public device_t
{
public:
	zevio_gpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto irq_cb() { return m_irq_cb.bind(); }

	// display lists, vertices, textures and VRAM all live in SDRAM
	void set_ram(u32 *base, u32 phys_base, u32 size) { m_ram = base; m_ram_base = phys_base; m_ram_size = size; }

	u32 regs_r(offs_t offset);                       // 0xA8000000
	void regs_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	u32 cvt_r(offs_t offset);                        // 0xA4000000 number converters
	void cvt_w(offs_t offset, u32 data, u32 mem_mask = ~0);

	static float f16_to_float(u16 v);
	static u16 float_to_f16(float f);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	struct vertex_t
	{
		float x, y, z;       // object/eye space, or screen space after projection
		float nx, ny, nz;
		float u, v;
		float r, g, b;       // 0-1
		float w;             // 1/z for perspective correction
		u16 hi;
		bool lit;            // has a normal: apply lighting
	};

	// memory
	u32 rd32(u32 addr) const;
	u16 rd16(u32 addr) const { return rd32(addr & ~3) >> (BIT(addr, 1) * 16); }
	u8 rd8(u32 addr) const { return rd32(addr & ~3) >> ((addr & 3) * 8); }
	void wr32(u32 addr, u32 data);
	void wr16(u32 addr, u16 data);
	u32 vram_addr(int x, int y) const;
	u16 vram_r(int x, int y) const { return rd16(vram_addr(x, y)); }
	void vram_w(int x, int y, u16 data);

	void run_list(u32 addr);
	void update_irq();

	// 3D
	int vertex_words(u8 fmt) const;
	vertex_t fetch_vertex(u32 addr, u8 fmt) const;
	void transform(vertex_t &v) const;
	bool project(vertex_t &v) const;
	void prim3d(u32 header, u32 &addr);
	void transform_to_memory(u32 header, u32 &addr);
	void draw_triangle(const vertex_t &a, const vertex_t &b, const vertex_t &c, bool textured);
	void draw_line(const vertex_t &a, const vertex_t &b);
	void draw_3d_tri(vertex_t a, vertex_t b, vertex_t c);

	// 2D
	void prim2d(u32 header, u32 &addr);
	void sprite(u32 header, u32 &addr);

	// pixels
	bool texel(float u, float v, bool normalized, float &r, float &g, float &b, float &a) const;
	void plot(int x, int y, float z, float r, float g, float b, float a, bool depth);
	bool in_clip(int x, int y) const;

	float reg_f(int r) const { return f16_to_float(m_r[r & 0xff]); }

	devcb_write_line m_irq_cb;
	u32 *m_ram;
	u32 m_ram_base;
	u32 m_ram_size;

	u32 m_regs[0x400 / 4];
	u32 m_cvt[0x40 / 4];

	// state
	u16 m_r[256];          // 0x20 register block values (mostly f16)
	u32 m_w1c, m_w1e, m_w84, m_w88, m_w81, m_w85, m_w86, m_w87, m_w8d;
	u32 m_clip0, m_clip1;
	u32 m_clut[6];         // 0x8F-0x94
	u32 m_vtxbase, m_xfer;

	std::unique_ptr<float[]> m_zbuf; // 1024 x 1024, GPU coordinates
	bool m_no3d = false;
	int m_stat_tris = 0, m_stat_pixels = 0, m_stat_fills = 0, m_stat_dbg = 0, m_stat_dbg2 = 0, m_total_tris = 0, m_total_pixels = 0;
};

DECLARE_DEVICE_TYPE(ZEVIO_GPU, zevio_gpu_device)

#endif // MAME_VTECH_ZEVIO_GPU_H
