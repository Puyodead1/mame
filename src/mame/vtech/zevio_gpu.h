// license:BSD-3-Clause
// copyright-holders:Riley
#ifndef MAME_VTECH_ZEVIO_GPU_H
#define MAME_VTECH_ZEVIO_GPU_H

#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

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
	virtual void device_stop() override ATTR_COLD;
	virtual void device_pre_save() override;

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
	// GPU rows y >= HIWORD(+0x94) are RAM at +0x98 + tiled(x, y - HIWORD(+0x94));
	// rows above are a separate area that the CPU cannot address (see zevio_gpu.cpp)
	static u32 tiled_offs(int x, int y)
	{
		const u32 xb = (x & 1023) * 2;
		return (y >> 3) * 16384 + (xb >> 6) * 512 + (y & 7) * 64 + (xb & 63);
	}
	bool vram_in_aux(int y) const { return y < m_vram_y0; }
	u32 vram_addr(int x, int y) const { return m_vram_base + tiled_offs(x, y - m_vram_y0); }
	u16 &aux_px(int x, int y) const { return m_aux[(y & (AUX_H - 1)) * 1024 + (x & 1023)]; }
	u16 vram_r(int x, int y) const
	{
		if (vram_in_aux(y))
			return aux_px(x, y);
		const u32 off = vram_addr(x, y) - m_ram_base;
		if (off >= m_ram_size)
			return 0;
#ifdef LSB_FIRST
		return reinterpret_cast<const u16 *>(m_ram)[off >> 1];
#else
		return m_ram[off >> 2] >> (BIT(off, 1) * 16);
#endif
	}
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
	void draw_3d_tri(vertex_t a, vertex_t b, vertex_t c);

	// 2D
	void prim2d(u32 header, u32 &addr);
	void sprite(u32 header, u32 &addr);
	void log_texture();

	float reg_f(int r) const { return f16_to_float(m_r[r & 0xff]); }

	/*
	    Rendering. List parsing, transforms and FA's queue bookkeeping run on
	    the emulation thread; drawing is recorded as commands with a snapshot
	    of the render state and executed in order on a render thread, split
	    into horizontal bands on worker threads (bands touch disjoint rows of
	    VRAM and the depth buffer, so the result does not depend on timing).
	    sync() waits for it: before VRAM is read or copied, on screen update
	    and on save.
	*/
public:
	void sync();
	// once per frame: wait for the work queued before the previous fence (or for
	// everything if newer work draws into the given area); false if everything was finished (or rendering is synchronous)
	bool frame_fence(int x0, int y0, int x1, int y1);
private:
	struct rstate
	{
		u32 w84, w86, w87, w88, w8d, clip0, clip1, clut0, clut3;
	};
	struct rcache // per band
	{
		u16 pal[256];
		u32 key = ~0U;
	};
	enum : u8 { CMD_TRI3D, CMD_TRI2D, CMD_LINE, CMD_POINT, CMD_SPRITE, CMD_FILL, CMD_ZCLEAR };
	struct rcmd
	{
		u8 type;
		bool textured;
		rstate st;
		int ymin, ymax;        // rows touched (for band culling)
		vertex_t v[3];
		u32 a0, a1, a2, a3;    // sprite: x0/y0, w/h, uv, col | fill: base, size, col
		u8 flags;
	};
	rstate cur_state() const;
	void add_cmd(rcmd &&c);
	void submit();
	void exec_batch(const std::vector<rcmd> &cmds);
	void exec_band(const std::vector<rcmd> &cmds, int y0, int y1);
	void render_thread();

	bool in_clip(const rstate &s, int x, int y) const;
	bool texel(const rstate &s, rcache &pc, float u, float v, bool normalized, float &r, float &g, float &b, float &a) const;
	bool texel_point(const rstate &s, rcache &pc, int tu, int tv, float &r, float &g, float &b, float &a) const;
	int texel_raw(const rstate &s, rcache &pc, int tu, int tv) const;
	void plot(const rstate &s, int x, int y, float z, float r, float g, float b, float a, bool depth);
	void draw_triangle(const rstate &s, rcache &pc, const vertex_t &a, const vertex_t &b, const vertex_t &c, bool textured, int y0, int y1);
	void draw_tri2d(const rstate &s, rcache &pc, const vertex_t &a, const vertex_t &b, const vertex_t &c, bool textured, int y0, int y1);
	void draw_line(const rstate &s, const vertex_t &a, const vertex_t &b, int y0, int y1);
	void draw_sprite(const rstate &s, rcache &pc, const rcmd &c, int y0, int y1);
	void draw_fill(const rcmd &c, int y0, int y1);
	void draw_zclear(const rcmd &c, int y0, int y1);
	int m_zdirty_y0 = 0, m_zdirty_y1 = -1; // rows written with depth since the last clear

	static constexpr int MAX_BANDS = 48;
	struct band_job { zevio_gpu_device *gpu; const std::vector<rcmd> *cmds; int y0, y1; std::atomic<int> *left; };
	static void *band_cb(void *param, int threadid);
	std::vector<rcmd> m_cmds;              // being recorded
	osd_work_queue *m_queue = nullptr;
	int m_bands = 1;
	bool m_count_stats = false;
	bool m_async = true;
	std::thread m_thread;
	std::mutex m_mutex;
	std::condition_variable m_cv;
	std::deque<std::vector<rcmd>> m_jobs;
	bool m_busy = false, m_quit = false;

	// drawing time: a CPU that kicks large batches waits as if the engine
	// were drawing, so loops that redraw whenever it is done run at a real rate
	device_execute_interface *m_stall = nullptr;
	u64 m_work_px = 0; // pixels queued, roughly
	double m_px_debt = 0, m_px_time = 0;
public:
	void set_stall_cpu(device_execute_interface *cpu) { m_stall = cpu; }
private:
	u64 m_seq_submitted = 0, m_seq_done = 0, m_fence = 0; // job counts, under m_mutex
	std::vector<std::array<int, 4>> m_drawn; // areas drawn since the last fence
	std::atomic<u32> m_pal_gen{ 0 };       // bumped when a palette may have changed

	devcb_write_line m_irq_cb;
	u32 *m_ram;
	u32 m_ram_base;
	u32 m_ram_size;

	// +0x94/+0x98 coordinate mapping, latched (after a sync) when written
	static constexpr int AUX_H = 4096;
	int m_vram_y0 = 0;
	u32 m_vram_base = 0;
	std::unique_ptr<u16[]> m_aux;    // GPU rows below HIWORD(+0x94), 1024 x AUX_H
	void vram_map_update();
	int abs_row(int y) const;        // GPU row -> row of the RAM surface at 0x0FFD0000

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

	// FA display list queue HLE (see fa_queue_hle)
	bool m_defer_irq = false;
	bool m_hle_handled = false; // the last kick's lists completed through fa_queue_hle
	u32 m_fa_handler = 0;
	double m_fa_next_scan = 0;
	u32 fa_find_irq_handler();
	bool fa_queue_hle();
	int m_stat_tris = 0, m_stat_pixels = 0, m_stat_fills = 0, m_stat_dbg = 0, m_stat_dbg2 = 0, m_total_tris = 0, m_total_pixels = 0;
};

DECLARE_DEVICE_TYPE(ZEVIO_GPU, zevio_gpu_device)

#endif // MAME_VTECH_ZEVIO_GPU_H
