// license:BSD-3-Clause
// copyright-holders:Riley
#ifndef MAME_VTECH_ZEVIO_DSP_H
#define MAME_VTECH_ZEVIO_DSP_H

#pragma once

// High-level emulation of the Zevio 1020's ZSP400 media DSP as driven by
// the V.Flash software: the MJP movie program (and, TODO, the JPEG program)
class zevio_dsp_hle_device : public device_t
{
public:
	zevio_dsp_hle_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto msg_irq_cb() { return m_msg_irq_cb.bind(); }     // IRQ 17: DSP message (0x104)
	auto reply_irq_cb() { return m_reply_irq_cb.bind(); } // IRQ 18: DSP reply (0x110)

	void set_ram(u32 *base, u32 phys_base, u32 size) { m_ram = base; m_ram_base = phys_base; m_ram_size = size; }

	void program_loaded(u32 words) { m_prog_words = words; }
	void run_w(u32 data);        // 0x900A0020
	void windows_w(u32 data);    // 0x900A0024
	u32 mbox_r(offs_t offset);   // 0x900A0100-0x900A011C
	void mbox_w(offs_t offset, u32 data, u32 mem_mask = ~0);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	enum : u32 { MSG = 0x104 / 4, DOORBELL = 0x108 / 4, REPLY = 0x110 / 4, STATUS = 0x114 / 4 };

	u32 rd32(u32 addr) const;
	u8 rd8(u32 addr) const { return rd32(addr & ~3) >> ((addr & 3) * 8); }
	void wr32(u32 addr, u32 data);
	void wr8(u32 addr, u8 data);

	void post_reply();
	void post_message();
	void command(u32 cmd);
	TIMER_CALLBACK_MEMBER(frame_cb);
	bool decode_frame();

	// MJP stream access in the CD ring (wraps at m_ring_end)
	u32 ring_wrap(u32 addr) const { return (addr >= m_ring_end) ? addr - (m_ring_end - m_ring_start) : addr; }
	u8 ring8(u32 addr) const { return rd8(ring_wrap(addr)); }
	u32 ring32(u32 addr) const;
	bool decode_jpeg(const std::vector<u8> &jpeg, u32 ybase, u32 ubase, u32 vbase);

	devcb_write_line m_msg_irq_cb;
	devcb_write_line m_reply_irq_cb;
	u32 *m_ram;
	u32 m_ram_base, m_ram_size;

	u32 m_mbox[8];
	u32 m_prog_words;
	bool m_running;
	u32 m_w;              // shared-memory base
	emu_timer *m_frame_timer;

	// MJP state
	bool m_playing, m_paused, m_awaiting_ack;
	u32 m_ring_start, m_ring_end, m_rd;
	u32 m_out[3];         // Y, U, V
	u32 m_aud[2], m_aud_wp[2], m_aud_size;
	u32 m_width, m_height, m_frames, m_frame;
	std::vector<u8> m_jpeg_header;
};

DECLARE_DEVICE_TYPE(ZEVIO_DSP_HLE, zevio_dsp_hle_device)

#endif // MAME_VTECH_ZEVIO_DSP_H
