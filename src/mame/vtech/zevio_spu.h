// license:BSD-3-Clause
// copyright-holders:Riley
#ifndef MAME_VTECH_ZEVIO_SPU_H
#define MAME_VTECH_ZEVIO_SPU_H

#pragma once

class zevio_spu_device : public device_t, public device_sound_interface
{
public:
	zevio_spu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 44100);

	auto irq_cb() { return m_irq_cb.bind(); }

	// sample memory is SDRAM, addressed by absolute CPU addresses
	void set_ram(const u32 *base, u32 phys_base, u32 size) { m_ram = base; m_ram_base = phys_base; m_ram_size = size; }

	u32 read(offs_t offset);
	void write(offs_t offset, u32 data, u32 mem_mask = ~0);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override;
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	struct voice_t
	{
		u32 regs[16];
		bool active;
		u32 addr;
		u32 phase;
		s32 sample;     // current decoded sample
		s32 prev;       // the one before it (output interpolates between them)
		u32 release, release_len;   // key-off fade: samples left / total (0 = not releasing)
		// IMA ADPCM (Apple ima4 packets of 34 bytes)
		u32 packet;     // address of the current packet
		int nibble;     // 0-63 within the packet
		s32 pred;
		int index;
	};

	u8 ram_r8(u32 addr) const;
	u16 ram_r16(u32 addr) const { return ram_r8(addr) | (ram_r8(addr + 1) << 8); }
	void key_on(int v);
	void voice_fetch(voice_t &vc);
	bool voice_advance(voice_t &vc);
	void ima_start_packet(voice_t &vc, u32 addr);
	void ima_decode_nibble(voice_t &vc);
	void update_irq();
	void arm_timer();
	TIMER_CALLBACK_MEMBER(timer_tick);

	devcb_write_line m_irq_cb;
	emu_timer *m_timer;
	sound_stream *m_stream;
	const u32 *m_ram;
	u32 m_ram_base;
	u32 m_ram_size;

	voice_t m_voice[64];
	u32 m_global[0x1000 / 4];   // registers 0xB0001000-0xB0001FFF
	u64 m_pause;
	u32 m_irq_status;           // 0xB0001004 bits 0-5
};

DECLARE_DEVICE_TYPE(ZEVIO_SPU, zevio_spu_device)

#endif // MAME_VTECH_ZEVIO_SPU_H
