// license:BSD-3-Clause
// copyright-holders:Riley
/***************************************************************************

    LSI Zevio 1020 sound block (0xB0000000)

    64 voices mixed to 44.1 kHz stereo, reading samples straight from SDRAM.
    Register knowledge comes from the V.Flash boot ROM and the games' FA.VFF
    sound driver; nothing here is from a datasheet.

    Per voice (base + v * 0x40):
    +0x00 start address, +0x04 end address (inclusive, last sample),
    +0x08 loop address, +0x0C mode, +0x10 pitch (13 bits, 0x1000 = 1:1),
    +0x14/+0x18 envelope (constant in practice), +0x1C (always ~0),
    +0x20 levels (byte 0 left, byte 1 right), +0x24 unknown,
    +0x28 current address (read), +0x34 status (bit 16 = playing)
    Mode: bit 0 = 8-bit PCM (unsigned), bit 1 = IMA ADPCM (Apple ima4 packets),
    bits 2-3 = rate divider (1, 2, 4, 8), bit 4 = loop; neither bit 0 nor 1
    = signed 16-bit PCM.

    Global (0xB0001000+):
    +0x004 IRQ: bits 0-5 status (write 1 to clear), bits 8-13 enables
    +0x008/+0x00C pause masks (voices 0-31 / 32-63)
    +0x010/+0x014 key on, +0x018/+0x01C key off (write-1 masks)
    +0x020 IRQ address: source 0 fires when a voice reaches this address
    +0x184 master volume ramp rate, +0x188 master volume (15 bits per side),
    +0x248 current master volume (read)

    TODO:
    - envelope fields, key-off release, levels bytes 2-3 (effect sends?)
    - reverb (never enabled by known software)
    - interrupt sources 1-5

***************************************************************************/

#include "emu.h"
#include "zevio_spu.h"

#define VERBOSE 0
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ZEVIO_SPU, zevio_spu_device, "zevio_spu", "Zevio 1020 sound")

namespace {

const s16 ima_steps[89] =
{
	7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
	50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
	253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
	1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327,
	3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
	11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
	32767
};

const int ima_index_adjust[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

enum
{
	V_START = 0x00 / 4,
	V_END = 0x04 / 4,
	V_LOOP = 0x08 / 4,
	V_MODE = 0x0c / 4,
	V_PITCH = 0x10 / 4,
	V_LEVELS = 0x20 / 4,
	V_CURADDR = 0x28 / 4,
	V_STATUS = 0x34 / 4
};

} // anonymous namespace


zevio_spu_device::zevio_spu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, ZEVIO_SPU, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	m_irq_cb(*this),
	m_stream(nullptr),
	m_ram(nullptr),
	m_ram_base(0),
	m_ram_size(0)
{
}

void zevio_spu_device::device_start()
{
	m_stream = stream_alloc(0, 2, clock());

	for (int v = 0; v < 64; v++)
	{
		save_item(NAME(m_voice[v].regs), v);
		save_item(NAME(m_voice[v].active), v);
		save_item(NAME(m_voice[v].addr), v);
		save_item(NAME(m_voice[v].phase), v);
		save_item(NAME(m_voice[v].sample), v);
		save_item(NAME(m_voice[v].packet), v);
		save_item(NAME(m_voice[v].nibble), v);
		save_item(NAME(m_voice[v].pred), v);
		save_item(NAME(m_voice[v].index), v);
	}
	save_item(NAME(m_global));
	save_item(NAME(m_pause));
	save_item(NAME(m_irq_status));
}

void zevio_spu_device::device_reset()
{
	for (auto &vc : m_voice)
	{
		std::fill(std::begin(vc.regs), std::end(vc.regs), 0);
		vc.active = false;
		vc.addr = vc.phase = 0;
		vc.sample = 0;
		vc.packet = 0;
		vc.nibble = 0;
		vc.pred = 0;
		vc.index = 0;
	}
	std::fill(std::begin(m_global), std::end(m_global), 0);
	m_pause = 0;
	m_irq_status = 0;
	update_irq();
}

u8 zevio_spu_device::ram_r8(u32 addr) const
{
	const u32 off = (addr & 0x7fffffff) - m_ram_base;
	if (!m_ram || off >= m_ram_size)
		return 0;
	return m_ram[off >> 2] >> ((off & 3) * 8);
}

void zevio_spu_device::update_irq()
{
	const u32 enables = (m_global[0x004 / 4] >> 8) & 0x3f;
	m_irq_cb((m_irq_status & enables) ? ASSERT_LINE : CLEAR_LINE);
}

void zevio_spu_device::ima_start_packet(voice_t &vc, u32 addr)
{
	vc.packet = addr;
	const u16 header = (ram_r8(addr) << 8) | ram_r8(addr + 1); // big-endian
	vc.pred = s16(header & 0xff80);
	vc.index = std::min(header & 0x7f, 88);
	vc.nibble = 0;
	vc.addr = addr + 2;
}

void zevio_spu_device::ima_decode_nibble(voice_t &vc)
{
	const u8 byte = ram_r8(vc.packet + 2 + (vc.nibble >> 1));
	const int n = BIT(vc.nibble, 0) ? (byte >> 4) : (byte & 0x0f);
	const int step = ima_steps[vc.index];
	int diff = step >> 3;
	if (n & 1) diff += step >> 2;
	if (n & 2) diff += step >> 1;
	if (n & 4) diff += step;
	vc.pred = std::clamp<s32>((n & 8) ? vc.pred - diff : vc.pred + diff, -32768, 32767);
	vc.index = std::clamp(vc.index + ima_index_adjust[n & 7], 0, 88);
	vc.sample = vc.pred;
	vc.addr = vc.packet + 2 + (vc.nibble >> 1);
}

void zevio_spu_device::voice_fetch(voice_t &vc)
{
	const u32 mode = vc.regs[V_MODE];
	if (BIT(mode, 1))
		return; // IMA samples are produced by ima_decode_nibble
	if (BIT(mode, 0))
		vc.sample = (s32(ram_r8(vc.addr)) - 0x80) << 8;
	else
		vc.sample = s16(ram_r16(vc.addr));
}

void zevio_spu_device::key_on(int v)
{
	voice_t &vc = m_voice[v];
	vc.active = true;
	vc.phase = 0;
	const u32 start = vc.regs[V_START];
	if (BIT(vc.regs[V_MODE], 1))
	{
		ima_start_packet(vc, start);
		ima_decode_nibble(vc);
	}
	else
	{
		vc.addr = start;
		if (start > vc.regs[V_END])
			vc.active = false;
		else
			voice_fetch(vc);
	}
}

// step one source sample; returns false when the voice stopped
bool zevio_spu_device::voice_advance(voice_t &vc)
{
	const u32 mode = vc.regs[V_MODE];
	const u32 end = vc.regs[V_END];
	const u32 old = vc.addr;
	if (BIT(mode, 1))
	{
		if (++vc.nibble >= 64)
		{
			const u32 next = vc.packet + 34;
			if (next > end)
			{
				if (!BIT(mode, 4))
				{
					vc.active = false;
					return false;
				}
				ima_start_packet(vc, vc.regs[V_LOOP]);
			}
			else
				ima_start_packet(vc, next);
		}
		ima_decode_nibble(vc);
	}
	else
	{
		u32 next = vc.addr + (BIT(mode, 0) ? 1 : 2);
		if (next > end)
		{
			if (!BIT(mode, 4))
			{
				vc.active = false;
				return false;
			}
			next = vc.regs[V_LOOP];
		}
		vc.addr = next;
		voice_fetch(vc);
	}

	// interrupt source 0: a voice reached the address in +0x020
	const u32 target = m_global[0x020 / 4];
	if (target && (vc.addr == target || (old < target && vc.addr > target)))
		m_irq_status |= 1;
	return true;
}

void zevio_spu_device::sound_stream_update(sound_stream &stream)
{
	const u32 master = m_global[0x188 / 4];
	const s32 master_l = master & 0x7fff, master_r = (master >> 16) & 0x7fff;
	const u32 irq_before = m_irq_status;

	for (int s = 0; s < stream.samples(); s++)
	{
		s32 left = 0, right = 0;
		for (int v = 0; v < 64; v++)
		{
			voice_t &vc = m_voice[v];
			if (!vc.active || BIT(m_pause, v))
				continue;
			const u32 mode = vc.regs[V_MODE];
			const u32 step = 0x1000U << ((mode >> 2) & 3);
			vc.phase += vc.regs[V_PITCH] & 0x1fff;
			bool alive = true;
			while (vc.phase >= step && alive)
			{
				vc.phase -= step;
				alive = voice_advance(vc);
			}
			if (!alive)
				continue;
			const u32 levels = vc.regs[V_LEVELS];
			left += (vc.sample * s32(levels & 0xff)) >> 8;
			right += (vc.sample * s32((levels >> 8) & 0xff)) >> 8;
		}
		left = (left * master_l) >> 15;
		right = (right * master_r) >> 15;
		stream.put_int_clamp(0, s, left, 32768);
		stream.put_int_clamp(1, s, right, 32768);
	}

	if (m_irq_status != irq_before)
		update_irq();
}

u32 zevio_spu_device::read(offs_t offset)
{
	m_stream->update();
	if (offset < 0x1000 / 4)
	{
		const voice_t &vc = m_voice[offset >> 4];
		switch (offset & 15)
		{
		case V_CURADDR: return vc.addr;
		case V_STATUS: return vc.active ? 0x10000 : 0;
		default: return vc.regs[offset & 15];
		}
	}
	const offs_t g = offset - 0x1000 / 4;
	switch (g)
	{
	case 0x004 / 4: return (m_global[g] & ~0x3f) | m_irq_status;
	case 0x248 / 4: return m_global[0x188 / 4]; // master volume ramps instantly
	default: return m_global[g];
	}
}

void zevio_spu_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	m_stream->update();
	if (offset < 0x1000 / 4)
	{
		voice_t &vc = m_voice[offset >> 4];
		COMBINE_DATA(&vc.regs[offset & 15]);
		return;
	}
	const offs_t g = offset - 0x1000 / 4;
	switch (g)
	{
	case 0x004 / 4:
		m_irq_status &= ~(data & 0x3f);
		COMBINE_DATA(&m_global[g]);
		update_irq();
		return;
	case 0x008 / 4: m_pause = (m_pause & 0xffffffff00000000ULL) | data; break;
	case 0x00c / 4: m_pause = (m_pause & 0x00000000ffffffffULL) | (u64(data) << 32); break;
	case 0x010 / 4: case 0x014 / 4:
		for (int i = 0; i < 32; i++)
			if (BIT(data, i))
				key_on(i + (g == 0x014 / 4 ? 32 : 0));
		break;
	case 0x018 / 4: case 0x01c / 4:
		for (int i = 0; i < 32; i++)
			if (BIT(data, i))
				m_voice[i + (g == 0x01c / 4 ? 32 : 0)].active = false;
		break;
	default:
		break;
	}
	COMBINE_DATA(&m_global[g]);
}
