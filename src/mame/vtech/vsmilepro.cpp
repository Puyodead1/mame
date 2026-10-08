// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***************************************************************************

    VTech V.Flash (USA) / V.Smile Pro (Europe) CD system

    Hardware:
    - LSI Zevio 1020 SoC: ARM926EJ-S CPU, ZSP400 DSP, "Hanabi" 3D/2D GPU,
      display controller with TV encoder, 64-voice sound, DMA engines,
      CD sector decoder, SPI, two 16550-style UARTs (controller ports)
    - Sony CXD3059AR CD servo/DSP
    - 2 MB AMIC A29L160ATV NOR boot ROM
    - 16 MB SDRAM at 0x10000000
    - SST25VF512/010 or PMC Pm25LV512/010 SPI serial flash (saves, settings)
    - Optional AT45DB642D DataFlash memory card on the same SPI bus

    Register knowledge comes from disassembly of the boot ROM and of the games'
    FA.VFF operating system. Nothing here is from a datasheet.

    Memory map:
    0x00000000  NOR boot ROM
    0x10000000  SDRAM (16 MB). The GPU and display see it as a tiled surface
                1024 pixels (2048 bytes) wide, origin 0x0FFD0000, in 32x8 tiles
    0x8FFF0000  SDRAM controller
    0x90000000  GPIO bank 0                0x90020000  UART0 (controller 1, debug)
    0x90030000  UART1 (controller 2)       0x90080000  key/unlock registers
    0x90090000  RTC / always-on block      0x900A0000  system control
    0x900B0000  PLL                        0x900C0000  timers
    0x900D0000  GPIO bank 1                0xA0000000  external (factory) bus
    0xA1000000  SPI controller             0xA4000000  GPU number converters
    0xA8000000  GPU command processor      0xB0000000  sound
    0xB8000000  display / TV encoder / scaler, on-chip SRAM at +0x800
    0xBC000000  memory DMA                 0xC0000000  descriptor DMA
    0xC4000000  CD sector decoder / servo serial link
    0xDC000000  interrupt controller       0xDC020000  DSP memory window

    TODO:
    - everything marked TODO below; GPU 3D pipeline, sound output, CD servo,
      ZSP400 DSP (JPEG / MJP video)

****************************************************************************/

#include "emu.h"

#include "zevio_spu.h"

#include "cpu/arm7/arm7.h"
#include "imagedev/cdromimg.h"
#include "machine/nvram.h"

#include "cdrom.h"
#include "screen.h"
#include "softlist.h"
#include "speaker.h"

#define LOG_UNKNOWN (1U << 1)
#define LOG_INTC    (1U << 2)
#define LOG_GPU     (1U << 3)
#define LOG_VIDEO   (1U << 4)
#define LOG_SPI     (1U << 5)
#define LOG_CD      (1U << 6)
#define LOG_DMA     (1U << 7)
#define LOG_SOUND   (1U << 8)
#define LOG_UART    (1U << 9)
#define LOG_SYS     (1U << 10)
#define LOG_DEBUGTX (1U << 11)

#define VERBOSE (LOG_UNKNOWN | LOG_DEBUGTX | LOG_CD)
#include "logmacro.h"

#include <deque>


namespace {

class vsmilpro_state : public driver_device
{
public:
	vsmilpro_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_cdrom(*this, "cdrom"),
		m_nvram(*this, "spiflash"),
		m_spu(*this, "spu"),
		m_sdram(*this, "sdram"),
		m_spi_region(*this, "spi"),
		m_io_keys(*this, "KEYS"),
		m_io_straps(*this, "STRAPS"),
		m_io_joy_buttons(*this, "JOY%u_BUTTONS", 1U),
		m_io_joy_stick(*this, "JOY%u_STICK", 1U)
	{ }

	void vsmilpro(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	static constexpr u32 SDRAM_BASE = 0x10000000;
	static constexpr u32 SDRAM_SIZE = 0x01000000;
	static constexpr u32 VRAM_ORIGIN = 0x0ffd0000;
	static constexpr u32 PERIPH_CLOCK = 37'500'000; // PLL out3 at boot settings
	static constexpr u32 SFLASH_SIZE = 0x10000;

	// interrupt sources
	enum
	{
		IRQ_UART0 = 1,
		IRQ_UART1 = 2,
		IRQ_RTC = 4,
		IRQ_TICK = 5,
		IRQ_TIMER = 6,
		IRQ_GPU = 8,
		IRQ_SOUND = 9,
		IRQ_BCDMA = 12,
		IRQ_CD = 14,
		IRQ_MBOX0 = 17,
		IRQ_MBOX1 = 18,
		IRQ_VIDEO = 21
	};

	struct uart_t
	{
		u8 ier, lcr, mcr, scr, dll, dlm, fcr;
		bool thre_pending;
		std::deque<u8> rx;
		std::string line;
	};

	required_device<arm9_cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<cdrom_image_device> m_cdrom;
	required_device<nvram_device> m_nvram;
	required_device<zevio_spu_device> m_spu;
	required_shared_ptr<u32> m_sdram;
	optional_memory_region m_spi_region;
	required_ioport m_io_keys;
	required_ioport m_io_straps;
	required_ioport_array<2> m_io_joy_buttons;
	required_ioport_array<2> m_io_joy_stick;

	// memory helpers (SDRAM is the only memory the bus masters use)
	bool in_sdram(u32 addr) const { return addr - SDRAM_BASE < SDRAM_SIZE; }
	u8 ram_r8(u32 addr) { return in_sdram(addr) ? u8(m_sdram[(addr - SDRAM_BASE) >> 2] >> ((addr & 3) * 8)) : m_maincpu->space(AS_PROGRAM).read_byte(addr); }
	u16 ram_r16(u32 addr) { return in_sdram(addr) ? m_sdram[(addr - SDRAM_BASE) >> 2] >> (BIT(addr, 1) * 16) : m_maincpu->space(AS_PROGRAM).read_word(addr); }
	u32 ram_r32(u32 addr) { return in_sdram(addr) ? m_sdram[(addr - SDRAM_BASE) >> 2] : m_maincpu->space(AS_PROGRAM).read_dword(addr); }
	void ram_w8(u32 addr, u8 data)
	{
		if (in_sdram(addr))
		{
			const int shift = (addr & 3) * 8;
			u32 &w = m_sdram[(addr - SDRAM_BASE) >> 2];
			w = (w & ~(0xffU << shift)) | (u32(data) << shift);
		}
		else
			m_maincpu->space(AS_PROGRAM).write_byte(addr, data);
	}
	void ram_w16(u32 addr, u16 data)
	{
		if (in_sdram(addr))
		{
			const int shift = BIT(addr, 1) * 16;
			u32 &w = m_sdram[(addr - SDRAM_BASE) >> 2];
			w = (w & ~(0xffffU << shift)) | (u32(data) << shift);
		}
		else
			m_maincpu->space(AS_PROGRAM).write_word(addr, data);
	}
	void ram_w32(u32 addr, u32 data) { if (in_sdram(addr)) m_sdram[(addr - SDRAM_BASE) >> 2] = data; else m_maincpu->space(AS_PROGRAM).write_dword(addr, data); }

	// tiled surface: byte column xb (0-2047), row y, relative to base
	static u32 tiled_addr(u32 base, u32 xb, u32 y)
	{
		return base + (y >> 3) * 16384 + ((xb >> 6) & 31) * 512 + (y & 7) * 64 + (xb & 63);
	}

	// interrupt controller
	u32 m_int_raw;      // level of each source
	u32 m_int_pending;  // currently the same as m_int_raw
	u32 m_int_enable[2];
	u32 m_int_regs[0x400 / 4];
	void set_irq(int source, int state);
	void update_cpu_irqs();
	u32 intc_r(offs_t offset);
	void intc_w(offs_t offset, u32 data, u32 mem_mask);

	// system control / PLL / RTC / GPIO
	u32 m_sysctl[0x1000 / 4];
	u32 m_pll[0x40 / 4];
	u32 m_rtc[0x100 / 4];
	u32 m_gpio_dir[2][4], m_gpio_out[2][4];
	u32 sysctl_r(offs_t offset);
	void sysctl_w(offs_t offset, u32 data, u32 mem_mask);
	u32 pll_r(offs_t offset);
	void pll_w(offs_t offset, u32 data, u32 mem_mask);
	u32 rtc_r(offs_t offset);
	void rtc_w(offs_t offset, u32 data, u32 mem_mask);
	template <int Bank> u32 gpio_r(offs_t offset);
	template <int Bank> void gpio_w(offs_t offset, u32 data, u32 mem_mask);
	void update_mbox_irqs();

	// timers
	u32 m_timer_regs[0x40 / 4];
	emu_timer *m_tick_timer;
	attotime m_ch1_start;
	u32 m_ch1_period_ticks;  // input clocks per count
	bool m_ch1_running;
	emu_timer *m_ch1_wrap_timer;
	u32 timer_r(offs_t offset);
	void timer_w(offs_t offset, u32 data, u32 mem_mask);
	void update_timer_irq();
	TIMER_CALLBACK_MEMBER(tick_cb);
	TIMER_CALLBACK_MEMBER(ch1_wrap_cb);
	u16 ch1_count();

	// UARTs
	uart_t m_uart[2];

	// controllers (HLE of the serial protocol)
	struct joy_t
	{
		std::vector<u8> cmd;
		bool detected;
		u8 last[5];
		attotime last_sent;
	};
	joy_t m_joy[2];
	emu_timer *m_joy_timer;
	void joy_from_console(int port, u8 data);
	void joy_send(int port, const u8 *payload, int len);
	void joy_build_report(int port, u8 *raw);
	TIMER_CALLBACK_MEMBER(joy_cb);
	template <int N> u32 uart_r(offs_t offset);
	template <int N> void uart_w(offs_t offset, u32 data, u32 mem_mask);
	void uart_update_irq(int n);
	u8 uart_iir(int n);

	// SPI and serial flash
	u32 m_spi_regs[0x40 / 4];
	std::unique_ptr<u8[]> m_sflash;
	u8 m_sflash_status;
	u32 m_sflash_aai_addr;
	u32 spi_r(offs_t offset);
	void spi_w(offs_t offset, u32 data, u32 mem_mask);
	void spi_transfer(int nbytes);
	u8 sflash_xfer(int index, u8 data, std::vector<u8> &cmd);
	u8 dataflash_xfer(int index, u8 data, std::vector<u8> &cmd);

	// GPU
	u32 m_gpu_regs[0x400 / 4];
	u32 m_gpu_clip[2];
	u32 m_gpu_offset;
	u32 m_gpu_texmode, m_gpu_texpos, m_gpu_clutpos;
	u32 m_gpu_xfer_addr;
	u32 m_gpu_state[0x100];
	u32 gpu_r(offs_t offset);
	void gpu_w(offs_t offset, u32 data, u32 mem_mask);
	void gpu_run_list(u32 addr);
	void gpu_update_irq();
	u16 gpu_vram_r16(u32 x, u32 y) { return ram_r16(tiled_addr(m_gpu_regs[0x98 / 4], x * 2, y)); }
	void gpu_vram_w16(u32 x, u32 y, u16 data) { ram_w16(tiled_addr(m_gpu_regs[0x98 / 4], x * 2, y), data); }
	void gpu_plot(int x, int y, u16 color);
	u16 gpu_texel(u32 u, u32 v, bool &transparent);
	void gpu_sprite(u32 flags, const u32 *args, int nargs);
	u32 m_gpu_cvt[0x40 / 4];
	u32 gpucvt_r(offs_t offset);
	void gpucvt_w(offs_t offset, u32 data, u32 mem_mask);

	// display
	u32 m_disp_regs[0x3000 / 4];
	u32 m_disp_status;
	emu_timer *m_line_timer;
	u32 disp_r(offs_t offset);
	void disp_w(offs_t offset, u32 data, u32 mem_mask);
	void disp_update_irq();
	TIMER_CALLBACK_MEMBER(line_cb);
	void disp_update_screen_size();
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	// DMA
	u32 m_bcdma_regs[0x200 / 4];
	u32 m_dcdma_regs[0x200 / 4];
	u32 m_bcdma_busy;
	emu_timer *m_bcdma_timer[8];
	void bcdma_update_irq();
	TIMER_CALLBACK_MEMBER(bcdma_done_cb);
	u32 bcdma_r(offs_t offset);
	void bcdma_w(offs_t offset, u32 data, u32 mem_mask);
	u32 dcdma_r(offs_t offset);
	void dcdma_w(offs_t offset, u32 data, u32 mem_mask);

	// sound (registers only for now)

	// CD
	u32 m_cd_regs[0x100 / 4];
	u32 cd_r(offs_t offset);
	void cd_w(offs_t offset, u32 data, u32 mem_mask);
	void cd_update_irq();
	s32 m_cd_head;      // absolute frame under the pickup (LBA + 150)
	s32 m_cd_dir;       // direction of the last sled command
	s32 m_cd_jump_n;    // 2N track jump count from the last $7x command
	bool m_cd_sled_armed;
	u32 m_cd_ptr;       // decoder write pointer
	s32 m_cd_len;
	bool m_cd_running;
	emu_timer *m_cd_timer;
	bool cd_sens(u8 select);
	void cd_jump_tracks(s32 tracks);
	void cd_servo_write(int nbits, u64 value, bool sens);
	void cd_latch_subq();
	void cd_queue_push(u32 data_addr, s32 frame);
	TIMER_CALLBACK_MEMBER(cd_sector_cb);

	// DSP memory window
	std::unique_ptr<u16[]> m_dsp_mem;

	u32 unk_r(offs_t offset);
	void unk_w(offs_t offset, u32 data, u32 mem_mask);

	void vsmilpro_map(address_map &map) ATTR_COLD;
};


/***************************************************************************
    Interrupt controller (0xDC000000)
***************************************************************************/

void vsmilpro_state::set_irq(int source, int state)
{
	// sources are level sensitive: the ROM's tick handler only acknowledges the
	// timer block, never the controller
	if (state)
		m_int_raw |= 1U << source;
	else
		m_int_raw &= ~(1U << source);
	m_int_pending = m_int_raw;
	update_cpu_irqs();
}

void vsmilpro_state::update_cpu_irqs()
{
	m_maincpu->set_input_line(arm7_cpu_device::ARM7_IRQ_LINE, (m_int_pending & m_int_enable[0]) ? ASSERT_LINE : CLEAR_LINE);
	m_maincpu->set_input_line(arm7_cpu_device::ARM7_FIRQ_LINE, (m_int_pending & m_int_enable[1]) ? ASSERT_LINE : CLEAR_LINE);
}

u32 vsmilpro_state::intc_r(offs_t offset)
{
	switch (offset)
	{
	case 0x000 / 4: case 0x100 / 4: return m_int_pending;
	case 0x008 / 4: return m_int_enable[0];
	case 0x108 / 4: return m_int_enable[1];
	case 0x024 / 4: case 0x124 / 4:
	{
		// TODO: probably returns the highest-priority pending source
		const u32 p = m_int_pending & m_int_enable[BIT(offset, 6)];
		for (int i = 0; i < 32; i++)
			if (BIT(p, i))
				return i;
		return 0;
	}

	default:
		return m_int_regs[offset];
	}
}

void vsmilpro_state::intc_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_int_regs[offset]);
	switch (offset)
	{
	case 0x004 / 4: case 0x104 / 4:
		// write 1 to clear; level sources stay pending while asserted
		m_int_pending = m_int_raw;
		break;
	case 0x008 / 4: m_int_enable[0] |= data; break;
	case 0x00c / 4: m_int_enable[0] &= ~data; break;
	case 0x108 / 4: m_int_enable[1] |= data; break;
	case 0x10c / 4: m_int_enable[1] &= ~data; break;
	default:
		LOGMASKED(LOG_INTC, "%08x: intc %03x = %08x\n", m_maincpu->pc(), offset * 4, data);
		break;
	}
	update_cpu_irqs();
}


/***************************************************************************
    System control (0x900A0000), PLL, RTC, GPIO
***************************************************************************/

void vsmilpro_state::update_mbox_irqs()
{
	set_irq(IRQ_MBOX0, m_sysctl[0x104 / 4] != 0);
	set_irq(IRQ_MBOX1, m_sysctl[0x110 / 4] != 0);
}

u32 vsmilpro_state::sysctl_r(offs_t offset)
{
	switch (offset)
	{
	case 0x00c / 4: return 0; // reset cause: cold boot
	case 0x018 / 4: return m_io_keys->read();
	case 0x01c / 4: return 0;
	case 0x030 / 4: return m_sysctl[offset];
	default:
		if (offset >= 0x100 / 4 && offset < 0x120 / 4)
			LOGMASKED(LOG_SYS, "%08x: mailbox read %03x = %08x\n", m_maincpu->pc(), offset * 4, m_sysctl[offset]);
		return m_sysctl[offset];
	}
}

void vsmilpro_state::sysctl_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x008 / 4:
		if (data == 2)
		{
			logerror("%08x: power off\n", m_maincpu->pc());
			machine().schedule_exit();
		}
		break;
	case 0x02c / 4:
		// write back what was read to acknowledge
		m_sysctl[offset] &= ~data;
		set_irq(IRQ_TICK, m_sysctl[offset] != 0);
		return;
	case 0x030 / 4:
		m_sysctl[offset] &= ~data;
		update_timer_irq();
		return;
	case 0x038 / 4:
		COMBINE_DATA(&m_sysctl[offset]);
		update_timer_irq();
		return;
	default:
		break;
	}
	COMBINE_DATA(&m_sysctl[offset]);
	if (offset >= 0x100 / 4 && offset < 0x120 / 4)
	{
		LOGMASKED(LOG_SYS, "%08x: mailbox write %03x = %08x\n", m_maincpu->pc(), offset * 4, data);
		update_mbox_irqs();
	}
}

u32 vsmilpro_state::pll_r(offs_t offset)
{
	if (offset == 0x14 / 4)
		return 1; // PLL locked
	return m_pll[offset];
}

void vsmilpro_state::pll_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_pll[offset]);
}

u32 vsmilpro_state::rtc_r(offs_t offset)
{
	// +0x00: free-running seconds counter; the ROM waits for it to tick on a
	// cold boot before marking the always-on block with 0xC01DB007 at +0x88
	if (offset == 0)
		return m_rtc[0] + u32(machine().time().seconds());
	return m_rtc[offset];
}

void vsmilpro_state::rtc_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_rtc[offset]);
}

// four 8-bit lanes per bank at +0x40*lane: +0x10 dir, +0x14 out, +0x18 readback, +0x20 pins
template <int Bank>
u32 vsmilpro_state::gpio_r(offs_t offset)
{
	const int lane = (offset >> 4) & 3;
	switch ((offset & 0xf) * 4)
	{
	case 0x10: return m_gpio_dir[Bank][lane];
	case 0x14: return m_gpio_out[Bank][lane];
	case 0x18:
	case 0x20:
	{
		u32 val = m_gpio_out[Bank][lane];
		if (Bank == 1 && lane == 0)
			val = (val & ~0x0f) | (m_io_straps->read() & 0x0f);
		if (Bank == 0 && lane == 1)
		{
			// bit 14: low = disc lid closed (tied to a mounted image); bit 10: unknown switch
			val &= ~0x44;
			if (!m_cdrom->exists())
				val |= 0x40;
			val |= BIT(m_io_straps->read(), 4) << 2;
		}
		return val;
	}
	default:
		LOGMASKED(LOG_UNKNOWN, "%08x: gpio%d read %02x\n", m_maincpu->pc(), Bank, offset * 4);
		return 0;
	}
}

template <int Bank>
void vsmilpro_state::gpio_w(offs_t offset, u32 data, u32 mem_mask)
{
	const int lane = (offset >> 4) & 3;
	switch ((offset & 0xf) * 4)
	{
	case 0x10: m_gpio_dir[Bank][lane] = data & 0xff; break;
	case 0x14: case 0x18: m_gpio_out[Bank][lane] = data & 0xff; break;
	default:
		LOGMASKED(LOG_UNKNOWN, "%08x: gpio%d write %02x = %08x\n", m_maincpu->pc(), Bank, offset * 4, data);
		break;
	}
}


/***************************************************************************
    Timers (0x900C0000)
    ch0: periodic tick (1 ms in practice), raises the 0x900A002C tick source
    ch1: 16-bit free-running down counter, wrap flag in 0x900A0030 bit 1
***************************************************************************/

void vsmilpro_state::update_timer_irq()
{
	set_irq(IRQ_TIMER, (m_sysctl[0x30 / 4] & m_sysctl[0x38 / 4] & 0x3f) != 0);
}

TIMER_CALLBACK_MEMBER(vsmilpro_state::tick_cb)
{
	m_sysctl[0x2c / 4] |= 1;
	m_sysctl[0x30 / 4] |= 1;
	set_irq(IRQ_TICK, 1);
	update_timer_irq();
}

TIMER_CALLBACK_MEMBER(vsmilpro_state::ch1_wrap_cb)
{
	m_sysctl[0x30 / 4] |= 2;
	update_timer_irq();
	m_ch1_start = machine().time();
	m_ch1_wrap_timer->adjust(attotime::from_ticks(u64(m_ch1_period_ticks) * 0x10000, PERIPH_CLOCK));
}

u16 vsmilpro_state::ch1_count()
{
	if (!m_ch1_running)
		return m_timer_regs[0x0c / 4];
	const u64 elapsed = (machine().time() - m_ch1_start).as_ticks(PERIPH_CLOCK) / m_ch1_period_ticks;
	return u16(0xffff - (elapsed & 0xffff));
}

u32 vsmilpro_state::timer_r(offs_t offset)
{
	if (offset == 0x0c / 4)
		return ch1_count();
	return m_timer_regs[offset];
}

void vsmilpro_state::timer_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_timer_regs[offset]);
	switch (offset)
	{
	case 0x08 / 4:
		if (data & 1)
		{
			const u64 ticks = u64(m_timer_regs[0] ? m_timer_regs[0] : 1) * (m_timer_regs[1] + 1);
			const attotime period = attotime::from_ticks(ticks, PERIPH_CLOCK);
			m_tick_timer->adjust(period, 0, period);
		}
		else
			m_tick_timer->adjust(attotime::never);
		break;
	case 0x14 / 4:
		if (data & 2)
		{
			m_ch1_period_ticks = m_timer_regs[0x10 / 4] + 1;
			m_ch1_running = true;
			m_ch1_start = machine().time();
			m_ch1_wrap_timer->adjust(attotime::from_ticks(u64(m_ch1_period_ticks) * 0x10000, PERIPH_CLOCK));
		}
		else
		{
			m_ch1_running = false;
			m_ch1_wrap_timer->adjust(attotime::never);
		}
		break;
	}
}


/***************************************************************************
    UARTs (16550-like, 32-bit spaced registers)
    UART0 = controller port 1 and the 38400 baud debug console
    UART1 = controller port 2
***************************************************************************/

u8 vsmilpro_state::uart_iir(int n)
{
	uart_t &u = m_uart[n];
	const u8 fifo = (u.fcr & 1) ? 0xc0 : 0x00;
	if ((u.ier & 1) && !u.rx.empty())
		return fifo | 0x04;
	if ((u.ier & 2) && u.thre_pending)
		return fifo | 0x02;
	return fifo | 0x01;
}

void vsmilpro_state::uart_update_irq(int n)
{
	set_irq(n == 0 ? IRQ_UART0 : IRQ_UART1, !(uart_iir(n) & 1));
}

template <int N>
u32 vsmilpro_state::uart_r(offs_t offset)
{
	uart_t &u = m_uart[N];
	const bool dlab = BIT(u.lcr, 7);
	u32 ret = 0;
	switch (offset)
	{
	case 0:
		if (dlab)
			ret = u.dll;
		else if (!u.rx.empty())
		{
			ret = u.rx.front();
			if (!machine().side_effects_disabled())
			{
				u.rx.pop_front();
				uart_update_irq(N);
			}
		}
		break;
	case 1: ret = dlab ? u.dlm : u.ier; break;
	case 2:
		ret = uart_iir(N);
		if (!machine().side_effects_disabled() && (ret & 0x0f) == 0x02)
		{
			u.thre_pending = false;
			uart_update_irq(N);
		}
		break;
	case 3: ret = u.lcr; break;
	case 4: ret = u.mcr; break;
	case 5: ret = 0x60 | (u.rx.empty() ? 0 : 1); break;
	case 6: ret = 0xb0; break;
	case 7: ret = u.scr; break;
	}
	return ret;
}

template <int N>
void vsmilpro_state::uart_w(offs_t offset, u32 data, u32 mem_mask)
{
	uart_t &u = m_uart[N];
	const bool dlab = BIT(u.lcr, 7);
	data &= 0xff;
	switch (offset)
	{
	case 0:
		if (dlab)
			u.dll = data;
		else
		{
			LOGMASKED(LOG_UART, "uart%d tx %02x\n", N, data);
			const u32 divisor = (u.dlm << 8) | u.dll;
			if (divisor >= 200)
				joy_from_console(N, data); // 9600 baud: controller port
			else if (N == 0)
			{
				// 38400 baud: debug console on controller port 1's pins
				if (data == '\n')
				{
					LOGMASKED(LOG_DEBUGTX, "DEBUG: %s\n", u.line);
					u.line.clear();
				}
				else if (data >= 0x20 && data < 0x7f)
					u.line += char(data);
			}
			u.thre_pending = true;
			uart_update_irq(N);
		}
		break;
	case 1:
		if (dlab)
			u.dlm = data;
		else
		{
			if ((data & 2) && !(u.ier & 2))
				u.thre_pending = true;
			u.ier = data;
			uart_update_irq(N);
		}
		break;
	case 2:
		u.fcr = data;
		if (data & 2)
			u.rx.clear();
		uart_update_irq(N);
		break;
	case 3: u.lcr = data; break;
	case 4: u.mcr = data; break;
	case 7: u.scr = data; break;
	}
}


/***************************************************************************
    Controllers: framed serial protocol at 9600 baud on each UART

    Console -> controller: FF 01 02, value v as 0x40|v[3:0], 0x40|v[9:4],
    0x40|v[15:10], then the checksum lo(v)+hi(v)+3 encoded the same way.
    Controller -> console: FF 01 len, len/2 triples each carrying two bytes
    (b0 = t0[3:0] | t1[3:0]<<4, b1 = t1[5:4] | t2[5:0]<<2), then a checksum
    triple holding the u16 sum of the bytes + 1 + len.
    - command 0x340 (detect) is answered with status bytes 0x20, 0x08
    - len 6 frames carry the button/stick report: a 40-bit stream with
      axis 2 at bits 4-11, axis 1 at bits 13-20, buttons at bits 22-37
***************************************************************************/

void vsmilpro_state::joy_send(int port, const u8 *payload, int len)
{
	uart_t &u = m_uart[port];
	auto triple = [&u] (u8 b0, u8 b1)
	{
		u.rx.push_back(0x40 | (b0 & 0x0f));
		u.rx.push_back(0x40 | (b0 >> 4) | ((b1 & 3) << 4));
		u.rx.push_back(0x40 | (b1 >> 2));
	};
	u.rx.push_back(0xff);
	u.rx.push_back(0x01);
	u.rx.push_back(len);
	u16 sum = 1 + len;
	for (int i = 0; i < len; i += 2)
	{
		triple(payload[i], payload[i + 1]);
		sum += payload[i] + payload[i + 1];
	}
	triple(sum & 0xff, sum >> 8);
	uart_update_irq(port);
}

void vsmilpro_state::joy_from_console(int port, u8 data)
{
	joy_t &j = m_joy[port];
	if (j.cmd.empty() && data != 0xff)
		return;
	j.cmd.push_back(data);
	if (j.cmd.size() < 9)
		return;
	const u16 v = (j.cmd[3] & 0x0f) | ((j.cmd[4] & 0x3f) << 4) | ((j.cmd[5] & 0x3f) << 10);
	LOGMASKED(LOG_UNKNOWN, "joy%d frame %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", port + 1,
			j.cmd[0], j.cmd[1], j.cmd[2], j.cmd[3], j.cmd[4], j.cmd[5], j.cmd[6], j.cmd[7], j.cmd[8]);
	j.cmd.clear();
	LOGMASKED(LOG_UART, "joy%d command %04x\n", port + 1, v);
	if (v == 0x340)
	{
		static const u8 status[2] = { 0x20, 0x08 };
		joy_send(port, status, 2);
		j.detected = true;
		j.last_sent = attotime::zero;
	}
	else
		LOGMASKED(LOG_UNKNOWN, "joy%d unhandled command %04x\n", port + 1, v);
}

void vsmilpro_state::joy_build_report(int port, u8 *raw)
{
	const u16 buttons = m_io_joy_buttons[port]->read();
	const u8 stick = m_io_joy_stick[port]->read();
	// axis 1 (report byte 5) is horizontal: left = key code 1, right = key code 2
	// axis 2 (report byte 6) is vertical: positive = key code 0x8005, negative = 0x8006
	// TODO: confirm the vertical sign on hardware
	const u8 a1 = BIT(stick, 2) ? u8(-100) : BIT(stick, 3) ? 100 : 0;
	const u8 a2 = BIT(stick, 0) ? 100 : BIT(stick, 1) ? u8(-100) : 0;
	raw[0] = (a2 & 0x0f) << 4;
	raw[1] = (a2 >> 4) | ((a1 & 7) << 5);
	raw[2] = (a1 >> 3) | ((buttons & 3) << 6);
	raw[3] = buttons >> 2;
	raw[4] = buttons >> 10;
}

TIMER_CALLBACK_MEMBER(vsmilpro_state::joy_cb)
{
	for (int port = 0; port < 2; port++)
	{
		joy_t &j = m_joy[port];
		if (!j.detected)
			continue;
		u8 raw[6];
		joy_build_report(port, raw);
		raw[5] = 0;
		const bool changed = memcmp(raw, j.last, 5) != 0;
		if (changed || (machine().time() - j.last_sent) > attotime::from_msec(100))
		{
			memcpy(j.last, raw, 5);
			j.last_sent = machine().time();
			joy_send(port, raw, 6);
		}
	}
}


/***************************************************************************
    SPI controller (0xA1000000) with the on-board serial flash and the
    DataFlash memory card. GPIO0 bit 19 selects: high = serial flash.
    +0x04 = ((bits-1) << 16) | 1 starts a full-duplex frame of up to 8 bytes
    held MSB-first in +0x1C (bytes 0-3) and +0x20 (bytes 4-7).
***************************************************************************/

u32 vsmilpro_state::spi_r(offs_t offset)
{
	if (offset == 0x10 / 4)
		return 1; // transfer done
	return m_spi_regs[offset];
}

void vsmilpro_state::spi_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_spi_regs[offset]);
	if (offset == 0x04 / 4 && (data & 1))
	{
		const int nbits = ((data >> 16) & 0x3f) + 1;
		spi_transfer((nbits + 7) / 8);
	}
}

void vsmilpro_state::spi_transfer(int nbytes)
{
	const bool sflash = BIT(m_gpio_out[0][2], 3); // bank 0 bit 19
	u8 tx[8], rx[8];
	for (int i = 0; i < 4; i++)
	{
		tx[i] = m_spi_regs[0x1c / 4] >> (24 - 8 * i);
		tx[i + 4] = m_spi_regs[0x20 / 4] >> (24 - 8 * i);
	}
	std::vector<u8> cmd;
	for (int i = 0; i < nbytes; i++)
		rx[i] = sflash ? sflash_xfer(i, tx[i], cmd) : dataflash_xfer(i, tx[i], cmd);
	for (int i = nbytes; i < 8; i++)
		rx[i] = 0xff;

	// short frames come back right-justified in +0x1C (status = low byte after
	// a 2-byte frame); longer frames keep bytes 0-3 in +0x1C and 4-7 in +0x20
	u32 w0 = 0, w1 = 0;
	if (nbytes <= 4)
	{
		for (int i = 0; i < nbytes; i++)
			w0 = (w0 << 8) | rx[i];
	}
	else
	{
		for (int i = 0; i < 4; i++)
			w0 = (w0 << 8) | rx[i];
		for (int i = 4; i < 8; i++)
			w1 = (w1 << 8) | rx[i];
	}
	m_spi_regs[0x1c / 4] = w0;
	m_spi_regs[0x20 / 4] = w1;
	LOGMASKED(LOG_SPI, "%08x: spi %s %d bytes tx %02x %02x %02x %02x %02x %02x %02x %02x -> %08x %08x\n", m_maincpu->pc(), sflash ? "sflash" : "card", nbytes,
			tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7], m_spi_regs[0x1c / 4], m_spi_regs[0x20 / 4]);
}

// SST25VF512A-like serial flash, one command per frame (chip select per frame)
u8 vsmilpro_state::sflash_xfer(int index, u8 data, std::vector<u8> &cmd)
{
	cmd.push_back(data);
	const u8 op = cmd[0];
	const u32 addr = cmd.size() > 3 ? ((cmd[1] << 16) | (cmd[2] << 8) | cmd[3]) & (SFLASH_SIZE - 1) : 0;
	switch (op)
	{
	case 0x03: // read
	case 0x0b: // fast read (one dummy byte)
	{
		const int first = (op == 0x0b) ? 5 : 4;
		if (index >= first)
			return m_sflash[(addr + index - first) & (SFLASH_SIZE - 1)];
		return 0xff;
	}
	case 0x05: // read status
		return index >= 1 ? m_sflash_status : 0xff;
	case 0x06: m_sflash_status |= 0x02; return 0xff; // write enable
	case 0x04: m_sflash_status &= ~0x02; return 0xff; // write disable
	case 0x50: return 0xff; // enable write status
	case 0x01: if (index == 1) m_sflash_status = (m_sflash_status & 0x03) | (data & 0xbc); return 0xff;
	case 0x90: case 0xab: // read id: manufacturer, device, repeated
		if (index >= 4)
			return ((index - 4) & 1) ? 0x48 : 0xbf; // SST25VF512A
		return 0xff;
	case 0x9f: // JEDEC id
		if (index == 1) return 0xbf;
		if (index == 2) return 0x48;
		return 0xff;
	case 0x02: // byte program
		if (index == 4 && (m_sflash_status & 2))
		{
			m_sflash[addr] &= data;
			m_sflash_status &= ~0x02;
		}
		return 0xff;
	case 0xaf: // auto address increment program
		if (index == 4 && (m_sflash_status & 2))
		{
			m_sflash_aai_addr = addr;
			m_sflash[m_sflash_aai_addr++ & (SFLASH_SIZE - 1)] &= data;
		}
		else if (cmd.size() == 2 && index == 1)
			m_sflash[m_sflash_aai_addr++ & (SFLASH_SIZE - 1)] &= data;
		return 0xff;
	case 0x20: case 0xd7: // 4K sector erase
		if (index == 3 && (m_sflash_status & 2))
		{
			const u32 a = ((cmd[1] << 16) | (cmd[2] << 8) | cmd[3]) & (SFLASH_SIZE - 1) & ~0xfff;
			std::fill_n(&m_sflash[a], 0x1000, 0xff);
			m_sflash_status &= ~0x02;
		}
		return 0xff;
	case 0x52: case 0xd8: // 32K block erase
		if (index == 3 && (m_sflash_status & 2))
		{
			const u32 a = ((cmd[1] << 16) | (cmd[2] << 8) | cmd[3]) & (SFLASH_SIZE - 1) & ~0x7fff;
			std::fill_n(&m_sflash[a], 0x8000, 0xff);
			m_sflash_status &= ~0x02;
		}
		return 0xff;
	case 0x60: case 0xc7: // chip erase
		if (index == 0 && (m_sflash_status & 2))
		{
			std::fill_n(&m_sflash[0], SFLASH_SIZE, 0xff);
			m_sflash_status &= ~0x02;
		}
		return 0xff;
	default:
		if (index == 0)
			LOGMASKED(LOG_UNKNOWN, "%08x: serial flash unknown command %02x\n", m_maincpu->pc(), op);
		return 0xff;
	}
}

// TODO: AT45DB642D memory card. For now no card is inserted.
u8 vsmilpro_state::dataflash_xfer(int index, u8 data, std::vector<u8> &cmd)
{
	cmd.push_back(data);
	return 0xff;
}


/***************************************************************************
    GPU ("Hanabi") command processor at 0xA8000000
    Display lists are executed instantly when kicked.
***************************************************************************/

void vsmilpro_state::gpu_update_irq()
{
	const u32 mask = m_gpu_regs[0x10 / 4];
	set_irq(IRQ_GPU, (m_gpu_regs[0x14 / 4] & ~mask & 0x3f) != 0);
}

u32 vsmilpro_state::gpu_r(offs_t offset)
{
	switch (offset)
	{
	case 0x00 / 4: return m_gpu_regs[0] & 0xffff; // engine idle
	default: return m_gpu_regs[offset];
	}
}

void vsmilpro_state::gpu_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x00 / 4:
		LOGMASKED(LOG_GPU, "%08x: gpu cmd %x list %08x\n", m_maincpu->pc(), data, m_gpu_regs[0x28 / 4]);
		if (data == 1)
			gpu_run_list(m_gpu_regs[0x28 / 4]);
		else if (data == 3)
		{
			m_gpu_regs[0x14 / 4] = 0;
			gpu_update_irq();
		}
		return;
	case 0x14 / 4:
		// acknowledge: writing the source bits clears them along with the tag
		if (data & 0x3f)
			m_gpu_regs[0x14 / 4] &= ~(data & 0x3f);
		if (!(m_gpu_regs[0x14 / 4] & 0x3f))
			m_gpu_regs[0x14 / 4] = 0;
		gpu_update_irq();
		return;
	case 0x10 / 4:
		COMBINE_DATA(&m_gpu_regs[offset]);
		gpu_update_irq();
		return;
	default:
		COMBINE_DATA(&m_gpu_regs[offset]);
		return;
	}
}

void vsmilpro_state::gpu_plot(int x, int y, u16 color)
{
	const int cx0 = m_gpu_clip[0] & 0xfff, cy0 = (m_gpu_clip[0] >> 12) & 0xfff;
	const int cx1 = m_gpu_clip[1] & 0xfff, cy1 = (m_gpu_clip[1] >> 12) & 0xfff;
	if (x < cx0 || x > cx1 || y < cy0 || y > cy1 || x < 0 || y < 0 || x > 1023)
		return;
	gpu_vram_w16(x, y, color);
}

// texture mode (0x86): bits 0-2 log2(width/8), bits 3-5 log2(height/8),
// bit 8 + bits 9-10 colour format (from the engine's use; partly guessed)
u16 vsmilpro_state::gpu_texel(u32 u, u32 v, bool &transparent)
{
	const u32 w = 8 << (m_gpu_texmode & 7);
	const u32 h = 8 << ((m_gpu_texmode >> 3) & 7);
	u &= w - 1;
	v &= h - 1;
	const u32 tx = m_gpu_texpos & 0xfff, ty = (m_gpu_texpos >> 12) & 0xfff;
	const u32 bppsel = (m_gpu_texmode >> 9) & 3;
	const u32 base = m_gpu_regs[0x98 / 4];
	u16 color;
	if (bppsel == 0)
		color = ram_r16(tiled_addr(base, tx * 2 + u * 2, ty + v));
	else
	{
		u32 index;
		if (bppsel == 1)
			index = ram_r8(tiled_addr(base, tx * 2 + u, ty + v));
		else
		{
			const u8 b = ram_r8(tiled_addr(base, tx * 2 + (u >> 1), ty + v));
			index = BIT(u, 0) ? (b >> 4) : (b & 0x0f);
		}
		const u32 cx = m_gpu_clutpos & 0xfff, cy = (m_gpu_clutpos >> 12) & 0xfff;
		color = ram_r16(tiled_addr(base, cx * 2 + index * 2, cy));
	}
	transparent = (color == 0);
	return color;
}

void vsmilpro_state::gpu_sprite(u32 flags, const u32 *args, int nargs)
{
	// args: colour/z, y<<16|x, h<<16|w, [v<<16|u]
	if (nargs < 3)
		return;
	const int ox = m_gpu_offset & 0xfff, oy = (m_gpu_offset >> 12) & 0xfff;
	const int x = s16(args[1] & 0xffff) + ox, y = s16(args[1] >> 16) + oy;
	const int w = args[2] & 0xffff, h = args[2] >> 16;
	const bool textured = BIT(flags, 0) && nargs >= 4;
	const u32 u0 = textured ? args[3] & 0xffff : 0, v0 = textured ? args[3] >> 16 : 0;
	for (int j = 0; j < h; j++)
		for (int i = 0; i < w; i++)
		{
			if (textured)
			{
				bool transparent;
				const u16 c = gpu_texel(u0 + i, v0 + j, transparent);
				if (!transparent)
					gpu_plot(x + i, y + j, c);
			}
			else
				gpu_plot(x + i, y + j, args[0] & 0xffff);
		}
}

void vsmilpro_state::gpu_run_list(u32 addr)
{
	std::vector<u32> callstack;
	int guard = 0;
	while (guard++ < 0x200000)
	{
		const u32 cmd = ram_r32(addr);
		addr += 4;
		const u8 op = cmd >> 24;
		switch (op)
		{
		case 0x04: // end of a called sub-list
			if (!callstack.empty())
			{
				addr = callstack.back();
				callstack.pop_back();
			}
			break;
		case 0x95: case 0x96: // flush/terminate primitive
			break;
		case 0x08: // jump
			addr = ram_r32(addr);
			break;
		case 0x09: // call
			callstack.push_back(addr + 4);
			addr = ram_r32(addr);
			break;
		case 0x0a: // jump over inline data
			addr = ram_r32(addr);
			break;
		case 0x0c: // address for the next transfer
			m_gpu_xfer_addr = ram_r32(addr);
			addr += 4;
			break;
		case 0x0e: // fetch words: handled by the image commands
			addr += 4;
			break;
		case 0x10: // fence
			m_gpu_regs[0x5c / 4] = cmd & 0xffffff;
			break;
		case 0x14: // end of list, interrupt with tag in bits 8-15
			addr += 4;
			m_gpu_regs[0x14 / 4] = (cmd & 0xff00) | 0x08;
			gpu_update_irq();
			return;
		case 0x1c: case 0x1e:
			m_gpu_state[op] = cmd & 0xffffff;
			break;
		case 0x20: // register block write
		{
			const u32 reg = (cmd >> 16) & 0xff, n = cmd & 0xffff;
			for (u32 i = 0; i < n; i++)
				m_gpu_state[(reg + i) & 0xff] = ram_r32(addr + i * 4);
			addr += n * 4;
			break;
		}
		case 0x81: m_gpu_state[op] = cmd & 0xffffff; break;
		case 0x82: m_gpu_clip[0] = cmd & 0xffffff; break;
		case 0x83: m_gpu_clip[1] = cmd & 0xffffff; break;
		case 0x85: m_gpu_offset = cmd & 0xffffff; break;
		case 0x86: m_gpu_texmode = cmd & 0xffffff; break;
		case 0x87: m_gpu_texpos = cmd & 0xffffff; break;
		case 0x8f: m_gpu_clutpos = cmd & 0xffffff; break;
		case 0x84: case 0x88: case 0x89: case 0x8a: case 0x8b: case 0x8c: case 0x8d: case 0x8e:
		case 0x90: case 0x91: case 0x92: case 0x93: case 0x94:
			m_gpu_state[op] = cmd & 0xffffff;
			break;
		case 0xd0: case 0xd4: // load image into VRAM: pos, size, then 0x0E fetch of n words from src
		{
			const u32 pos = ram_r32(addr), size = ram_r32(addr + 4);
			u32 fetch = ram_r32(addr + 8), src = ram_r32(addr + 12);
			addr += 16;
			const u32 x = pos & 0xffff, y = pos >> 16, w = size & 0xffff, h = size >> 16;
			LOGMASKED(LOG_GPU, "gpu load image (%d,%d) %dx%d from %08x (%08x)\n", x, y, w, h, src, fetch);
			if ((fetch >> 24) != 0x0e)
				LOGMASKED(LOG_UNKNOWN, "gpu load image: unexpected %08x\n", fetch);
			for (u32 j = 0; j < h; j++)
				for (u32 i = 0; i < w; i++)
					gpu_vram_w16((x + i) & 1023, y + j, ram_r16(src + (j * w + i) * 2));
			break;
		}
		case 0xd1: // store image from VRAM to the 0x0C address
		{
			const u32 pos = ram_r32(addr), size = ram_r32(addr + 4);
			addr += 8;
			const u32 x = pos & 0xffff, y = pos >> 16, w = size & 0xffff, h = size >> 16;
			for (u32 j = 0; j < h; j++)
				for (u32 i = 0; i < w; i++)
					ram_w16(m_gpu_xfer_addr + (j * w + i) * 2, gpu_vram_r16((x + i) & 1023, y + j));
			break;
		}
		case 0xd3: // fill rectangle
		{
			const u32 pos = ram_r32(addr), size = ram_r32(addr + 4), col = ram_r32(addr + 8);
			addr += 12;
			const u32 x = pos & 0xffff, y = pos >> 16, w = size & 0xffff, h = size >> 16;
			LOGMASKED(LOG_GPU, "gpu fill (%d,%d) %dx%d %04x\n", x, y, w, h, col & 0xffff);
			for (u32 j = 0; j < h; j++)
				for (u32 i = 0; i < w; i++)
					gpu_vram_w16((x + i) & 1023, y + j, col);
			break;
		}
		case 0xc8: // sprite
		{
			const int n = cmd & 0xff;
			u32 args[8] = { };
			for (int i = 0; i < n && i < 8; i++)
				args[i] = ram_r32(addr + i * 4);
			addr += n * 4;
			gpu_sprite((cmd >> 16) & 0xff, args, n);
			break;
		}
		default:
			if (op >= 0xc0 && op <= 0xdf)
			{
				// other primitives: skip their arguments
				// TODO: polygons, lines, points, 3D primitive kinds
				const int n = cmd & 0xff;
				LOGMASKED(LOG_UNKNOWN, "gpu unhandled primitive %08x (%d words)\n", cmd, n);
				addr += n * 4;
				if (op == 0xc0 || op == 0xc2)
				{
					// strips end with 0xFF000000
					while (ram_r32(addr) != 0xff000000 && guard++ < 0x200000)
						addr += 4;
					addr += 4;
				}
			}
			else
			{
				LOGMASKED(LOG_UNKNOWN, "gpu unknown command %08x at %08x, stopping list\n", cmd, addr - 4);
				m_gpu_regs[0x14 / 4] = 0xfd08;
				gpu_update_irq();
				return;
			}
			break;
		}
	}
	LOGMASKED(LOG_UNKNOWN, "gpu list runaway\n");
}

// number converters (0xA4000000)
u32 vsmilpro_state::gpucvt_r(offs_t offset)
{
	const u32 v = m_gpu_cvt[offset];
	switch (offset)
	{
	case 0x0c / 4: case 0x1c / 4: // f16 (1.6.9, bias 31) -> 4.12 fixed
	{
		if (!(v & 0x7fff))
			return 0;
		const int e = ((v >> 9) & 0x3f) - 31;
		double f = ldexp(1.0 + (v & 0x1ff) / 512.0, e);
		if (v & 0x8000)
			f = -f;
		return u32(s32(f * 4096.0));
	}
	case 0x08 / 4: // fixed (x.12) -> f16
	{
		const s32 s = util::sext(v, 27);
		if (!s)
			return 0;
		double f = std::abs(s) / 4096.0;
		int e;
		const double m = frexp(f, &e); // f = m * 2^e, m in [0.5,1)
		const int exp = std::clamp(e - 1 + 31, 0, 63);
		const u32 mant = u32((m * 2.0 - 1.0) * 512.0) & 0x1ff;
		return (s < 0 ? 0x8000 : 0) | (exp << 9) | mant;
	}
	default:
		return v;
	}
}

void vsmilpro_state::gpucvt_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_gpu_cvt[offset]);
}


/***************************************************************************
    Display controller (0xB8000000)
***************************************************************************/

void vsmilpro_state::disp_update_irq()
{
	set_irq(IRQ_VIDEO, (m_disp_status & m_disp_regs[0x004 / 4]) != 0);
}

void vsmilpro_state::disp_update_screen_size()
{
	const u32 size = m_disp_regs[0x00c / 4];
	int w = (size & 0x7ff) + 1, h = ((size >> 16) & 0x7ff) + 1;
	if (w < 64 || w > 1024 || h < 64 || h > 576)
		return;
	const bool pal = m_disp_regs[0x008 / 4] == 3;
	const int lines = pal ? 312 : 262;
	h = std::min(h, lines - 1);
	rectangle visarea(0, w - 1, 0, h - 1);
	m_screen->configure(w, lines, visarea, attotime::from_hz(pal ? 50 : 60));
}

TIMER_CALLBACK_MEMBER(vsmilpro_state::line_cb)
{
	const int line = m_screen->vpos();
	const rectangle &vis = m_screen->visible_area();
	if (line == vis.bottom() + 1)
	{
		m_disp_status |= 0x02; // vblank
		disp_update_irq();
	}
	if (line == int(m_disp_regs[0x1048 / 4] & 0x7ff))
	{
		m_disp_status |= 0x40; // line compare
		disp_update_irq();
	}
	if (line == int((m_disp_regs[0x020 / 4] >> 16) & 0x7ff))
	{
		m_disp_status |= 0x01; // TODO: guessed bit for the +0x020 line interrupt
		disp_update_irq();
	}
	m_line_timer->adjust(m_screen->time_until_pos((line + 1) % m_screen->height()));
}

u32 vsmilpro_state::disp_r(offs_t offset)
{
	switch (offset)
	{
	case 0x000 / 4: return m_disp_status;
	case 0x024 / 4: return (m_screen->vpos() << 16) | (m_screen->hpos() & 0xffff); // TODO: unconfirmed layout
	default:
		return m_disp_regs[offset];
	}
}

void vsmilpro_state::disp_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x000 / 4:
		m_disp_status &= ~data;
		disp_update_irq();
		return;
	case 0x004 / 4:
		COMBINE_DATA(&m_disp_regs[offset]);
		disp_update_irq();
		return;
	default:
		break;
	}
	COMBINE_DATA(&m_disp_regs[offset]);
	LOGMASKED(LOG_VIDEO, "%08x: disp %04x = %08x\n", m_maincpu->pc(), offset * 4, data);
	if (offset == 0x00c / 4 || offset == 0x008 / 4)
		disp_update_screen_size();
}

static inline rgb_t bgr555(u16 p)
{
	return rgb_t(pal5bit(p & 0x1f), pal5bit((p >> 5) & 0x1f), pal5bit((p >> 10) & 0x1f));
}

/*
    Layers:
    - main (4): 16-bit tiled surface at +0x160, window +0x130/+0x134
    - 0-3: tile layers. +0x140+4n = tile map (u16 entries, row length from
      the 2-bit code at +0x104 byte n: 16/32/64/128 entries), +0x150+4n = 8bpp
      8x8 tile pixels (64 bytes per tile), +0x110+4n = scroll (y<<16|x).
      Windows: layer 0 +0x170, 1 +0x120, 2 +0x128, 3 +0x178 (x0,y0 / x1,y1)
    - palettes: two banks of 256 BGR555 colours at +0x800 / +0xA00
    - backdrop colour at +0x10C
    TODO: priorities, palette bank select (+0x02C?), tile entry attribute
    bits, transparency rule, layer 5 (cursor) at +0xC00
*/
u32 vsmilpro_state::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const u32 enable = m_disp_regs[0x100 / 4];
	bitmap.fill(bgr555(m_disp_regs[0x10c / 4]), cliprect);

	auto in_window = [this] (int reg, int x, int y)
	{
		const u32 a = m_disp_regs[reg / 4], b = m_disp_regs[reg / 4 + 1];
		return x >= int(a & 0x7ff) && x <= int(b & 0x7ff) && y >= int((a >> 16) & 0x7ff) && y <= int((b >> 16) & 0x7ff);
	};

	// main 16-bit layer from the tiled surface at the display start address
	if (BIT(enable, 5) || !(enable & 0x0f))
	{
		const u32 start = m_disp_regs[0x160 / 4];
		const u32 off = start - VRAM_ORIGIN;
		const u32 y0 = (off / 16384) * 8 + ((off % 16384) % 512) / 64;
		const u32 xb0 = ((off % 16384) / 512) * 64 + (off % 64);
		for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
		{
			u32 *dst = &bitmap.pix(y);
			for (int x = cliprect.left(); x <= cliprect.right(); x++)
			{
				if (!in_window(0x130, x, y))
					continue;
				const u32 addr = tiled_addr(VRAM_ORIGIN, (xb0 + x * 2) & 2047, y0 + y);
				const u16 p = in_sdram(addr) ? ram_r16(addr) : 0;
				dst[x] = bgr555(p);
			}
		}
	}

	static const int window_regs[4] = { 0x170, 0x120, 0x128, 0x178 };
	for (int layer = 0; layer < 4; layer++)
	{
		if (!BIT(enable, layer))
			continue;
		const u32 map = m_disp_regs[(0x140 / 4) + layer];
		const u32 tiles = m_disp_regs[(0x150 / 4) + layer];
		const u32 scroll = m_disp_regs[(0x110 / 4) + layer];
		const int mapw = 16 << ((m_disp_regs[0x104 / 4] >> (layer * 8)) & 3);
		const int sx = scroll & 0xfff, sy = (scroll >> 16) & 0xfff;
		for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
		{
			u32 *dst = &bitmap.pix(y);
			for (int x = cliprect.left(); x <= cliprect.right(); x++)
			{
				if (!in_window(window_regs[layer], x, y))
					continue;
				const int lx = (x + sx) & (mapw * 8 - 1), ly = (y + sy) & (mapw * 8 - 1);
				const u16 entry = ram_r16(map + ((ly >> 3) * mapw + (lx >> 3)) * 2);
				const u8 index = ram_r8(tiles + (entry & 0xfff) * 64 + (ly & 7) * 8 + (lx & 7));
				const u16 color = m_disp_regs[(0x800 / 4) + (index >> 1)] >> (BIT(index, 0) * 16);
				if (color == 0x83e0) // UI transparency key
					continue;
				dst[x] = bgr555(color);
			}
		}
	}
	return 0;
}


/***************************************************************************
    DMA
***************************************************************************/

// memory DMA (0xBC000000), 8 channels, instant transfers
u32 vsmilpro_state::bcdma_r(offs_t offset)
{
	if (offset == 0x0c / 4)
		return m_bcdma_busy;
	return m_bcdma_regs[offset];
}

void vsmilpro_state::bcdma_update_irq()
{
	set_irq(IRQ_BCDMA, (m_bcdma_regs[0x1c / 4] & m_bcdma_regs[0x18 / 4] & 0xff) != 0);
}

TIMER_CALLBACK_MEMBER(vsmilpro_state::bcdma_done_cb)
{
	m_bcdma_busy &= ~(1 << param);
	m_bcdma_regs[0x1c / 4] |= 1 << param;
	bcdma_update_irq();
}

void vsmilpro_state::bcdma_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset == 0x1c / 4)
	{
		m_bcdma_regs[offset] &= ~data;
		bcdma_update_irq();
		return;
	}
	COMBINE_DATA(&m_bcdma_regs[offset]);
	if (offset == 0x10 / 4)
	{
		for (int ch = 0; ch < 8; ch++)
		{
			if (!BIT(data, ch))
				continue;
			const u32 cfg = m_bcdma_regs[(0x100 + ch * 16) / 4];
			const u32 src = m_bcdma_regs[(0x104 + ch * 16) / 4], dst = m_bcdma_regs[(0x108 + ch * 16) / 4];
			static const int unit_sizes[4] = { 1, 2, 4, 4 };
			const int unit = unit_sizes[(cfg >> 16) & 3];
			const u32 count = cfg & 0x3ff;
			// TODO: confirm unit semantics; memcpy uses count = words with width field 3
			const int width = (m_bcdma_regs[0x08 / 4] >> (ch * 3)) & 7;
			const int bytes_per = (width == 3) ? 4 : unit;
			LOGMASKED(LOG_DMA, "bcdma ch%d %08x -> %08x count %d x %d (cfg %08x)\n", ch, src, dst, count, bytes_per, cfg);
			for (u32 i = 0; i < count * bytes_per; i++)
				ram_w8(dst + i, ram_r8(src + i));
			// the data moves at once; completion is signalled after a short
			// delay (~4 bytes per 75 MHz bus clock) so drivers see it busy first
			m_bcdma_busy |= 1 << ch;
			m_bcdma_timer[ch]->adjust(attotime::from_ticks(count * bytes_per / 4 + 16, 75'000'000), ch);
		}
	}
}

// descriptor DMA (0xC0000000), linked 32-byte descriptors, instant transfers
u32 vsmilpro_state::dcdma_r(offs_t offset)
{
	switch (offset)
	{
	case 0x00 / 4: return m_dcdma_regs[0x18 / 4]; // armed channels read back ready
	case 0x40 / 4: return 0;
	default:
		if (offset >= 0x80 / 4 && ((offset * 4) & 0x3f) == 0x04)
			return 0x10; // done
		return m_dcdma_regs[offset];
	}
}

void vsmilpro_state::dcdma_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_dcdma_regs[offset]);
	if (offset == 0x10 / 4)
	{
		for (int ch = 0; ch < 2; ch++)
		{
			if (!BIT(data, ch))
				continue;
			u32 desc = m_dcdma_regs[(0x88 + ch * 0x40) / 4];
			int guard = 0;
			while (desc && guard++ < 4096)
			{
				const u32 src = ram_r32(desc + 8), dst = ram_r32(desc + 12);
				const u32 next = ram_r32(desc + 16), count = ram_r32(desc + 20);
				const s32 sstride = ram_r32(desc + 24), dstride = ram_r32(desc + 28);
				LOGMASKED(LOG_DMA, "dcdma ch%d desc %08x: %08x -> %08x count %d (%08x %08x)\n", ch, desc, src, dst, count, ram_r32(desc), ram_r32(desc + 4));
				for (u32 i = 0; i < count; i++)
				{
					const u32 d = dst + i * dstride;
					const u16 v = ram_r16(src + i * sstride);
					if ((d & 0xfff00000) == 0xdc000000)
						m_dsp_mem[((d - 0xdc000000) >> 1) & 0x7ffff] = v;
					else
						ram_w16(d, v);
				}
				desc = next;
			}
			m_dcdma_regs[0x18 / 4] &= ~(1 << ch);
		}
	}
}


/***************************************************************************
    CD sector decoder and servo serial link (0xC4000000)

    The servo DSP (Sony CXD3059AR) is modelled at the level the ROM's MAIKO
    driver observes it: SENS answers for the status selects it polls, a head
    position in absolute frames (LBA + 150, lead-in negative) moved by sled
    and track-jump commands, and a subcode-Q stream. The decoder streams user
    data from the CD image to RAM.

    Servo link: +0x00 TX bit count, +0x04 RX bit count, +0x08/+0x0C TX word
    (command byte is the most significant byte of the n-bit word), +0x10 RX,
    +0x14 control (bit 0 start, bit 1 read, bit 3 latch SENS), +0x18 bit 1 SENS.
    Decoder: +0x1C start MSF (BCD, bit 24 = continue), +0x20 length (bit 24 =
    unlimited), +0x24 destination / write pointer, +0x28/+0x2C ring,
    +0x3C record queue (4 x 1 KB), +0x40 control, +0x54-+0x5C subcode Q,
    +0x64 IRQ status (W1C), +0x68 IRQ enable, +0x70/+0x74 queue status/control,
    +0x80/+0x84 watermark.
    IRQ bits: 0x1/0x2 servo write/read done, 0x10 transfer end, 0x20 subcode Q,
    0x1000 record queued, 0x10000 watermark reached.
***************************************************************************/

static constexpr s32 CD_HOME = -3000; // sled inner limit, in the lead-in

static u8 to_bcd(int v) { return ((v / 10) << 4) | (v % 10); }
static int from_bcd(u8 v) { return (v >> 4) * 10 + (v & 15); }

static u32 frames_to_bcd_msf(s32 f)
{
	if (f < 0)
		f = 0;
	return (to_bcd(f / (75 * 60)) << 16) | (to_bcd((f / 75) % 60) << 8) | to_bcd(f % 75);
}

// spiral track number for an absolute frame (the ROM's Servo_framesToTrackNo)
static s32 cd_track_of(s32 l)
{
	const double v = std::floor(double(l) * 3448.3571 + 0.5) + 244140625.0;
	return s32(std::floor(std::sqrt(std::max(v, 0.0)))) - 15625;
}

static s32 cd_frame_of(s32 t)
{
	const double tt = double(t) + 15625.0;
	return s32((tt * tt - 244140625.0) / 3448.3571);
}

void vsmilpro_state::cd_update_irq()
{
	set_irq(IRQ_CD, (m_cd_regs[0x64 / 4] & m_cd_regs[0x68 / 4]) != 0);
}

bool vsmilpro_state::cd_sens(u8 select)
{
	switch (select & 0xf0)
	{
	case 0x30: return m_cd_head <= CD_HOME;  // sled inner limit switch
	case 0x40: return true;                  // XBUSY: sequence finished
	case 0x50: return m_cdrom->exists();     // FOK
	case 0xa0: return m_cdrom->exists();     // GFS
	default: return false;                   // 0xB0 traverse count reached, 0xE0 spindle
	}
}

void vsmilpro_state::cd_jump_tracks(s32 tracks)
{
	m_cd_head = cd_frame_of(cd_track_of(m_cd_head) + tracks);
}

void vsmilpro_state::cd_servo_write(int nbits, u64 value, bool sens)
{
	if (nbits <= 0 || nbits > 40)
		return;
	const u8 cmd = u8(value >> (nbits - 8));
	switch (nbits)
	{
	case 8:
		if (cmd == 0x20)
			m_cd_sled_armed = false;
		else if (cmd == 0x22) // sled out
		{
			m_cd_sled_armed = true;
			m_cd_dir = 1;
			m_cd_head = std::max(m_cd_head, CD_HOME + 1000);
		}
		else if (cmd == 0x23) // sled in
		{
			m_cd_sled_armed = true;
			m_cd_dir = -1;
			m_cd_head = std::max(CD_HOME, m_cd_head - 1000);
		}
		break;
	case 16:
		switch (cmd)
		{
		case 0x48: cd_jump_tracks(1); break;
		case 0x49: cd_jump_tracks(-1); break;
		case 0x4a: cd_jump_tracks(10); break;
		case 0x4b: cd_jump_tracks(-10); break;
		case 0x4c: cd_jump_tracks(2 * m_cd_jump_n); break;
		case 0x4d: cd_jump_tracks(-2 * m_cd_jump_n); break;
		}
		break;
	case 24:
		if ((cmd & 0xf0) == 0x70)
			m_cd_jump_n = (value >> 4) & 0xffff;
		break;
	case 32:
		// traverse count for a long sled move; only meaningful right after a
		// $22/$23 sled command (the init sequence also writes $B0 values)
		if ((cmd & 0xf0) == 0xb0 && m_cd_sled_armed)
		{
			m_cd_sled_armed = false;
			const s32 c = (value >> 8) & 0xfffff;
			const s32 tracks = (c <= 2399) ? c * 100 / 60 : c * 100 / 90;
			cd_jump_tracks(m_cd_dir * tracks);
		}
		break;
	}
	if (sens)
	{
		m_cd_regs[0x18 / 4] = (m_cd_regs[0x18 / 4] & ~2) | (cd_sens(cmd) ? 2 : 0);
		LOGMASKED(LOG_CD, "cd sens %02x = %d (head %d)\n", cmd, BIT(m_cd_regs[0x18 / 4], 1), m_cd_head);
	}
}

// subcode Q for the current head position
void vsmilpro_state::cd_latch_subq()
{
	const s32 l = m_cd_head;
	u8 q[10];
	const int ntracks = m_cdrom->get_last_track();
	if (l < 0)
	{
		// lead-in: TOC entries A0, A1, A2, then each track, every point ~3 times
		const int npoints = 3 + ntracks;
		const int point = ((-l) / 3) % npoints;
		const u32 runtime = frames_to_bcd_msf(-l);
		q[0] = (m_cdrom->get_track_type(0) == cdrom_file::CD_TRACK_AUDIO) ? 0x01 : 0x41;
		q[1] = 0x00;
		q[3] = runtime >> 16; q[4] = runtime >> 8; q[5] = runtime;
		q[6] = 0;
		u32 p;
		if (point == 0)
		{
			q[2] = 0xa0;
			p = to_bcd(1) << 16;
		}
		else if (point == 1)
		{
			q[2] = 0xa1;
			p = to_bcd(ntracks) << 16;
		}
		else if (point == 2)
		{
			q[2] = 0xa2;
			p = frames_to_bcd_msf(m_cdrom->get_track_start(0xaa) + 150);
		}
		else
		{
			const int t = point - 3;
			q[0] = (m_cdrom->get_track_type(t) == cdrom_file::CD_TRACK_AUDIO) ? 0x01 : 0x41;
			q[2] = to_bcd(t + 1);
			p = frames_to_bcd_msf(m_cdrom->get_track_start(t) + 150);
		}
		q[7] = p >> 16; q[8] = p >> 8; q[9] = p;
	}
	else
	{
		const s32 lba = l - 150;
		const u32 leadout = m_cdrom->get_track_start(0xaa);
		int track;
		if (lba >= s32(leadout))
			track = -1;
		else if (lba < 0)
			track = 0;
		else
			track = m_cdrom->get_track(lba);
		const u32 start = (track >= 0) ? m_cdrom->get_track_start(track) : leadout;
		q[0] = (track >= 0 && m_cdrom->get_track_type(track) == cdrom_file::CD_TRACK_AUDIO) ? 0x01 : 0x41;
		q[1] = (track >= 0) ? to_bcd(track + 1) : 0xaa;
		q[2] = (lba < s32(start)) ? 0x00 : 0x01;
		const u32 rel = frames_to_bcd_msf(std::abs(lba - s32(start)));
		q[3] = rel >> 16; q[4] = rel >> 8; q[5] = rel;
		q[6] = 0;
		const u32 abs = frames_to_bcd_msf(l);
		q[7] = abs >> 16; q[8] = abs >> 8; q[9] = abs;
	}
	m_cd_regs[0x54 / 4] = 0x10000 | (q[0] << 8) | q[1];
	m_cd_regs[0x58 / 4] = (q[2] << 24) | (q[3] << 16) | (q[4] << 8) | q[5];
	m_cd_regs[0x5c / 4] = (q[6] << 24) | (q[7] << 16) | (q[8] << 8) | q[9];
}

void vsmilpro_state::cd_queue_push(u32 data_addr, s32 frame)
{
	u32 st = m_cd_regs[0x70 / 4];
	const int head = st & 3, count = (st >> 2) & 7;
	if (count >= 4)
		return;
	const int slot = (head + count) & 3;
	const u32 rec = m_cd_regs[0x3c / 4] + slot * 1024;
	const u32 msf = frames_to_bcd_msf(frame);
	ram_w32(rec + 0, 1);
	ram_w32(rec + 4, data_addr);
	ram_w8(rec + 20, msf >> 16);
	ram_w8(rec + 21, msf >> 8);
	ram_w8(rec + 22, msf);
	ram_w8(rec + 23, 1); // mode 1
	m_cd_regs[0x70 / 4] = (st & ~0x1c) | ((count + 1) << 2);
	m_cd_regs[0x64 / 4] |= 0x1000;
}

TIMER_CALLBACK_MEMBER(vsmilpro_state::cd_sector_cb)
{
	if (!m_cdrom->exists() || !(m_cd_regs[0x40 / 4] & 1))
		return;

	const u32 ctrl = m_cd_regs[0x40 / 4];
	if ((ctrl & 4) && m_cd_running)
	{
		// deliver the sector under the head
		const u32 mode = ctrl & 0x18;
		const bool capture = (mode == 0x08) && (ctrl & 0x80);
		const s32 lba = m_cd_head - 150;
		u32 size;
		if (capture)
		{
			size = 4;
			const u32 msf = frames_to_bcd_msf(m_cd_head);
			ram_w8(m_cd_ptr + 0, msf >> 16);
			ram_w8(m_cd_ptr + 1, msf >> 8);
			ram_w8(m_cd_ptr + 2, msf);
			ram_w8(m_cd_ptr + 3, 1);
			cd_queue_push(m_cd_ptr, m_cd_head);
		}
		else
		{
			u8 raw[2352];
			std::fill(std::begin(raw), std::end(raw), 0);
			if (mode == 0x18)
			{
				size = 2048;
				if (lba >= 0)
					m_cdrom->read_data(lba, raw, cdrom_file::CD_TRACK_MODE1);
			}
			else
			{
				size = (mode == 0x10) ? 2336 : 2352;
				if (lba >= 0)
					m_cdrom->read_data(lba, raw, cdrom_file::CD_TRACK_RAW_DONTCARE);
				if (size == 2336)
					memmove(raw, raw + 16, 2336);
			}
			for (u32 i = 0; i < size; i++)
				ram_w8(m_cd_ptr + i, raw[i]);
		}
		LOGMASKED(LOG_CD, "cd sector lba %d -> %08x (%d bytes)\n", lba, m_cd_ptr, size);

		m_cd_ptr += size;
		if ((ctrl & 0x40) && m_cd_ptr > m_cd_regs[0x2c / 4])
			m_cd_ptr = m_cd_regs[0x28 / 4];

		if (!(m_cd_regs[0x20 / 4] & 0x1000000))
		{
			m_cd_len -= size;
			if (m_cd_len <= 0)
			{
				m_cd_running = false;
				m_cd_regs[0x64 / 4] |= 0x10;
			}
		}
		if (m_cd_running && m_cd_ptr == m_cd_regs[0x80 / 4])
		{
			if (m_cd_regs[0x84 / 4] & 1)
			{
				m_cd_running = false;
				m_cd_regs[0x64 / 4] |= 0x10;
			}
			else
				m_cd_regs[0x64 / 4] |= 0x10000;
		}
	}

	m_cd_head++;
	cd_latch_subq();
	m_cd_regs[0x64 / 4] |= 0x20;
	cd_update_irq();
}

u32 vsmilpro_state::cd_r(offs_t offset)
{
	switch (offset)
	{
	case 0x10 / 4: return 0; // servo RX data: calibration monitor reads accept anything
	case 0x24 / 4: return m_cd_ptr;
	default:
		return m_cd_regs[offset];
	}
}

void vsmilpro_state::cd_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x64 / 4:
		m_cd_regs[offset] &= ~data;
		cd_update_irq();
		return;
	case 0x14 / 4:
		COMBINE_DATA(&m_cd_regs[offset]);
		if (data & 1)
		{
			const bool read = BIT(data, 1);
			const int nbits = m_cd_regs[0];
			const u64 value = (u64(m_cd_regs[3]) << 32 | m_cd_regs[2]) & util::make_bitmask<u64>(std::min(nbits, 64));
			LOGMASKED(LOG_CD, "%08x: cd servo %s %d bits tx %09x\n", m_maincpu->pc(), read ? "read" : "write", nbits, value);
			if (!read)
				cd_servo_write(nbits, value, BIT(m_cd_regs[0x14 / 4], 3));
			m_cd_regs[0x14 / 4] &= ~3; // start and read bits clear when the transfer completes
			m_cd_regs[0x64 / 4] |= read ? 2 : 1;
			cd_update_irq();
		}
		return;
	case 0x24 / 4:
		COMBINE_DATA(&m_cd_regs[offset]);
		m_cd_ptr = m_cd_regs[offset];
		return;
	case 0x40 / 4:
	{
		const u32 old = m_cd_regs[offset];
		COMBINE_DATA(&m_cd_regs[offset]);
		LOGMASKED(LOG_CD, "%08x: cd control %08x\n", m_maincpu->pc(), m_cd_regs[offset]);
		if (!(old & 4) && (m_cd_regs[offset] & 4))
		{
			const u32 start = m_cd_regs[0x1c / 4];
			if (!(start & 0x1000000))
				m_cd_head = (from_bcd(start >> 16) * 60 + from_bcd(start >> 8)) * 75 + from_bcd(start);
			m_cd_ptr = m_cd_regs[0x24 / 4];
			m_cd_len = m_cd_regs[0x20 / 4] & 0xffffff;
			m_cd_running = true;
			LOGMASKED(LOG_CD, "cd run from frame %d to %08x len %06x\n", m_cd_head, m_cd_ptr, m_cd_len);
		}
		else if (!(m_cd_regs[offset] & 4) && m_cd_running)
		{
			// stopping an active transfer completes it with a transfer-end
			// interrupt; Cd_startRead relies on this to restart with a new request
			m_cd_running = false;
			m_cd_regs[0x64 / 4] |= 0x10;
			cd_update_irq();
		}
		return;
	}
	case 0x74 / 4:
	{
		u32 st = m_cd_regs[0x70 / 4];
		int head = st & 3, count = (st >> 2) & 7;
		if ((data & 1) && count)
		{
			head = (head + 1) & 3;
			count--;
		}
		if (data & 2)
			count = 0;
		m_cd_regs[0x70 / 4] = (st & ~0x1f) | (count << 2) | head;
		m_cd_regs[0x64 / 4] &= ~0x1000;
		cd_update_irq();
		return;
	}
	default:
		COMBINE_DATA(&m_cd_regs[offset]);
		LOGMASKED(LOG_CD, "%08x: cd %02x = %08x\n", m_maincpu->pc(), offset * 4, data);
		cd_update_irq();
		return;
	}
}


/***************************************************************************
    Unknown blocks
***************************************************************************/

u32 vsmilpro_state::unk_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_UNKNOWN, "%08x: unknown read\n", m_maincpu->pc());
	return 0;
}

void vsmilpro_state::unk_w(offs_t offset, u32 data, u32 mem_mask)
{
}


/***************************************************************************
    Address map
***************************************************************************/

void vsmilpro_state::vsmilpro_map(address_map &map)
{
	map(0x00000000, 0x001fffff).rom().region("maincpu", 0);
	map(0x10000000, 0x10ffffff).ram().share("sdram");

	map(0x8fff0000, 0x8fff001f).noprw(); // SDRAM controller

	map(0x90000000, 0x900000ff).rw(FUNC(vsmilpro_state::gpio_r<0>), FUNC(vsmilpro_state::gpio_w<0>));
	map(0x90010000, 0x9001003f).noprw(); // second timer unit, unused
	map(0x90020000, 0x9002001f).rw(FUNC(vsmilpro_state::uart_r<0>), FUNC(vsmilpro_state::uart_w<0>));
	map(0x90030000, 0x9003001f).rw(FUNC(vsmilpro_state::uart_r<1>), FUNC(vsmilpro_state::uart_w<1>));
	map(0x90080000, 0x9008003f).noprw(); // write-once key
	map(0x90090000, 0x900900ff).rw(FUNC(vsmilpro_state::rtc_r), FUNC(vsmilpro_state::rtc_w));
	map(0x900a0000, 0x900a0fff).rw(FUNC(vsmilpro_state::sysctl_r), FUNC(vsmilpro_state::sysctl_w));
	map(0x900b0000, 0x900b003f).rw(FUNC(vsmilpro_state::pll_r), FUNC(vsmilpro_state::pll_w));
	map(0x900c0000, 0x900c003f).rw(FUNC(vsmilpro_state::timer_r), FUNC(vsmilpro_state::timer_w));
	map(0x900d0000, 0x900d00ff).rw(FUNC(vsmilpro_state::gpio_r<1>), FUNC(vsmilpro_state::gpio_w<1>));

	map(0xa0000000, 0xa00fffff).lr32(NAME([] () { return 0; })); // nothing on the factory bus
	map(0xa1000000, 0xa100003f).rw(FUNC(vsmilpro_state::spi_r), FUNC(vsmilpro_state::spi_w));
	map(0xa4000000, 0xa400003f).rw(FUNC(vsmilpro_state::gpucvt_r), FUNC(vsmilpro_state::gpucvt_w));
	map(0xa8000000, 0xa80003ff).rw(FUNC(vsmilpro_state::gpu_r), FUNC(vsmilpro_state::gpu_w));
	map(0xb0000000, 0xb0001fff).rw(m_spu, FUNC(zevio_spu_device::read), FUNC(zevio_spu_device::write));
	map(0xb8000000, 0xb8002fff).rw(FUNC(vsmilpro_state::disp_r), FUNC(vsmilpro_state::disp_w)); // +0x800-0xFFF on-chip SRAM / palettes
	map(0xbc000000, 0xbc0001ff).rw(FUNC(vsmilpro_state::bcdma_r), FUNC(vsmilpro_state::bcdma_w));
	map(0xc0000000, 0xc00001ff).rw(FUNC(vsmilpro_state::dcdma_r), FUNC(vsmilpro_state::dcdma_w));
	map(0xc4000000, 0xc40000ff).rw(FUNC(vsmilpro_state::cd_r), FUNC(vsmilpro_state::cd_w));
	map(0xdc000000, 0xdc0003ff).rw(FUNC(vsmilpro_state::intc_r), FUNC(vsmilpro_state::intc_w));
	map(0xdc020000, 0xdc0fffff).lrw16(
			NAME([this] (offs_t offset) { return m_dsp_mem[offset & 0x7ffff]; }),
			NAME([this] (offs_t offset, u16 data, u16 mem_mask) { COMBINE_DATA(&m_dsp_mem[offset & 0x7ffff]); }));
	map(0xf8000000, 0xf8001fff).rw(FUNC(vsmilpro_state::unk_r), FUNC(vsmilpro_state::unk_w)); // IRQ 20 block (USB?)
}


/***************************************************************************
    Machine
***************************************************************************/

void vsmilpro_state::machine_start()
{
	m_sflash = std::make_unique<u8[]>(SFLASH_SIZE);
	std::fill_n(&m_sflash[0], SFLASH_SIZE, 0xff);
	if (m_spi_region)
		std::copy_n(m_spi_region->base(), std::min<u32>(m_spi_region->bytes(), SFLASH_SIZE), &m_sflash[0]);
	m_nvram->set_base(&m_sflash[0], SFLASH_SIZE);

	m_dsp_mem = std::make_unique<u16[]>(0x80000);
	m_spu->set_ram(m_sdram.target(), SDRAM_BASE, SDRAM_SIZE);

	m_tick_timer = timer_alloc(FUNC(vsmilpro_state::tick_cb), this);
	for (auto &t : m_bcdma_timer)
		t = timer_alloc(FUNC(vsmilpro_state::bcdma_done_cb), this);
	save_item(NAME(m_bcdma_busy));
	m_ch1_wrap_timer = timer_alloc(FUNC(vsmilpro_state::ch1_wrap_cb), this);
	m_line_timer = timer_alloc(FUNC(vsmilpro_state::line_cb), this);
	m_joy_timer = timer_alloc(FUNC(vsmilpro_state::joy_cb), this);
	m_cd_timer = timer_alloc(FUNC(vsmilpro_state::cd_sector_cb), this);

	// Speedup: the boot ROM's idle task (0x1001098C) is an endless counting
	// loop whose counter lives at 0x101E05E4; sleep until the next interrupt
	// TODO: equivalent for FA's idle task
	m_maincpu->space(AS_PROGRAM).install_write_tap(0x101e05e4, 0x101e05e7, "idle_tap",
			[this] (offs_t offset, u32 &data, u32 mem_mask)
			{
				const u32 pc = m_maincpu->pc();
				if (pc >= 0x10010990 && pc <= 0x100109cc && ram_r32(0x100109a0) == 0xe58d3000)
					m_maincpu->spin_until_interrupt();
			});

	save_item(NAME(m_int_raw));
	save_item(NAME(m_int_pending));
	save_item(NAME(m_int_enable));
	save_item(NAME(m_int_regs));
	save_item(NAME(m_sysctl));
	save_item(NAME(m_pll));
	save_item(NAME(m_rtc));
	save_item(NAME(m_gpio_dir));
	save_item(NAME(m_gpio_out));
	save_item(NAME(m_timer_regs));
	save_item(NAME(m_spi_regs));
	save_item(NAME(m_gpu_regs));
	save_item(NAME(m_disp_regs));
	save_item(NAME(m_disp_status));
	save_item(NAME(m_bcdma_regs));
	save_item(NAME(m_dcdma_regs));
	save_item(NAME(m_cd_regs));
	save_item(NAME(m_cd_head));
	save_item(NAME(m_cd_dir));
	save_item(NAME(m_cd_jump_n));
	save_item(NAME(m_cd_ptr));
	save_item(NAME(m_cd_len));
	save_item(NAME(m_cd_running));
	save_pointer(NAME(m_dsp_mem), 0x80000);
}

void vsmilpro_state::machine_reset()
{
	m_int_raw = m_int_pending = 0;
	m_int_enable[0] = m_int_enable[1] = 0;
	std::fill(std::begin(m_int_regs), std::end(m_int_regs), 0);
	std::fill(std::begin(m_sysctl), std::end(m_sysctl), 0);
	std::fill(std::begin(m_pll), std::end(m_pll), 0);
	std::fill(std::begin(m_rtc), std::end(m_rtc), 0);
	// The always-on block survives power-off; the ROM marks it with 0xC01DB007
	// after the first cold start and otherwise powers down until a key press
	m_rtc[0x88 / 4] = 0xc01db007;
	// +0x8C = 0x1230FFAB records a power-key wake-up; without it the startup
	// task treats the boot as spurious and returns to standby
	m_rtc[0x8c / 4] = 0x1230ffab;
	for (int b = 0; b < 2; b++)
		for (int l = 0; l < 4; l++)
			m_gpio_dir[b][l] = m_gpio_out[b][l] = 0;
	std::fill(std::begin(m_timer_regs), std::end(m_timer_regs), 0);
	std::fill(std::begin(m_spi_regs), std::end(m_spi_regs), 0);
	std::fill(std::begin(m_gpu_regs), std::end(m_gpu_regs), 0);
	std::fill(std::begin(m_gpu_state), std::end(m_gpu_state), 0);
	std::fill(std::begin(m_gpu_cvt), std::end(m_gpu_cvt), 0);
	std::fill(std::begin(m_disp_regs), std::end(m_disp_regs), 0);
	std::fill(std::begin(m_bcdma_regs), std::end(m_bcdma_regs), 0);
	m_bcdma_busy = 0;
	for (auto &t : m_bcdma_timer)
		t->adjust(attotime::never);
	std::fill(std::begin(m_dcdma_regs), std::end(m_dcdma_regs), 0);
	std::fill(std::begin(m_cd_regs), std::end(m_cd_regs), 0);
	m_gpu_clip[0] = 0;
	m_gpu_clip[1] = 0xffffff;
	m_gpu_offset = m_gpu_texmode = m_gpu_texpos = m_gpu_clutpos = m_gpu_xfer_addr = 0;
	m_disp_status = 0;
	m_sflash_status = 0;
	m_sflash_aai_addr = 0;
	m_ch1_running = false;
	m_ch1_period_ticks = 1;
	for (auto &u : m_uart)
	{
		u.ier = u.lcr = u.mcr = u.scr = u.dll = u.dlm = u.fcr = 0;
		u.thre_pending = false;
		u.rx.clear();
		u.line.clear();
	}
	m_tick_timer->adjust(attotime::never);
	m_ch1_wrap_timer->adjust(attotime::never);
	m_line_timer->adjust(m_screen->time_until_pos(0));
	for (auto &j : m_joy)
	{
		j.cmd.clear();
		j.detected = false;
		memset(j.last, 0, sizeof(j.last));
		j.last_sent = attotime::zero;
	}
	m_joy_timer->adjust(attotime::from_hz(100), 0, attotime::from_hz(100));
	m_cd_head = -3000;
	m_cd_dir = 1;
	m_cd_jump_n = 0;
	m_cd_sled_armed = false;
	m_cd_ptr = 0;
	m_cd_len = 0;
	m_cd_running = false;
	m_cd_timer->adjust(attotime::from_hz(75 * 4), 0, attotime::from_hz(75 * 4)); // TODO: 1x/4x speed switching
}


static INPUT_PORTS_START( vsmilpro )
	PORT_START("KEYS") // 0x900A0018; active level unconfirmed
	PORT_BIT( 0x00000001, IP_ACTIVE_HIGH, IPT_VOLUME_DOWN )
	PORT_BIT( 0x00000002, IP_ACTIVE_HIGH, IPT_VOLUME_UP )
	PORT_BIT( 0x00000100, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_NAME("Panel button 8")
	PORT_BIT( 0x00010000, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_NAME("Panel button 16")
	PORT_BIT( 0x01000000, IP_ACTIVE_HIGH, IPT_BUTTON3 ) PORT_NAME("Panel button 24")
	PORT_BIT( 0x02000000, IP_ACTIVE_HIGH, IPT_POWER_OFF )
	PORT_BIT( 0xfcfefefc, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("STRAPS") // 0x900D0018 bits 0-3, wiring unknown
	// bit 0 set skips the disc task entirely (the ROM then waits on nothing);
	// with it clear the ROM waits for the CD drive init and boots discs
	PORT_DIPNAME( 0x01, 0x00, "Boot mode strap" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On ) )
	PORT_DIPNAME( 0x02, 0x00, "Strap 1" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On ) )
	PORT_DIPNAME( 0x04, 0x00, "Skip intro check" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On ) )
	PORT_DIPNAME( 0x08, 0x00, "Strap 3" )
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On ) )
	PORT_DIPNAME( 0x10, 0x00, "GPIO0 bit 10" ) // read alongside the lid switch
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On ) )

	// controller button word; names give the key code the boot ROM menu assigns
	PORT_START("JOY1_BUTTONS")
	PORT_BIT( 0x0001, IP_ACTIVE_HIGH, IPT_BUTTON9 )  PORT_PLAYER(1) PORT_NAME("P1 Button (code 1)")
	PORT_BIT( 0x0002, IP_ACTIVE_HIGH, IPT_BUTTON10 ) PORT_PLAYER(1) PORT_NAME("P1 Button (code 2)")
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_BUTTON1 )  PORT_PLAYER(1) PORT_NAME("P1 Enter (code 7)")
	PORT_BIT( 0x0008, IP_ACTIVE_HIGH, IPT_BUTTON11 ) PORT_PLAYER(1) PORT_NAME("P1 Button (code 8)")
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_BUTTON6 )  PORT_PLAYER(1) PORT_NAME("P1 Button (code 9)")
	PORT_BIT( 0x0020, IP_ACTIVE_HIGH, IPT_BUTTON7 )  PORT_PLAYER(1) PORT_NAME("P1 Button (code 16)")
	PORT_BIT( 0x0040, IP_ACTIVE_HIGH, IPT_BUTTON8 )  PORT_PLAYER(1) PORT_NAME("P1 Button (code 17)")
	PORT_BIT( 0x0080, IP_ACTIVE_HIGH, IPT_BUTTON12 ) PORT_PLAYER(1) PORT_NAME("P1 Button (code 18)")
	PORT_BIT( 0x0100, IP_ACTIVE_HIGH, IPT_BUTTON13 ) PORT_PLAYER(1) PORT_NAME("P1 Button (code 19)")
	PORT_BIT( 0x0200, IP_ACTIVE_HIGH, IPT_BUTTON14 ) PORT_PLAYER(1) PORT_NAME("P1 Button (code 20)")
	PORT_BIT( 0x0400, IP_ACTIVE_HIGH, IPT_BUTTON15 ) PORT_PLAYER(1) PORT_NAME("P1 Button (code 21)")
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_BUTTON2 )  PORT_PLAYER(1) PORT_NAME("P1 Colour 1 (code 22)")
	PORT_BIT( 0x1000, IP_ACTIVE_HIGH, IPT_BUTTON3 )  PORT_PLAYER(1) PORT_NAME("P1 Colour 2 (code 23)")
	PORT_BIT( 0x2000, IP_ACTIVE_HIGH, IPT_BUTTON4 )  PORT_PLAYER(1) PORT_NAME("P1 Colour 3 (code 24)")
	PORT_BIT( 0x4000, IP_ACTIVE_HIGH, IPT_BUTTON5 )  PORT_PLAYER(1) PORT_NAME("P1 Colour 4 (code 25)")
	PORT_BIT( 0x8000, IP_ACTIVE_HIGH, IPT_BUTTON16 ) PORT_PLAYER(1) PORT_NAME("P1 Button (byte 4)")

	PORT_START("JOY1_STICK")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(1)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(1)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(1)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1)

	PORT_START("JOY2_BUTTONS")
	PORT_BIT( 0x0004, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_PLAYER(2) PORT_NAME("P2 Enter (code 7)")
	PORT_BIT( 0x0800, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_PLAYER(2) PORT_NAME("P2 Colour 1 (code 22)")
	PORT_BIT( 0x1000, IP_ACTIVE_HIGH, IPT_BUTTON3 ) PORT_PLAYER(2) PORT_NAME("P2 Colour 2 (code 23)")
	PORT_BIT( 0x2000, IP_ACTIVE_HIGH, IPT_BUTTON4 ) PORT_PLAYER(2) PORT_NAME("P2 Colour 3 (code 24)")
	PORT_BIT( 0x4000, IP_ACTIVE_HIGH, IPT_BUTTON5 ) PORT_PLAYER(2) PORT_NAME("P2 Colour 4 (code 25)")
	PORT_BIT( 0x87f3, IP_ACTIVE_HIGH, IPT_UNUSED )

	PORT_START("JOY2_STICK")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP )    PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN )  PORT_PLAYER(2)
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT )  PORT_PLAYER(2)
	PORT_BIT( 0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
INPUT_PORTS_END

void vsmilpro_state::vsmilpro(machine_config &config)
{
	ARM9(config, m_maincpu, 150'000'000); // ARM926EJ-S
	m_maincpu->set_addrmap(AS_PROGRAM, &vsmilpro_state::vsmilpro_map);

	NVRAM(config, m_nvram, nvram_device::DEFAULT_NONE);

	SCREEN(config, m_screen);
	m_screen->set_refresh_hz(60);
	m_screen->set_size(320, 262);
	m_screen->set_visarea(0, 320 - 1, 0, 240 - 1);
	m_screen->set_screen_update(FUNC(vsmilpro_state::screen_update));

	SPEAKER(config, "speaker", 2).front();

	ZEVIO_SPU(config, m_spu);
	m_spu->irq_cb().set([this] (int state) { set_irq(IRQ_SOUND, state); });
	m_spu->add_route(0, "speaker", 1.0, 0);
	m_spu->add_route(1, "speaker", 1.0, 1);

	CDROM(config, m_cdrom).set_interface("vsmile_vdisk");

	SOFTWARE_LIST(config, "cd_list").set_original("vsmile_cd");
}

// ROM definition
ROM_START( vsmilpro )
	ROM_REGION( 0x200000, "maincpu", 0 )
	ROM_LOAD( "70004.bin", 0x000000, 0x200000, CRC(b9161eac) SHA1(8d75fdeda8c4e228a0b1efd35011f9f667f9fb23) )
ROM_END

ROM_START( vsmilprof ) // 35-70000-131-203 707399_8 PCB
	ROM_REGION( 0x200000, "maincpu", 0 )
	ROM_LOAD( "a29l160atv.u6", 0x000000, 0x200000, CRC(7a266f2a) SHA1(94337677b2d17eaeef53e010bbd26588f677cca4) )

	ROM_REGION( 0x10000, "spi", 0 )
	ROM_LOAD( "sst25vf512.u5", 0x00000, 0x10000, CRC(a895a5f6) SHA1(4cc246c8c753a7f11a4de76ebc221dc5fe0798c8) )
ROM_END

// "V.Flash Boot Rom ver. 0006 14/9/06"
ROM_START( vflash )
	ROM_REGION( 0x200000, "maincpu", 0 )
	ROM_LOAD( "a29l160atv.bin", 0x000000, 0x200000, CRC(f80ac0ae) SHA1(6ea840280fe411ffb9f571e521fca90a63939ca1) )
ROM_END

} // anonymous namespace


// Driver

//    YEAR  NAME       PARENT    COMPAT  MACHINE   INPUT     CLASS           INIT        COMPANY  FULLNAME                 FLAGS
COMP( 2007, vsmilpro,  0,        0,      vsmilpro, vsmilpro, vsmilpro_state, empty_init, "VTech", "V.Smile Pro (Germany)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
COMP( 2007, vsmilprof, vsmilpro, 0,      vsmilpro, vsmilpro, vsmilpro_state, empty_init, "VTech", "V.Smile Pro (France)",  MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
COMP( 2006, vflash,    vsmilpro, 0,      vsmilpro, vsmilpro, vsmilpro_state, empty_init, "VTech", "V.Flash (USA)",         MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
