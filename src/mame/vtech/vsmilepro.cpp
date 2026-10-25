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

#include "zevio_dsp.h"
#include "zevio_gpu.h"
#include "zevio_spu.h"

#include "cpu/arm7/arm7.h"
#include "imagedev/cdromimg.h"
#include "machine/nvram.h"

#include "cdrom.h"
#include "multibyte.h"
#include "osdfile.h"
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
#define LOG_CDR     (1U << 12)

#define VERBOSE (LOG_UNKNOWN)
#include "logmacro.h"

#include <bit>
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
		m_cardnv(*this, "memcard"),
		m_spu(*this, "spu"),
		m_gpu(*this, "gpu"),
		m_dsp(*this, "dsp"),
		m_sdram(*this, "sdram"),
		m_spi_region(*this, "spi"),
		m_io_keys(*this, "KEYS"),
		m_io_straps(*this, "STRAPS"),
		m_io_conf(*this, "CONF"),
		m_io_joy_buttons(*this, "JOY%u_BUTTONS", 1U),
		m_io_joy_stick(*this, "JOY%u_STICK", 1U)
	{ }

	void vsmilpro(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	void soc_reset();

private:
	static constexpr u32 SDRAM_BASE = 0x10000000;
	static constexpr u32 SDRAM_SIZE = 0x01000000;
	static constexpr u32 VRAM_ORIGIN = 0x0ffd0000;
	// display +0x010 at power-on: bit 16 set (16-bit layer tiled). Inferred: no
	// game code writes the register, and the boot ROM only ever clears bit 16
	static constexpr u32 DISP_010_RESET = 0x00010000;
	static constexpr u32 PERIPH_CLOCK = 37'500'000; // PLL out3 at boot settings
	static constexpr u32 SFLASH_SIZE = 0x10000;
	// AT45DB642D memory card: 8192 pages of 1056 bytes, or 1024 in power-of-2 mode
	static constexpr u32 CARD_PAGES = 8192;
	static constexpr u32 CARD_PAGE_MAX = 1056;

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
		u8 lsr_err;     // overrun (bit 1), cleared by reading LSR
		std::deque<u8> rx;
		std::string line;
	};

	required_device<arm9_cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<cdrom_image_device> m_cdrom;
	required_device<nvram_device> m_nvram;
	required_device<nvram_device> m_cardnv;
	required_device<zevio_spu_device> m_spu;
	required_device<zevio_gpu_device> m_gpu;
	required_device<zevio_dsp_hle_device> m_dsp;
	required_shared_ptr<u32> m_sdram;
	optional_memory_region m_spi_region;
	required_ioport m_io_keys;
	required_ioport m_io_straps;
	required_ioport m_io_conf;
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
	u32 m_int_pending;
	u32 m_int_latched;  // edge sources
	static constexpr u32 EDGE_SOURCES = (1U << 17) | (1U << 18);
	u32 m_int_enable[2];
	u32 m_int_regs[0x400 / 4];
	u8 m_int_limit[2];  // +0x02C: only sources with a smaller +0x300 priority pass
	u8 m_int_prev[2];   // +0x028: the limit before the last +0x024 acknowledge
	int int_current(int group) const;
	void set_irq(int source, int state);
	void update_cpu_irqs();
	u32 intc_r(offs_t offset);
	void intc_w(offs_t offset, u32 data, u32 mem_mask);

	// system control / PLL / RTC / GPIO
	u32 m_sysctl[0x1000 / 4];
	bool m_resume;
	u32 m_pll[0x40 / 4];
	u32 m_rtc[0x100 / 4];
	u32 m_gpio_dir[2][4], m_gpio_out[2][4], m_gpio_irqcfg[2][4];
	u32 sysctl_r(offs_t offset);
	void sysctl_w(offs_t offset, u32 data, u32 mem_mask);
	u32 pll_r(offs_t offset);
	void pll_w(offs_t offset, u32 data, u32 mem_mask);
	u32 rtc_r(offs_t offset);
	void rtc_w(offs_t offset, u32 data, u32 mem_mask);
	template <int Bank> u32 gpio_r(offs_t offset);
	template <int Bank> void gpio_w(offs_t offset, u32 data, u32 mem_mask);

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

	// serial link for homebrew loaders: VFLASH_SERIAL=<controller port 1|2>:<tcp port> listens on
	// 127.0.0.1:<tcp port>; the UART's TX goes to the socket and bytes from the socket arrive at the UART's
	// baud rate into its 16-byte FIFO (overrun when the CPU does not keep up). No controller on that port.
	int m_ser_port;
	osd_file::ptr m_ser_file;
	std::deque<u8> m_ser_in;
	emu_timer *m_ser_timer;
	TIMER_CALLBACK_MEMBER(ser_cb);

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
	std::unique_ptr<u8[]> m_card;           // CARD_PAGES * CARD_PAGE_MAX bytes, page p at p * CARD_PAGE_MAX
	u8 m_card_buf[2][CARD_PAGE_MAX];
	u8 m_card_pow2;                         // 1 = 1024-byte pages (one-time setting on the real chip)
	u32 card_page_size() const { return m_card_pow2 ? 1024 : 1056; }
	void card_decode(const std::vector<u8> &cmd, u32 &page, u32 &byte) const;


	// display
	u32 m_disp_regs[0x3000 / 4];
	u32 m_disp_shown[0xc00 / 4] = {}, m_disp_prev[0xc00 / 4] = {}; // display registers as of the previous frame
	// layer-5 YCbCr planes: V - Y of the last set written whole, and whether the newest Y came without its V
	u32 m_l5_yv_delta = 0;
	bool m_l5_v_missing = false;
	u64 m_disp_frame = ~0ULL;
	bool m_disp_delayed = false;
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
	u8 m_bcdma_irq_on_done[8];
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
	s32 m_cd_expect;    // next frame the decoder accepts (header match)
	bool m_cd_mute;     // $A audio control MUTE: no data reaches the decoder
	u32 m_cd_tmo;       // sector timeout countdown armed by +0x60 bits 31-24
	s32 m_cd_dir;       // direction of the last sled command
	s32 m_cd_jump_n;    // 2N track jump count from the last $7x command
	bool m_cd_sled_armed;
	u8 m_cd_cal;         // last $3AA8xx calibration value
	u32 m_cd_rx;        // servo read result
	u32 m_cd_ptr;       // decoder write pointer
	u32 m_cd_ptr2;      // second write pointer (dual buffer mode)
	s32 m_cd_len;
	bool m_cd_running;
	bool m_cd_found;    // a sector was taken since the run bit went up
	emu_timer *m_cd_timer;
	bool cd_sens(u8 select);
	void cd_jump_tracks(s32 tracks);
	void cd_servo_write(int nbits, u64 value, bool sens);
	void cd_latch_subq();
	bool cd_read_raw(s32 lba, u8 *raw, bool parity);
	bool cd_queue_push(u32 data_addr, const u8 *raw);
	void cd_ram_write(u32 addr, const u8 *src, u32 size, bool ring);
	TIMER_CALLBACK_MEMBER(cd_sector_cb);
	void line_timer_arm(int line);
	void prof_dump()
	{
		std::vector<std::pair<u32, u32>> v;
		u64 total = 0;
		for (auto &e : m_prof) { v.emplace_back(e.second, e.first); total += e.second; }
		std::sort(v.rbegin(), v.rend());
		for (int i = 0; i < std::min<int>(v.size(), getenv("ZPROF_N") ? atoi(getenv("ZPROF_N")) : 30); i++)
			printf("PROF %08x %5.1f%%\n", v[i].second, 100.0 * v[i].first / std::max<u64>(total, 1));
		const double secs = std::max(machine().time().as_double() - m_prof_start, 0.001);
		for (int i = 0; i < 32; i++)
			if (m_prof_irq[i])
				printf("IRQ %2d serviced %u (%.0f/s)\n", i, m_prof_irq[i], m_prof_irq[i] / secs);
	}
	bool m_prof_on = false;
	int m_irq_line = -1, m_fiq_line = -1; // last state given to the CPU (-1: none yet)
	double m_prof_start = 0;
	std::map<u32, u32> m_prof;
	u32 m_prof_irq[32] = { };

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
	// Most sources are level sensitive: the ROM's tick handler only
	// acknowledges the timer block, never the controller. The DSP mailbox
	// sources latch on a rising edge until written to +0x04 (software sets
	// 0xDC000204 bits 17/18 for them; FA counts each IRQ 17 as one message).
	const u32 bit = 1U << source;
	// ZINT_STATS: count rising edges per source, printed at exit
	static u64 s_int_edges[32];
	static struct int_stats_dump { ~int_stats_dump() {
		if (getenv("ZINT_STATS"))
			for (int i = 0; i < 32; i++)
				if (s_int_edges[i])
					printf("IRQ source %2d: %llu raised%c", i, (unsigned long long)s_int_edges[i], 10);
	} } s_int_stats_dump;
	if (state && !(m_int_raw & bit))
		s_int_edges[source & 31]++;
	if (state && !(m_int_raw & bit) && (EDGE_SOURCES & bit))
		m_int_latched |= bit;
	if (state)
		m_int_raw |= bit;
	else
		m_int_raw &= ~bit;
	m_int_pending = (m_int_raw & ~EDGE_SOURCES) | m_int_latched;
	update_cpu_irqs();
}

// The pending, enabled source with the most urgent (smallest) priority below
// the group's limit; ties go to the lower source number. -1 if none.
// FA's handler (10A0BC20) reads +0x024, keeps +0x028, re-enables CPU IRQs
// while its handlers run and writes the kept value back to +0x02C at exit:
// the acknowledge raises the limit to the source's priority, so only more
// urgent sources nest, and the exit restores it.
int vsmilpro_state::int_current(int group) const
{
	int cur = -1, limit = m_int_limit[group];
	for (u32 p = m_int_pending & m_int_enable[group]; p; p &= p - 1)
	{
		const int i = std::countr_zero(p);
		if (int(m_int_regs[0x300 / 4 + i] & 7) < limit)
		{
			cur = i;
			limit = m_int_regs[0x300 / 4 + i] & 7;
		}
	}
	return cur;
}

void vsmilpro_state::update_cpu_irqs()
{
	// only touch the lines when they change: each set_input_line synchronizes
	static const bool noprio = getenv("ZINT_NOPRIO") != nullptr;
	const int irq = (noprio ? (m_int_pending & m_int_enable[0]) != 0 : int_current(0) >= 0) ? ASSERT_LINE : CLEAR_LINE;
	const int fiq = (noprio ? (m_int_pending & m_int_enable[1]) != 0 : int_current(1) >= 0) ? ASSERT_LINE : CLEAR_LINE;
	if (irq != m_irq_line)
		m_maincpu->set_input_line(arm7_cpu_device::ARM7_IRQ_LINE, m_irq_line = irq);
	if (fiq != m_fiq_line)
		m_maincpu->set_input_line(arm7_cpu_device::ARM7_FIRQ_LINE, m_fiq_line = fiq);
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
		// acknowledge: the most urgent source, and the limit rises to its priority
		const int g = BIT(offset, 6);
		const int i = int_current(g);
		if (i < 0)
			return 0;
		if (!machine().side_effects_disabled())
		{
			if (m_prof_on)
			{
				if (!m_prof_start)
					m_prof_start = machine().time().as_double();
				m_prof_irq[i]++;
			}
			m_int_prev[g] = m_int_limit[g];
			m_int_limit[g] = m_int_regs[0x300 / 4 + i] & 7;
			update_cpu_irqs();
		}
		return i;
	}
	case 0x028 / 4: case 0x128 / 4: return m_int_prev[BIT(offset, 6)];
	case 0x02c / 4: case 0x12c / 4: return m_int_limit[BIT(offset, 6)];

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
		m_int_latched &= ~data;
		m_int_pending = (m_int_raw & ~EDGE_SOURCES) | m_int_latched;
		break;
	case 0x008 / 4: m_int_enable[0] |= data; break;
	case 0x00c / 4: m_int_enable[0] &= ~data; break;
	case 0x108 / 4: m_int_enable[1] |= data; break;
	case 0x10c / 4: m_int_enable[1] &= ~data; break;
	case 0x02c / 4: case 0x12c / 4: m_int_limit[BIT(offset, 6)] = m_int_regs[offset] & 15; break;
	default:
		LOGMASKED(LOG_INTC, "%08x: intc %03x = %08x\n", m_maincpu->pc(), offset * 4, data);
		break;
	}
	update_cpu_irqs();
}


/***************************************************************************
    System control (0x900A0000), PLL, RTC, GPIO
***************************************************************************/

u32 vsmilpro_state::sysctl_r(offs_t offset)
{
	switch (offset)
	{
	case 0x000 / 4: // status: FA's media player refuses to open a movie unless bit 4 is set
		return m_sysctl[offset] | 0x10;
	case 0x00c / 4: // reset cause: bit 1 = resume from standby, bit 0 = warm restart
	{
		const u32 cause = m_resume ? 2 : 0;
		if (!machine().side_effects_disabled())
			m_resume = false;
		return cause;
	}
	case 0x018 / 4: return m_io_keys->read();
	case 0x01c / 4: return 0;
	case 0x030 / 4: return m_sysctl[offset];
	default:
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
		else if (data == 1)
		{
			// Standby. Games use it to start their OS: vboot installs FA's
			// vectors at 0x1000FF00 and enters the ROM's standby routine; on
			// wake the reset code sees "resume" in +0x0C and jumps to the RAM
			// resume handler. TODO: wake on the power key instead of at once
			logerror("%08x: standby, waking with resume flag\n", m_maincpu->pc());
			m_resume = true;
			// waking resets the on-chip blocks; among others this restores
			// display +0x010 bit 16, which every ROM menu screen clears and FA
			// needs set (it never writes it)
			soc_reset();
			m_maincpu->pulse_input_line(INPUT_LINE_RESET, attotime::zero);
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
	if (offset == 0x020 / 4)
		m_dsp->run_w(m_sysctl[offset]);
	else if (offset == 0x024 / 4)
		m_dsp->windows_w(m_sysctl[offset]);
}

u32 vsmilpro_state::pll_r(offs_t offset)
{
	if (offset == 0x14 / 4)
		return 1; // PLL locked
	return m_pll[offset];
}

void vsmilpro_state::pll_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (getenv("ZPLL_LOG"))
		logerror("%s: pll %02x = %08x\n", machine().describe_context(), offset * 4, data);
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

// four 8-bit lanes per bank at +0x40*lane: +0x10 direction (1 = input, as the
// firmware uses it [FW-backed INT]), +0x14 out, +0x18 pin level, +0x20 a
// register of the pin-interrupt block (firmware read-modify-writes it while
// arming the lid interrupt; it is never read as pins) [FW, system.md GPIO table]
template <int Bank>
u32 vsmilpro_state::gpio_r(offs_t offset)
{
	const int lane = (offset >> 4) & 3;
	switch ((offset & 0xf) * 4)
	{
	case 0x10: return m_gpio_dir[Bank][lane];
	case 0x14: return m_gpio_out[Bank][lane];
	case 0x20: return m_gpio_irqcfg[Bank][lane];
	case 0x18:
	{
		u32 val = m_gpio_out[Bank][lane];
		if (Bank == 1 && lane == 0)
		{
			// bits 0-3 are board straps; bit 4 is an input that games' vboot
			// checks before starting FA (set = refuse to boot, enter standby)
			val = (val & ~0x1f) | (m_io_straps->read() & 0x0f);
		}
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
	case 0x20: m_gpio_irqcfg[Bank][lane] = data & 0xff; break;
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
	case 5:
		ret = 0x60 | (u.rx.empty() ? 0 : 1) | u.lsr_err;
		if (!machine().side_effects_disabled())
			u.lsr_err = 0;
		break;
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
			if (N == m_ser_port)
			{
				if (m_ser_file)
				{
					const u8 b = data;
					u32 actual;
					m_ser_file->write(&b, 0, 1, actual);
				}
			}
			else if (divisor >= 200)
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
	if (port == m_ser_port)
		return;
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
	if (m_prof_on)
	{
		// detuned from the 100 Hz tick so samples are not aligned with its IRQ
		if (m_joy_timer->period() != attotime::from_hz(97))
			m_joy_timer->adjust(attotime::from_hz(97), 0, attotime::from_hz(97));
		// ZPROF_FROM=<seconds> starts sampling later
		static const double from = getenv("ZPROF_FROM") ? atof(getenv("ZPROF_FROM")) : 0.0;
		const u32 pc = m_maincpu->pc();
		if (machine().time().as_double() >= from)
			m_prof[pc < 0x1000 ? pc : (pc & ~0x3f)]++;
	}
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


TIMER_CALLBACK_MEMBER(vsmilpro_state::ser_cb)
{
	uart_t &u = m_uart[m_ser_port];
	if (m_ser_in.empty())
	{
		u8 buf[4096];
		u32 actual = 0;
		if (!m_ser_file->read(buf, 0, sizeof(buf), actual) && actual)
			m_ser_in.insert(m_ser_in.end(), buf, buf + actual);
	}
	if (!m_ser_in.empty())
	{
		if (u.rx.size() >= 16)
			u.lsr_err |= 2; // overrun: the byte is lost
		else
			u.rx.push_back(m_ser_in.front());
		m_ser_in.pop_front();
		uart_update_irq(m_ser_port);
	}
	// one 8N1 character (10 bits) at the programmed rate: f3 / (16 x divisor), f3 = 37.5 MHz
	const u32 divisor = (u.dlm << 8) | u.dll;
	m_ser_timer->adjust(m_ser_in.empty() ? attotime::from_msec(1)
			: attotime::from_ticks(160 * std::max<u32>(divisor, 1), 37'500'000));
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

	// +0x1C and +0x20 are shift registers: +0x1C shifts first (sends bit 31, takes MISO in at bit 0), then +0x20
	// for the bytes after the fourth, so received bytes end up right-aligned in the last word used: a 2-byte
	// frame's byte 1 is the low byte of +0x1C, a 6-byte frame's byte 5 the low byte of +0x20 (HARDWARE.md 13,
	// consistent with every firmware use)
	u32 w0 = m_spi_regs[0x1c / 4], w1 = m_spi_regs[0x20 / 4];
	for (int i = 0; i < nbytes; i++)
	{
		if (i < 4)
			w0 = (w0 << 8) | rx[i];
		else
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
	// SO is not driven during opcode/address bytes and reads 0: FA's AAI
	// program waits for bit 0 to clear after a 2-byte 03 00 frame
	static constexpr u8 UNDRIVEN = 0x00;
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
		return UNDRIVEN;
	}
	case 0x05: // read status
		return index >= 1 ? m_sflash_status : UNDRIVEN;
	case 0x06: m_sflash_status |= 0x02; return UNDRIVEN; // write enable
	case 0x04: m_sflash_status &= ~0x02; return UNDRIVEN; // write disable
	case 0x50: return UNDRIVEN; // enable write status
	case 0x01: if (index == 1) m_sflash_status = (m_sflash_status & 0x03) | (data & 0xbc); return UNDRIVEN;
	case 0x90: case 0xab: // read id: manufacturer, device, repeated
		if (index >= 4)
			return ((index - 4) & 1) ? 0x48 : 0xbf; // SST25VF512A
		return UNDRIVEN;
	case 0x9f: // JEDEC id
		if (index == 1) return 0xbf;
		if (index == 2) return 0x48;
		return UNDRIVEN;
	case 0x02: // byte program
		if (index == 4 && (m_sflash_status & 2))
		{
			m_sflash[addr] &= data;
			m_sflash_status &= ~0x02;
		}
		return UNDRIVEN;
	case 0xaf: // auto address increment program
		if (index == 4 && (m_sflash_status & 2))
		{
			m_sflash_aai_addr = addr;
			m_sflash[m_sflash_aai_addr++ & (SFLASH_SIZE - 1)] &= data;
		}
		else if (cmd.size() == 2 && index == 1)
			m_sflash[m_sflash_aai_addr++ & (SFLASH_SIZE - 1)] &= data;
		return UNDRIVEN;
	case 0x20: case 0xd7: // 4K sector erase
		if (index == 3 && (m_sflash_status & 2))
		{
			const u32 a = ((cmd[1] << 16) | (cmd[2] << 8) | cmd[3]) & (SFLASH_SIZE - 1) & ~0xfff;
			std::fill_n(&m_sflash[a], 0x1000, 0xff);
			m_sflash_status &= ~0x02;
		}
		return UNDRIVEN;
	case 0x52: case 0xd8: // 32K block erase
		if (index == 3 && (m_sflash_status & 2))
		{
			const u32 a = ((cmd[1] << 16) | (cmd[2] << 8) | cmd[3]) & (SFLASH_SIZE - 1) & ~0x7fff;
			std::fill_n(&m_sflash[a], 0x8000, 0xff);
			m_sflash_status &= ~0x02;
		}
		return UNDRIVEN;
	case 0x60: case 0xc7: // chip erase
		if (index == 0 && (m_sflash_status & 2))
		{
			std::fill_n(&m_sflash[0], SFLASH_SIZE, 0xff);
			m_sflash_status &= ~0x02;
		}
		return UNDRIVEN;
	default:
		if (index == 0)
			LOGMASKED(LOG_UNKNOWN, "%08x: serial flash unknown command %02x\n", m_maincpu->pc(), op);
		return UNDRIVEN;
	}
}

// AT45DB642D DataFlash memory card (Atmel datasheet 3542; the commands the firmware uses are listed in
// HARDWARE.md section 13). One command per SPI frame: the controller drives chip select once per frame, so
// every frame starts a new command. Opcode and address bytes clock out 0xff.
void vsmilpro_state::card_decode(const std::vector<u8> &cmd, u32 &page, u32 &byte) const
{
	const u32 a = (cmd[1] << 16) | (cmd[2] << 8) | cmd[3];
	if (m_card_pow2)
	{
		page = (a >> 10) & (CARD_PAGES - 1);
		byte = a & 0x3ff;
	}
	else
	{
		page = (a >> 11) & (CARD_PAGES - 1);
		byte = a & 0x7ff;
	}
}

u8 vsmilpro_state::dataflash_xfer(int index, u8 data, std::vector<u8> &cmd)
{
	if (!BIT(m_io_conf->read(), 0))
		return 0x00; // no card: MISO reads 0 (the firmware takes a 0 ID as "no card")
	cmd.push_back(data);
	const u8 op = cmd[0];
	const u32 psize = card_page_size();
	u32 page = 0, byte = 0;
	if (cmd.size() >= 4)
		card_decode(cmd, page, byte);
	u8 *const mem = &m_card[page * CARD_PAGE_MAX];
	switch (op)
	{
	case 0x9f: // manufacturer and device id: Atmel, AT45DB642D
	{
		static const u8 id[] = { 0xff, 0x1f, 0x28, 0x00, 0x01, 0x00 };
		return index < 6 ? id[index] : 0x00;
	}
	case 0xd7: // status: ready, density 1111 (64 Mbit), bit 0 = power-of-2 page size
		return index >= 1 ? u8(0x80 | 0x3c | m_card_pow2) : 0xff;
	case 0x03: case 0x0b: case 0xe8: // continuous array read (0x0b: 1 dummy byte, 0xe8: 4 dummy bytes)
	{
		const int first = op == 0x03 ? 4 : op == 0x0b ? 5 : 8;
		if (index < first)
			return 0xff;
		u32 p = page, b = byte + (index - first);
		p = (p + b / psize) & (CARD_PAGES - 1);
		b %= psize;
		return m_card[p * CARD_PAGE_MAX + b];
	}
	case 0xd2: // main memory page read (4 dummy bytes), wraps within the page
		return index >= 8 ? mem[(byte + index - 8) % psize] : 0xff;
	case 0x53: case 0x55: // main memory page to buffer 1 / 2
		if (index == 3)
			std::copy_n(mem, CARD_PAGE_MAX, m_card_buf[op == 0x55]);
		return 0xff;
	case 0xd4: case 0xd6: // buffer 1 / 2 read, 1 dummy byte
	case 0xd1: case 0xd3: // buffer 1 / 2 read, low frequency, no dummy byte
	{
		const int first = (op == 0xd4 || op == 0xd6) ? 5 : 4;
		if (index < first)
			return 0xff;
		const u32 off = ((cmd[2] << 8) | cmd[3]) & 0x7ff;
		return m_card_buf[op == 0xd6 || op == 0xd3][(off + index - first) % psize];
	}
	case 0x84: case 0x87: // buffer 1 / 2 write
		if (index >= 4)
		{
			const u32 off = ((cmd[2] << 8) | cmd[3]) & 0x7ff;
			m_card_buf[op == 0x87][(off + index - 4) % psize] = data;
		}
		return 0xff;
	case 0x83: case 0x86: // buffer to main memory page with built-in erase
		if (index == 3)
			std::copy_n(m_card_buf[op == 0x86], psize, mem);
		return 0xff;
	case 0x88: case 0x89: // buffer to main memory page without erase (programs 1 -> 0)
		if (index == 3)
			for (u32 i = 0; i < psize; i++)
				mem[i] &= m_card_buf[op == 0x89][i];
		return 0xff;
	case 0x81: // page erase
		if (index == 3)
			std::fill_n(mem, CARD_PAGE_MAX, 0xff);
		return 0xff;
	case 0x50: // block erase: 8 pages
		if (index == 3)
			std::fill_n(&m_card[(page & ~7u) * CARD_PAGE_MAX], 8 * CARD_PAGE_MAX, 0xff);
		return 0xff;
	case 0x7c: // sector erase: sector 0a = pages 0-7, 0b = 8-255, sector n = 256 pages
		if (index == 3)
		{
			u32 first = page & ~255u, count = 256;
			if (first == 0)
			{
				first = page < 8 ? 0 : 8;
				count = page < 8 ? 8 : 248;
			}
			std::fill_n(&m_card[first * CARD_PAGE_MAX], count * CARD_PAGE_MAX, 0xff);
		}
		return 0xff;
	case 0xc7: // chip erase (C7 94 80 9A)
		if (index == 3 && cmd[1] == 0x94 && cmd[2] == 0x80 && cmd[3] == 0x9a)
			std::fill_n(&m_card[0], CARD_PAGES * CARD_PAGE_MAX, 0xff);
		return 0xff;
	case 0x3d: // 3D 2A 80 A6: switch to 1024-byte pages (one-time programmable on the chip)
		if (index == 3 && cmd[1] == 0x2a && cmd[2] == 0x80 && cmd[3] == 0xa6)
			m_card_pow2 = 1;
		return 0xff;
	default:
		if (index == 0)
			LOGMASKED(LOG_UNKNOWN, "%08x: memory card unknown command %02x\n", m_maincpu->pc(), op);
		return 0xff;
	}
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
	// Status bit 1 is raised by the +0x020 beam-position compare, not by a
	// separate vblank: both the boot ROM and FA program +0x020 (y<<16 | x,
	// y = 236 for 240 lines) right before enabling only bit 1, and FA flips its
	// buffers on bit 1 (analysis/re/reports/display.md, +0x020 / section 3.3).
	// Bit 6 follows the +0x1048 line compare (ROM Vid_waitLine230).
	// TODO: the x field of +0x020 is ignored; bit 0 and bits 2-5, 7 have no
	// known source
	const int line = m_screen->vpos();
	if (line == int((m_disp_regs[0x020 / 4] >> 16) & 0x7ff))
	{
		m_disp_status |= 0x02;
		disp_update_irq();
	}
	if (line == int(m_disp_regs[0x1048 / 4] & 0x7ff))
	{
		m_disp_status |= 0x40; // line compare
		disp_update_irq();
	}
	line_timer_arm(line);
}

// next line with something to do: +0x020 position compare, +0x1048 line compare
void vsmilpro_state::line_timer_arm(int line)
{
	const int lines[2] = { int((m_disp_regs[0x020 / 4] >> 16) & 0x7ff), int(m_disp_regs[0x1048 / 4] & 0x7ff) };
	const int height = m_screen->height();
	int best = -1, bestd = height + 1;
	for (int l : lines)
	{
		if (l < 0 || l >= height)
			continue;
		int d = l - line;
		if (d <= 0)
			d += height;
		if (d < bestd)
		{
			bestd = d;
			best = l;
		}
	}
	m_line_timer->adjust(best >= 0 ? m_screen->time_until_pos(best) : attotime::never);
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
	// ZDISP_LOG=t0,t1: every display register write in that emulated time window
	static const char *const dlog = getenv("ZDISP_LOG");
	static const double dlog0 = dlog ? atof(dlog) : 0, dlog1 = (dlog && strchr(dlog, ',')) ? atof(strchr(dlog, ',') + 1) : 1e9;
	if (dlog && machine().time().as_double() >= dlog0 && machine().time().as_double() < dlog1)
		logerror("DISPW %.4f %08x: %04x = %08x line %d\n", machine().time().as_double(), m_maincpu->pc(), offset * 4, data, m_screen->vpos());
	static const bool l5_log = getenv("ZL5_LOG") != nullptr;
	if (l5_log && offset >= 0x164 / 4 && offset <= 0x16c / 4)
		logerror("L5 %03x = %08x at %.4f s line %d\n", offset * 4, data, machine().time().as_double(), m_screen->vpos());
	// [EMU workaround] Wacky Races' movie player sets the three planes of each new frame with
	// Disp_setL5Planes, which skips zero arguments; once a second it passes no V plane, so the frame
	// would show with the previous frame's V plane (a purple or washed-out flash). Its three plane sets
	// all keep V at the same offset from Y, so a Y without its V uses that offset.
	if (offset == 0x164 / 4)
		m_l5_v_missing = true;
	else if (offset == 0x16c / 4)
	{
		m_l5_v_missing = false;
		if (data && m_disp_regs[0x164 / 4])
			m_l5_yv_delta = data - m_disp_regs[0x164 / 4];
	}
	if (offset == 0x00c / 4 || offset == 0x008 / 4)
		disp_update_screen_size();
	if (offset == 0x00c / 4 || offset == 0x008 / 4 || offset == 0x1048 / 4 || offset == 0x020 / 4)
		line_timer_arm(m_screen->vpos());
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
	// threaded rendering: the GPU layer shows the previous frame's buffer, whose
	// drawing is finished while this frame renders. If work still queued draws
	// into that buffer, finish everything instead.
	if (screen.frame_number() != m_disp_frame)
	{
		m_disp_frame = screen.frame_number();
		std::copy_n(m_disp_prev, std::size(m_disp_prev), m_disp_shown);
		const u32 off = m_disp_shown[0x160 / 4] - VRAM_ORIGIN;
		const int sy = (off / 16384) * 8 + ((off % 16384) % 512) / 64;
		const int sx = (((off % 16384) / 512) * 64 + (off % 64)) / 2;
		const rectangle &vis = screen.visible_area();
		const bool wraps = sx + vis.right() > 1023;
		m_disp_delayed = m_gpu->frame_fence(wraps ? 0 : sx, sy, wraps ? 1023 : sx + vis.right(), sy + vis.bottom());
		std::copy_n(m_disp_regs, std::size(m_disp_prev), m_disp_prev);
	}
	const u32 *const regs = m_disp_regs;
	const u32 *const gregs = m_disp_delayed ? m_disp_shown : m_disp_regs; // GPU layer
	const u32 enable = regs[0x100 / 4];
	bitmap.fill(bgr555(regs[0x10c / 4]), cliprect);

	auto in_window = [] (const u32 *r, int reg, int x, int y)
	{
		const u32 a = r[reg / 4], b = r[reg / 4 + 1];
		return x >= int(a & 0x7ff) && x <= int(b & 0x7ff) && y >= int((a >> 16) & 0x7ff) && y <= int((b >> 16) & 0x7ff);
	};

	// layer 5: planar YCbCr 4:2:0 at +0x164/+0x168/+0x16C, window +0x138/+0x13C,
	// stride = window width (DSP movie and photo output)
	if (BIT(enable, 5))
	{
		const u32 a = regs[0x138 / 4], b = regs[0x13c / 4];
		const int x0 = a & 0x7ff, y0 = (a >> 16) & 0x7ff, x1 = b & 0x7ff, y1 = (b >> 16) & 0x7ff;
		const int w = x1 - x0 + 1;
		const u32 yb = regs[0x164 / 4], ub = regs[0x168 / 4];
		const u32 vb = (m_l5_v_missing && m_l5_yv_delta && !BIT(enable, 14)) ? yb + m_l5_yv_delta : regs[0x16c / 4];
		if (BIT(enable, 14))
		{
			// +0x100 bit 14: linear BGR555 instead of YCbCr (the boot ROM's
			// V.Flash splash while a disc starts)
			if (w > 0 && in_sdram(yb))
				for (int y = std::max(y0, cliprect.top()); y <= std::min(y1, cliprect.bottom()); y++)
				{
					u32 *dst = &bitmap.pix(y);
					for (int x = std::max(x0, cliprect.left()); x <= std::min(x1, cliprect.right()); x++)
					{
						// bit 15 set = transparent (the ROM's splash pixels all have it clear;
						// homebrew drawn with it set shows only the backdrop on hardware)
						const u16 p = ram_r16(yb + ((y - y0) * w + (x - x0)) * 2);
						if (!BIT(p, 15))
							dst[x] = bgr555(p);
					}
				}
		}
		else if (w > 2 && in_sdram(yb) && in_sdram(ub) && in_sdram(vb))
		{
			// YCbCr -> RGB through the table at +0xC00 (written by FA and the boot ROM's photo viewer):
			// 64 entries each for Y, Cb and Cr, indexed by the sample's top 6 bits, each word three signed
			// 10-bit terms B<<20 | G<<10 | R summed per channel, half scale (Y 235 -> 128), clamped. FA's table
			// is BT.601 studio range; Disney Princess' movie on the console fits it (console = 1.13 x Y - 32
			// over its frames), not full-range JFIF, and its darkest pixels (Y 13-15) stay black.
			s32 lut[3][64][3];
			bool have_table = false;
			for (int t = 0; t < 3; t++)
				for (int i = 0; i < 64; i++)
				{
					const u32 wd = regs[(0xc00 + t * 0x100 + i * 4) / 4];
					have_table |= wd != 0;
					for (int c = 0; c < 3; c++)
					{
						const s32 v = (wd >> (c * 10)) & 0x3ff;
						lut[t][i][c] = (v < 0x200) ? v : v - 0x400;
					}
				}
			for (int y = std::max(y0, cliprect.top()); y <= std::min(y1, cliprect.bottom()); y++)
			{
				u32 *dst = &bitmap.pix(y);
				const int ly = y - y0;
				for (int x = std::max(x0, cliprect.left()); x <= std::min(x1, cliprect.right()); x++)
				{
					const int lx = x - x0;
					const u8 yy = ram_r8(yb + ly * w + lx);
					const u8 cb = ram_r8(ub + (ly / 2) * (w / 2) + lx / 2);
					const u8 cr = ram_r8(vb + (ly / 2) * (w / 2) + lx / 2);
					if (have_table)
					{
						const s32 *ty = lut[0][yy >> 2], *tb = lut[1][cb >> 2], *tr = lut[2][cr >> 2];
						dst[x] = rgb_t(std::clamp((ty[0] + tb[0] + tr[0]) * 2, 0, 255),
								std::clamp((ty[1] + tb[1] + tr[1]) * 2, 0, 255),
								std::clamp((ty[2] + tb[2] + tr[2]) * 2, 0, 255));
					}
					else
					{
						// no table written: full-range JFIF
						const float fcb = cb - 128.0f, fcr = cr - 128.0f;
						dst[x] = rgb_t(std::clamp(int(yy + 1.402f * fcr), 0, 255),
								std::clamp(int(yy - 0.344136f * fcb - 0.714136f * fcr), 0, 255),
								std::clamp(int(yy + 1.772f * fcb), 0, 255));
					}
				}
			}
		}
	}

	// main 16-bit layer at the display start address, over the video layer
	// (+0x100 bit 4; FA enables 0x18). +0x010 bit 16 selects the layout: set (the
	// power-on value, never written by FA) = the GPU's tiled surface; clear = linear
	// lines one window wide, which is what the boot ROM's UI draws (UI_fillRect:
	// buf + 2 * (320 * y + x)) after its UI screens clear the bit (ROM 0x10025EAC).
	// Confirmed on hardware: tiled pixels shown after the menu came out as lines.
	if (BIT(gregs[0x100 / 4], 4) && !BIT(gregs[0x010 / 4], 16))
	{
		const u32 start = gregs[0x160 / 4];
		const u32 a = gregs[0x130 / 4], b = gregs[0x134 / 4];
		const int x0 = a & 0x7ff, y0 = (a >> 16) & 0x7ff, x1 = b & 0x7ff;
		const int w = x1 - x0 + 1;
		if (w > 0)
			for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
			{
				u32 *dst = &bitmap.pix(y);
				for (int x = cliprect.left(); x <= cliprect.right(); x++)
				{
					if (!in_window(gregs, 0x130, x, y))
						continue;
					const u32 addr = start + ((y - y0) * w + (x - x0)) * 2;
					const u16 p = in_sdram(addr) ? ram_r16(addr) : 0x8000;
					if (!BIT(p, 15)) // bit 15 set = transparent
						dst[x] = bgr555(p);
				}
			}
	}
	else if (BIT(gregs[0x100 / 4], 4))
	{
		const u32 start = gregs[0x160 / 4];
		const u32 off = start - VRAM_ORIGIN;
		const u32 y0 = (off / 16384) * 8 + ((off % 16384) % 512) / 64;
		const u32 xb0 = ((off % 16384) / 512) * 64 + (off % 64);
		for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
		{
			u32 *dst = &bitmap.pix(y);
			for (int x = cliprect.left(); x <= cliprect.right(); x++)
			{
				if (!in_window(gregs, 0x130, x, y))
					continue;
				const u32 addr = tiled_addr(VRAM_ORIGIN, (xb0 + x * 2) & 2047, y0 + y);
				const u16 p = in_sdram(addr) ? ram_r16(addr) : 0;
				if (BIT(p, 15))
					continue; // transparent: confirmed on hardware with CPU-drawn pixels
				dst[x] = bgr555(p);
			}
		}
	}

	static const int window_regs[4] = { 0x170, 0x120, 0x128, 0x178 };
	for (int layer = 0; layer < 4; layer++)
	{
		if (!BIT(enable, layer))
			continue;
		const u32 map = regs[(0x140 / 4) + layer];
		const u32 tiles = regs[(0x150 / 4) + layer];
		// FA leaves layer 3 enabled with null pointers; show nothing for it
		// TODO: check what the hardware fetches in that case
		if (!in_sdram(map) || !in_sdram(tiles))
			continue;
		const u32 scroll = regs[(0x110 / 4) + layer];
		// +0x104 byte n (analysis/re/reports/display.md 2.1/2.2) [INT, strong]:
		// bits 0-1 map size (16/32/64/128 entries square), bits 2-3 tile format
		// (0 = 8bpp as in the ROM UI, 2 = 4bpp as in the FA console), bit 4
		// palette bank (0 = +0x800, 1 = +0xA00). Map entries: bits 0-10 tile
		// number, bits 11-14 the 16-colour sub-palette in 4bpp mode
		const u8 mode = regs[0x104 / 4] >> (layer * 8);
		const int mapw = 16 << (mode & 3);
		const bool bpp4 = ((mode >> 2) & 3) == 2;
		const u32 *const pal = &regs[(BIT(mode, 4) ? 0xa00 : 0x800) / 4];
		const int sx = scroll & 0xfff, sy = (scroll >> 16) & 0xfff;
		for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
		{
			u32 *dst = &bitmap.pix(y);
			for (int x = cliprect.left(); x <= cliprect.right(); x++)
			{
				if (!in_window(regs, window_regs[layer], x, y))
					continue;
				const int lx = (x + sx) & (mapw * 8 - 1), ly = (y + sy) & (mapw * 8 - 1);
				const u16 entry = ram_r16(map + ((ly >> 3) * mapw + (lx >> 3)) * 2);
				u8 index;
				if (bpp4)
				{
					const u8 b = ram_r8(tiles + (entry & 0x7ff) * 32 + (ly & 7) * 4 + ((lx & 7) >> 1));
					index = BIT(lx, 0) ? (b >> 4) : (b & 0x0f);
					// [guess] +0x028 bit (16 * bank + sub-palette) makes colour 0 of
					// that sub-palette transparent: FA sets it for its console, whose
					// colour 0 is 0x0000 and must be see-through
					if (!index && BIT(regs[0x028 / 4], BIT(mode, 4) * 16 + ((entry >> 11) & 15)))
						continue;
					index |= ((entry >> 11) & 15) << 4;
				}
				else
					index = ram_r8(tiles + (entry & 0x7ff) * 64 + (ly & 7) * 8 + (lx & 7));
				const u16 color = pal[index >> 1] >> (BIT(index, 0) * 16);
				// bit 15 = transparent as in the 16-bit layer [INT: the ROM UI
				// palettes mark their key colour 0x83E0 this way]. TODO: 0x0000 is
				// also skipped, which is [EMU] only (opaque black on layer 4)
				if (BIT(color, 15) || color == 0x0000)
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
	// [INT] completion status (and IRQ 12) only for transfers started with
	// control bit 31: the driver sets it exactly when a callback is attached,
	// and polled transfers wait on +0x0C instead
	if (m_bcdma_irq_on_done[param])
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
			// control word +0x100+16ch (analysis/re/reports/dma_spi_misc.md 1.2.1) [FW]:
			// bits 0-9 count, bits 16-17 unit {1, 2, 4} bytes (the driver
			// invalidates count * unit bytes at the destination; code 3 is
			// rejected by the driver, treated as 4 here), bits 18-19 a second
			// width code (always equal), bit 31 interrupt on completion. The 3-bit
			// +0x08 field per channel is not a width (memcpy sets 3 there and
			// width code 2 here); its meaning is unknown
			static const int unit_sizes[4] = { 1, 2, 4, 4 };
			const int bytes_per = unit_sizes[(cfg >> 16) & 3];
			const u32 count = cfg & 0x3ff;
			LOGMASKED(LOG_DMA, "bcdma ch%d %08x -> %08x count %d x %d (cfg %08x)\n", ch, src, dst, count, bytes_per, cfg);
			static const bool reclog = getenv("ZCD_RECLOG") != nullptr;
			if (reclog)
				logerror("BCDMA %.4f ch%d %08x -> %08x %d bytes\n", machine().time().as_double(), ch, src, dst, count * bytes_per);
			for (u32 i = 0; i < count * bytes_per; i++)
				ram_w8(dst + i, ram_r8(src + i));
			// the data moves at once; completion is signalled after a short
			// delay (~4 bytes per 75 MHz bus clock) so drivers see it busy first
			m_bcdma_busy |= 1 << ch;
			m_bcdma_irq_on_done[ch] = BIT(cfg, 31);
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
				// DSP side of the address map [FW, sound_dsp.md 2.4]: 0xDC020000 + 2a
				// = program word a, 0xDC000000 + 2a = data word a. The data range is
				// not the CPU's interrupt controller [INT, strong], so these writes
				// go to DSP memory (m_dsp_mem: data at 0, program at 0x10000)
				if (dst == 0xdc020000)
				{
					m_dsp->program_loaded(count); // DSP program memory segment
					if (getenv("ZDSP_PROGDUMP")) // the program as loaded: dsp_prog_<words>.bin
						if (FILE *f = fopen(util::string_format("dsp_prog_%x.bin", count).c_str(), "wb"))
						{
							for (u32 i = 0; i < count; i++)
							{
								const u16 v = ram_r16(src + i * sstride);
								fwrite(&v, 2, 1, f);
							}
							fclose(f);
						}
				}
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
			// ZDSP_MEMDUMP: DSP data and program memory after every transfer (dsp_d.bin, dsp_p.bin;
			// 64K little-endian words each), to run the loaded program in an offline ZSP400 interpreter
			if (getenv("ZDSP_MEMDUMP"))
				for (int seg = 0; seg < 2; seg++)
					if (FILE *f = fopen(seg ? "dsp_p.bin" : "dsp_d.bin", "wb"))
					{
						for (u32 i = 0; i < 0x10000; i++)
						{
							const u16 v = m_dsp_mem[seg * 0x10000 + i];
							const u8 b[2] = { u8(v), u8(v >> 8) };
							fwrite(b, 1, 2, f);
						}
						fclose(f);
					}
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
	if (select == 0x30)
		return m_cd_head <= CD_HOME;         // sled inner limit switch
	switch (select & 0xf0)
	{
	case 0x30: return false;                 // other $3x selects: calibration comparators (FA loops on $38 until 0)
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
		else if (cmd == 0x3a && ((value >> 8) & 0xff) == 0xa8)
			m_cd_cal = value & 0xff; // swept by FA's balance calibration (sub_109E5610)
		break;
	case 32:
		// $Ax audio control: D21 (MUTE) also cuts the data the decoder
		// sees, which FA relies on to pause right after the requested sectors
		if ((value >> 26) == (0xa0 >> 2))
			m_cd_mute = BIT(value, 25);
		// $9x function specification: D22 (DSPB) is set only for the ROM's speed code 4, which
		// FA requests after the TOC and logs as "To 4X command" (HARDWARE.md 12). 4x is also what
		// FA's movie start needs: it waits for 290 sectors before configuring the DSP and the game
		// gives up after 2 s (Disney Princess's intro; 2x takes 1.95 s plus the seek)
		if ((cmd & 0xf0) == 0x90)
		{
			const int speed = BIT(value, 26) ? 4 : 1;
			m_cd_timer->adjust(m_cd_timer->remaining(), 0, attotime::from_hz(75 * speed));
		}
		// traverse count for a long sled move (count in bits 27-12). The init
		// sequence also writes $B0 values, so only act while the disc spins.
		// The direction comes from a preceding $22/$23 when there is one;
		// FA's seek sends none, so fall back to the decoder's target MSF.
		// TODO: find the direction bit
		if ((cmd & 0xf0) == 0xb0 && (m_cd_sled_armed || (m_cd_regs[0x40 / 4] & 1)))
		{
			s32 dir = m_cd_dir;
			if (!m_cd_sled_armed)
			{
				const u32 t = m_cd_regs[0x1c / 4];
				const s32 target = (from_bcd(t >> 16) * 60 + from_bcd(t >> 8)) * 75 + from_bcd(t);
				dir = (target >= m_cd_head) ? 1 : -1;
			}
			m_cd_sled_armed = false;
			const s32 c = (value >> 12) & 0xffff;
			const s32 tracks = (c <= 2399) ? c * 100 / 60 : c * 100 / 90;
			cd_jump_tracks(dir * tracks);
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

// The whole 2352-byte sector as it comes off the disc. Images that only keep
// the 2048 user bytes get the sync, header, EDC and P/Q parity rebuilt, since
// FA's software ECC checks every sector against them.
bool vsmilpro_state::cd_read_raw(s32 lba, u8 *raw, bool parity)
{
	std::fill_n(raw, 2352, 0);
	if (lba < 0)
		return false;
	const u32 track = m_cdrom->get_track(lba);
	if (m_cdrom->get_toc().tracks[track].datasize >= 2352)
	{
		if (!m_cdrom->read_data(lba, raw, cdrom_file::CD_TRACK_RAW_DONTCARE))
			return false;
		// CHDs keep CD-DA big-endian; the disc (and the decoder) has it little-endian
		if (m_cdrom->get_track_type(track) == cdrom_file::CD_TRACK_AUDIO)
			for (int i = 0; i < 2352; i += 2)
				std::swap(raw[i], raw[i + 1]);
		return true;
	}
	if (!m_cdrom->read_data(lba, raw + 16, cdrom_file::CD_TRACK_MODE1))
		return false;
	static const u8 sync[12] = { 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };
	std::copy(std::begin(sync), std::end(sync), raw);
	const u32 msf = frames_to_bcd_msf(lba + 150);
	raw[12] = msf >> 16;
	raw[13] = msf >> 8;
	raw[14] = msf;
	raw[15] = 1;
	if (!parity)
		return true;
	u32 edc = 0; // CRC-32, polynomial 0x8001801B, LSB first
	for (int i = 0; i < 0x810; i++)
	{
		edc ^= raw[i];
		for (int b = 0; b < 8; b++)
			edc = (edc >> 1) ^ ((edc & 1) ? 0xd8018001 : 0);
	}
	put_u32le(&raw[0x810], edc);
	cdrom_file::ecc_generate(raw);
	return true;
}

// Writes decoder data to RAM; in ring mode (+0x40 bit 6) the bytes past the
// ring end continue at its start, so a sector can straddle the wrap
void vsmilpro_state::cd_ram_write(u32 addr, const u8 *src, u32 size, bool ring)
{
	const u32 start = m_cd_regs[0x28 / 4], end = m_cd_regs[0x2c / 4] + 4;
	for (u32 i = 0; i < size; i++)
	{
		if (ring && addr >= end && end > start)
			addr = start + (addr - end);
		ram_w8(addr++, src[i]);
	}
}

// Fills the next of the 4 slot records (1 KB each at +0x3C). Besides the data
// address and the header, FA's software ECC (Cd_eccCorrectPQ) takes raw bytes
// 0x810-0x92F (EDC, zeros, P and Q parity) from +0x18 and one erasure (C2) bit
// per raw byte from +0x138. Returns false, and sets +0x70 bit 5, when all four
// slots are full: the sector is not taken and Cd_msgTransferEnd retries.
bool vsmilpro_state::cd_queue_push(u32 data_addr, const u8 *raw)
{
	u32 st = m_cd_regs[0x70 / 4];
	const int head = st & 3, count = (st >> 2) & 7;
	if (count >= 4)
	{
		m_cd_regs[0x70 / 4] |= 0x20;
		return false;
	}
	const int slot = (head + count) & 3;
	const u32 rec = m_cd_regs[0x3c / 4] + slot * 1024;
	ram_w32(rec + 0, 1);
	ram_w32(rec + 4, data_addr);
	for (int i = 0; i < 4; i++)
		ram_w8(rec + 0x14 + i, raw[12 + i]);
	for (int i = 0; i < 0x120; i++)
		ram_w8(rec + 0x18 + i, raw[0x810 + i]);
	for (int i = 0; i < 2352 / 8; i++)
		ram_w8(rec + 0x138 + i, 0);
	m_cd_regs[0x70 / 4] = (st & ~0x1c) | ((count + 1) << 2);
	m_cd_regs[0x64 / 4] |= 0x1000;
	return true;
}

TIMER_CALLBACK_MEMBER(vsmilpro_state::cd_sector_cb)
{
	if (!m_cdrom->exists() || !(m_cd_regs[0x40 / 4] & 1))
		return;

	const u32 ctrl = m_cd_regs[0x40 / 4];
	const u32 mode = ctrl & 0x18;
	const bool capture = (mode == 0x08) && (ctrl & 0x80);
	// the decoder only takes the sector whose header matches the next
	// expected address; FA pauses by seeking back and letting the drive
	// re-read sectors the decoder already has
	// +0x60 bits 31-24 arm a timeout (IRQ 0x8000) for a running decoder that
	// sees no data (counted in EFM frames, 98 per sector); FA uses it to end
	// a muted pause, which makes it take the last buffered sector and restart
	bool deliver = (ctrl & 4) && m_cd_running && !m_cd_mute && (capture || m_cd_head == m_cd_expect);
	if ((ctrl & 4) && m_cd_running && (m_cd_regs[0x60 / 4] >> 24))
	{
		if (!m_cd_mute)
			m_cd_tmo = m_cd_regs[0x60 / 4] >> 24;
		else if (m_cd_tmo)
		{
			m_cd_tmo = (m_cd_tmo > 98) ? (m_cd_tmo - 98) : 0;
			if (!m_cd_tmo)
				m_cd_regs[0x64 / 4] |= 0x8000;
		}
	}
	const s32 lba = m_cd_head - 150;
	if (lba >= 0 && m_cdrom->get_track_type(m_cdrom->get_track(lba)) != cdrom_file::CD_TRACK_AUDIO)
		m_cd_regs[0x4c / 4] = 0x80000000 | frames_to_bcd_msf(m_cd_head); // last header seen
	// With +0x40 bit 8 every sector taken probably also fills a slot record: FA
	// sets it for its formats 3 (2048 bytes, software ECC on every record) and
	// 4 (the records are flushed), not for 5 (2048, no records) or the raw
	// formats. With all four slots unread the sector is dropped.
	// Opt-in (ZCD_REC=1) until it works: with it, Incredibles' loading stalls
	// (2026-10-25). Its ring read of LBA 29-48 copies the requested sector
	// out, but the driver never reports the request complete, so the next one
	// never comes. Without records FA treats written data as verified
	// (Cd_syncWritePtr) and records only exist for header capture.
	static const bool norec = !getenv("ZCD_REC") || getenv("ZCD_NOREC");
	static const bool reclog = getenv("ZCD_RECLOG") != nullptr;
	const bool record = deliver && (capture || ((ctrl & 0x100) && !norec));
	u8 raw[2352];
	if (deliver)
		cd_read_raw(lba, raw, record); // EDC and parity only matter to the records
	if (record && !cd_queue_push(m_cd_ptr, raw))
	{
		if (reclog)
			logerror("CDREC overrun at lba %d\n", lba);
		deliver = false;
	}
	else if (deliver && reclog && (ctrl & 0x100))
		logerror("CDREC lba %d ctrl %03x ecc %s mode %d slots %02x\n", lba, ctrl & 0x1ff, cdrom_file::ecc_verify(raw) ? "ok" : "BAD", raw[15], m_cd_regs[0x70 / 4] & 0x3f);
	if (deliver)
	{
		// deliver the sector under the head
		if (!capture)
			m_cd_expect++;
		if (!m_cd_found)
		{
			m_cd_found = true;
			m_cd_regs[0x64 / 4] |= 0x04; // target found
		}
		u32 size;
		if (capture)
		{
			size = 4;
			cd_ram_write(m_cd_ptr, raw + 12, 4, false);
		}
		else if ((ctrl & 0x20) && mode == 0)
		{
			// dual buffer (CD-DA): left samples to +0x24 (ring +0x28-0x2C),
			// right samples to +0x30 (ring +0x34-0x38); the ROM's audio CD
			// player plays the two rings on two sound voices
			size = 1176;
			const bool ring = ctrl & 0x40;
			for (int i = 0; i < 2352; i += 4)
			{
				cd_ram_write(m_cd_ptr + i / 2, raw + i, 2, ring);
				u32 a = m_cd_ptr2 + i / 2;
				const u32 start = m_cd_regs[0x34 / 4], end = m_cd_regs[0x38 / 4] + 4;
				if (ring && a >= end && end > start)
					a = start + (a - end);
				ram_w8(a, raw[i + 2]);
				ram_w8(a + 1, raw[i + 3]);
			}
			m_cd_ptr2 += size;
			if (ring && m_cd_ptr2 > m_cd_regs[0x38 / 4])
				m_cd_ptr2 = m_cd_regs[0x34 / 4] + (m_cd_ptr2 - m_cd_regs[0x38 / 4] - 4);
		}
		else
		{
			size = (mode == 0x18) ? 2048 : (mode == 0x10) ? 2336 : 2352;
			cd_ram_write(m_cd_ptr, raw + 2352 - size - ((size == 2048) ? 288 : 0), size, ctrl & 0x40);
		}
		LOGMASKED(LOG_CD, "cd sector lba %d -> %08x (%d bytes)\n", lba, m_cd_ptr, size);
		// ZCD_RATE: time and write pointer every 75 sectors (stream rate checks)
		static const bool rate_log = getenv("ZCD_RATE") != nullptr;
		static u32 delivered;
		if (rate_log && (++delivered % 75 == 0 || atoi(getenv("ZCD_RATE")) >= 2))
			logerror("CDRATE %.3f s lba %d ptr %08x\n", machine().time().as_double(), lba, m_cd_ptr);

		// the watermark fires when the write pointer reaches it, which
		// includes passing it in this step (ring wrap aware)
		const u32 old_ptr = m_cd_ptr;
		m_cd_ptr += size;
		if ((ctrl & 0x40) && m_cd_ptr > m_cd_regs[0x2c / 4])
			m_cd_ptr = m_cd_regs[0x28 / 4] + (m_cd_ptr - m_cd_regs[0x2c / 4] - 4);
		const u32 wm = m_cd_regs[0x80 / 4];
		const u32 ring_size = (ctrl & 0x40) ? (m_cd_regs[0x2c / 4] + 4 - m_cd_regs[0x28 / 4]) : 0;
		u32 dist = wm - old_ptr;
		if (ring_size && dist >= ring_size)
			dist += ring_size; // wm behind old_ptr in the ring
		const bool wm_hit = (dist != 0) && (dist <= size);

		if (!(m_cd_regs[0x20 / 4] & 0x1000000))
		{
			m_cd_len -= size;
			if (m_cd_len <= 0)
			{
				// the decoder stops by itself: the run bit drops too, so
				// setting it again starts a new transfer
				m_cd_running = false;
				m_cd_regs[0x40 / 4] &= ~4;
				m_cd_regs[0x64 / 4] |= 0x10;
			}
		}
		if (m_cd_running && wm_hit)
		{
			if (m_cd_regs[0x84 / 4] & 1)
			{
				m_cd_running = false;
				m_cd_regs[0x40 / 4] &= ~4;
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
	if (!machine().side_effects_disabled() && offset != 0x18 / 4 && offset != 0x14 / 4)
		LOGMASKED(LOG_CDR, "%08x: cd read %02x = %08x (head %d)\n", m_maincpu->pc(), offset * 4, offset == 0x24 / 4 ? m_cd_ptr : m_cd_regs[offset], m_cd_head);
	switch (offset)
	{
	case 0x10 / 4: return m_cd_rx; // servo RX data
	case 0x24 / 4: return m_cd_ptr;
	case 0x30 / 4: return m_cd_ptr2;
	default:
		return m_cd_regs[offset];
	}
}

void vsmilpro_state::cd_w(offs_t offset, u32 data, u32 mem_mask)
{
	static const bool reclog = getenv("ZCD_RECLOG") != nullptr;
	if (reclog && offset != 0x00 / 4 && offset != 0x08 / 4 && offset != 0x0c / 4 && offset != 0x14 / 4)
		logerror("CDW %.4f %08x: +%02x = %08x (64 %05x 68 %05x 70 %02x)\n", machine().time().as_double(), m_maincpu->pc(), offset * 4, data,
				m_cd_regs[0x64 / 4], m_cd_regs[0x68 / 4], m_cd_regs[0x70 / 4]);
	switch (offset)
	{
	case 0x64 / 4:
		m_cd_regs[offset] &= ~data;
		if (m_cd_regs[0x70 / 4] & 0x1c)
			m_cd_regs[offset] |= 0x1000; // level while records remain
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
			else
			{
				// monitor reads ($39 aa 00): calibration loops accept any value
				// except FA's sweep of $3AA8xx, which waits for monitor $1F to
				// enter a window; let it follow the swept value
				const u8 cmd = u8(value >> (nbits - 8)), addr = u8(value >> (nbits - 16));
				m_cd_rx = (cmd == 0x39 && addr == 0x1f) ? u8(m_cd_cal - 0x40) : 0;
			}
			m_cd_regs[0x14 / 4] &= ~3; // start and read bits clear when the transfer completes
			m_cd_regs[0x64 / 4] |= read ? 2 : 1;
			cd_update_irq();
		}
		return;
	case 0x24 / 4:
		COMBINE_DATA(&m_cd_regs[offset]);
		m_cd_ptr = m_cd_regs[offset];
		return;
	case 0x30 / 4:
		COMBINE_DATA(&m_cd_regs[offset]);
		m_cd_ptr2 = m_cd_regs[offset];
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
			{
				m_cd_expect = (from_bcd(start >> 16) * 60 + from_bcd(start >> 8)) * 75 + from_bcd(start);
				// the seek model is coarse: if the head is not shortly before
				// the target, put it there
				if (m_cd_head > m_cd_expect || m_cd_expect - m_cd_head > 300)
					m_cd_head = m_cd_expect;
			}
			m_cd_ptr = m_cd_regs[0x24 / 4];
			m_cd_ptr2 = m_cd_regs[0x30 / 4];
			m_cd_len = m_cd_regs[0x20 / 4] & 0xffffff;
			m_cd_running = true;
			m_cd_found = false;
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
	case 0x60 / 4:
	{
		const u32 old = m_cd_regs[offset];
		COMBINE_DATA(&m_cd_regs[offset]);
		if ((m_cd_regs[offset] >> 24) && !(old >> 24))
			m_cd_tmo = m_cd_regs[offset] >> 24;
		LOGMASKED(LOG_CD, "%08x: cd 60 = %08x\n", m_maincpu->pc(), m_cd_regs[offset]);
		return;
	}
	case 0x84 / 4:
		COMBINE_DATA(&m_cd_regs[offset]);
		// a watermark programmed where the decoder already is takes effect now
		if (m_cd_running && m_cd_ptr == m_cd_regs[0x80 / 4])
		{
			if (m_cd_regs[offset] & 1)
			{
				m_cd_running = false;
				m_cd_regs[0x40 / 4] &= ~4;
				m_cd_regs[0x64 / 4] |= 0x10;
			}
			else
				m_cd_regs[0x64 / 4] |= 0x10000;
			cd_update_irq();
		}
		return;
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
		{
			count = 0;
			st &= ~0x20; // overrun flag goes with the flush
		}
		m_cd_regs[0x70 / 4] = (st & ~0x1f) | (count << 2) | head;
		// "record queued" is a level: it stays up while unread records remain.
		// FA masks it while its software ECC runs and relies on it to pick up
		// records that arrived meanwhile (Incredibles stalled loading without)
		if (count)
			m_cd_regs[0x64 / 4] |= 0x1000;
		else
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
	map(0x900a0100, 0x900a011f).rw(m_dsp, FUNC(zevio_dsp_hle_device::mbox_r), FUNC(zevio_dsp_hle_device::mbox_w));
	map(0x900b0000, 0x900b003f).rw(FUNC(vsmilpro_state::pll_r), FUNC(vsmilpro_state::pll_w));
	map(0x900c0000, 0x900c003f).rw(FUNC(vsmilpro_state::timer_r), FUNC(vsmilpro_state::timer_w));
	map(0x900d0000, 0x900d00ff).rw(FUNC(vsmilpro_state::gpio_r<1>), FUNC(vsmilpro_state::gpio_w<1>));

	map(0xa0000000, 0xa00fffff).lr32(NAME([] () { return 0; })); // nothing on the factory bus
	map(0xa1000000, 0xa100003f).rw(FUNC(vsmilpro_state::spi_r), FUNC(vsmilpro_state::spi_w));
	map(0xa4000000, 0xa400003f).rw(m_gpu, FUNC(zevio_gpu_device::cvt_r), FUNC(zevio_gpu_device::cvt_w));
	map(0xa8000000, 0xa80003ff).rw(m_gpu, FUNC(zevio_gpu_device::regs_r), FUNC(zevio_gpu_device::regs_w));
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
	m_gpu->set_stall_cpu(m_maincpu); // waits out the engine's drawing time
	m_sflash = std::make_unique<u8[]>(SFLASH_SIZE);
	std::fill_n(&m_sflash[0], SFLASH_SIZE, 0xff);
	if (m_spi_region)
		std::copy_n(m_spi_region->base(), std::min<u32>(m_spi_region->bytes(), SFLASH_SIZE), &m_sflash[0]);
	m_nvram->set_base(&m_sflash[0], SFLASH_SIZE);
	// memory card: a blank card in power-of-2 mode (the boot ROM converts every card it sees, HARDWARE.md 13);
	// the page-size flag is kept in the byte after the last page so it survives in the nvram file
	m_card = std::make_unique<u8[]>(CARD_PAGES * CARD_PAGE_MAX + 1);
	std::fill_n(&m_card[0], CARD_PAGES * CARD_PAGE_MAX + 1, 0xff);
	m_cardnv->set_base(&m_card[0], CARD_PAGES * CARD_PAGE_MAX + 1);
	m_card_pow2 = 1;
	save_pointer(NAME(m_card), CARD_PAGES * CARD_PAGE_MAX + 1);
	save_item(NAME(m_card_buf));
	save_item(NAME(m_card_pow2));

	m_dsp_mem = std::make_unique<u16[]>(0x80000);
	m_spu->set_ram(m_sdram.target(), SDRAM_BASE, SDRAM_SIZE);
	m_gpu->set_ram(m_sdram.target(), SDRAM_BASE, SDRAM_SIZE);
	m_dsp->set_ram(m_sdram.target(), SDRAM_BASE, SDRAM_SIZE);

	m_tick_timer = timer_alloc(FUNC(vsmilpro_state::tick_cb), this);
	for (auto &t : m_bcdma_timer)
		t = timer_alloc(FUNC(vsmilpro_state::bcdma_done_cb), this);
	save_item(NAME(m_bcdma_busy));
	save_item(NAME(m_bcdma_irq_on_done));
	m_ch1_wrap_timer = timer_alloc(FUNC(vsmilpro_state::ch1_wrap_cb), this);
	m_line_timer = timer_alloc(FUNC(vsmilpro_state::line_cb), this);
	m_joy_timer = timer_alloc(FUNC(vsmilpro_state::joy_cb), this);
	m_cd_timer = timer_alloc(FUNC(vsmilpro_state::cd_sector_cb), this);

	m_ser_port = -1;
	m_ser_timer = timer_alloc(FUNC(vsmilpro_state::ser_cb), this);
	if (const char *ser = getenv("VFLASH_SERIAL"))
	{
		int port = 0, tcp = 0;
		if (sscanf(ser, "%d:%d", &port, &tcp) == 2 && (port == 1 || port == 2) && tcp > 0)
		{
			u64 size;
			if (osd_file::open(util::string_format("socket.127.0.0.1:%d", tcp),
					OPEN_FLAG_READ | OPEN_FLAG_WRITE | OPEN_FLAG_CREATE, m_ser_file, size))
				osd_printf_error("VFLASH_SERIAL: cannot listen on 127.0.0.1:%d\n", tcp);
			else
			{
				m_ser_port = port - 1;
				osd_printf_info("VFLASH_SERIAL: controller port %d <-> 127.0.0.1:%d\n", port, tcp);
				m_ser_timer->adjust(attotime::from_msec(1));
			}
		}
		else
			osd_printf_error("VFLASH_SERIAL must be <controller port 1|2>:<tcp port>\n");
	}

	// Speedup: the boot ROM's idle task (0x1001098C) is an endless counting
	// loop whose counter lives at 0x101E05E4; sleep until the next interrupt
	// direct SDRAM access for the CPU core, except the boot ROM idle counter
	// (watched by the tap below); off with the debugger so watchpoints work
	if (!(machine().debug_flags & DEBUG_FLAG_ENABLED))
	{
		u8 *const sdram = reinterpret_cast<u8 *>(m_sdram.target());
		m_maincpu->add_fastram(0x101e05e8, 0x10ffffff, sdram + 0x1e05e8); // FA code, data and stacks: check first
		m_maincpu->add_fastram(0x10000000, 0x101e05e3, sdram);
	}

	// CPU memory timing measured on a console (sdk/tests/timing, analysis/re/hw_verified.md HW #18): ARM926EJ-S with
	// an 8 KB D-cache and a 16 KB I-cache (4-way, 32-byte lines, CP15 cache type 0x1D112152); a load that misses
	// costs 31 cycles in all, cold code 4.13 per instruction (a line fill every 8), every store about 6.3 as the
	// write-through cache drains it to SDRAM, and device registers 12-20 cycles per access. Without this the
	// emulated CPU did about twice the console's work per frame (Bratz's character select and store).
	// VFLASH_NO_MEMTIMING turns it off.
	if (!getenv("VFLASH_NO_MEMTIMING"))
	{
		arm7_cpu_device::mem_timing t;
		t.dmiss = 30;    // 1.49 per hit, 31.0 per miss (loop overhead 0.5)
		t.imiss = 25;    // 4.13 per instruction = 1 + 25 / 8
		t.wb_drain = 6;  // stores 6.29 per op back to back
		t.wb_entries = 4;
		auto io = [&t] (int region, int rd, int wr) { t.io_read[region] = rd; t.io_write[region] = wr; };
		io(0x10, 1, 6);  // SDRAM outside the fast RAM ranges (the idle counter word; all of it with the debugger)
		for (int r = 0x8f; r <= 0x90; r++)
			io(r, 20, 20); // SDRAM controller, APB: GPIO, UARTs, RTC, sysctl 20.49, PLL, timers 20.50
		io(0xa0, 20, 20); io(0xa1, 20, 20); // factory bus, SPI
		io(0xa4, 12, 14); // number converters: read 12.49, write 14.50
		io(0xa8, 12, 14); // GPU: read 12.49
		io(0xb0, 14, 16); // SPU: read 14.51
		io(0xb8, 14, 16); // display: read 14.64, write 16.68
		io(0xbc, 14, 16); io(0xc0, 14, 16); io(0xc4, 14, 16); // DMA, descriptor DMA, CD (as the AHB devices)
		io(0xdc, 14, 16); // interrupt controller / DSP window: read 14.50
		m_maincpu->set_mem_timing(t);
	}

	m_maincpu->space(AS_PROGRAM).install_write_tap(0x101e05e4, 0x101e05e7, "idle_tap",
			[this] (offs_t offset, u32 &data, u32 mem_mask)
			{
				const u32 pc = m_maincpu->pc();
				if (pc >= 0x10010990 && pc <= 0x100109cc && ram_r32(0x100109a0) == 0xe58d3000)
					m_maincpu->spin_until_interrupt();
			});

	// debug: ZPROF=1 samples the CPU PC at 100 Hz (in joy_cb) and prints a
	// histogram (64-byte buckets) at exit
	m_prof_on = getenv("ZPROF") != nullptr;
	if (m_prof_on)
		machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&vsmilpro_state::prof_dump, this));

	machine().save().register_postload(save_prepost_delegate(NAME([this] () { m_irq_line = m_fiq_line = -1; update_cpu_irqs(); })));
	save_item(NAME(m_int_raw));
	save_item(NAME(m_int_pending));
	save_item(NAME(m_int_latched));
	save_item(NAME(m_int_enable));
	save_item(NAME(m_int_regs));
	save_item(NAME(m_int_limit));
	save_item(NAME(m_int_prev));
	save_item(NAME(m_sysctl));
	save_item(NAME(m_resume));
	m_resume = false;
	save_item(NAME(m_pll));
	save_item(NAME(m_rtc));
	save_item(NAME(m_gpio_dir));
	save_item(NAME(m_gpio_out));
	save_item(NAME(m_gpio_irqcfg));
	save_item(NAME(m_timer_regs));
	save_item(NAME(m_ch1_start));
	save_item(NAME(m_ch1_period_ticks));
	save_item(NAME(m_ch1_running));
	save_item(NAME(m_spi_regs));
	save_item(NAME(m_disp_regs));
	save_item(NAME(m_disp_status));
	save_item(NAME(m_bcdma_regs));
	save_item(NAME(m_dcdma_regs));
	save_item(NAME(m_cd_regs));
	save_item(NAME(m_cd_head));
	save_item(NAME(m_cd_expect));
	save_item(NAME(m_cd_mute));
	save_item(NAME(m_cd_tmo));
	save_item(NAME(m_cd_dir));
	save_item(NAME(m_cd_jump_n));
	save_item(NAME(m_cd_ptr));
	save_item(NAME(m_cd_ptr2));
	save_item(NAME(m_cd_len));
	save_item(NAME(m_cd_running));
	save_item(NAME(m_cd_found));
	save_pointer(NAME(m_dsp_mem), 0x80000);
}

void vsmilpro_state::soc_reset()
{
	// on-chip blocks: reset at power-on and again when the chip wakes from
	// standby (the ROM's standby routine puts SDRAM in self-refresh, gates the
	// clocks and writes 0x900A0008 = 1; wake goes through the reset vector with
	// "resume" in the reset cause register). External chips and the always-on
	// block keep their state across standby.
	m_irq_line = m_fiq_line = -1;
	m_int_raw = m_int_pending = m_int_latched = 0;
	m_int_enable[0] = m_int_enable[1] = 0;
	std::fill(std::begin(m_int_regs), std::end(m_int_regs), 0);
	m_int_limit[0] = m_int_limit[1] = 8; // reset value unknown; Intc_init writes 8
	m_int_prev[0] = m_int_prev[1] = 8;
	std::fill(std::begin(m_sysctl), std::end(m_sysctl), 0);
	std::fill(std::begin(m_pll), std::end(m_pll), 0);
	for (int b = 0; b < 2; b++)
		for (int l = 0; l < 4; l++)
			m_gpio_dir[b][l] = m_gpio_out[b][l] = m_gpio_irqcfg[b][l] = 0;
	std::fill(std::begin(m_timer_regs), std::end(m_timer_regs), 0);
	std::fill(std::begin(m_spi_regs), std::end(m_spi_regs), 0);
	std::fill(std::begin(m_disp_regs), std::end(m_disp_regs), 0);
	m_disp_regs[0x010 / 4] = DISP_010_RESET;
	std::fill(std::begin(m_bcdma_regs), std::end(m_bcdma_regs), 0);
	m_bcdma_busy = 0;
	std::fill(std::begin(m_bcdma_irq_on_done), std::end(m_bcdma_irq_on_done), 0);
	for (auto &t : m_bcdma_timer)
		t->adjust(attotime::never);
	std::fill(std::begin(m_dcdma_regs), std::end(m_dcdma_regs), 0);
	std::fill(std::begin(m_cd_regs), std::end(m_cd_regs), 0);
	m_disp_status = 0;
	m_ch1_running = false;
	m_ch1_period_ticks = 1;
	// UARTs come up with IER = 0 and empty FIFOs. A game started from the ROM
	// menu depends on this: the menu leaves UART1 IER = 1 and the detected
	// controller keeps sending reports, but FA enables IRQ 2 before it installs
	// its UART1 receive callback, so stale bytes would hold the level IRQ forever
	for (auto &u : m_uart)
	{
		u.ier = u.lcr = u.mcr = u.scr = u.dll = u.dlm = u.fcr = 0;
		u.thre_pending = false;
		u.lsr_err = 0;
		u.rx.clear();
		u.line.clear();
	}
	m_tick_timer->adjust(attotime::never);
	m_ch1_wrap_timer->adjust(attotime::never);
	m_line_timer->adjust(m_screen->time_until_pos(0));
	// CD decoder (the servo and pickup are external)
	m_cd_expect = 0;
	m_cd_tmo = 0;
	m_cd_ptr = m_cd_ptr2 = 0;
	m_cd_len = 0;
	m_cd_running = false;
	m_cd_found = false;
	update_cpu_irqs();
}

void vsmilpro_state::machine_reset()
{
	soc_reset();
	std::fill(std::begin(m_rtc), std::end(m_rtc), 0);
	// The always-on block survives power-off; the ROM marks it with 0xC01DB007
	// after the first cold start and otherwise powers down until a key press
	m_rtc[0x88 / 4] = 0xc01db007;
	// +0x8C = 0x1230FFAB records a power-key wake-up; without it the startup
	// task treats the boot as spurious and returns to standby
	m_rtc[0x8c / 4] = 0x1230ffab;
	m_sflash_status = 0;
	m_sflash_aai_addr = 0;
	for (auto &j : m_joy)
	{
		j.cmd.clear();
		j.detected = false;
		memset(j.last, 0, sizeof(j.last));
		j.last_sent = attotime::zero;
	}
	m_joy_timer->adjust(attotime::from_hz(100), 0, attotime::from_hz(100));
	m_cd_head = -3000;
	m_cd_mute = false;
	m_cd_dir = 1;
	m_cd_jump_n = 0;
	m_cd_sled_armed = false;
	m_cd_cal = 0;
	m_cd_rx = 0;
	m_cd_timer->adjust(attotime::from_hz(75), 0, attotime::from_hz(75));
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

	PORT_START("CONF")
	PORT_CONFNAME( 0x01, 0x01, "Memory card" )
	PORT_CONFSETTING(    0x00, "None" )
	PORT_CONFSETTING(    0x01, "Inserted" )

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
	// the ARM926 has caches, so the ARM7 prefetch buffer model is not needed
	m_maincpu->set_direct_fetch(true);
	// CP15 c0 as read on a console (HW #18): ARM926EJ-S r0p4; 8 KB D-cache, 16 KB I-cache, 4-way, 32-byte lines
	m_maincpu->set_copro_id(0x41069264);
	m_maincpu->set_cache_type(0x1d112152);
	m_maincpu->set_drc(!getenv("VFLASH_NO_DRC"));
	// the boot ROM kernel's idle task (copied to 0x10010994) counts forever
	m_maincpu->add_idle_loop(0x100109a4, 0x10010994, {
			0xe3a03000, 0xe5cd3007, 0xe3a03000, 0xe58d3000, 0xe59d3000, 0xe35300fe, 0x9a000000, 0xeafffff7,
			0xe5dd3007, 0xe2833001, 0xe5cd3007, 0xe59d3000, 0xe2833001, 0xe58d3000, 0xeafffff4 });
	// FA's idle task (Task_idle, 0x109C8BBC in Bratz; the same five words in
	// every game kernel seen) counts to 0xFE forever; it is entered at the
	// MOV and at the ADD (the BLS target)
	static const std::vector<u32> fa_idle = { 0xe3a03000, 0xe2833001, 0xe35300fe, 0x9afffffc, 0xeafffffa };
	m_maincpu->add_idle_pattern(0, fa_idle);
	m_maincpu->add_idle_pattern(4, fa_idle);
	// Wacky Races (Jumpin' Bean Island): the main loop's frame pacing spins until the timer at
	// 0x900C000C has moved 0x30 counts on from the last frame; each pass only reads the time
	m_maincpu->add_idle_loop(0x10b520c4, 0x10b520c4, {
			0xeb001f92, 0xe1a03000, 0xe1a00003, 0xebff13f0, 0xe1a02000, 0xe59d3008, 0xe5933124, 0xe0633002,
			0xe3530030, 0x9afffff5 });
	m_maincpu->set_addrmap(AS_PROGRAM, &vsmilpro_state::vsmilpro_map);

	NVRAM(config, m_nvram, nvram_device::DEFAULT_NONE);
	NVRAM(config, m_cardnv, nvram_device::DEFAULT_ALL_1);

	SCREEN(config, m_screen);
	m_screen->set_refresh_hz(60);
	m_screen->set_size(320, 262);
	m_screen->set_visarea(0, 320 - 1, 0, 240 - 1);
	m_screen->set_screen_update(FUNC(vsmilpro_state::screen_update));

	SPEAKER(config, "speaker", 2).front();

	ZEVIO_DSP_HLE(config, m_dsp);
	m_dsp->msg_irq_cb().set([this] (int state) { set_irq(IRQ_MBOX0, state); });
	m_dsp->reply_irq_cb().set([this] (int state) { set_irq(IRQ_MBOX1, state); });

	ZEVIO_GPU(config, m_gpu);
	m_gpu->irq_cb().set([this] (int state) { set_irq(IRQ_GPU, state); });

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
COMP( 2007, vsmilpro,  0,        0,      vsmilpro, vsmilpro, vsmilpro_state, empty_init, "VTech", "V.Smile Pro (Germany)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
COMP( 2007, vsmilprof, vsmilpro, 0,      vsmilpro, vsmilpro, vsmilpro_state, empty_init, "VTech", "V.Smile Pro (France)",  MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
COMP( 2006, vflash,    vsmilpro, 0,      vsmilpro, vsmilpro, vsmilpro_state, empty_init, "VTech", "V.Flash (USA)",         MACHINE_SUPPORTS_SAVE )
