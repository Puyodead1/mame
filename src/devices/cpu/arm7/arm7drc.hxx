// license:BSD-3-Clause
// copyright-holders:Steve Ellenoff,R. Belmont,Ryan Holtz
/*****************************************************************************

    ARM dynamic recompiler (ARM state only)

    Blocks of up to DRC_MAX_INSNS instructions are compiled to UML. The most
    common instructions (data processing with an immediate or a register
    operand, B and BL) are generated directly; every other instruction calls
    the interpreter for just that instruction. After each instruction the
    pending interrupt check and the cycle count are done in the same order
    as the interpreter does them, so both produce identical results and the
    recompiler can be checked against the interpreter.

    Blocks are hashed by (CPSR mode bits 3-0, virtual PC). Thumb state runs
    in the interpreter. Changes to translation (CP15) flush everything; a
    store to a 1K page holding compiled code invalidates the blocks there.

    Register file, CPSR and the cycle counter stay in the device; generated
    code reaches them with LOAD/STORE, which accept far pointers.

*****************************************************************************/

#include "emuopts.h"
#include "cpu/drcuml.h"
#include "cpu/drcumlsh.h"

namespace {

constexpr uint32_t DRC_EXECUTE_OUT_OF_CYCLES = 0;
constexpr uint32_t DRC_EXECUTE_MISSING_CODE  = 1;
constexpr uint32_t DRC_EXECUTE_RESET_CACHE   = 3;
constexpr uint32_t DRC_EXECUTE_INTERP        = 100; // run one instruction in the interpreter (Thumb, unfetchable code)

constexpr int DRC_MAX_INSNS = 64;
constexpr size_t DRC_CACHE_SIZE = 256 * 1024 * 1024;

} // anonymous namespace


void arm7_cpu_device::drc_start()
{
	m_drc = m_drc_requested && m_direct_fetch && allow_drc()
			&& !(machine().debug_flags & DEBUG_FLAG_ENABLED) && !(m_archFlags & ARCHFLAG_MODE26);
	if (!m_drc)
		return;
	m_drc_cache = std::make_unique<drc_cache>(DRC_CACHE_SIZE);
	m_drc_cache->allocate_cache(mconfig().options().drc_rwx());
	m_drcuml = std::make_unique<drcuml_state>(*this, *m_drc_cache, 0, 16, 32, 2, DRC_MAX_INSNS * 4);
	m_drc_dirty = true;
}

// ARM_DRC_STATS=2: count the instructions the compiled code hands to the interpreter, by
// decode bits 27-20 and 7-4, and print the most frequent at exit
// ARM_DRC_OFF=<mask>: leave native paths to the interpreter (for bisecting differences):
// 1 write-back/post-index forms, 2 halfword/signed, 4 LDR PC, 8 MOV PC, 16 MRS, 32 ALU with Rm = PC, 64 all loads/stores,
// 128 LDM/STM
static const unsigned s_drc_off = getenv("ARM_DRC_OFF") ? strtoul(getenv("ARM_DRC_OFF"), nullptr, 0) : 0;
static const bool s_drc_hist_on = getenv("ARM_DRC_STATS") && atoi(getenv("ARM_DRC_STATS")) >= 2;
static uint64_t s_drc_hist[4096];
static uint32_t s_drc_hist_insn[4096];
static std::unordered_map<uint32_t, uint64_t> s_drc_hist_pc;   // by PC
static std::unordered_map<uint32_t, uint32_t> s_drc_hist_lr;   // last LR seen there
static uint64_t s_drc_hist_region[256];                        // interpreted LDR/STR by address bits 31-24
static std::unordered_map<uint32_t, uint32_t> s_drc_hist_addr; // last LDR/STR address seen there; bit 0: fast RAM through a cached translation
static struct drc_hist_dump { ~drc_hist_dump() {
	if (!s_drc_hist_on) return;
	uint64_t total = 0;
	std::vector<int> k;
	for (int i = 0; i < 4096; i++) { total += s_drc_hist[i]; if (s_drc_hist[i]) k.push_back(i); }
	std::sort(k.begin(), k.end(), [] (int a, int b) { return s_drc_hist[a] > s_drc_hist[b]; });
	printf("DRC interpreter fallbacks %llu\n", (unsigned long long)total);
	for (size_t i = 0; i < k.size() && i < 30; i++)
		printf("  %03x %5.1f%%  e.g. %08x\n", k[i], 100.0 * s_drc_hist[k[i]] / total, s_drc_hist_insn[k[i]]);
	std::vector<std::pair<uint32_t, uint64_t>> pcs(s_drc_hist_pc.begin(), s_drc_hist_pc.end());
	std::sort(pcs.begin(), pcs.end(), [] (auto &a, auto &b) { return a.second > b.second; });
	for (size_t i = 0; i < pcs.size() && i < 90; i++)
		printf("  pc %08x %5.1f%%  lr %08x  addr %08x\n", pcs[i].first, 100.0 * pcs[i].second / total, s_drc_hist_lr[pcs[i].first], s_drc_hist_addr[pcs[i].first]);
	for (int r = 0; r < 256; r++)
		if (s_drc_hist_region[r])
			printf("  LDR/STR region %02x...... %5.1f%%\n", r, 100.0 * s_drc_hist_region[r] / total);
} } s_drc_hist_dump;

void arm7_cpu_device::drc_interp_one(void *param)
{
	arm7_cpu_device &cpu = *reinterpret_cast<arm7_cpu_device *>(param);
	if (s_drc_hist_on)
	{
		const uint32_t key = ((cpu.m_drc_insn >> 16) & 0xff0) | ((cpu.m_drc_insn >> 4) & 0xf);
		s_drc_hist[key]++;
		s_drc_hist_insn[key] = cpu.m_drc_insn;
		s_drc_hist_pc[cpu.m_r[eR15]]++;
		s_drc_hist_lr[cpu.m_r[eR15]] = cpu.m_r[sRegisterTable[cpu.m_r[eCPSR] & 0xf][14]];
		// single loads/stores with an immediate offset: which 16 MB region they touch
		const uint32_t i = cpu.m_drc_insn;
		if ((i & 0x0e000000) == 0x04000000 && ((i >> 16) & 15) != 15)
		{
			uint32_t a = cpu.m_r[sRegisterTable[cpu.m_r[eCPSR] & 0xf][(i >> 16) & 15]];
			if (i & INSN_SDT_P)
				a = (i & INSN_SDT_U) ? a + (i & 0xfff) : a - (i & 0xfff);
			s_drc_hist_region[a >> 24]++;
			offs_t pa = a;
			const bool hit = cpu.xlat_hit(pa, ARM7_TLB_ABORT_D | ((i & INSN_SDT_L) ? ARM7_TLB_READ : ARM7_TLB_WRITE)) && cpu.fastram_ptr(pa);
			s_drc_hist_addr[cpu.m_r[eR15]] = (a & ~1U) | (hit ? 1 : 0);
		}
	}
	cpu.exec_arm(cpu.m_drc_insn);
	if (cpu.m_pending_interrupt)
		cpu.arm7_check_irq_state();
	cpu.m_icount -= 3;
}

void arm7_cpu_device::drc_check_irq(void *param)
{
	reinterpret_cast<arm7_cpu_device *>(param)->arm7_check_irq_state();
}

// LDM/STM (m_drc_insn, condition already passed) on fast RAM, exactly as HandleMemBlock does it:
// the same addresses, registers, base write-back, loaded PC and cycles. Only the plain case is
// done here (S clear, base not PC, a non-empty list, word-aligned, every word on fast RAM through a
// cached translation, no compiled code in a stored page, no PC in an STM list, no switch to Thumb
// by a loaded PC); m_drc_ok = 0 then, with nothing changed, and the interpreter does it
void arm7_cpu_device::drc_block_transfer(void *param)
{
	reinterpret_cast<arm7_cpu_device *>(param)->drc_block_transfer_impl();
}

void arm7_cpu_device::drc_block_transfer_impl()
{
	const uint32_t insn = m_drc_insn;
	const uint32_t rb = (insn >> 16) & 15, list = insn & 0xffff;
	const bool load = insn & INSN_BDT_L, up = insn & INSN_BDT_U, pre = insn & INSN_BDT_P;
	m_drc_ok = 0;
	if (!list || (insn & INSN_BDT_S) || rb == 15 || !m_fast_mem || (m_archFlags & ARCHFLAG_MODE26) || !(m_r[eCPSR] & SR_MODE32))
		return;
	if (!load && (list & 0x8000))
		return;
	const uint32_t n = std::popcount(list);
	const uint32_t base = GetRegister(rb);
	if (base & 3)
		return;
	// lowest address; registers go to ascending addresses in every mode
	const uint32_t lo = up ? (pre ? base + 4 : base) : (pre ? base - n * 4 : base - (n - 1) * 4);
	uint32_t *p[16];
	const int flags = ARM7_TLB_ABORT_D | (load ? ARM7_TLB_READ : ARM7_TLB_WRITE);
	uint32_t run = 0; // words after p[k - 1] in the same page and fast RAM region
	for (uint32_t k = 0; k < n; k++)
	{
		// the smallest page is 1 KB, so within one the words are contiguous in host memory too
		if (run)
		{
			p[k] = p[k - 1] + 1;
			run--;
			continue;
		}
		const offs_t va = lo + k * 4;
		offs_t pa = va;
		if (!xlat_hit(pa, flags))
			return;
		uint32_t after;
		uint32_t *const host = fastram_words(pa, after);
		if (!host)
			return;
		run = std::min<uint32_t>(after, (0x3fc - (va & 0x3ff)) / 4);
		if (!load && m_drc_page_flags)
		{
			const offs_t page = (pa - m_drc_pages_base) >> 10;
			if (page < m_drc_pages_count && m_drc_page_flags[page])
				return;
		}
		p[k] = host;
	}
	if (load && (list & 0x8000) && (*p[n - 1] & 1) && m_archRev >= 5)
		return; // to Thumb

	uint32_t k = 0;
	if (load)
	{
		for (int i = 0; i < 16; i++)
			if (BIT(list, i))
			{
				const uint32_t data = *p[k++];
				if (i == 15)
					m_r[eR15] = data & ~3;
				else
					SetRegister(i, data);
			}
		if ((insn & INSN_BDT_W) && !BIT(list, rb))
			SetRegister(rb, up ? base + n * 4 : base - n * 4);
		m_icount -= n + 2 + ((list & 0x8000) ? 2 : 0);   // nS + 1N + 1I, PC 2 more
	}
	else
	{
		for (int i = 0; i < 16; i++)
			if (BIT(list, i))
				*p[k++] = GetRegister(i);
		if (insn & INSN_BDT_W)
			SetRegister(rb, up ? base + n * 4 : base - n * 4);
		m_icount -= n + 1;                                 // (n-1)S + 2N
	}
	m_drc_ok = 1;
}

static uint64_t s_drc_compiles, s_drc_invalidates, s_drc_dirty_flushes, s_drc_compile_insns;
static struct drc_stats_dump { ~drc_stats_dump() { if (getenv("ARM_DRC_STATS")) printf("DRC compiles %llu (%llu insns) invalidated pages %llu flushes %llu\n", (unsigned long long)s_drc_compiles, (unsigned long long)s_drc_compile_insns, (unsigned long long)s_drc_invalidates, (unsigned long long)s_drc_dirty_flushes); } } s_drc_stats_dump;

void arm7_cpu_device::drc_code_written(offs_t page)
{
	s_drc_invalidates++;
	const uint8_t flags = m_drc_page_flags[page];
	m_drc_page_flags[page] = 0;
	if (flags & 2)
	{
		// code reached through a remapped page: we do not know its virtual address
		m_drc_dirty = true;
		return;
	}
	const offs_t start = m_drc_pages_base + (page << 10);
	m_drcuml->hash_invalidate_range(start, start + 0x3ff);
}

void arm7_cpu_device::drc_flush()
{
	m_drcuml->reset();
	m_drc_dirty = false;
	m_drc_flush_count++;
	s_drc_dirty_flushes++;

	// page flags cover the fast RAM (set up by the driver after device_start)
	if (!m_drc_page_flags && m_fastram_count)
	{
		offs_t lo = ~offs_t(0), hi = 0;
		for (unsigned i = 0; i < m_fastram_count; i++)
		{
			lo = std::min(lo, m_fastram[i].start);
			hi = std::max(hi, m_fastram[i].start + m_fastram[i].span);
		}
		m_drc_pages_base = lo & ~offs_t(0x3ff);
		m_drc_pages_count = ((hi - m_drc_pages_base) >> 10) + 1;
		m_drc_page_flags = std::make_unique<uint8_t[]>(m_drc_pages_count);
	}
	if (m_drc_page_flags)
		std::fill_n(&m_drc_page_flags[0], m_drc_pages_count, 0);

	try
	{
		drc_static_generate();
	}
	catch (drcuml_block::abort_compilation &)
	{
		fatalerror("%s: unable to generate static ARM recompiler code\n", tag());
	}
}

void arm7_cpu_device::drc_static_generate()
{
	drcuml_block &block(m_drcuml->begin_block(64));

	if (!m_drc_entry)
		m_drc_entry = m_drcuml->handle_alloc("entry");
	if (!m_drc_nocode)
		m_drc_nocode = m_drcuml->handle_alloc("nocode");
	if (!m_drc_out_of_cycles)
		m_drc_out_of_cycles = m_drcuml->handle_alloc("out_of_cycles");

	// entry: dispatch on the current mode and PC (Thumb goes to the interpreter)
	UML_HANDLE(block, *m_drc_entry);
	UML_LOAD(block, uml::I0, &m_r[eCPSR], 0, uml::SIZE_DWORD, uml::SCALE_x1);
	UML_TEST(block, uml::I0, T_MASK);
	block.append().exit(uml::COND_NZ, uml::parameter(DRC_EXECUTE_INTERP));
	UML_AND(block, uml::I0, uml::I0, 0xf);
	UML_LOAD(block, uml::I1, &m_r[eR15], 0, uml::SIZE_DWORD, uml::SCALE_x1);
	UML_HASHJMP(block, uml::I0, uml::I1, *m_drc_nocode);

	// no code for this PC yet: R15 = PC, compile it
	UML_HANDLE(block, *m_drc_nocode);
	UML_GETEXP(block, uml::I0);
	UML_STORE(block, &m_r[eR15], 0, uml::I0, uml::SIZE_DWORD, uml::SCALE_x1);
	UML_EXIT(block, DRC_EXECUTE_MISSING_CODE);

	// out of cycles: R15 already holds the next PC
	UML_HANDLE(block, *m_drc_out_of_cycles);
	UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);

	block.end();
}

void arm7_cpu_device::drc_execute_run()
{
	if (m_drc_dirty)
		drc_flush();

	while (true)
	{
		if (T_IS_SET(GET_CPSR))
		{
			do
				execute_run_interp(true);
			while (m_icount > 0 && T_IS_SET(GET_CPSR));
			if (m_icount <= 0)
				return;
			if (m_drc_dirty)
				drc_flush();
			continue;
		}

		const int result = m_drcuml->execute(*m_drc_entry);
		if (result == DRC_EXECUTE_OUT_OF_CYCLES)
			return;
		else if (result == DRC_EXECUTE_MISSING_CODE)
			drc_compile(GET_MODE, R15);
		else if (result == DRC_EXECUTE_INTERP)
		{
			execute_run_interp(true);
			if (m_icount <= 0)
				return;
		}
		else if (result == DRC_EXECUTE_RESET_CACHE)
			m_drc_dirty = true;

		if (m_drc_dirty)
			drc_flush();
	}
}


/***************************************************************************
    BLOCK COMPILER
***************************************************************************/

void arm7_cpu_device::drc_compile(uint32_t mode, offs_t startpc)
{
	using namespace uml;

	try
	{
		drcuml_block &block(m_drcuml->begin_block(DRC_MAX_INSNS * 96 + 64));
		code_label nextlabel = 1;
		std::vector<std::function<void ()>> stubs; // out-of-line code, emitted after the block

		const int *const regmap = sRegisterTable[mode];
		auto reg = [this, regmap] (int r) { return &m_r[regmap[r]]; };
		auto load = [&block] (uml::parameter dst, const void *p) { UML_LOAD(block, dst, p, 0, SIZE_DWORD, SCALE_x1); };
		auto store = [&block] (void *p, uml::parameter src) { UML_STORE(block, p, 0, src, SIZE_DWORD, SCALE_x1); };

		// dispatch to whatever R15 and CPSR now say
		auto dynamic_dispatch = [&] ()
		{
			load(I0, &m_r[eCPSR]);
			UML_TEST(block, I0, T_MASK);
			block.append().exit(uml::COND_NZ, uml::parameter(DRC_EXECUTE_INTERP));
			UML_AND(block, I0, I0, 0xf);
			load(I1, &m_r[eR15]);
			UML_HASHJMP(block, I0, I1, *m_drc_nocode);
		};

		// after a natively compiled instruction: cycles. The interpreter checks
		// for interrupts first, but the instructions compiled here neither
		// access devices nor change the mode or the I/F bits (those that do,
		// device loads/stores and MSR CPSR, check themselves), so nothing can
		// become pending or unmasked: that check would always do nothing.
		// Execution continues after the tail.
		auto tail = [&] (int cost, offs_t nextpc)
		{
			const code_label ooc = nextlabel++;
			load(I0, &m_icount);
			UML_SUB(block, I0, I0, cost);
			store(&m_icount, I0);
			UML_CMP(block, I0, 0);
			UML_JMPc(block, uml::COND_LE, ooc);

			stubs.emplace_back([&, ooc, nextpc] ()
			{
				UML_LABEL(block, ooc);
				UML_MOV(block, I0, nextpc);
				store(&m_r[eR15], I0);
				UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
			});
		};

		// condition check: jumps to the returned label when it fails
		auto condition = [&] (uint32_t cond) -> code_label
		{
			const code_label skip = nextlabel++;
			load(I0, &m_r[eCPSR]);
			UML_SHR(block, I0, I0, 28);
			UML_SHR(block, I1, uint32_t(s_cond_table[cond]), I0);
			UML_TEST(block, I1, 1);
			UML_JMPc(block, uml::COND_Z, skip);
			return skip;
		};

		// every other instruction: the interpreter does it, including the
		// interrupt check and cycles; then check where we are
		auto fallback = [&] (offs_t pc, uint32_t insn)
		{
			const code_label out = nextlabel++, dyn = nextlabel++, reset = nextlabel++;
			UML_MOV(block, I0, pc);
			store(&m_r[eR15], I0);
			UML_MOV(block, I0, insn);
			store(&m_drc_insn, I0);
			UML_CALLC(block, &arm7_cpu_device::drc_interp_one, this);
			load(I0, &m_icount);
			UML_CMP(block, I0, 0);
			UML_JMPc(block, uml::COND_LE, out);
			UML_LOAD(block, I0, &m_drc_dirty, 0, SIZE_BYTE, SCALE_x1);
			UML_TEST(block, I0, 0xff);
			UML_JMPc(block, uml::COND_NZ, reset);
			load(I1, &m_r[eR15]);
			UML_CMP(block, I1, pc + 4);
			UML_JMPc(block, uml::COND_NE, dyn);
			load(I0, &m_r[eCPSR]);
			UML_AND(block, I0, I0, 0x2f);
			UML_CMP(block, I0, mode);
			UML_JMPc(block, uml::COND_NE, dyn);

			stubs.emplace_back([&, out, dyn, reset] ()
			{
				UML_LABEL(block, out);
				UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
				UML_LABEL(block, reset);
				UML_EXIT(block, DRC_EXECUTE_RESET_CACHE);
				UML_LABEL(block, dyn);
				dynamic_dispatch();
			});
		};

		UML_HASH(block, mode, startpc);
		s_drc_compiles++;

		for (const auto &il : m_idle_loops)
		{
			if (il.pc != startpc)
				continue;
			bool match = true;
			for (size_t i = 0; i < il.sig.size() && match; i++)
			{
				offs_t a = il.sig_start + i * 4;
				if ((m_control & COPRO_CTRL_MMU_EN) && !translate_fast(a, ARM7_TLB_ABORT_P | ARM7_TLB_READ))
					match = false;
				else
				{
					const uint8_t *const p = fastram_ptr(a);
					match = p && *reinterpret_cast<const uint32_t *>(p) == il.sig[i];
				}
			}
			if (match)
			{
				// nothing to do until an interrupt: the first instruction's
				// cycle check ends the timeslice
				UML_MOV(block, I0, 0);
				store(&m_icount, I0);
			}
		}

		offs_t pc = startpc;
		bool ended = false;
		for (int n = 0; n < DRC_MAX_INSNS && !ended; n++, pc += 4)
		{
			s_drc_compile_insns++;
			// fetch the way the interpreter does
			offs_t phys = pc;
			if ((m_control & COPRO_CTRL_MMU_EN) && !translate_fast(phys, ARM7_TLB_ABORT_P | ARM7_TLB_READ))
			{
				// let the interpreter take the prefetch abort
				UML_MOV(block, I0, pc);
				store(&m_r[eR15], I0);
				UML_EXIT(block, DRC_EXECUTE_INTERP);
				ended = true;
				break;
			}
			const uint8_t *const host = fastram_ptr(phys & ~3);
			if (!host && m_program->get_write_ptr(phys & ~3))
			{
				// writable memory outside the fast RAM: no write tracking there,
				// so do not keep compiled code for it
				UML_MOV(block, I0, pc);
				store(&m_r[eR15], I0);
				UML_EXIT(block, DRC_EXECUTE_INTERP);
				ended = true;
				break;
			}
			const uint32_t insn = host ? *reinterpret_cast<const uint32_t *>(host) : insn_fetch_word(phys & ~3);
			if (host && m_drc_page_flags)
			{
				const offs_t page = (phys - m_drc_pages_base) >> 10;
				if (page < m_drc_pages_count)
					m_drc_page_flags[page] |= ((phys ^ pc) & ~offs_t(0x3ff)) ? 2 : 1;
			}

			const uint32_t cond = insn >> 28;
			const uint32_t cls = insn & 0x0e000000;
			const uint32_t rdn = (insn >> 12) & 15, rnn = (insn >> 16) & 15, opcode = (insn >> 21) & 15;
			const bool s = insn & INSN_S;
			const bool logical = (opcode & 6) == 0 || opcode >= 12 || opcode == 9 || opcode == 8; // AND EOR TST TEQ ORR MOV BIC MVN

			// shifter operand for data processing that can be generated here
			bool alu_ok = false;
			uint32_t shift_type = 0, shift_amount = 0;
			int sc_const = -1; // shifter carry: -1 = C unchanged, 0/1 = known
			if (cond != 15 && rdn != 15 && (insn & 0x01900000) != 0x01000000)
			{
				if (cls == 0x02000000)
				{
					alu_ok = true;
					const uint32_t by = (insn >> 8) & 15;
					if (by)
						sc_const = BIT(std::rotr<uint32_t>(insn & 0xff, by << 1), 31);
				}
				else if (cls == 0 && !(insn & 0x10) && !((insn & 15) == 15 && (s_drc_off & 32)))
				{
					shift_type = (insn >> 5) & 3;
					shift_amount = (insn >> 7) & 31;
					// LSR/ASR #0 mean #32 and ROR #0 is RRX; flag-setting logical ops
					// with a shift need the shifter carry: leave those to the interpreter
					alu_ok = (shift_amount != 0 || shift_type == 0) && !(s && logical && shift_amount != 0);
				}
				if (alu_ok && rnn == 15 && (opcode & 0xd) != 0xd)
					alu_ok = true; // Rn = PC reads PC + 8
			}

			if (alu_ok)
			{
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);

				// I1 = operand 2
				if (cls == 0x02000000)
				{
					const uint32_t by = (insn >> 8) & 15;
					UML_MOV(block, I1, std::rotr<uint32_t>(insn & 0xff, by << 1));
				}
				else
				{
					if ((insn & 15) == 15)
						UML_MOV(block, I1, pc + 8);   // Rm = PC with an immediate shift reads PC + 8
					else
						load(I1, reg(insn & 15));
					if (shift_amount)
					{
						switch (shift_type)
						{
						case 0: UML_SHL(block, I1, I1, shift_amount); break;
						case 1: UML_SHR(block, I1, I1, shift_amount); break;
						case 2: UML_SAR(block, I1, I1, shift_amount); break;
						case 3: UML_ROR(block, I1, I1, shift_amount); break;
						}
					}
				}
				// I0 = Rn
				if ((opcode & 0xd) != 0xd)
				{
					if (rnn == 15)
						UML_MOV(block, I0, pc + 8);
					else
						load(I0, reg(rnn));
				}

				// I2 = result; arithmetic flags take (a, b) = (I0, I1) or (I1, I0)
				bool add = false, sub = false, swap = false;
				switch (opcode)
				{
				case OPCODE_AND: case OPCODE_TST: UML_AND(block, I2, I0, I1); break;
				case OPCODE_EOR: case OPCODE_TEQ: UML_XOR(block, I2, I0, I1); break;
				case OPCODE_SUB: case OPCODE_CMP: UML_SUB(block, I2, I0, I1); sub = true; break;
				case OPCODE_RSB: UML_SUB(block, I2, I1, I0); sub = true; swap = true; break;
				case OPCODE_ADD: case OPCODE_CMN: UML_ADD(block, I2, I0, I1); add = true; break;
				case OPCODE_ADC:
					load(I3, &m_r[eCPSR]);
					UML_ROLAND(block, I3, I3, 3, 1); // C
					UML_ADD(block, I2, I0, I1);
					UML_ADD(block, I2, I2, I3);
					add = true;
					break;
				case OPCODE_SBC:
					load(I3, &m_r[eCPSR]);
					UML_ROLAND(block, I3, I3, 3, 1);
					UML_XOR(block, I3, I3, 1);       // borrow = !C
					UML_SUB(block, I2, I0, I1);
					UML_SUB(block, I2, I2, I3);
					sub = true;
					break;
				case OPCODE_RSC:
					load(I3, &m_r[eCPSR]);
					UML_ROLAND(block, I3, I3, 3, 1);
					UML_XOR(block, I3, I3, 1);
					UML_SUB(block, I2, I1, I0);
					UML_SUB(block, I2, I2, I3);
					sub = true;
					swap = true;
					break;
				case OPCODE_ORR: UML_OR(block, I2, I0, I1); break;
				case OPCODE_MOV: UML_MOV(block, I2, I1); break;
				case OPCODE_BIC: UML_XOR(block, I3, I1, ~uint32_t(0)); UML_AND(block, I2, I0, I3); break;
				default: /* MVN */ UML_XOR(block, I2, I1, ~uint32_t(0)); break;
				}

				if (s)
				{
					// N and Z from the result (as HandleALUNZFlags)
					UML_AND(block, I5, I2, 0x80000000);
					UML_CMP(block, I2, 0);
					UML_SETc(block, uml::COND_Z, I6);
					UML_SHL(block, I6, I6, Z_BIT);
					UML_OR(block, I5, I5, I6);
					load(I7, &m_r[eCPSR]);
					if (add || sub)
					{
						const uml::parameter a = swap ? I1 : I0, b = swap ? I0 : I1;
						if (add)
						{
							// C = (a & b) | (a & ~r) | (b & ~r), V = ~(a ^ b) & (a ^ r)
							UML_XOR(block, I4, I2, ~uint32_t(0));
							UML_AND(block, I3, a, b);
							UML_AND(block, I6, a, I4);
							UML_OR(block, I3, I3, I6);
							UML_AND(block, I6, b, I4);
							UML_OR(block, I3, I3, I6);
							UML_XOR(block, I4, a, b);
							UML_XOR(block, I4, I4, ~uint32_t(0));
							UML_XOR(block, I6, a, I2);
							UML_AND(block, I4, I4, I6);
						}
						else
						{
							// C = (a & ~b) | (a & ~r) | (~b & ~r), V = (a ^ b) & (a ^ r)
							UML_XOR(block, I4, b, ~uint32_t(0)); // ~b
							UML_AND(block, I3, a, I4);
							UML_XOR(block, I6, I2, ~uint32_t(0)); // ~r
							UML_AND(block, I4, I4, I6);           // ~b & ~r
							UML_OR(block, I3, I3, I4);
							UML_AND(block, I6, a, I6);            // a & ~r
							UML_OR(block, I3, I3, I6);
							UML_XOR(block, I4, a, b);
							UML_XOR(block, I6, a, I2);
							UML_AND(block, I4, I4, I6);
						}
						UML_ROLAND(block, I3, I3, 30, C_MASK);  // bit 31 -> C (bit 29)
						UML_ROLAND(block, I4, I4, 29, V_MASK);  // bit 31 -> V (bit 28)
						UML_AND(block, I7, I7, ~(N_MASK | Z_MASK | C_MASK | V_MASK));
						UML_OR(block, I7, I7, I3);
						UML_OR(block, I7, I7, I4);
					}
					else
					{
						// logical: C from the shifter (or unchanged), V unchanged
						if (sc_const >= 0)
						{
							UML_AND(block, I7, I7, ~(N_MASK | Z_MASK | C_MASK));
							if (sc_const)
								UML_OR(block, I7, I7, C_MASK);
						}
						else
							UML_AND(block, I7, I7, ~(N_MASK | Z_MASK));
					}
					UML_OR(block, I7, I7, I5);
					store(&m_r[eCPSR], I7);
				}
				if ((insn & 0x01800000) != 0x01000000) // not TST/TEQ/CMP/CMN
					store(reg(rdn), I2);

				tail(1, pc + 4); // HandleALU: +2, then -3
				if (cond != 14)
				{
					const code_label done = nextlabel++;
					UML_JMP(block, done);
					UML_LABEL(block, skip);
					tail(1, pc + 4); // unexecuted: +2, then -3
					UML_LABEL(block, done);
				}
				continue;
			}

			// MUL/MLA and UMULL/UMLAL/SMULL/SMLAL without S, no PC operands, with HandleMul's and
			// HandleU/SMulLong's cycles: 1 + m (m = 1..4 from the size of Rs), +1 for a long result,
			// +1 for accumulating (net of their +3 and the interpreter fallback's -3)
			const bool mul32 = (insn & 0x0fd000f0) == 0x00000090 && m_archRev >= 3;
			const bool mul64 = (insn & 0x0f9000f0) == 0x00800090 && m_archRev >= 4;
			if ((mul32 || mul64) && cond != 15 && !(s_drc_off & 1024) && rdn != 15 && rnn != 15
					&& (insn & 15) != 15 && ((insn >> 8) & 15) != 15)
			{
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				const bool acc = insn & INSN_MUL_A, sgn = mul64 && (insn & 0x00400000);
				load(I0, reg(insn & 15));          // Rm
				load(I1, reg((insn >> 8) & 15));   // Rs
				if (mul32)
				{
					UML_MULULW(block, I2, I0, I1);
					if (acc)
					{
						load(I3, reg(rdn));            // Rn (bits 15-12)
						UML_ADD(block, I2, I2, I3);
					}
					store(reg(rnn), I2);               // Rd (bits 19-16)
				}
				else
				{
					if (sgn)
						UML_MULS(block, I2, I3, I0, I1);
					else
						UML_MULU(block, I2, I3, I0, I1);
					if (acc)
					{
						load(I4, reg(rdn));            // RdLo
						load(I5, reg(rnn));            // RdHi
						UML_ADD(block, I2, I2, I4);
						UML_ADDC(block, I3, I3, I5);
					}
					store(reg(rnn), I3);               // RdHi, then RdLo (which wins if they are the same)
					store(reg(rdn), I2);
				}
				// m from Rs: MUL takes |Rs| as unsigned, SMULL |Rs| as signed (so 0x80000000 counts as small),
				// UMULL Rs as it is
				if (mul32 || sgn)
				{
					UML_SAR(block, I2, I1, 31);
					UML_XOR(block, I1, I1, I2);
					UML_SUB(block, I1, I1, I2);
				}
				const uml::condition_t ge = sgn ? uml::COND_GE : uml::COND_AE;
				UML_MOV(block, I4, 1 + (mul64 ? 1 : 0) + (acc ? 1 : 0) + 1);
				for (const uint32_t t : { 0x100U, 0x10000U, 0x1000000U })
				{
					UML_CMP(block, I1, t);
					UML_SETc(block, ge, I2);
					UML_ADD(block, I4, I4, I2);
				}
				{
					const code_label ooc = nextlabel++;
					load(I0, &m_icount);
					UML_SUB(block, I0, I0, I4);
					store(&m_icount, I0);
					UML_CMP(block, I0, 0);
					UML_JMPc(block, uml::COND_LE, ooc);
					stubs.emplace_back([&, ooc, pc] ()
					{
						UML_LABEL(block, ooc);
						UML_MOV(block, I0, pc + 4);
						store(&m_r[eR15], I0);
						UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
					});
				}
				if (cond != 14)
				{
					const code_label done = nextlabel++;
					UML_JMP(block, done);
					UML_LABEL(block, skip);
					tail(1, pc + 4); // unexecuted: +2, then -3
					UML_LABEL(block, done);
				}
				continue;
			}

			// Loads and stores, Rd != PC, inline:
			//   LDR/STR(B) Rd, [Rn, #+-imm]{!} and [Rn], #+-imm (not the T forms)
			//   LDR/STR(B) Rd, [Rn, +-Rm, LSL #n]{!} and [Rn], +-Rm, LSL #n (Rn, Rm not PC)
			//   LDRH/STRH/LDRSB/LDRSH the same ways, Rm unshifted (not LDRD/STRD)
			// on fast RAM directly, elsewhere through the address space. Anything else
			// (translation miss, unaligned word or halfword, store to a code page)
			// goes through the interpreter, which then does the whole instruction
			// including the base write-back.
			const uint32_t sh = (insn >> 5) & 3, rmn = insn & 15;
			const bool sdt_word = (cls == 0x04000000 || (cls == 0x06000000 && !(insn & 0x70) && rmn != 15 && rnn != 15))
					&& ((insn & INSN_SDT_P) || !(insn & INSN_SDT_W));
			const bool sdt_half = cls == 0 && (insn & 0x00000090) == 0x00000090 && sh != 0
					&& ((insn & 0x00400000) || (!(insn & 0xf00) && rmn != 15 && rnn != 15))
					&& ((insn & INSN_SDT_L) || sh == 1) && ((insn & INSN_SDT_P) || !(insn & INSN_SDT_W));
			const bool sdt_regoff = sdt_word ? cls == 0x06000000 : !(insn & 0x00400000);
			const bool sdt_wb = !(insn & INSN_SDT_P) || (insn & INSN_SDT_W);
			const bool sdt_topc = rdn == 15 && sdt_word && (insn & INSN_SDT_L) && !(insn & INSN_SDT_B) && rnn != 15;
			if ((sdt_word || sdt_half) && !(s_drc_off & 64) && !(sdt_wb && (s_drc_off & 1)) && !(sdt_half && (s_drc_off & 2))
					&& !(sdt_topc && (s_drc_off & 4)) && !(sdt_regoff && (s_drc_off & 256)) && (rdn != 15 || sdt_topc) && cond != 15
					&& !(sdt_wb && rnn == 15) && m_fast_mem && m_fastram_count)
			{
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				const code_label slow = nextlabel++, resume = nextlabel++;
				const bool isload = insn & INSN_SDT_L, pre = insn & INSN_SDT_P, up = insn & INSN_SDT_U;
				// access size in bytes, and sign extension for loads
				const int size = sdt_word ? ((insn & INSN_SDT_B) ? 1 : 4) : (sh == 2 ? 1 : 2);
				const bool sext = sdt_half && sh >= 2;
				const uint32_t off = sdt_regoff ? 0 : sdt_word ? (insn & 0xfff) : (((insn >> 4) & 0xf0) | (insn & 0x0f));
				const uml::operand_size usize = size == 1 ? SIZE_BYTE : size == 2 ? SIZE_WORD : SIZE_DWORD;

				// I5 = register offset (Rm, LSL #n)
				if (sdt_regoff)
				{
					load(I5, reg(rmn));
					if (sdt_word && ((insn >> 7) & 31))
						UML_SHL(block, I5, I5, (insn >> 7) & 31);
				}

				// I0 = virtual address (the base for post-indexing)
				if (rnn == 15)
					UML_MOV(block, I0, up ? pc + 8 + off : pc + 8 - off);
				else
				{
					load(I0, reg(rnn));
					if ((off || sdt_regoff) && pre)
					{
						if (up)
							UML_ADD(block, I0, I0, sdt_regoff ? uml::parameter(I5) : uml::parameter(off));
						else
							UML_SUB(block, I0, I0, sdt_regoff ? uml::parameter(I5) : uml::parameter(off));
					}
				}
				if (size > 1)
				{
					UML_TEST(block, I0, size - 1);
					UML_JMPc(block, uml::COND_NZ, slow);
				}
				// I1 = physical address through the translation cache
				if (m_control & COPRO_CTRL_MMU_EN)
				{
					const int flags = ARM7_TLB_ABORT_D | (isload ? ARM7_TLB_READ : ARM7_TLB_WRITE);
					UML_SHR(block, I2, I0, 10);
					UML_SHR(block, I3, I0, 24);           // I3 = xlat_slot(I0)
					UML_XOR(block, I3, I3, I2);
					UML_AND(block, I3, I3, XLAT_CACHE_SIZE - 1);
					if (mode & 0xf)
						UML_OR(block, I2, I2, 0x80000000);
					UML_LOAD(block, I4, &m_xlat_cache[flags & 0xf][0].tag, I3, SIZE_DWORD, SCALE_x8);
					UML_CMP(block, I4, I2);
					UML_JMPc(block, uml::COND_NE, slow);
					UML_LOAD(block, I1, &m_xlat_cache[flags & 0xf][0].ppage, I3, SIZE_DWORD, SCALE_x8);
					UML_AND(block, I2, I0, 0x3ff);
					UML_OR(block, I1, I1, I2);
				}
				else
					UML_MOV(block, I1, I0);

				// base write-back (after the access; for a load into the base the loaded value wins, as in the interpreter)
				auto writeback = [&] ()
				{
					if (!sdt_wb)
						return;
					if (pre || (!off && !sdt_regoff))
						store(reg(rnn), I0);
					else
					{
						if (up)
							UML_ADD(block, I2, I0, sdt_regoff ? uml::parameter(I5) : uml::parameter(off));
						else
							UML_SUB(block, I2, I0, sdt_regoff ? uml::parameter(I5) : uml::parameter(off));
						store(reg(rnn), I2);
					}
				};

				// fast RAM range (I2 = offset into it); code page check for stores
				const code_label found = nextlabel++;
				for (unsigned r = 0; r < m_fastram_count; r++)
				{
					const code_label next = nextlabel++;
					UML_SUB(block, I2, I1, m_fastram[r].start);
					UML_CMP(block, I2, m_fastram[r].span - (size - 1));
					UML_JMPc(block, uml::COND_A, next);
					if (!isload && m_drc_page_flags)
					{
						UML_SUB(block, I3, I1, m_drc_pages_base);
						UML_SHR(block, I3, I3, 10);
						UML_LOAD(block, I4, &m_drc_page_flags[0], I3, SIZE_BYTE, SCALE_x1);
						UML_TEST(block, I4, 0xff);
						UML_JMPc(block, uml::COND_NZ, slow);
					}
					if (isload && sdt_topc)
					{
						// to PC: an ARMv5 load with bit 0 set switches to Thumb, leave that to the interpreter
						// (nothing has been changed yet); otherwise PC = value & ~3
						UML_LOAD(block, I3, m_fastram[r].base, I2, usize, SCALE_x1);
						UML_TEST(block, I3, 1);
						UML_JMPc(block, uml::COND_NZ, slow);
						writeback();
						UML_AND(block, I3, I3, ~uint32_t(3));
						store(&m_r[eR15], I3);
					}
					else if (isload)
					{
						UML_LOAD(block, I3, m_fastram[r].base, I2, usize, SCALE_x1);
						if (sext)
							UML_SEXT(block, I3, I3, usize);
						writeback();
						store(reg(rdn), I3);
					}
					else
					{
						load(I3, reg(rdn));
						UML_STORE(block, m_fastram[r].base, I2, I3, usize, SCALE_x1);
						writeback();
					}
					UML_JMP(block, found);
					UML_LABEL(block, next);
				}
				if (sdt_topc || (s_drc_off & 512))
					UML_JMP(block, slow);
				else
				{
					// other memory (devices): through the address space, as the interpreter does it
					// (the translation hit, so nothing can abort). A device may raise an interrupt,
					// or write code by DMA: check as the interpreter fallback does
					const code_label check = nextlabel++;
					if (isload)
					{
						UML_READ(block, I3, I1, usize, SPACE_PROGRAM);
						if (sext)
							UML_SEXT(block, I3, I3, usize);
						writeback();
						store(reg(rdn), I3);
					}
					else
					{
						load(I3, reg(rdn));
						UML_WRITE(block, I1, I3, usize, SPACE_PROGRAM);
						writeback();
					}
					UML_LOAD(block, I0, &m_pending_interrupt, 0, SIZE_BYTE, SCALE_x1);
					UML_LOAD(block, I1, &m_drc_dirty, 0, SIZE_BYTE, SCALE_x1);
					UML_OR(block, I0, I0, I1);
					UML_TEST(block, I0, 0xff);
					UML_JMPc(block, uml::COND_NZ, check);

					stubs.emplace_back([&, check, pc, isload] ()
					{
						const code_label ooc = nextlabel++, reset = nextlabel++;
						UML_LABEL(block, check);
						UML_MOV(block, I0, pc + 4);
						store(&m_r[eR15], I0);
						UML_CALLC(block, &arm7_cpu_device::drc_check_irq, this);
						load(I0, &m_icount);
						UML_SUB(block, I0, I0, isload ? 3 : 2);
						store(&m_icount, I0);
						UML_CMP(block, I0, 0);
						UML_JMPc(block, uml::COND_LE, ooc);
						UML_LOAD(block, I0, &m_drc_dirty, 0, SIZE_BYTE, SCALE_x1);
						UML_TEST(block, I0, 0xff);
						UML_JMPc(block, uml::COND_NZ, reset);
						dynamic_dispatch();
						UML_LABEL(block, ooc);
						UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
						UML_LABEL(block, reset);
						UML_EXIT(block, DRC_EXECUTE_RESET_CACHE);
					});
				}
				UML_LABEL(block, found);
				if (sdt_topc)
				{
					// LDR PC: 5 cycles (HandleMemSingle), then wherever it goes; the interpreter
					// fallback (slow) continues through its own dynamic dispatch
					const code_label ooc = nextlabel++;
					load(I0, &m_icount);
					UML_SUB(block, I0, I0, 5);
					store(&m_icount, I0);
					UML_CMP(block, I0, 0);
					UML_JMPc(block, uml::COND_LE, ooc);
					load(I1, &m_r[eR15]);
					UML_HASHJMP(block, mode, I1, *m_drc_nocode);
					stubs.emplace_back([&, ooc] ()
					{
						UML_LABEL(block, ooc);
						UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
					});
				}
				else
					tail(isload ? 3 : 2, pc + 4); // loads 3, stores 2 (HandleMemSingle, HandleHalfWordDT)
				UML_LABEL(block, resume);
				if (sdt_topc && cond == 14)
				{
					// back from the interpreter at PC + 4 (its own dispatch handles any other target)
					UML_MOV(block, I0, pc + 4);
					store(&m_r[eR15], I0);
					UML_HASHJMP(block, mode, pc + 4, *m_drc_nocode);
					ended = true;
				}

				stubs.emplace_back([&, slow, resume, pc, insn] ()
				{
					UML_LABEL(block, slow);
					fallback(pc, insn);
					UML_JMP(block, resume);
				});

				if (cond != 14)
				{
					const code_label done = nextlabel++;
					UML_JMP(block, done);
					UML_LABEL(block, skip);
					tail(1, pc + 4);
					UML_LABEL(block, done);
				}
				continue;
			}

			// MRS Rd, SPSR (Rd != PC; CPSR in modes without one, as the register table maps it): 1 cycle
			if ((insn & 0x0fff0fff) == 0x014f0000 && rdn != 15 && cond != 15 && !(s_drc_off & 16))
			{
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				load(I0, reg(SPSR));
				store(reg(rdn), I0);
				tail(1, pc + 4);
				if (cond != 14)
				{
					const code_label done = nextlabel++;
					UML_JMP(block, done);
					UML_LABEL(block, skip);
					tail(1, pc + 4);
					UML_LABEL(block, done);
				}
				continue;
			}

			// MSR CPSR/SPSR, fields, Rm or #imm (Rm != PC): HandlePSRTransfer's field rules, 1 cycle. A CPSR
			// write that would change the mode or the T bit goes to the interpreter (checked before anything
			// changes); one that can unmask interrupts checks for them, as the interpreter fallback does
			const bool msr_reg = (insn & 0x0fb0fff0) == 0x0120f000 && (insn & 15) != 15;
			const bool msr_imm = (insn & 0x0fb0f000) == 0x0320f000;
			if ((msr_reg || msr_imm) && cond != 15 && !(s_drc_off & 2048) && m_archRev < 6
					&& !(m_archFlags & (ARCHFLAG_MODE26 | ARCHFLAG_ONLY26)))
			{
				const bool spsr = insn & 0x00400000;
				const bool priv = (mode & MODE_FLAG) != eARM7_MODE_USER;
				const bool has_spsr = priv && (mode & MODE_FLAG) != eARM7_MODE_SYS;
				uint32_t mask = 0;
				if (spsr ? has_spsr : priv)
					mask |= (BIT(insn, 16) ? 0x000000ffU : 0) | (BIT(insn, 17) ? 0x0000ff00U : 0) | (BIT(insn, 18) ? 0x00ff0000U : 0);
				if ((spsr ? has_spsr : true) && BIT(insn, 19))
					mask |= 0xf8000000U;
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				const code_label slow = nextlabel++, resume = nextlabel++, irq = nextlabel++;
				const bool cfield = !spsr && (mask & 0xff);
				if (mask)
				{
					if (msr_imm)
						UML_MOV(block, I1, std::rotr<uint32_t>(insn & 0xff, ((insn >> 8) & 15) * 2) & mask);
					else
					{
						load(I1, reg(insn & 15));
						UML_AND(block, I1, I1, mask);
					}
					load(I0, spsr ? reg(SPSR) : &m_r[eCPSR]);
					UML_AND(block, I2, I0, ~mask);
					UML_OR(block, I2, I2, I1);
					if (!spsr)
						UML_OR(block, I2, I2, 0x10);            // set_cpsr: 32-bit modes only
					if (cfield)
					{
						UML_XOR(block, I3, I2, I0);
						UML_TEST(block, I3, MODE_FLAG | 0x10 | T_MASK);
						UML_JMPc(block, uml::COND_NZ, slow);
					}
					store(spsr ? reg(SPSR) : &m_r[eCPSR], I2);
					if (cfield)
					{
						UML_LOAD(block, I0, &m_pending_interrupt, 0, SIZE_BYTE, SCALE_x1);
						UML_TEST(block, I0, 0xff);
						UML_JMPc(block, uml::COND_NZ, irq);
					}
				}
				tail(1, pc + 4);
				UML_LABEL(block, resume);
				if (cfield)
				{
					stubs.emplace_back([&, slow, resume, irq, pc, insn] ()
					{
						const code_label ooc = nextlabel++;
						UML_LABEL(block, slow);
						fallback(pc, insn);
						UML_JMP(block, resume);
						UML_LABEL(block, irq);
						UML_MOV(block, I0, pc + 4);
						store(&m_r[eR15], I0);
						UML_CALLC(block, &arm7_cpu_device::drc_check_irq, this);
						load(I0, &m_icount);
						UML_SUB(block, I0, I0, 1);
						store(&m_icount, I0);
						UML_CMP(block, I0, 0);
						UML_JMPc(block, uml::COND_LE, ooc);
						dynamic_dispatch();
						UML_LABEL(block, ooc);
						UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
					});
				}
				if (cond != 14)
				{
					const code_label done = nextlabel++;
					UML_JMP(block, done);
					UML_LABEL(block, skip);
					tail(1, pc + 4);
					UML_LABEL(block, done);
				}
				continue;
			}

			// MRS Rd, CPSR (Rd != PC): 1 cycle (PSR transfer)
			if ((insn & 0x0fff0fff) == 0x010f0000 && rdn != 15 && cond != 15 && !(s_drc_off & 16))
			{
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				load(I0, &m_r[eCPSR]);
				store(reg(rdn), I0);
				tail(1, pc + 4);
				if (cond != 14)
				{
					const code_label done = nextlabel++;
					UML_JMP(block, done);
					UML_LABEL(block, skip);
					tail(1, pc + 4);
					UML_LABEL(block, done);
				}
				continue;
			}

			// MOV PC, Rm (Rm != PC, no S, no shift): return; 3 cycles as the interpreter
			// (HandleALU: PC written)
			if ((insn & 0x0ffffff0) == 0x01a0f000 && (insn & 15) != 15 && cond != 15 && !(s_drc_off & 8))
			{
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				const code_label ooc = nextlabel++;
				load(I1, reg(insn & 15));
				UML_AND(block, I1, I1, ~uint32_t(3));
				store(&m_r[eR15], I1);
				load(I0, &m_icount);
				UML_SUB(block, I0, I0, 3);
				store(&m_icount, I0);
				UML_CMP(block, I0, 0);
				UML_JMPc(block, uml::COND_LE, ooc);
				UML_HASHJMP(block, mode, I1, *m_drc_nocode);
				stubs.emplace_back([&, ooc] ()
				{
					UML_LABEL(block, ooc);
					UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
				});
				if (cond != 14)
				{
					UML_LABEL(block, skip);
					tail(1, pc + 4);
				}
				else
					ended = true;
				continue;
			}

			// LDM / STM: drc_block_transfer does the plain case (cycles included); anything it
			// leaves goes through the interpreter
			if (cls == 0x08000000 && cond != 15 && !(s_drc_off & 128))
			{
				const bool topc = (insn & INSN_BDT_L) && (insn & 0x8000);
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				const code_label slow = nextlabel++, resume = nextlabel++, ooc = nextlabel++;
				UML_MOV(block, I0, insn);
				store(&m_drc_insn, I0);
				UML_CALLC(block, &arm7_cpu_device::drc_block_transfer, this);
				UML_LOAD(block, I0, &m_drc_ok, 0, SIZE_BYTE, SCALE_x1);
				UML_TEST(block, I0, 0xff);
				UML_JMPc(block, uml::COND_Z, slow);
				load(I0, &m_icount);
				UML_CMP(block, I0, 0);
				UML_JMPc(block, uml::COND_LE, ooc);
				if (topc)
				{
					// R15 = the loaded PC: go there (the mode is unchanged)
					load(I1, &m_r[eR15]);
					UML_HASHJMP(block, mode, I1, *m_drc_nocode);
				}
				UML_LABEL(block, resume);
				if (topc && cond == 14)
				{
					// back from the interpreter at PC + 4 (its own dispatch handles any other target)
					UML_MOV(block, I0, pc + 4);
					store(&m_r[eR15], I0);
					UML_HASHJMP(block, mode, pc + 4, *m_drc_nocode);
					ended = true;
				}

				stubs.emplace_back([&, slow, resume, ooc, pc, insn, topc] ()
				{
					UML_LABEL(block, ooc);
					if (!topc)
					{
						UML_MOV(block, I0, pc + 4);
						store(&m_r[eR15], I0);
					}
					UML_EXIT(block, DRC_EXECUTE_OUT_OF_CYCLES);
					UML_LABEL(block, slow);
					fallback(pc, insn);
					UML_JMP(block, resume);
				});

				if (cond != 14)
				{
					const code_label done = nextlabel++;
					UML_JMP(block, done);
					UML_LABEL(block, skip);
					tail(1, pc + 4);
					UML_LABEL(block, done);
				}
				continue;
			}

			// B / BL
			if (cls == 0x0a000000 && cond != 15)
			{
				const offs_t target = pc + 8 + util::sext((insn & INSN_BRANCH) << 2, 26);
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				if (insn & INSN_BL)
				{
					UML_MOV(block, I0, pc + 4);
					store(reg(14), I0);
				}
				tail(3, target);
				UML_MOV(block, I0, target);
				store(&m_r[eR15], I0);
				UML_HASHJMP(block, mode, target, *m_drc_nocode);
				if (cond != 14)
				{
					UML_LABEL(block, skip);
					tail(1, pc + 4);
				}
				else
					ended = true;
				continue;
			}

			// anything else
			fallback(pc, insn);

			// stop after instructions that always leave the sequence
			if (cond == 14)
			{
				const bool writes_pc =
						((cls == 0 || cls == 0x02000000) && rdn == 15 && (insn & 0x01900000) != 0x01000000) // data processing to PC
						|| ((cls == 0x04000000 || cls == 0x06000000) && (insn & INSN_SDT_L) && rdn == 15) // LDR PC
						|| (cls == 0x08000000 && (insn & INSN_BDT_L) && (insn & 0x8000))                   // LDM with PC
						|| ((insn & 0x0ffffff0) == 0x012fff10)                                               // BX
						|| ((insn & 0x0f000000) == 0x0f000000);                                              // SWI
				if (writes_pc)
				{
					UML_MOV(block, I0, pc + 4);
					store(&m_r[eR15], I0);
					UML_HASHJMP(block, mode, pc + 4, *m_drc_nocode);
					ended = true;
				}
			}
		}

		if (!ended)
		{
			UML_MOV(block, I0, pc);
			store(&m_r[eR15], I0);
			UML_HASHJMP(block, mode, pc, *m_drc_nocode);
		}

		for (size_t i = 0; i < stubs.size(); i++)
		{
			auto stub = std::move(stubs[i]); // stubs may add stubs
			stub();
		}

		block.end();
	}
	catch (drcuml_block::abort_compilation &)
	{
		// out of cache: start over
		m_drc_dirty = true;
	}
}
