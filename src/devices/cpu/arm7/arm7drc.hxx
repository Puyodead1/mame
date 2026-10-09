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

void arm7_cpu_device::drc_interp_one(void *param)
{
	arm7_cpu_device &cpu = *reinterpret_cast<arm7_cpu_device *>(param);
	cpu.exec_arm(cpu.m_drc_insn);
	if (cpu.m_pending_interrupt)
		cpu.arm7_check_irq_state();
	cpu.m_icount -= 3;
}

void arm7_cpu_device::drc_check_irq(void *param)
{
	reinterpret_cast<arm7_cpu_device *>(param)->arm7_check_irq_state();
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
		// access memory nor change the mode or the I/F bits, so nothing can
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
				else if (cls == 0 && !(insn & 0x10) && (insn & 15) != 15)
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

			// LDR/STR(B) Rd, [Rn, #+-imm] (no write-back, Rd != PC): fast RAM
			// inline, anything else (other memory, translation miss, unaligned
			// word, store to a code page) through the interpreter
			if ((insn & 0x0f200000) == 0x05000000 && rdn != 15 && cond != 15 && m_fast_mem && m_fastram_count)
			{
				const code_label skip = (cond != 14) ? condition(cond) : code_label(0);
				const code_label slow = nextlabel++, resume = nextlabel++;
				const bool isload = insn & INSN_SDT_L, isbyte = insn & INSN_SDT_B;
				const uint32_t off = insn & 0xfff;

				// I0 = virtual address
				if (rnn == 15)
					UML_MOV(block, I0, (insn & INSN_SDT_U) ? pc + 8 + off : pc + 8 - off);
				else
				{
					load(I0, reg(rnn));
					if (off)
					{
						if (insn & INSN_SDT_U)
							UML_ADD(block, I0, I0, off);
						else
							UML_SUB(block, I0, I0, off);
					}
				}
				if (!isbyte)
				{
					UML_TEST(block, I0, 3);
					UML_JMPc(block, uml::COND_NZ, slow);
				}
				// I1 = physical address through the translation cache
				if (m_control & COPRO_CTRL_MMU_EN)
				{
					const int flags = ARM7_TLB_ABORT_D | (isload ? ARM7_TLB_READ : ARM7_TLB_WRITE);
					UML_SHR(block, I2, I0, 10);
					UML_AND(block, I3, I2, XLAT_CACHE_SIZE - 1);
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

				// fast RAM range (I2 = offset into it); code page check for stores
				const code_label found = nextlabel++;
				for (unsigned r = 0; r < m_fastram_count; r++)
				{
					const code_label next = nextlabel++;
					UML_SUB(block, I2, I1, m_fastram[r].start);
					UML_CMP(block, I2, m_fastram[r].span - (isbyte ? 0 : 3));
					UML_JMPc(block, uml::COND_A, next);
					if (!isload && m_drc_page_flags)
					{
						UML_SUB(block, I3, I1, m_drc_pages_base);
						UML_SHR(block, I3, I3, 10);
						UML_LOAD(block, I4, &m_drc_page_flags[0], I3, SIZE_BYTE, SCALE_x1);
						UML_TEST(block, I4, 0xff);
						UML_JMPc(block, uml::COND_NZ, slow);
					}
					if (isload)
					{
						UML_LOAD(block, I3, m_fastram[r].base, I2, isbyte ? SIZE_BYTE : SIZE_DWORD, SCALE_x1);
						store(reg(rdn), I3);
					}
					else
					{
						load(I3, reg(rdn));
						UML_STORE(block, m_fastram[r].base, I2, I3, isbyte ? SIZE_BYTE : SIZE_DWORD, SCALE_x1);
					}
					UML_JMP(block, found);
					UML_LABEL(block, next);
				}
				UML_JMP(block, slow);
				UML_LABEL(block, found);
				tail(isload ? 3 : 2, pc + 4); // loads 3, stores 2 (HandleMemSingle)
				UML_LABEL(block, resume);

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
