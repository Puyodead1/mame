// license:BSD-3-Clause
// copyright-holders:Steve Ellenoff,R. Belmont,Ryan Holtz
/* ARM7 core helper Macros / Functions */

/* Macros that need to be defined according to the cpu implementation specific need */
#define ARM7REG(reg)        m_r[reg]
#define ARM7_ICOUNT         m_icount


/***************
 * helper funcs
 ***************/

// TODO LD:
//  - SIGN_BITS_DIFFER = THUMB_SIGN_BITS_DIFFER
//  - do while (0)
//  - HandleALUAddFlags = HandleThumbALUAddFlags except for PC incr
//  - HandleALUSubFlags = HandleThumbALUSubFlags except for PC incr

#define IsNeg(i) ((i) >> 31)
#define IsPos(i) ((~(i)) >> 31)

/* Set NZCV flags for ADDS / SUBS */
#define HandleALUAddFlags(rd, rn, op2)                                                \
	if (insn & INSN_S)                                                                  \
	set_cpsr_nzcv(((GET_CPSR & ~(N_MASK | Z_MASK | V_MASK | C_MASK))                       \
				| (((!SIGN_BITS_DIFFER(rn, op2)) && SIGN_BITS_DIFFER(rn, rd)) << V_BIT) \
				| (((IsNeg(rn) & IsNeg(op2)) | (IsNeg(rn) & IsPos(rd)) | (IsNeg(op2) & IsPos(rd))) ? C_MASK : 0) \
				| HandleALUNZFlags(rd)));                                               \
	R15 += 4;

#define HandleThumbALUAddFlags(rd, rn, op2)                                                       \
	set_cpsr_nzcv(((GET_CPSR & ~(N_MASK | Z_MASK | V_MASK | C_MASK))                                   \
				| (((!THUMB_SIGN_BITS_DIFFER(rn, op2)) && THUMB_SIGN_BITS_DIFFER(rn, rd)) << V_BIT) \
				| (((IsNeg(rn) & IsNeg(op2)) | (IsNeg(rn) & IsPos(rd)) | (IsNeg(op2) & IsPos(rd))) ? C_MASK : 0) \
				| HandleALUNZFlags(rd)));                                                           \
	R15 += 2;

#define DRCHandleThumbALUAddFlags(rd, rn, op2)                                                  \
	UML_AND(block, DRC_CPSR, DRC_CPSR, ~(N_MASK | Z_MASK | V_MASK | C_MASK));                   \
	DRCHandleALUNZFlags(rd);                                                                    \
	UML_XOR(block, uml::I1, rn, ~0);                                                            \
	UML_CMP(block, uml::I1, op2);                                                               \
	UML_MOVc(block, uml::COND_B, uml::I1, C_BIT);                                               \
	UML_MOVc(block, uml::COND_AE, uml::I1, 0);                                                  \
	UML_OR(block, uml::I0, uml::I0, uml::I1);                                                   \
	UML_XOR(block, uml::I1, rn, op2);                                                           \
	UML_XOR(block, uml::I2, rn, rd);                                                            \
	UML_AND(block, uml::I1, uml::I1, uml::I2);                                                  \
	UML_TEST(block, uml::I1, 1 << 31);                                                          \
	UML_MOVc(block, uml::COND_NZ, uml::I1, V_BIT);                                              \
	UML_MOVc(block, uml::COND_Z, uml::I1, 0);                                                   \
	UML_OR(block, uml::I0, uml::I0, uml::I1);                                                   \
	UML_OR(block, DRC_CPSR, DRC_CPSR, uml::I0);                                                  \
	UML_ADD(block, DRC_PC, DRC_PC, 2);

#define HandleALUSubFlags(rd, rn, op2)                                                                         \
	if (insn & INSN_S)                                                                                           \
	set_cpsr_nzcv(((GET_CPSR & ~(N_MASK | Z_MASK | V_MASK | C_MASK))                                                \
				| ((SIGN_BITS_DIFFER(rn, op2) && SIGN_BITS_DIFFER(rn, rd)) << V_BIT)                             \
				| (((IsNeg(rn) & IsPos(op2)) | (IsNeg(rn) & IsPos(rd)) | (IsPos(op2) & IsPos(rd))) ? C_MASK : 0) \
				| HandleALUNZFlags(rd)));                                                                        \
	R15 += 4;

#define HandleThumbALUSubFlags(rd, rn, op2)                                                                    \
	set_cpsr_nzcv(((GET_CPSR & ~(N_MASK | Z_MASK | V_MASK | C_MASK))                                                \
				| ((THUMB_SIGN_BITS_DIFFER(rn, op2) && THUMB_SIGN_BITS_DIFFER(rn, rd)) << V_BIT)                 \
				| (((IsNeg(rn) & IsPos(op2)) | (IsNeg(rn) & IsPos(rd)) | (IsPos(op2) & IsPos(rd))) ? C_MASK : 0) \
				| HandleALUNZFlags(rd)));                                                                        \
	R15 += 2;

#define DRCHandleThumbALUSubFlags(rd, rn, op2)                                                  \
	UML_AND(block, DRC_CPSR, DRC_CPSR, ~(N_MASK | Z_MASK | V_MASK | C_MASK));                   \
	DRCHandleALUNZFlags(rd);                                                                    \
	UML_XOR(block, uml::I1, rn, op2);                                                           \
	UML_XOR(block, uml::I2, rn, rd);                                                            \
	UML_AND(block, uml::I1, uml::I1, uml::I2);                                                  \
	UML_TEST(block, uml::I1, 1 << 31);                                                          \
	UML_MOVc(block, uml::COND_NZ, uml::I1, V_BIT);                                              \
	UML_MOVc(block, uml::COND_Z, uml::I1, 0);                                                   \
	UML_OR(block, uml::I0, uml::I0, uml::I1);                                                   \
	UML_OR(block, DRC_CPSR, DRC_CPSR, uml::I0);                                                 \
	UML_AND(block, uml::I0, rd, 1 << 31);                                                       \
	UML_AND(block, uml::I1, op2, 1 << 31);                                                      \
	UML_AND(block, uml::I2, rn, 1 << 31);                                                       \
	UML_XOR(block, uml::I2, uml::I2, ~0);                                                       \
	UML_AND(block, uml::I1, uml::I1, uml::I2);                                                  \
	UML_AND(block, uml::I2, uml::I2, uml::I0);                                                  \
	UML_OR(block, uml::I1, uml::I1, uml::I2);                                                   \
	UML_AND(block, uml::I2, op2, 1 << 31);                                                      \
	UML_AND(block, uml::I2, uml::I2, uml::I0);                                                  \
	UML_OR(block, uml::I1, uml::I1, uml::I2);                                                   \
	UML_TEST(block, uml::I1, 1 << 31);                                                          \
	UML_MOVc(block, uml::COND_NZ, uml::I0, C_MASK);                                             \
	UML_MOVc(block, uml::COND_Z, uml::I0, 0);                                                   \
	UML_OR(block, DRC_CPSR, DRC_CPSR, uml::I0);                                                 \
	UML_ADD(block, DRC_PC, DRC_PC, 2);

/* Set NZC flags for logical operations. */

// This macro (which I didn't write) - doesn't make it obvious that the SIGN BIT = 31, just as the N Bit does,
// therefore, N is set by default
#define HandleALUNZFlags(rd)               \
	(((rd) & SIGN_BIT) | ((!(rd)) << Z_BIT))

#define DRCHandleALUNZFlags(rd)                            \
	UML_AND(block, uml::I0, rd, SIGN_BIT);                 \
	UML_CMP(block, rd, 0);                                 \
	UML_MOVc(block, uml::COND_E, uml::I1, 1);              \
	UML_MOVc(block, uml::COND_NE, uml::I1, 0);             \
	UML_ROLINS(block, uml::I0, uml::I1, Z_BIT, 1 << Z_BIT);

// Long ALU Functions use bit 63
#define HandleLongALUNZFlags(rd)                            \
	((((rd) & ((uint64_t)1 << 63)) >> 32) | ((!(rd)) << Z_BIT))

#define HandleALULogicalFlags(rd, sc)                  \
	if (insn & INSN_S)                                   \
	set_cpsr_nzcv(((GET_CPSR & ~(N_MASK | Z_MASK | C_MASK)) \
				| HandleALUNZFlags(rd)                   \
				| (((sc) != 0) << C_BIT)));              \
	R15 += 4;

#define DRC_RD      uml::mem(&GetRegister(rd))
#define DRC_RS      uml::mem(&GetRegister(rs))
#define DRC_CPSR    uml::mem(&GET_CPSR)
#define DRC_PC      uml::mem(&R15)
#define DRC_REG(i)  uml::mem(&m_r[(i)])

#define DRCHandleALULogicalFlags(rd, sc)                                \
	if (insn & INSN_S)                                                  \
	{                                                                   \
		UML_AND(block, DRC_CPSR, DRC_CPSR, ~(N_MASK | Z_MASK | C_MASK); \
		DRCHandleALUNZFlags(rd);                                        \
		UML_TEST(block, sc, ~0);                                        \
		UML_MOVc(block, uml::COND_Z, uml::I1, C_BIT);                   \
		UML_MOVc(block, uml::COND_NZ, uml::I1, 0);                      \
		UML_OR(block, uml::I0, uml::I0, uml::I1);                       \
		UML_OR(block, DRC_CPSR, DRC_CPSR, uml::I0);                     \
	}                                                                   \
	UML_ADD(block, DRC_PC, DRC_PC, 4);


// used to be functions, but no longer a need, so we'll use define for better speed.
#define GetRegister(rIndex)        m_r[m_reg_group[rIndex]]
#define SetRegister(rIndex, value) m_r[m_reg_group[rIndex]] = value

#define GetModeRegister(mode, rIndex)        m_r[sRegisterTable[mode][rIndex]]
#define SetModeRegister(mode, rIndex, value) m_r[sRegisterTable[mode][rIndex]] = value


/* Macros that can be re-defined for custom cpu implementations - The core expects these to be defined */
/* In this case, we are using the default arm7 handlers (supplied by the core)
   - but simply changes these and define your own if needed for cpu implementation specific needs */
// condition flag updates keep the mode and T bits, so on 32-bit-only cores
// they do not need set_cpsr's mode switching
#define set_cpsr_nzcv(val) do { const uint32_t v_ = (val); if (!(m_archFlags & ARCHFLAG_MODE26)) m_r[eCPSR] = v_; else set_cpsr(v_); } while (0)

#define READ8(addr)         fast_read8(addr)
#define WRITE8(addr,data)   fast_write8(addr,data)
#define READ16(addr)        fast_read16(addr)
#define WRITE16(addr,data)  fast_write16(addr,data)
#define READ32(addr)        fast_read32(addr)
#define WRITE32(addr,data)  fast_write32(addr,data)

// inline fast path for fast RAM (see add_fastram): a cached translation and
// a host pointer, else the normal (virtual) access functions
inline bool arm7_cpu_device::xlat_hit(offs_t &addr, const int flags) const
{
	if (!(m_control & COPRO_CTRL_MMU_EN))
		return true;
	if (addr < 0x2000000)
		return false;
	const uint32_t xtag = (addr >> 10) | ((m_r[eCPSR] & 0xf) ? 0x80000000U : 0);
	const xlat_cache_entry &xe = m_xlat_cache[flags & 0xf][(addr >> 10) & (XLAT_CACHE_SIZE - 1)];
	if (xe.tag != xtag)
		return false;
	addr = xe.ppage | (addr & 0x3ff);
	return true;
}

inline uint32_t arm7_cpu_device::fast_read32(offs_t addr)
{
	offs_t pa = addr;
	if (m_fast_mem && xlat_hit(pa, ARM7_TLB_ABORT_D | ARM7_TLB_READ))
		if (const uint8_t *const p = fastram_ptr(pa & ~3))
			return std::rotr(*reinterpret_cast<const uint32_t *>(p), 8 * (addr & 3));
	return arm7_cpu_read32(addr);
}

inline uint32_t arm7_cpu_device::fast_read16(offs_t addr)
{
	offs_t pa = addr;
	if (m_fast_mem && xlat_hit(pa, ARM7_TLB_ABORT_D | ARM7_TLB_READ))
		if (const uint8_t *const p = fastram_ptr(pa & ~1))
		{
			uint32_t result = *reinterpret_cast<const uint16_t *>(p);
			if (addr & 1)
				result = ((result >> 8) & 0xff) | ((result & 0xff) << 24);
			return result;
		}
	return arm7_cpu_read16(addr);
}

inline uint8_t arm7_cpu_device::fast_read8(offs_t addr)
{
	offs_t pa = addr;
	if (m_fast_mem && xlat_hit(pa, ARM7_TLB_ABORT_D | ARM7_TLB_READ))
		if (const uint8_t *const p = fastram_ptr(pa))
			return *p;
	return arm7_cpu_read8(addr);
}

inline void arm7_cpu_device::fast_write32(offs_t addr, uint32_t data)
{
	offs_t pa = addr;
	if (m_fast_mem && xlat_hit(pa, ARM7_TLB_ABORT_D | ARM7_TLB_WRITE))
		if (uint8_t *const p = fastram_ptr(pa & ~3))
		{
			*reinterpret_cast<uint32_t *>(p) = data;
			return;
		}
	arm7_cpu_write32(addr, data);
}

inline void arm7_cpu_device::fast_write16(offs_t addr, uint16_t data)
{
	offs_t pa = addr;
	if (m_fast_mem && xlat_hit(pa, ARM7_TLB_ABORT_D | ARM7_TLB_WRITE))
		if (uint8_t *const p = fastram_ptr(pa & ~1))
		{
			*reinterpret_cast<uint16_t *>(p) = data;
			return;
		}
	arm7_cpu_write16(addr, data);
}

inline void arm7_cpu_device::fast_write8(offs_t addr, uint8_t data)
{
	offs_t pa = addr;
	if (m_fast_mem && xlat_hit(pa, ARM7_TLB_ABORT_D | ARM7_TLB_WRITE))
		if (uint8_t *const p = fastram_ptr(pa))
		{
			*p = data;
			return;
		}
	arm7_cpu_write8(addr, data);
}
#define PTR_READ32          &arm7_cpu_read32
#define PTR_WRITE32         &arm7_cpu_write32
