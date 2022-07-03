#include "core.h"
#include <stdbool.h>
#include <stddef.h>
#include "reg_macros.h"

#define MASK(n) (~((~0U << (n))))
#define unlikely(x) __builtin_expect((x),0)

const char* core_error_str(core_error_t err) {
    static const char* errors [] = {
        "NONE",
        "EXCEPTION",
        "USER",
    };
    if (err >= 0 && err < (sizeof(errors) / sizeof(*errors)))
        return errors[err];
    return "UNKNOWN";
}

const char* core_exc_cause_str(uint32_t err) {
    static const char* errors [] = {
        "Instruction address misaligned",
        "Instruction access fault",
        "Illegal instruction",
        "Breakpoint",
        "Load address misaligned",
        "Load access fault",
        "Store/AMO address misaligned",
        "Store/AMO access fault",
        "Environment call from U-mode",
        "Environment call from S-mode",
        NULL,
        NULL,
        "Instruction page fault",
        "Load page fault",
        NULL,
        "Store/AMO page fault",
    };
    if (err < (sizeof(errors) / sizeof(*errors)) && errors[err])
        return errors[err];
    return "[Unknown]";
}


// INSTRUCTION DECODING

uint32_t __core_dec_u(uint32_t instr) {
    return instr & (MASK(20) << 12);
}
uint32_t __core_dec_i(uint32_t instr) {
    return (uint32_t)(((int32_t)instr) >> 20);
}
uint32_t __core_dec_j(uint32_t instr) {
    int32_t sign = ((int32_t)instr) & (1 << 31);
    return (uint32_t)(sign >> 11) |
        (instr & (MASK(8) << 12)) |
        ((instr & (MASK(1) << 20)) >> 9) |
        ((instr & (MASK(10) << 21)) >> 20);
}
uint32_t __core_dec_b(uint32_t instr) {
    int32_t sign = ((int32_t)instr) & (1 << 31);
    return (uint32_t)(sign >> 20) |
        ((instr & (MASK(4) << 8)) >> 7) |
        ((instr & (MASK(1) << 7)) << 4) |
        ((instr & (MASK(6) << 25)) >> 20);
}
uint32_t __core_dec_s(uint32_t instr) {
    return (uint32_t)(((int32_t)(instr & (MASK(7) << 25))) >> 20) | ((instr >> 7) & MASK(5));
}

uint16_t __core_dec_i_unsigned(uint32_t instr) {
    return instr >> 20;
}
uint8_t __core_dec_rd(uint32_t instr) {
    return (instr >> 7) & MASK(5);
}
uint8_t __core_dec_rs1(uint32_t instr) {
    return (instr >> 15) & MASK(5);
}
uint8_t __core_dec_rs2(uint32_t instr) {
    return (instr >> 20) & MASK(5);
}
uint8_t __core_dec_func3(uint32_t instr) {
    return (instr >> 12) & MASK(3);
}
uint8_t __core_dec_func5(uint32_t instr) {
    return instr >> 27;
}

uint32_t __core_read_rs1(const core_t* core, uint32_t instr) {
    return core->x_regs[__core_dec_rs1(instr)];
}
uint32_t __core_read_rs2(const core_t* core, uint32_t instr) {
    return core->x_regs[__core_dec_rs2(instr)];
}

#define __negBit (instr & (1 << 30))


// VIRTUAL ADDRESSING

// we pre-verify the root page table, so that
// at most 1 page table access is done at translation time
void __core_mmu_set(core_t* core, uint32_t satp) {
    if (satp >> 31) {
        uint32_t* page_table = core->mem_page_table(core, satp & MASK(22));
        if (!page_table) return;
        core->page_table = page_table;
        satp &= ~(MASK(9) << 22);
    } else {
        core->page_table = NULL;
        satp = 0;
    }
    core->satp = satp;
}

#define __PTE_ITERATION(page_table, vpn, additional_checks) \
    *pte = &(page_table)[vpn]; \
    switch ((**pte) & MASK(4)) { \
        case 0b0001: \
            break; /* pointer to next level */ \
        case 0b0011: \
        case 0b0111: \
        case 0b1001: \
        case 0b1011: \
        case 0b1111: \
            *ppn = (**pte) >> 10; \
            additional_checks \
            return true; /* leaf entry */ \
        case 0b0101: \
        case 0b1101: \
        default: \
            *pte = NULL; \
            return true; /* not valid */ \
    }

// assumes core->page_table is set!
// if there's an error fetching a page table, returns false.
// otherwise returns true and:
//   - in case of valid leaf: sets *pte and *ppn
//   - none found (page fault): sets *pte to NULL
bool __core_mmu_lookup(const core_t* core, uint32_t vpn, uint32_t** pte, uint32_t *ppn) {
    __PTE_ITERATION(core->page_table, vpn >> 10,
        if (unlikely((*ppn) & MASK(10))) // misaligned superpage
            *pte = NULL;
        else
            *ppn |= vpn & MASK(10);
    )
    uint32_t* page_table = core->mem_page_table(core, (**pte) >> 10);
    if (!page_table) return false;
    __PTE_ITERATION(page_table, vpn & MASK(10),)
    *pte = NULL;
    return true;
}

void __core_mmu_translate(core_t* core, uint32_t *addr, uint32_t access_bits, uint32_t set_bits, bool skip_privilege_test, uint8_t fault, uint8_t pfault) {
    core->exc_val = *addr; // hack: save virtual address, for physical accesses to set exception
    if (!core->page_table)
        return;

    uint32_t *pte_ref;
    uint32_t ppn;
    bool ok = __core_mmu_lookup(core, (*addr) >> RISCV_PAGE_SHIFT, &pte_ref, &ppn);
    if (unlikely(!ok)) {
        core_set_exception(core, fault, *addr);
        return;
    }

    uint32_t pte;
    if (!(
        pte_ref // PTE lookup was successful
        && !(ppn >> 20) // PPN is valid
        && (pte = *pte_ref, pte & access_bits) // access type is allowed
        && (!(pte & (1 << 4)) == core->s_mode || skip_privilege_test) // privilege matches
    )) {
        core_set_exception(core, pfault, *addr);
        return;
    }

    uint32_t new_pte = pte | set_bits;
    if (new_pte != pte)
        *pte_ref = new_pte;

    *addr = ((*addr) & MASK(RISCV_PAGE_SHIFT)) | (ppn << RISCV_PAGE_SHIFT);
}

void __core_mmu_fence(core_t* /*core*/, uint32_t /*instr*/) {
    // no-op for now
}

void __core_mmu_fetch(core_t* core, uint32_t addr, uint32_t* value) {
    __core_mmu_translate(core, &addr,
        (1 << 3),
        (1 << 6),
        false,
        RISCV_EXC_FETCH_FAULT, RISCV_EXC_FETCH_PFAULT);
    if (core->error) return;
    core->mem_fetch(core, addr, value);
}

void __core_mmu_load(core_t* core, uint32_t addr, uint8_t width, uint32_t* value, bool reserved) {
    __core_mmu_translate(core, &addr,
        (1 << 1) | (core->sstatus_mxr ? (1 << 3) : 0),
        (1 << 6),
        core->sstatus_sum && core->s_mode,
        RISCV_EXC_LOAD_FAULT, RISCV_EXC_LOAD_PFAULT);
    if (core->error) return;
    core->mem_load(core, addr, width, value);
    if (core->error) return;

    if (unlikely(reserved))
        core->lr_reservation = addr | 1;
}

bool __core_mmu_store(core_t* core, uint32_t addr, uint8_t width, uint32_t value, bool conditional) {
    __core_mmu_translate(core, &addr,
        (1 << 2),
        (1 << 6) | (1 << 7),
        core->sstatus_sum && core->s_mode,
        RISCV_EXC_STORE_FAULT, RISCV_EXC_STORE_PFAULT);
    if (core->error) return false;

    if (unlikely(conditional)) {
        if (core->lr_reservation != (addr | 1))
            return false;
        core->lr_reservation = 0;
    } else {
        if (unlikely(core->lr_reservation & 1) && (core->lr_reservation & ~3) == (addr & ~3))
            core->lr_reservation = 0;
    }
    core->mem_store(core, addr, width, value);
    return true;
}


// EXCEPTIONS, TRAPS, INTERRUPTS

void core_set_exception(core_t* core, uint32_t cause, uint32_t val) {
    core->error = ERR_EXCEPTION;
    core->exc_cause = cause;
    core->exc_val = val;
}

void core_trap(core_t* core) {
    // copy exception fields
    core->scause = core->exc_cause;
    core->stval = core->exc_val;

    // save stuff to stack
    core->sstatus_spie = core->sstatus_sie;
    core->sstatus_spp = core->s_mode;
    core->sepc = core->current_pc;

    // set stuff
    core->sstatus_sie = false;
    core->s_mode = true;
    core->pc = core->stvec_addr;
    if (core->stvec_vectored)
        core->pc += core->scause * 4;

    core->error = ERR_NONE;
}

void __core_do_sret(core_t* core) {
    // restore stuff from stack
    core->pc = core->sepc;
    core->s_mode = core->sstatus_spp;
    core->sstatus_sie = core->sstatus_spie;

    // reset stack
    core->sstatus_spp = false;
    core->sstatus_spie = true;
}

void __core_do_privileged(core_t* core, uint32_t instr) {
    if ((instr >> 25) == RISCV_PRIV___SFENCE_VMA) {
        __core_mmu_fence(core, instr);
        return;
    }
    if (instr & ( (MASK(5) << 7) | (MASK(5) << 15) )) {
        core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0);
        return;
    }
    switch (__core_dec_i_unsigned(instr)) {
        case RISCV_PRIV_EBREAK:
            core_set_exception(core, RISCV_EXC_BREAKPOINT, core->current_pc); break;
        case RISCV_PRIV_ECALL:
            core_set_exception(core, core->s_mode ? RISCV_EXC_ECALL_S : RISCV_EXC_ECALL_U, 0); break;
        case RISCV_PRIV_SRET:
            __core_do_sret(core); break;
        default:
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); break;
    }
}


// CSR INSTRUCTIONS

void __core_set_dest(core_t* core, uint32_t instr, uint32_t x);

#define CSR_BITFIELD_CASE(R, W, CASE, CODE) \
    case (CASE): \
        R(*value = 0;) \
        CODE \
        break;

#define CSR_BITFIELD_BOOL(R, W, BIT, LVALUE) \
    R((LVALUE) && (*value |= 1 << (BIT));) \
    W((LVALUE) = (value & (1 << (BIT))) != 0;)

#define SIE_MASK ( RISCV_INT_SEI_BIT | RISCV_INT_STI_BIT | RISCV_INT_SSI_BIT )
#define SIP_MASK (                 0 |                 0 | RISCV_INT_SSI_BIT )

#define __csr_body(R, W) \
    R( \
        if ((addr >> 8) == 0xC) { \
            uint16_t idx = addr & MASK(7); \
            if (idx >= 0x20 || !((core->scounteren >> idx) & 1)) \
                core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); \
            else \
                /* we'll use the instruction counter for all of the counters. ideally,
                reads should return the value before the increment, and writes should
                set the value after the increment. but we don't expose any way to write
                the counters, so I don't think it'll cause visible issues */ \
                *value = (addr & (1 << 7)) ? core->instr_count_h : core->instr_count; \
            return; \
        } \
    ) \
    if (!core->s_mode) { \
        core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); \
        return; \
    } \
    switch (addr) { \
        CSR_BITFIELD_CASE(R, W, RISCV_CSR_SSTATUS, \
            CSR_BITFIELD_BOOL(R, W, 1, core->sstatus_sie) \
            CSR_BITFIELD_BOOL(R, W, 5, core->sstatus_spie) \
            CSR_BITFIELD_BOOL(R, W, 8, core->sstatus_spp) \
            CSR_BITFIELD_BOOL(R, W, 18, core->sstatus_sum) \
            CSR_BITFIELD_BOOL(R, W, 19, core->sstatus_mxr) \
        ) \
        case RISCV_CSR_SIE: \
            W(value &= SIE_MASK;) \
            REG_LVALUE(R, W, core->sie) break; \
        case RISCV_CSR_SIP: \
            W(value &= SIP_MASK; value |= core->sip & ~SIP_MASK;) \
            REG_LVALUE(R, W, core->sip) break; \
        CSR_BITFIELD_CASE(R, W, RISCV_CSR_STVEC, \
            REG_LVALUE(R, W, core->stvec_addr) \
            W(core->stvec_addr &= ~0b11;) \
            CSR_BITFIELD_BOOL(R, W, 0, core->stvec_vectored) \
        ) \
        case RISCV_CSR_SATP: \
            R(*value = core->satp;) \
            W(__core_mmu_set(core, value);) \
            break; \
        REG_CASE_RW(R, W, RISCV_CSR_SCOUNTEREN, core->scounteren) \
        REG_CASE_RW(R, W, RISCV_CSR_SSCRATCH, core->sscratch) \
        REG_CASE_RW(R, W, RISCV_CSR_SEPC, core->sepc) \
        REG_CASE_RW(R, W, RISCV_CSR_SCAUSE, core->scause) \
        REG_CASE_RW(R, W, RISCV_CSR_STVAL, core->stval) \
        default: \
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); \
    }

REG_FUNCTIONS(void __core_csr, (core_t* core, uint16_t addr), uint32_t, __csr_body)

void __core_csr_rw(core_t* core, uint32_t instr, uint16_t csr, uint32_t wvalue) {
    uint8_t rd = __core_dec_rd(instr);
    if (rd) {
        uint32_t value;
        __core_csr_read(core, csr, &value);
        if (unlikely(core->error)) return;
        __core_set_dest(core, instr, value);
    }
    __core_csr_write(core, csr, wvalue);
}

void __core_csr_cs(core_t* core, uint32_t instr, uint16_t csr, uint32_t setmask, uint32_t clearmask) {
    uint32_t value;
    __core_csr_read(core, csr, &value);
    if (unlikely(core->error)) return;
    __core_set_dest(core, instr, value);
    if (__core_dec_rs1(instr))
        __core_csr_write(core, csr, (value & ~clearmask) | setmask);
}

void __core_do_system(core_t* core, uint32_t instr) {
    switch (__core_dec_func3(instr)) {
        // CSR
        case RISCV_SYS_CSRRW:
            __core_csr_rw(core, instr, __core_dec_i_unsigned(instr), __core_read_rs1(core, instr)); break;
        case RISCV_SYS_CSRRWI:
            __core_csr_rw(core, instr, __core_dec_i_unsigned(instr), __core_dec_rs1(instr)); break;
        case RISCV_SYS_CSRRS:
            __core_csr_cs(core, instr, __core_dec_i_unsigned(instr), __core_read_rs1(core, instr), 0); break;
        case RISCV_SYS_CSRRSI:
            __core_csr_cs(core, instr, __core_dec_i_unsigned(instr), __core_dec_rs1(instr), 0); break;
        case RISCV_SYS_CSRRC:
            __core_csr_cs(core, instr, __core_dec_i_unsigned(instr), 0, __core_read_rs1(core, instr)); break;
        case RISCV_SYS_CSRRCI:
            __core_csr_cs(core, instr, __core_dec_i_unsigned(instr), 0, __core_dec_rs1(instr)); break;

        // privileged instruction
        case RISCV_SYS_PRIV:
            __core_do_privileged(core, instr); break;

        default:
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); return;
    }
}


// UNPRIVILEGED INSTRUCTIONS

uint32_t __core_mul_op(uint32_t instr, uint32_t a, uint32_t b) {
    // FIXME: test ifunc7 zeros...
    switch (__core_dec_func3(instr)) {
        case RISCV_MUL_MUL: return a * b;
        case RISCV_MUL_MULH: return (uint32_t)((uint64_t)( ((int64_t)(int32_t)a) * ((int64_t)(int32_t)b) ) >> 32);
        case RISCV_MUL_MULHSU: return (uint32_t)((uint64_t)( ((int64_t)(int32_t)a) * ((int64_t)b) ) >> 32);
        case RISCV_MUL_MULHU: return (uint32_t)(( ((uint64_t)a) * ((uint64_t)b) ) >> 32);

        case RISCV_MUL_DIV: return b ? (uint32_t)( ((int32_t)a) / ((int32_t)b) ) : 0xFFFFFFFF;
        case RISCV_MUL_DIVU: return b ? (a / b) : 0xFFFFFFFF;

        case RISCV_MUL_REM: return b ? (uint32_t)( ((int32_t)a) % ((int32_t)b) ) : a;
        case RISCV_MUL_REMU: return b ? (a % b) : a;
    }
    __builtin_unreachable();
}

uint32_t __core_int_op(uint32_t instr, bool isReg, uint32_t a, uint32_t b) {
    // FIXME: test ifunc7 zeros...
    switch (__core_dec_func3(instr)) {
        case RISCV_IFUNC_ADD:  return a + ((isReg && __negBit) ? -b : b);

        case RISCV_IFUNC_SLT:  return ((int32_t)a) < ((int32_t)b);
        case RISCV_IFUNC_SLTU: return a < b;

        case RISCV_IFUNC_XOR:  return a ^ b;
        case RISCV_IFUNC_OR:   return a | b;
        case RISCV_IFUNC_AND:  return a & b;

        case RISCV_IFUNC_SLL:  return a << (b & MASK(5));
        case RISCV_IFUNC_SRL:  return __negBit ? (uint32_t)(((int32_t)a) >> (b & MASK(5))) : // SRA
                                                                      a  >> (b & MASK(5))  ; // SRL
    }
    __builtin_unreachable();
}

bool __core_jmp_op(core_t* core, uint32_t instr, uint32_t a, uint32_t b) {
    switch (__core_dec_func3(instr)) {
        case RISCV_BFUNC_BEQ:  return a == b;
        case RISCV_BFUNC_BNE:  return a != b;
        case RISCV_BFUNC_BLTU: return a  < b;
        case RISCV_BFUNC_BGEU: return a >= b;
        case RISCV_BFUNC_BLT:  return ((int32_t)a)  < ((int32_t)b);
        case RISCV_BFUNC_BGE:  return ((int32_t)a) >= ((int32_t)b);
    }
    core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0);
    return false;
}

void __core_set_dest(core_t* core, uint32_t instr, uint32_t x) {
    uint8_t rd = __core_dec_rd(instr);
    if (rd) core->x_regs[rd] = x;
}

void __core_jump(core_t* core, uint32_t addr) {
    if (unlikely(addr & 0b11))
        core_set_exception(core, RISCV_EXC_PC_MISALIGN, addr);
    else
        core->pc = addr;
}

void __core_jump_link(core_t* core, uint32_t instr, uint32_t addr) {
    __core_set_dest(core, instr, core->pc); // FIXME: if the jump fails (misalign), does rd need to be updated?
    __core_jump(core, addr);
}

#define __AMO_CASE(CASE, STORED_EXPR) \
    case (CASE): \
        value2 = __core_read_rs2(core, instr); \
        __core_mmu_load(core, addr, RISCV_MEM_LW, &value, false); \
        if (core->error) return; \
        __core_set_dest(core, instr, value); \
        __core_mmu_store(core, addr, RISCV_MEM_SW, (STORED_EXPR), false); \
        break;

void __core_do_amo(core_t* core, uint32_t instr) {
    if (unlikely(__core_dec_func3(instr) != RISCV_AMO__W))
        return core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0);
    uint32_t addr = __core_read_rs1(core, instr);
    uint32_t value, value2;
    switch (__core_dec_func5(instr)) {
        case RISCV_AMO_LR:
            if (addr & 0b11)
                return core_set_exception(core, RISCV_EXC_LOAD_MISALIGN, addr);
            if (__core_dec_rs2(instr))
                return core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0);
            __core_mmu_load(core, addr, RISCV_MEM_LW, &value, true);
            if (core->error) return;
            __core_set_dest(core, instr, value);
            break;
        case RISCV_AMO_SC:
            if (addr & 0b11)
                return core_set_exception(core, RISCV_EXC_STORE_MISALIGN, addr);
            bool ok = __core_mmu_store(core, addr, RISCV_MEM_SW, __core_read_rs2(core, instr), true);
            if (core->error) return;
            __core_set_dest(core, instr, ok ? 0 : 1);
            break;

        __AMO_CASE(RISCV_AMO_AMOSWAP, value2)
        __AMO_CASE(RISCV_AMO_AMOADD, value + value2)
        __AMO_CASE(RISCV_AMO_AMOXOR, value ^ value2)
        __AMO_CASE(RISCV_AMO_AMOAND, value & value2)
        __AMO_CASE(RISCV_AMO_AMOOR,  value | value2)
        __AMO_CASE(RISCV_AMO_AMOMIN, ((int32_t)value) < ((int32_t)value2) ? value : value2)
        __AMO_CASE(RISCV_AMO_AMOMAX, ((int32_t)value) > ((int32_t)value2) ? value : value2)
        __AMO_CASE(RISCV_AMO_AMOMINU, value < value2 ? value : value2)
        __AMO_CASE(RISCV_AMO_AMOMAXU, value > value2 ? value : value2)

        default:
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); return;
    }
}

void core_step(core_t* core) {
    if (unlikely(core->error)) return;
    core->current_pc = core->pc;

    if ((core->sstatus_sie || !core->s_mode) && (core->sip & core->sie)) {
        uint32_t applicable = (core->sip & core->sie);
        uint8_t idx = 32;
        while (!(applicable & (1 << --idx))); // FIXME: use a LUT
        core->exc_cause = (1 << 31) | idx;
        core->stval = 0;
        core_trap(core);
    }

    uint32_t instr;
    __core_mmu_fetch(core, core->pc, &instr);
    if (unlikely(core->error)) return;

    core->pc += 4;
    core->instr_count++;
    if (!core->instr_count) core->instr_count_h++;

    uint32_t instr_opcode = instr & MASK(7);
    uint32_t value;
    switch (instr_opcode) {
        case RISCV_I_OP_IMM:
            __core_set_dest(core, instr, __core_int_op(instr, false, __core_read_rs1(core, instr), __core_dec_i(instr))); break;
        case RISCV_I_OP:
            if (!(instr & (1 << 25)))
                __core_set_dest(core, instr, __core_int_op(instr, true, __core_read_rs1(core, instr), __core_read_rs2(core, instr)));
            else
                __core_set_dest(core, instr, __core_mul_op(instr,       __core_read_rs1(core, instr), __core_read_rs2(core, instr)));
            break;

        case RISCV_I_LUI:
            __core_set_dest(core, instr, __core_dec_u(instr)); break;
        case RISCV_I_AUIPC:
            __core_set_dest(core, instr, __core_dec_u(instr) + core->current_pc); break;

        case RISCV_I_JAL:
            __core_jump_link(core, instr, __core_dec_j(instr) + core->current_pc); break;
        case RISCV_I_JALR:
            __core_jump_link(core, instr, (__core_dec_i(instr) + __core_read_rs1(core, instr)) & ~1); break;

        case RISCV_I_BRANCH:
            if (__core_jmp_op(core, instr, __core_read_rs1(core, instr), __core_read_rs2(core, instr)))
                __core_jump(core, __core_dec_b(instr) + core->current_pc);
            break;

        case RISCV_I_LOAD:
            __core_mmu_load(core, __core_read_rs1(core, instr) + __core_dec_i(instr), __core_dec_func3(instr), &value, false);
            if (unlikely(core->error)) return;
            __core_set_dest(core, instr, value);
            break;
        case RISCV_I_STORE:
            __core_mmu_store(core, __core_read_rs1(core, instr) + __core_dec_s(instr), __core_dec_func3(instr), __core_read_rs2(core, instr), false);
            if (unlikely(core->error)) return;
            break;

        case RISCV_I_MISC_MEM:
            switch (__core_dec_func3(instr)) {
                case RISCV_MM_FENCE:
                case RISCV_MM_FENCE_I:
                    break; // currently no-op, we're singlethreaded
                default:
                    core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); break;
            }
            break;

        case RISCV_I_AMO:
            __core_do_amo(core, instr); break;

        case RISCV_I_SYSTEM:
            __core_do_system(core, instr); break;

        default:
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); break;
    }
}
