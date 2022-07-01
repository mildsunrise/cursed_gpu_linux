#include "core.h"
#include <stdbool.h>
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
    return (uint32_t)(sign >> 12) |
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

uint32_t __core_read_rs1(const core_t* core, uint32_t instr) {
    return core->x_regs[__core_dec_rs1(instr)];
}
uint32_t __core_read_rs2(const core_t* core, uint32_t instr) {
    return core->x_regs[__core_dec_rs2(instr)];
}

#define __negBit (instr & (1 << 30))


// EXCEPTIONS, TRAPS, INTERRUPTS

void core_set_exception(core_t* core, uint32_t cause, uint32_t val) {
    core->error = ERR_EXCEPTION;
    core->exc_cause = cause;
    core->exc_val = val;
}

void __core_do_privileged(core_t* core, uint32_t instr) {
    if (instr & ( (MASK(5) << 7) | (MASK(5) << 15) )) {
        core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0);
        return;
    }
    switch (__core_dec_i_unsigned(instr)) {
        case RISCV_PRIV_EBREAK:
            core_set_exception(core, RISCV_EXC_BREAKPOINT, core->current_pc); break;
        case RISCV_PRIV_ECALL:
            core_set_exception(core, core->s_mode ? RISCV_EXC_ECALL_S : RISCV_EXC_ECALL_U, 0); break;
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
        CSR_BITFIELD_CASE(R, W, RISCV_CSR_SATP, \
            R(*value = core->satp_addr >> 12;) \
            W(core->satp_addr = value << 12;) \
            CSR_BITFIELD_BOOL(R, W, 31, core->stvec_vectored) \
        ) \
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

uint32_t __core_int_op(uint32_t instr, bool isReg, uint32_t a, uint32_t b) {
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

void core_step(core_t* core) {
    if (unlikely(core->error)) return;
    core->current_pc = core->pc;

    uint32_t instr;
    core->mem_fetch(core, core->pc, &instr);
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
            __core_set_dest(core, instr, __core_int_op(instr, true, __core_read_rs1(core, instr), __core_read_rs2(core, instr))); break;

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
            core->mem_load(core, __core_read_rs1(core, instr) + __core_dec_i(instr), __core_dec_func3(instr), &value);
            if (unlikely(core->error)) return;
            __core_set_dest(core, instr, value);
            break;
        case RISCV_I_STORE:
            core->mem_store(core, __core_read_rs1(core, instr) + __core_dec_s(instr), __core_dec_func3(instr), __core_read_rs2(core, instr));
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

        case RISCV_I_SYSTEM:
            __core_do_system(core, instr); break;

        default:
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); break;
    }
}
