#include "core.h"
#include <stdbool.h>
#include "reg_macros.h"

#define MASK(n) (~((~0U << (n))))
#define unlikely(x) __builtin_expect((x),0)

const char* core_error_str(core_error_t err) {
    static const char* errors [] = {
        "NONE",
        "BAD_INSTRUCTION",
        "JUMP_UNALIGNED",
        "MEM_UNMAPPED",
        "MEM_PROTECTED",
        "SYSTEM",
    };
    if (err >= 0 && err < (sizeof(errors) / sizeof(*errors)))
        return errors[err];
    return "UNKNOWN";
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


// INSTRUCTIONS

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
    core->error = ERR_BAD_INSTRUCTION;
    return false;
}

void __core_set_dest(core_t* core, uint32_t instr, uint32_t x) {
    uint8_t rd = __core_dec_rd(instr);
    if (rd) core->x_regs[rd] = x;
}

void __core_jump(core_t* core, uint32_t addr) {
    if (unlikely(addr & 0b11))
        core->error = ERR_JUMP_UNALIGNED;
    else
        core->pc = addr;
}

void __core_jump_link(core_t* core, uint32_t instr, uint32_t addr) {
    __core_set_dest(core, instr, core->pc); // FIXME: if the jump fails (misalign), does rd need to be updated?
    __core_jump(core, addr);
}

#define __csr_body(R, W) \
    /* TODO */ \
    core->error = ERR_BAD_INSTRUCTION;

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

        // ECALL / EBREAK is the only non-CSR instruction
        case RISCV_SYS_ECALL:
            core->error = ERR_SYSTEM; break;

        default:
            core->error = ERR_BAD_INSTRUCTION; return;
    }
}

void core_step(core_t* core) {
    if (unlikely(core->error)) return;
    core->current_pc = core->pc;

    uint32_t instr;
    core->mem_fetch(core, core->pc, &instr);
    if (unlikely(core->error)) return;

    core->pc += 4;

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
                    core->error = ERR_BAD_INSTRUCTION; break;
            }
            break;

        case RISCV_I_SYSTEM:
            __core_do_system(core, instr); break;

        default:
            core->error = ERR_BAD_INSTRUCTION; break;
    }
}
