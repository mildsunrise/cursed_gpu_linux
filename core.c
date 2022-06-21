#include "core.h"
#include <stdbool.h>

#define MASK(n) (~((~0U << (n))))
#define unlikely(x) __builtin_expect((x),0)


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
    return (uint32_t)(sign >> 19) |
        ((instr & (MASK(4) << 8)) >> 7) |
        ((instr & (MASK(1) << 7)) << 4) |
        ((instr & (MASK(6) << 25)) >> 20);
}
uint32_t __core_dec_s(uint32_t instr) {
    int32_t sign = ((int32_t)instr) & (1 << 31);
    return (uint32_t)(sign >> 19) |
        ((instr & (MASK(5) << 7)) >> 7) |
        ((instr & (MASK(6) << 25)) >> 20);
}

uint32_t __core_read_rs1(const core_t* core, uint32_t instr) {
    return core->x_regs[(instr >> 7) & MASK(5)];
}
uint32_t __core_read_rs2(const core_t* core, uint32_t instr) {
    return core->x_regs[(instr >> 20) & MASK(5)];
}

uint8_t __core_dec_func3(uint32_t instr) {
    return (instr >> 12) & MASK(3);
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
    uint32_t rd = (instr >> 15) & MASK(5);
    if (rd) core->x_regs[rd] = x;
}

void __core_jump(core_t* core, uint32_t addr) {
    core->pc = addr;
    if (addr & 0b11)
        core->error = ERR_JUMP_UNALIGNED;
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

    uint32_t instr_opcode = instr & MASK(7);
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
            uint32_t value;
            core->mem_load(core, __core_read_rs1(core, instr) + __core_dec_s(instr), __core_dec_func3(instr), &value);
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
            core->error = ERR_SYSTEM; break;
        default:
            core->error = ERR_BAD_INSTRUCTION; break;
    }
}
