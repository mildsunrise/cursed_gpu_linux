#define RISCV_I_OP_IMM    0b0010011
#define RISCV_I_OP        0b0110011
#define RISCV_I_LUI       0b0110111
#define RISCV_I_AUIPC     0b0010111
#define RISCV_I_JAL       0b1101111
#define RISCV_I_JALR      0b1100111
#define RISCV_I_BRANCH    0b1100011
#define RISCV_I_LOAD      0b0000011
#define RISCV_I_STORE     0b0100011
#define RISCV_I_MISC_MEM  0b0001111
#define RISCV_I_SYSTEM    0b1110011

#define RISCV_IFUNC_ADD   0b000
#define RISCV_IFUNC_SLT   0b010
#define RISCV_IFUNC_SLTU  0b011
#define RISCV_IFUNC_XOR   0b100
#define RISCV_IFUNC_OR    0b110
#define RISCV_IFUNC_AND   0b111
#define RISCV_IFUNC_SLL   0b001
#define RISCV_IFUNC_SRL   0b101

#define RISCV_BFUNC_BEQ   0b000
#define RISCV_BFUNC_BNE   0b001
#define RISCV_BFUNC_BLT   0b100
#define RISCV_BFUNC_BGE   0b101
#define RISCV_BFUNC_BLTU  0b110
#define RISCV_BFUNC_BGEU  0b111

#define RISCV_MM_FENCE    0b000
#define RISCV_MM_FENCE_I  0b001

#define RISCV_MEM_LB      0b000
#define RISCV_MEM_LH      0b001
#define RISCV_MEM_LW      0b010
#define RISCV_MEM_LBU     0b100
#define RISCV_MEM_LHU     0b101
