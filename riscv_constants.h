// base ISA

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
#define RISCV_I_AMO       0b0101111

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

#define RISCV_MEM_SB      0b000
#define RISCV_MEM_SH      0b001
#define RISCV_MEM_SW      0b010

#define RISCV_MUL_MUL     0b000
#define RISCV_MUL_MULH    0b001
#define RISCV_MUL_MULHSU  0b010
#define RISCV_MUL_MULHU   0b011
#define RISCV_MUL_DIV     0b100
#define RISCV_MUL_DIVU    0b101
#define RISCV_MUL_REM     0b110
#define RISCV_MUL_REMU    0b111

#define RISCV_AMO__W      0b010
#define RISCV_AMO_LR      0b00010
#define RISCV_AMO_SC      0b00011
#define RISCV_AMO_AMOSWAP 0b00001
#define RISCV_AMO_AMOADD  0b00000
#define RISCV_AMO_AMOXOR  0b00100
#define RISCV_AMO_AMOAND  0b01100
#define RISCV_AMO_AMOOR   0b01000
#define RISCV_AMO_AMOMIN  0b10000
#define RISCV_AMO_AMOMAX  0b10100
#define RISCV_AMO_AMOMINU 0b11000
#define RISCV_AMO_AMOMAXU 0b11100

#define RISCV_SYS_CSRRW   0b001
#define RISCV_SYS_CSRRS   0b010
#define RISCV_SYS_CSRRC   0b011
#define RISCV_SYS_CSRRWI  0b101
#define RISCV_SYS_CSRRSI  0b110
#define RISCV_SYS_CSRRCI  0b111

#define RISCV_SYS_PRIV    0b000
#define RISCV_PRIV_ECALL  0b000000000000
#define RISCV_PRIV_EBREAK 0b000000000001
#define RISCV_PRIV_SRET   0b000100000010
#define RISCV_PRIV_MRET   0b001100000010
#define RISCV_PRIV_WFI    0b000100000101
#define RISCV_PRIV___SFENCE_VMA 0b0001001


// privileged ISA: CSRs

/// U-mode (RO counters)
#define RISCV_CSR_CYCLE            0xC00  // URO  Cycle counter for RDCYCLE instruction.
#define RISCV_CSR_TIME             0xC01  // URO  Timer for RDTIME instruction.
#define RISCV_CSR_INSTRET          0xC02  // URO  Instructions-retired counter for RDINSTRET instruction.
#define RISCV_CSR_HPMCOUNTER3      0xC03  // URO  Performance-monitoring counter.
// ...
#define RISCV_CSR_CYCLEH           0xC80  // URO  Upper 32 bits of cycle, RV32 only.
#define RISCV_CSR_TIMEH            0xC81  // URO  Upper 32 bits of time, RV32 only.
#define RISCV_CSR_INSTRETH         0xC82  // URO  Upper 32 bits of instret, RV32 only.
#define RISCV_CSR_HPMCOUNTER3H     0xC83  // URO  Upper 32 bits of hpmcounter3, RV32 only.
// ...

/// S-mode (Supervisor Trap Setup)
#define RISCV_CSR_SSTATUS          0x100  // SRW  Supervisor status register.
#define RISCV_CSR_SIE              0x104  // SRW  Supervisor interrupt-enable register.
#define RISCV_CSR_STVEC            0x105  // SRW  Supervisor trap handler base address.
#define RISCV_CSR_SCOUNTEREN       0x106  // SRW  Supervisor counter enable.
/// S-mode (Supervisor Configuration)
#define RISCV_CSR_SENVCFG          0x10A  // SRW  Supervisor environment configuration register.
/// S-mode (Supervisor Trap Handling)
#define RISCV_CSR_SSCRATCH         0x140  // SRW  Scratch register for supervisor trap handlers.
#define RISCV_CSR_SEPC             0x141  // SRW  Supervisor exception program counter.
#define RISCV_CSR_SCAUSE           0x142  // SRW  Supervisor trap cause.
#define RISCV_CSR_STVAL            0x143  // SRW  Supervisor bad address or instruction.
#define RISCV_CSR_SIP              0x144  // SRW  Supervisor interrupt pending.
/// S-mode (Supervisor Protection and Translation)
#define RISCV_CSR_SATP             0x180  // SRW  Supervisor address translation and protection

// privileged ISA: exception causes

#define RISCV_EXC_PC_MISALIGN    0  // Instruction address misaligned
#define RISCV_EXC_FETCH_FAULT    1  // Instruction access fault
#define RISCV_EXC_ILLEGAL_INSTR  2  // Illegal instruction
#define RISCV_EXC_BREAKPOINT     3  // Breakpoint
#define RISCV_EXC_LOAD_MISALIGN  4  // Load address misaligned
#define RISCV_EXC_LOAD_FAULT     5  // Load access fault
#define RISCV_EXC_STORE_MISALIGN 6  // Store/AMO address misaligned
#define RISCV_EXC_STORE_FAULT    7  // Store/AMO access fault
#define RISCV_EXC_ECALL_U        8  // Environment call from U-mode
#define RISCV_EXC_ECALL_S        9  // Environment call from S-mode
// 10–11 Reserved
#define RISCV_EXC_FETCH_PFAULT   12  // Instruction page fault
#define RISCV_EXC_LOAD_PFAULT    13  // Load page fault
// 14 Reserved
#define RISCV_EXC_STORE_PFAULT   15  // Store/AMO page fault

// privileged ISA: other

#define RISCV_PAGE_SHIFT 12
#define RISCV_PAGE_SIZE (1 << RISCV_PAGE_SHIFT)

#define RISCV_INT_SSI 1
#define RISCV_INT_SSI_BIT (1 << RISCV_INT_SSI)
#define RISCV_INT_STI 5
#define RISCV_INT_STI_BIT (1 << RISCV_INT_STI)
#define RISCV_INT_SEI 9
#define RISCV_INT_SEI_BIT (1 << RISCV_INT_SEI)


// SBI 0.2

#define SBI_SUCCESS                0
#define SBI_ERR_FAILED            -1
#define SBI_ERR_NOT_SUPPORTED     -2
#define SBI_ERR_INVALID_PARAM     -3
#define SBI_ERR_DENIED            -4
#define SBI_ERR_INVALID_ADDRESS   -5
#define SBI_ERR_ALREADY_AVAILABLE -6
#define SBI_ERR_ALREADY_STARTED   -7
#define SBI_ERR_ALREADY_STOPPED   -8

#define SBI_EID_BASE 0x10
#define SBI_BASE__GET_SBI_SPEC_VERSION 0
#define SBI_BASE__GET_SBI_IMPL_ID      1
#define SBI_BASE__GET_SBI_IMPL_VERSION 2
#define SBI_BASE__PROBE_EXTENSION      3
#define SBI_BASE__GET_MVENDORID        4
#define SBI_BASE__GET_MARCHID          5
#define SBI_BASE__GET_MIMPID           6

#define SBI_EID_TIMER 0x54494D45
#define SBI_TIMER__SET_TIMER 0

#define SBI_EID_HSM 0x48534D
#define SBI_HSM__HART_START           0
#define SBI_HSM__HART_STOP            1
#define SBI_HSM__HART_GET_STATUS      2
#define SBI_HSM__HART_SUSPEND         3  // SBI 0.3
#define SBI_HSM_STATE_STARTED         0
#define SBI_HSM_STATE_STOPPED         1
#define SBI_HSM_STATE_START_PENDING   2
#define SBI_HSM_STATE_STOP_PENDING    3
#define SBI_HSM_STATE_SUSPENDED       4
#define SBI_HSM_STATE_SUSPEND_PENDING 5
#define SBI_HSM_STATE_RESUME_PENDING  6

#define SBI_EID_RST 0x53525354
#define SBI_RST__SYSTEM_RESET         0
#define SBI_RST_TYPE_SHUTDOWN         0
#define SBI_RST_TYPE_COLD_REBOOT      1
#define SBI_RST_TYPE_WARM_REBOOT      2
#define SBI_RST_REASON_NONE           0
#define SBI_RST_REASON_SYSTEM_FAILURE 1
