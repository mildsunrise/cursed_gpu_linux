#pragma once

#include <stdint.h>
#include "riscv_constants.h"

typedef enum {
    ERR_NONE,
    ERR_BAD_INSTRUCTION, // an unknown instruction was decoded
    ERR_JUMP_UNALIGNED,  // misaligned jump attempted
    ERR_MEM_UNMAPPED,    // attempted to access unmapped memory
    ERR_MEM_PROTECTED,   // memory doesn't allow that access
    ERR_SYSTEM,          // EEI operation was hit
} core_error_t;

// ERR_SYSTEM is returned whenever an environment-specific instruction is executed.
// pc is already advanced, so after finishing the operation, execution can be
// resumed by just clearing error.
//
// you may also set ERR_SYSTEM from callbacks to signal that the instruction is halted
// pending operation from the environment. when doing this, it probably makes sense
// to also set core->pc = core->current_pc so that the instruction will be retried.

// core state. initialize with zeros (I'm too lazy to make an init function)
typedef struct _core_t core_t;
struct _core_t {
    uint32_t x_regs [32];
    uint32_t pc;
    // address of last instruction that began execution
    uint32_t current_pc;
    core_error_t error;

    // ENVIRONMENT SUPPLIED
    void* user_data;
    // memory access. sets core->error on failure, reads/writes `value` otherwise.
    void (*mem_fetch)(core_t* core, uint32_t addr, uint32_t* value);
    void (*mem_load)(core_t* core, uint32_t addr, uint8_t width, uint32_t* value);
    void (*mem_store)(core_t* core, uint32_t addr, uint8_t width, uint32_t value);
};

// (try to) emulate the next instruction. no-op if error is set.
void core_step(core_t* core);
