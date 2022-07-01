#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "riscv_constants.h"

typedef enum {
    ERR_NONE,
    ERR_EXCEPTION,  // RISC-V exception was raised (see additional fields)
    ERR_USER,       // user-specific error
} core_error_t;

const char* core_error_str(core_error_t err);

const char* core_exc_cause_str(uint32_t cause);

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
    // IMPORTANT: assumed to contain an aligned address at all times
    uint32_t pc;
    // address of last instruction that began execution
    uint32_t current_pc;
    // 'instructions executed' 64-bit counter. currently used
    // as "real-time clock", instruction-retired counter, and cycle counter.
    // should not be modified between logical resets.
    uint32_t instr_count, instr_count_h;
    // instruction execution state. if not NONE, then instruction execution
    // can't continue and core_step() exits
    core_error_t error;
    // if error == ERR_EXCEPTION, these specify values that will go in
    // scause & stval if turned into a trap. see RISC-V spec for meaning
    uint32_t exc_cause, exc_val;

    // supervisor state
    bool s_mode;
    bool sstatus_spp;     // state saved at trap
    bool sstatus_spie;
    uint32_t sepc;
    uint32_t scause;
    uint32_t stval;
    bool sstatus_mxr;     // alter MMU access rules
    bool sstatus_sum;
    bool sstatus_sie;     // interrupt state
    uint32_t sie;
    uint32_t sip;
    uint32_t stvec_addr;  // trap config
    bool stvec_vectored;
    uint32_t sscratch;    // misc
    uint32_t scounteren;
    bool satp_enabled;    // MMU
    uint32_t satp_addr;

    // ENVIRONMENT SUPPLIED
    void* user_data;
    // memory access. sets core->error on failure, reads/writes `value` otherwise.
    void (*mem_fetch)(core_t* core, uint32_t addr, uint32_t* value);
    void (*mem_load)(core_t* core, uint32_t addr, uint8_t width, uint32_t* value);
    void (*mem_store)(core_t* core, uint32_t addr, uint8_t width, uint32_t value);
};

// (try to) emulate the next instruction. no-op if error is set.
void core_step(core_t* core);

// raise a RISC-V exception. equivalent to setting core->error to EXC_EXCEPTION
// and setting the accompanying fields, but provided as a function for convenience
// and to prevent mistakes like forgetting to set a field (especially on future
// refactors)
void core_set_exception(core_t* core, uint32_t cause, uint32_t val);
