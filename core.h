#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "riscv_constants.h"

typedef enum {
    ERR_NONE,
    ERR_EXCEPTION,  // RISC-V exception was raised (see additional fields)
    ERR_USER,       // user-specific error
} core_error_t;

// returns the identifier for an error code as a string
const char* core_error_str(core_error_t err);

// returns a readable description for a RISC-V exception cause
const char* core_exc_cause_str(uint32_t cause);

// # Introduction
//
// to use the emulator, zero-initialize a core_t struct (I'm too lazy to
// make an init function) and set the environment-supplied callbacks.
// all of them are currently required.
//
// set any other field you need (argument registers and s_mode are frequent,
// though they all have sane defaults). some fields only admit a subset of
// values; make sure all restrictions are satisfied on all fields before
// starting emulation, or you may get UB.
//
// once everything is set up, you basically call core_step() in a loop.
// each call (attempts to) execute one instruction.
//
// if execution completes successfully then core->error will be ERR_NONE;
// otherwise execution is halted and core->error describes the error.
// you need to handle the emulation error before calling core_step()
// again (or it won't do anything). each possible error is described in a
// section below.
//
// ## RISC-V exceptions
//
// ERR_EXCEPTION means the instruction raised one of the exceptions defined
// in the spec. if set, then the additional fields `exc_cause` and `exc_val`
// must have also been set to values according to the spec.
//
// common way to handle exceptions include:
//
//  - environment call: since pc is already advanced, environment calls can be
//    handled by simply clearing error after finishing the operation.
//  - faults: when handling faults, also set core->pc = core->current_pc so
//    that the instruction will be retried.
//  - traps: exceptions can be delegated to the emulated code in the form of
//    S-mode traps, by calling core_trap(). it takes care of clearing error.
//
// ## User errors
//
// ERR_USER is never set from a core_*() function; it's reserved for user
// callbacks to halt instruction execution and cause core_step() to return.
// unless it's from a fetch, pc will be advanced, so the same applies.

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

// delegate the currently set exception to S-mode as a trap. this doesn't check
// core->error is EXC_EXCEPTION, it just assumes that `exc_cause` and `exc_val`
// are correctly set. sets core->error to ERR_NONE.
void core_trap(core_t* core);
