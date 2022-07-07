#define _GNU_SOURCE

#include "core.h"
#include "reg_macros.h"
#include <stdio.h>
#include <sys/mman.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "measure.c"

#define MASK(n) (~((~0U << (n))))
#define unlikely(x) __builtin_expect((x),0)
#define likely(x) __builtin_expect((x),1)

#define SBI_IMPL_ID 0x999
#define SBI_IMPL_VERSION 1
#define RISCV_MVENDORID 0x12345678
#define RISCV_MARCHID ((1 << 31) | 1)
#define RISCV_MIMPID 1


// main memory handlers (address must be relative, assumes it's within bounds)

#define MEM_CASE(func, width, code) \
    case (func): \
    if (unlikely((addr & ((width) - 1)))) { \
        core_set_exception(core, exc_cause, core->exc_val); \
        break; \
    } \
    offset = (addr & 0b11) * 8; \
    cell = &mem[addr >> 2]; \
    code; break;

#define __main_mem_body(R, W) \
    uint32_t *cell; \
    uint8_t offset; \
    const uint32_t exc_cause = R(RISCV_EXC_LOAD_MISALIGN) W(RISCV_EXC_STORE_MISALIGN); \
    switch (width) { \
    R( \
        MEM_CASE(RISCV_MEM_LW,  4, *value = *cell) \
        MEM_CASE(RISCV_MEM_LHU, 2, *value = (uint32_t)(uint16_t)((*cell) >> offset)) \
        MEM_CASE(RISCV_MEM_LH,  2, *value = (uint32_t)(int32_t)(int16_t)((*cell) >> offset)) \
        MEM_CASE(RISCV_MEM_LBU, 1, *value = (uint32_t)(uint8_t)((*cell) >> offset)) \
        MEM_CASE(RISCV_MEM_LB,  1, *value = (uint32_t)(int32_t)(int8_t)((*cell) >> offset)) \
    ) \
    W( \
        MEM_CASE(RISCV_MEM_SW,  4, *cell = value) \
        MEM_CASE(RISCV_MEM_SH,  2, *cell = ((*cell) & ~(MASK(16) << offset)) | (value & MASK(16)) << offset) \
        MEM_CASE(RISCV_MEM_SB,  1, *cell = ((*cell) & ~(MASK(8) << offset)) | (value & MASK(8)) << offset) \
    ) \
        default: \
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); \
            return; \
    }

REG_FUNCTIONS(void main_mem, (core_t* core, uint32_t* mem, uint32_t addr, uint8_t width), uint32_t, __main_mem_body)


// emulate 8250 (plain, without loopback mode support)

#define U8250_INT_MS   0
#define U8250_INT_THRE 1
#define U8250_INT_RDA  2
#define U8250_INT_LS   3

typedef struct {
    // divisor (ignored)
    uint8_t dll, dlh;
    // uart config (basically just for DLAB)
    uint8_t lcr;
    // interrupt config (basically just bit 1)
    uint8_t ier;
    // interrupt status
    uint8_t current_int;
    uint8_t pending_ints;
    // other output signals, loopback mode (ignored)
    uint8_t mcr;
} u8250_state_t;

void u8250_update_interrupts(u8250_state_t *uart) {
    // prevent generating any disabled interrupts in the first place
    uart->pending_ints &= uart->ier;
    // update current interrupt (higher bits -> more priority)
    if (uart->pending_ints) {
        uart->current_int = 8;
        while (!(uart->pending_ints & (1 << --uart->current_int))); // FIXME: use a LUT
    }
}

#define __u8250_body(R, W) \
    switch (addr) { \
        case 0: \
            if (uart->lcr & (1 << 7)) /* DLAB */ \
                { REG_LVALUE(R, W, uart->dll); break; } \
            W(putc(value, stdout); fflush(stdout);) /* FIXME */ \
            W(uart->pending_ints |= 1 << U8250_INT_THRE;) \
            R(*value = 0;) \
            break; \
        case 1: \
            if (uart->lcr & (1 << 7)) /* DLAB */ \
                { REG_LVALUE(R, W, uart->dlh); break; } \
            REG_LVALUE(R, W, uart->ier); \
            break; \
        R(case 2: \
            *value = (uart->current_int << 1) | (uart->pending_ints ? 0 : 1); \
            if (uart->current_int == U8250_INT_THRE) \
                uart->pending_ints &= ~(1 << uart->current_int); \
            break; \
        ) \
        REG_CASE_RW(R, W, 3, uart->lcr) \
        REG_CASE_RW(R, W, 4, uart->mcr) \
        REG_CASE_RO(R, W, 5, 0x60) /* LSR = no error, TX done & ready */ \
        REG_CASE_RO(R, W, 6, 0xb0) /* MSR = carrier detect, no ring, data ready, clear to send */ \
        /* no scratch register, so we should be detected as a plain 8250 */ \
        R(default: *value = 0;) \
    }

REG_FUNCTIONS(void u8250_reg, (u8250_state_t *uart, uint32_t addr), uint8_t, __u8250_body)

// we still need a wrapper for memory accesses
#define __u8250_wrap_body(R, W) \
    R(uint8_t u8value;) \
    switch (width) { \
    R( \
        case RISCV_MEM_LBU: \
            u8250_reg_read(uart, addr, &u8value); \
            *value = (uint32_t)u8value; \
            break; \
        case RISCV_MEM_LB: \
            u8250_reg_read(uart, addr, &u8value); \
            *value = (uint32_t)(int8_t)u8value; \
            break; \
    ) \
    W( \
        case RISCV_MEM_SB: \
            u8250_reg_write(uart, addr, value); \
            break; \
    ) \
        R(case RISCV_MEM_LW:) \
        R(case RISCV_MEM_LHU:) \
        R(case RISCV_MEM_LH:) \
        W(case RISCV_MEM_SW:) \
        W(case RISCV_MEM_SH:) \
            core_set_exception(core, R(RISCV_EXC_LOAD_MISALIGN) W(RISCV_EXC_STORE_MISALIGN), core->exc_val); \
            return; \
        default: \
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); \
            return; \
    }

REG_FUNCTIONS(void u8250_wrap_mem, (core_t *core, u8250_state_t *uart, uint32_t addr, uint8_t width), uint32_t, __u8250_wrap_body)


// PLIC
// we want it to be simple so: 32 interrupts, no priority

typedef struct {
    uint32_t masked;
    uint32_t ip;
    uint32_t ie;
    // state of input interrupt lines (level-triggered), set by environment
    uint32_t active;
} plic_state_t;

void plic_update_interrupts(core_t* core, plic_state_t* plic) {
    // update pending interrupts
    plic->ip |= plic->active & ~plic->masked;
    plic->masked |= plic->active;
    // send interrupt to target
    (plic->ip & plic->ie) ? (core->sip |= RISCV_INT_SEI_BIT) : (core->sip &= ~RISCV_INT_SEI_BIT);
}

#define __plic_body(R, W) \
    switch (addr) { \
        case  1: case  2: case  3: case  4: case  5: case  6: case  7: case  8: case  9: case 10: \
        case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19: case 20: \
        case 21: case 22: case 23: case 24: case 25: case 26: case 27: case 28: case 29: case 30: case 31: \
            R(*value = 1;) return true; /* no priority support -> source priority hardwired to 1 */ \
        R(case 0x400: *value = plic->ip; return true;) \
        case 0x800: \
            W(value &= ~1;) \
            REG_LVALUE(R, W, plic->ie); \
            return true; \
        case 0x80000: \
            R(*value = 0;) return true; /* no priority support -> target priority threshold hardwired to 0 */ \
        case 0x80001: \
            R( /* claim */ \
                *value = 0; \
                uint32_t candidates = plic->ip & plic->ie; \
                if (candidates) { \
                    while (!( candidates & (1 << (++(*value))) )); \
                    plic->ip &= ~(1 << (*value)); \
                } \
            ) \
            W( /* completion */ \
                if (plic->ie & (1 << value)) \
                    plic->masked &= ~(1 << value); \
            ) \
            return true; \
        default: return false; \
    }

REG_FUNCTIONS(bool plic_reg, (plic_state_t *plic, uint32_t addr), uint32_t, __plic_body)

// we still need a wrapper for memory accesses
#define __plic_wrap_body(R, W) \
    switch (width) { \
        case R(RISCV_MEM_LW) W(RISCV_MEM_SW): \
            if (!REG_FUNC(R, W, plic_reg)(plic, addr >> 2, value)) \
                core_set_exception(core, R(RISCV_EXC_LOAD_FAULT) W(RISCV_EXC_STORE_FAULT), core->exc_val); \
            break; \
        R(case RISCV_MEM_LBU:) \
        R(case RISCV_MEM_LB:) \
        R(case RISCV_MEM_LHU:) \
        R(case RISCV_MEM_LH:) \
        W(case RISCV_MEM_SB:) \
        W(case RISCV_MEM_SH:) \
            core_set_exception(core, R(RISCV_EXC_LOAD_MISALIGN) W(RISCV_EXC_STORE_MISALIGN), core->exc_val); \
            return; \
        default: \
            core_set_exception(core, RISCV_EXC_ILLEGAL_INSTR, 0); \
            return; \
    }

REG_FUNCTIONS(void plic_wrap_mem, (core_t *core, plic_state_t *plic, uint32_t addr, uint8_t width), uint32_t, __plic_wrap_body)


// memory mapping

#define RAM_SIZE (512 * 1024 * 1024)

#define IRQ_UART 1
#define IRQ_UART_BIT (1 << IRQ_UART)

typedef struct {
    uint32_t *ram;
    plic_state_t plic;
    u8250_state_t uart;
    uint32_t timer_l;
    uint32_t timer_h;
} emu_state_t;

// we define fetch separately because it's much simpler (width is fixed,
// alignment already checked, only main RAM is executable)
static void mem_fetch(core_t* core, uint32_t addr, uint32_t* value) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    if (unlikely(addr >= RAM_SIZE)) {
        // FIXME: check for other regions
        core_set_exception(core, RISCV_EXC_FETCH_FAULT, core->exc_val);
        return;
    }
    *value = data->ram[addr >> 2];
}

// similarly only main memory pages can be used as page_tables
static uint32_t* mem_page_table(const core_t* core, uint32_t ppn) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    if (ppn < (RAM_SIZE / RISCV_PAGE_SIZE))
        return &data->ram[ppn << (RISCV_PAGE_SHIFT - 2)];
    return NULL;
}

#define __mem_body(R, W) \
    emu_state_t *data = (emu_state_t *)core->user_data; \
    /* RAM at 0x00000000 + RAM_SIZE */ \
    if (addr < RAM_SIZE) { \
        REG_FUNC(R, W, main_mem)(core, data->ram, addr, width, value); return; \
    } \
    /* MMIO at 0xF_______ */ \
    if ((addr >> 28) == 0xF) { \
        /* 256 regions of 1MiB */ \
        switch ((addr >> 20) & MASK(8)) { \
            case 0x0: case 0x2: /* PLIC (0 - 0x3F) */ \
                REG_FUNC(R, W, plic_wrap_mem)(core, &data->plic, addr & 0x3FFFFFF, width, value); \
                plic_update_interrupts(core, &data->plic); \
                return; \
            case 0x40: /* UART */ \
                REG_FUNC(R, W, u8250_wrap_mem)(core, &data->uart, addr & 0xFFFFF, width, value); \
                u8250_update_interrupts(&data->uart); \
                data->uart.pending_ints ? (data->plic.active |= IRQ_UART_BIT) : (data->plic.active &= ~IRQ_UART_BIT); \
                plic_update_interrupts(core, &data->plic); \
                return; \
        } \
    } \
    core_set_exception(core, R(RISCV_EXC_LOAD_FAULT) W(RISCV_EXC_STORE_FAULT), core->exc_val);

REG_FUNCTIONS(void mem, (core_t *core, uint32_t addr, uint8_t width), uint32_t, __mem_body)


// SBI

typedef struct {
    int32_t error;
    int32_t value;
} sbiret_t;

sbiret_t handle_sbi_ecall_TIMER(core_t* core, int32_t fid) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    switch (fid) {
        case SBI_TIMER__SET_TIMER:
            data->timer_l = core->x_regs[RISCV_R_A0];
            data->timer_h = core->x_regs[RISCV_R_A1];
            return (sbiret_t){ SBI_SUCCESS, 0 };
    }
    return (sbiret_t){ SBI_ERR_NOT_SUPPORTED, 0 };
}

sbiret_t handle_sbi_ecall_BASE(core_t* core, int32_t fid) {
    switch (fid) {
        case SBI_BASE__GET_SBI_IMPL_ID:
            return (sbiret_t){ SBI_SUCCESS, SBI_IMPL_ID };
        case SBI_BASE__GET_SBI_IMPL_VERSION:
            return (sbiret_t){ SBI_SUCCESS, SBI_IMPL_VERSION };
        case SBI_BASE__GET_MVENDORID:
            return (sbiret_t){ SBI_SUCCESS, RISCV_MVENDORID };
        case SBI_BASE__GET_MARCHID:
            return (sbiret_t){ SBI_SUCCESS, RISCV_MARCHID };
        case SBI_BASE__GET_MIMPID:
            return (sbiret_t){ SBI_SUCCESS, RISCV_MIMPID };
        case SBI_BASE__GET_SBI_SPEC_VERSION:
            return (sbiret_t){ SBI_SUCCESS, (0 << 24) | (3) }; // v0.3
        case SBI_BASE__PROBE_EXTENSION:
            int32_t eid = (int32_t)core->x_regs[RISCV_R_A0];
            bool available = eid == SBI_EID_BASE || eid == SBI_EID_TIMER;
            return (sbiret_t){ SBI_SUCCESS, available };
    }
    return (sbiret_t){ SBI_ERR_NOT_SUPPORTED, 0 };
}

#define __SBI_CASE(SLUG) \
    case SBI_EID_ ## SLUG: \
        ret = handle_sbi_ecall_ ## SLUG(core, core->x_regs[RISCV_R_A6]); break;

void handle_sbi_ecall(core_t* core) {
    sbiret_t ret;
    switch (core->x_regs[RISCV_R_A7]) {
        __SBI_CASE(BASE)
        __SBI_CASE(TIMER)
        default:
            ret = (sbiret_t){ SBI_ERR_NOT_SUPPORTED, 0 };
    }
    core->x_regs[RISCV_R_A0] = (uint32_t)ret.error;
    core->x_regs[RISCV_R_A1] = (uint32_t)ret.value;
    // clear error to allow execution to continue
    core->error = ERR_NONE;
}


// main emulation

void read_file_into_ram(char** ram_cursor, const char* name) {
    FILE *input_file = fopen(name, "r");
    if (!input_file) {
        fprintf(stderr, "could not open %s\n", name);
        exit(2);
    }
    // FIXME: map instead of reading
    while (!feof(input_file)) {
        *ram_cursor += fread(*ram_cursor, sizeof(char), 1024 * 1024, input_file);
        assert(!ferror(input_file));
    }
    fclose(input_file);
}

int main() {
    // initialize emulator
    emu_state_t data;
    memset(&data, 0, sizeof(data));
    core_t core;
    memset(&core, 0, sizeof(core));
    core.user_data = &data;
    core.mem_fetch = mem_fetch;
    core.mem_load = mem_read;
    core.mem_store = mem_write;
    core.mem_page_table = mem_page_table;

    // set up RAM
    data.ram = mmap(NULL, RAM_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (data.ram == MAP_FAILED) {
        fprintf(stderr, "Could not map RAM\n");
        return 2;
    }
    assert(!(((uintptr_t)data.ram) & 0b11));

    char* ram_cursor = (char*) data.ram;
    read_file_into_ram(&ram_cursor, "linux/arch/riscv/boot/Image");
    // load at last MB to prevent kernel / initrd from overwriting it
    uint32_t dtb_addr = RAM_SIZE - 1024 * 1024;
    ram_cursor = ((char*)data.ram) + dtb_addr;
    read_file_into_ram(&ram_cursor, "linux_dtb");

    data.timer_h = data.timer_l = 0xFFFFFFFF;
    core.s_mode = true;
    core.x_regs[RISCV_R_A0] = 0; // hart ID (cpuid)
    core.x_regs[RISCV_R_A1] = dtb_addr; // FIXME: does it get overwritten?

    // emulate!
    int cycle_count_fd = simple_perf_counter(PERF_COUNT_HW_CPU_CYCLES);
    int instr_count_fd = simple_perf_counter(PERF_COUNT_HW_INSTRUCTIONS);
    struct timespec start, end;

    int res = clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    ioctl(cycle_count_fd, PERF_EVENT_IOC_ENABLE, 0);
    ioctl(instr_count_fd, PERF_EVENT_IOC_ENABLE, 0);

    while (1) {
        //printf("pc: %#08x, sp: %#08x\n", core.pc, core.x_regs[RISCV_R_SP]);

        bool timer_active = core.instr_count_h > data.timer_h || (core.instr_count_h == data.timer_h && core.instr_count > data.timer_l);
        timer_active ? (core.sip |= RISCV_INT_STI_BIT) : (core.sip &= ~RISCV_INT_STI_BIT);

        core_step(&core);
        if (likely(!core.error)) continue;

        if (core.error == ERR_EXCEPTION && core.exc_cause == RISCV_EXC_ECALL_S) {
            handle_sbi_ecall(&core);
            continue;
        }

        // for now, don't delegate any S-mode exceptions
        if (core.error == ERR_EXCEPTION) {
            core_trap(&core);
            continue;
        }

        fprintf(stderr, "core error %s: %s. val=%#x\n", core_error_str(core.error), core_exc_cause_str(core.exc_cause), core.exc_val);
        return 2;
    }

    res |= ioctl(cycle_count_fd, PERF_EVENT_IOC_DISABLE, 0);
    res |= ioctl(instr_count_fd, PERF_EVENT_IOC_DISABLE, 0);
    res |= clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);

    assert(!res);
    int64_t elapsed = ((int64_t)end.tv_sec - (int64_t)start.tv_sec) * 1000000000 + ((int64_t)end.tv_nsec - (int64_t)start.tv_nsec);
    long long host_instr_count = simple_perf_read(instr_count_fd);
    long long host_cycle_count = simple_perf_read(cycle_count_fd);
    printf("executed %u instructions in %.3f ms, %lld instructions, %lld cycles\n",
        core.instr_count, elapsed / 1e6, host_instr_count, host_cycle_count);
    printf("stats: %.3f c/i, %.1f i/I, %.1f c/I, %.1f MIps\n",
        ((double)host_cycle_count) / ((double)host_instr_count),
        ((double)host_instr_count) / ((double)core.instr_count),
        ((double)host_cycle_count) / ((double)core.instr_count),
        core.instr_count / (double)elapsed * 1e3);
}
