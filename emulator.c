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


// main memory handlers (address must be relative, assumes it's within bounds)

#define MEM_CASE(func, width, code) \
    case (func): \
    if (unlikely((addr & ((width) - 1)))) { \
        core->error = ERR_MEM_PROTECTED; /* FIXME: add error for misaligned access */ \
        break; \
    } \
    offset = (addr & 0b11) * 8; \
    cell = &mem[addr >> 2]; \
    code; break;

#define __main_mem_body(R, W) \
    uint32_t *cell; \
    uint8_t offset; \
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
            core->error = ERR_BAD_INSTRUCTION; \
            return; \
    }

REG_FUNCTIONS(void main_mem, (core_t* core, uint32_t* mem, uint32_t addr, uint8_t width), uint32_t, __main_mem_body)


// emulate 8250 (plain, without loopback mode support)

typedef struct {
    // divisor (ignored)
    uint8_t dll, dlh;
    // uart config (basically just for DLAB)
    uint8_t lcr;
    // interrupt config (basically just bit 1)
    uint8_t ier;
    // other output signals, loopback mode (ignored)
    uint8_t mcr;
} u8250_state_t;

#define __u8250_body(R, W) \
    switch (addr) { \
        case 0: \
            if (uart->lcr & (1 << 7)) /* DLAB */ \
                { REG_LVALUE(R, W, uart->dll); break; } \
            W(printf("incoming character on UART: %u\n", value);) /* FIXME */ \
            R(*value = 0;) \
            break; \
        case 1: \
            if (uart->lcr & (1 << 7)) /* DLAB */ \
                { REG_LVALUE(R, W, uart->dlh); break; } \
            REG_LVALUE(R, W, uart->ier); /* FIXME: support THE interrupt */ \
            break; \
        REG_CASE_RO(R, W, 2, 0x01) /* IIR = no pending interrupt */ \
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
            core->error = ERR_MEM_PROTECTED; /* FIXME: error for misaligned */ \
            return; \
        default: \
            core->error = ERR_BAD_INSTRUCTION; \
            return; \
    }

REG_FUNCTIONS(void u8250_wrap_mem, (core_t *core, u8250_state_t *uart, uint32_t addr, uint8_t width), uint32_t, __u8250_wrap_body)


// memory mapping

#define RAM_SIZE (512 * 1024 * 1024)

typedef struct {
    uint32_t *ram;
    u8250_state_t uart;
} emu_state_t;

// we define fetch separately because it's much simpler (width is fixed,
// alignment already checked, only main RAM is executable)
static void mem_fetch(core_t* core, uint32_t addr, uint32_t* value) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    if (unlikely(addr >= RAM_SIZE)) {
        // FIXME: check for other regions, issue MEM_PROTECTED if needed
        core->error = ERR_MEM_UNMAPPED;
    }
    *value = data->ram[addr >> 2];
}

#define __mem_body(R, W) \
    emu_state_t *data = (emu_state_t *)core->user_data; \
    /* RAM at 0x00000000 + RAM_SIZE */ \
    if (addr < RAM_SIZE) { \
        REG_FUNC(R, W, main_mem)(core, data->ram, addr, width, value); return; \
    } \
    /* MMIO at 0xF000____ */ \
    if ((addr & 0xFFFF0000) == 0xF0000000) { \
        /* 16 regions of 4kiB */ \
        switch ((addr & 0xF000) >> 12) { \
            case 0: /* UART */ \
                REG_FUNC(R, W, u8250_wrap_mem)(core, &data->uart, addr & 0xFFF, width, value); return; \
        } \
    } \
    core->error = ERR_MEM_UNMAPPED;

REG_FUNCTIONS(void mem, (core_t *core, uint32_t addr, uint8_t width), uint32_t, __mem_body)


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

    // set up RAM
    data.ram = mmap(NULL, RAM_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (data.ram == MAP_FAILED) {
        fprintf(stderr, "Could not map RAM\n");
        return 2;
    }
    assert(!(((uintptr_t)data.ram) & 0b11));

    char* ram_cursor = (char*) data.ram;
    read_file_into_ram(&ram_cursor, "linux/arch/riscv/boot/Image");
    uint32_t dtb_addr = ((uint32_t)(ram_cursor - (char*)data.ram) & 0x1000) + 0x1000;
    ram_cursor = ((char*)data.ram) + dtb_addr;
    read_file_into_ram(&ram_cursor, "linux_dtb");

    core.s_mode = true;
    core.x_regs[10] = 0; // hart ID (cpuid)
    core.x_regs[11] = dtb_addr; // FIXME: does it get overwritten?

    // emulate!
    int cycle_count_fd = simple_perf_counter(PERF_COUNT_HW_CPU_CYCLES);
    int instr_count_fd = simple_perf_counter(PERF_COUNT_HW_INSTRUCTIONS);
    struct timespec start, end;

    int res = clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    ioctl(cycle_count_fd, PERF_EVENT_IOC_ENABLE, 0);
    ioctl(instr_count_fd, PERF_EVENT_IOC_ENABLE, 0);

    while (1) {
        //printf("pc: %#08x, sp: %#08x\n", core.pc, core.x_regs[2]);
        core_step(&core);
        if (likely(!core.error)) continue;

        if (core.error == ERR_SYSTEM) break; // FIXME
        fprintf(stderr, "core error: %s\n", core_error_str(core.error));
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