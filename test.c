#define _GNU_SOURCE

#include "core.h"
#include <linux/elf.h>
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

// leave a 4 word unmapped gap, to detect stack underflows
#define STACK_INITIAL ((uint32_t)-16)
#define STACK_SIZE (1024 * 1024)

typedef struct {
    Elf32_Phdr phdr;
    uint32_t* data;
} segment_t;

typedef struct {
    segment_t *segments;
    size_t nsegments;
} emu_state_t;

static segment_t *find_segment_unchecked(core_t* core, uint32_t addr, Elf32_Word required_flag, uint8_t width) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    // search for appropriate segment
    size_t n;
    Elf32_Phdr *phdr;
    uint32_t end_addr = addr + width; // FIXME: what if we want to access last word, blah blah
    for (n = 0; n < data->nsegments; n++) {
        phdr = &data->segments[n].phdr;
        if (addr >= phdr->p_vaddr && end_addr - phdr->p_vaddr <= phdr->p_memsz)
            goto found_segment;
    }
    core->error = ERR_MEM_UNMAPPED;
    return NULL;
found_segment:
    // perform additional checks
    if (unlikely(!(phdr->p_flags & required_flag))) {
        core->error = ERR_MEM_PROTECTED;
        return NULL;
    }
    return &data->segments[n];
}

static segment_t *find_segment(core_t* core, uint32_t addr, Elf32_Word required_flag, uint8_t width) {
    // reject misaligned accesses (if we wanted to allow them, we'd at least have to check for overflow)
    if (unlikely((addr & (width - 1)))) {
        core->error = ERR_MEM_PROTECTED; // FIXME: add error for misaligned access
        return NULL;
    }
    return find_segment_unchecked(core, addr, required_flag, width);
}

static void mem_fetch(core_t* core, uint32_t addr, uint32_t* value) {
    segment_t *seg;
    if (!(seg = find_segment_unchecked(core, addr, PF_X, 4))) return;
    *value = seg->data[(addr - seg->phdr.p_vaddr) >> 2];
}

#define MEM_SWITCH(flag, cases) \
    const Elf32_Word required_flag = flag; \
    segment_t *seg; \
    uint32_t reladdr; \
    uint32_t *cell; \
    uint8_t offset; \
    switch (width) { \
        cases \
        default: \
            core->error = ERR_BAD_INSTRUCTION; \
            return; \
    }

#define MEM_CASE(func, width, code) \
    case (func): \
    if (!(seg = find_segment(core, addr, required_flag, (width)))) return; \
    reladdr = addr - seg->phdr.p_vaddr; \
    offset = (reladdr & 0b11) * 8; \
    cell = &seg->data[reladdr >> 2]; \
    code; break;

static void mem_load(core_t* core, uint32_t addr, uint8_t width, uint32_t* value) {
    MEM_SWITCH(PF_R,
        MEM_CASE(RISCV_MEM_LW, 4, *value = *cell)
        MEM_CASE(RISCV_MEM_LHU, 2, *value = (uint32_t)(uint16_t)((*cell) >> offset))
        MEM_CASE(RISCV_MEM_LH, 2, *value = (uint32_t)(int32_t)(int16_t)((*cell) >> offset))
        MEM_CASE(RISCV_MEM_LBU, 1, *value = (uint32_t)(uint8_t)((*cell) >> offset))
        MEM_CASE(RISCV_MEM_LB, 1, *value = (uint32_t)(int32_t)(int8_t)((*cell) >> offset))
    )
}

static void mem_store(core_t* core, uint32_t addr, uint8_t width, uint32_t value) {
    MEM_SWITCH(PF_W,
        MEM_CASE(RISCV_MEM_SW, 4, *cell = value)
        MEM_CASE(RISCV_MEM_SH, 2, *cell = ((*cell) & ~(MASK(16) << offset)) | (value & MASK(16)) << offset)
        MEM_CASE(RISCV_MEM_SB, 1, *cell = ((*cell) & ~(MASK(8) << offset)) | (value & MASK(8)) << offset)
    )
}

int main(int argc, char** argv) {
    // load ELF
    if (argc != 2) {
        fprintf(stderr, "Usage: ./core <elf file>\n");
        return 2;
    }
    FILE* elf = fopen(argv[1], "r");
    if (!elf) {
        fprintf(stderr, "Could not open ELF\n");
        return 2;
    }
    Elf32_Ehdr ehdr;
    if (!fread(&ehdr, sizeof(ehdr), 1, elf)) {
        fprintf(stderr, "Could not read ELF header\n");
        return 2;
    }
    if (memcmp(ehdr.e_ident, ELFMAG, 4) != 0) {
        fprintf(stderr, "Not an ELF\n");
        return 2;
    }
    if (ehdr.e_ident[EI_CLASS] != ELFCLASS32 || ehdr.e_ident[EI_DATA] != ELFDATA2LSB || ehdr.e_ident[EI_VERSION] != EV_CURRENT ||
        ehdr.e_machine != EM_RISCV || ehdr.e_type != ET_EXEC) {
        fprintf(stderr, "Not an RV32 LE executable\n");
        return 2;
    }

    // parse / map segments
    emu_state_t data;
    assert(ehdr.e_phentsize >= sizeof(Elf32_Phdr));
    data.segments = calloc(ehdr.e_phnum + /* stack */ 1, sizeof(*data.segments));
    data.nsegments = 0;
    assert(data.segments);
    for (size_t n = 0; n < ehdr.e_phnum; n++) {
        segment_t *segment = &data.segments[data.nsegments];
        Elf32_Phdr *phdr = &segment->phdr;
        if (fseek(elf, ehdr.e_phoff + n * ehdr.e_phentsize, SEEK_SET) < 0 ||
            fread(phdr, sizeof(*phdr), 1, elf) != 1) {
            fprintf(stderr, "Could not read PH\n");
            return 2;
        }
        if (phdr->p_type == PT_LOAD) {
            data.nsegments++;
            assert(phdr->p_memsz == phdr->p_filesz);
            assert(!(phdr->p_vaddr & 0b11));
            segment->data = (uint32_t*) mmap(NULL, phdr->p_filesz, PROT_READ | PROT_WRITE, MAP_PRIVATE, fileno(elf), phdr->p_offset);
            if (segment->data == MAP_FAILED) {
                fprintf(stderr, "Could not map PH %lu\n", n);
                return 2;
            }
            assert(!(((uintptr_t)segment->data) & 0b11));
        }
    }

    // map stack
    segment_t* stack_seg = &data.segments[data.nsegments++];
    Elf32_Phdr *phdr = &stack_seg->phdr;
    phdr->p_vaddr = STACK_INITIAL - STACK_SIZE;
    phdr->p_memsz = STACK_SIZE;
    phdr->p_flags = PF_R | PF_W;
    assert(!(phdr->p_vaddr & 0b11));
    stack_seg->data = mmap(NULL, phdr->p_memsz, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (stack_seg->data == MAP_FAILED) {
        fprintf(stderr, "Could not map stack\n");
        return 2;
    }
    assert(!(((uintptr_t)stack_seg->data) & 0b11));

    // FIXME: sort segments, check no overlap

    // initialize emulator
    core_t core;
    memset(&core, 0, sizeof(core));
    core.user_data = &data;
    core.mem_fetch = mem_fetch;
    core.mem_load = mem_load;
    core.mem_store = mem_store;

    core.pc = ehdr.e_entry;
    assert(!(core.pc & 0b11));
    core.x_regs[2] = STACK_INITIAL;

    // emulate!
    int cycle_count_fd = simple_perf_counter(PERF_COUNT_HW_CPU_CYCLES);
    int instr_count_fd = simple_perf_counter(PERF_COUNT_HW_INSTRUCTIONS);
    struct timespec start, end;

    int res = clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    ioctl(cycle_count_fd, PERF_EVENT_IOC_ENABLE, 0);
    ioctl(instr_count_fd, PERF_EVENT_IOC_ENABLE, 0);

    while (1) {
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
