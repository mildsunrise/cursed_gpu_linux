#define _GNU_SOURCE

#include "core.h"
#include "reg_macros.h"
#include "virtio_constants.h"
#include <stdio.h>
#include <sys/mman.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <sys/eventfd.h>
#include <stdatomic.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <sys/uio.h>
#include "measure.c"

#ifdef USE_VIRGLRENDERER
#include <virglrenderer.h>
#endif

#define MASK(n) (~((~0U << (n))))
#define unlikely(x) __builtin_expect((x),0)
#define likely(x) __builtin_expect((x),1)

#define SBI_IMPL_ID 0x999
#define SBI_IMPL_VERSION 1
#define RISCV_MVENDORID 0x12345678
#define RISCV_MARCHID ((1 << 31) | 1)
#define RISCV_MIMPID 1
#define VIRTIO_VENDOR_ID 0x12345678

// should be defined under "memory mapping", but defined here because of dependencies
#define RAM_SIZE (512 * 1024 * 1024)

#define TAP_INTERFACE "tap0"


inline uint64_t read_eventfd(int fd) {
    uint64_t value;
    int ret = read(fd, &value, sizeof(value));
    assert(ret == 8);
    return ret;
}

#define __checkerrno(expr, fmt, ...) \
    if (expr) { \
        fprintf(stderr, fmt " failed: %s\n", __VA_ARGS__ __VA_OPT__(,) strerror(errno)); \
        exit(2); \
    }


// SPSC circular queue, with separate write/commit steps

typedef struct {
    // the write head (atomic + local writer copy)
    _Atomic uint8_t write_head_at;
    uint8_t write_head;
    // the read head (local reader)
    uint8_t read_head;
    // an eventfd (written to by writer, after each commit to the atomic)
    int eventfd;
    // elements buffer (shared)
    uint8_t buffer [16];
} spsc_queue_t;

void spsc_queue_init(spsc_queue_t *queue) {
    memset(queue, 0, sizeof(*queue));
    queue->eventfd = eventfd(0, 0);
    assert(queue->eventfd >= 0);
    atomic_init(&queue->write_head_at, 0);
    assert(atomic_is_lock_free(&queue->write_head_at));
}

// writes an item to the queue
//
// warning: there are NO CHECKS for overflows, caller is responsible
// to make sure not to write more elements than queue capacity
// (sizeof(queue->buffer) / sizeof(*queue->buffer) - 1)
void spsc_queue_write(spsc_queue_t *queue, uint8_t value) {
    queue->buffer[queue->write_head] = value;
    queue->write_head++;
    if (queue->write_head == sizeof(queue->buffer) / sizeof(*queue->buffer))
        queue->write_head = 0;
}

// makes the written items visible to the reader end
void spsc_queue_commit(spsc_queue_t *queue) {
    atomic_store_explicit(&queue->write_head_at, queue->write_head, memory_order_release);

    uint64_t wake_value = 1;
    int ret = write(queue->eventfd, &wake_value, sizeof(wake_value));
    assert(ret == sizeof(wake_value));
}

// reads all pending items from the queue, returning true if there were items to read
bool spsc_queue_read_all(spsc_queue_t *queue, void (*cb)(uint8_t value, void *cookie), void *cookie) {
    uint8_t head = atomic_load_explicit(&queue->write_head_at, memory_order_relaxed);
    if (likely(queue->read_head == head))
        return false;
    atomic_thread_fence(memory_order_acquire);

    while (queue->read_head != head) {
        cb(queue->buffer[queue->read_head], cookie);
        queue->read_head++;
        if (queue->read_head == sizeof(queue->buffer) / sizeof(*queue->buffer))
            queue->read_head = 0;
    }
    return true;
}


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
    // I/O handling
    int in_fd, out_fd;
    bool in_ready;
    void (*need_more_data)(void *cookie);
    void *cookie;
} u8250_state_t;

void u8250_update_interrupts(u8250_state_t *uart) {
    // some interrupts are level-generated
    uart->in_ready ? (uart->pending_ints |= 1) : (uart->pending_ints &= ~1); // FIXME: does this also generate an LSR change interrupt?
    // prevent generating any disabled interrupts in the first place
    uart->pending_ints &= uart->ier;
    // update current interrupt (higher bits -> more priority)
    if (uart->pending_ints) {
        uart->current_int = 8;
        while (!(uart->pending_ints & (1 << --uart->current_int))); // FIXME: use a LUT
    }
}

void u8250_handle_out(u8250_state_t* uart, uint8_t value) {
    if (write(uart->out_fd, &value, 1) < 1)
        fprintf(stderr, "failed to write UART output: %s\n", strerror(errno));
}

uint8_t u8250_handle_in(u8250_state_t* uart) {
    uint8_t value = 0;
    if (!uart->in_ready) return value;
    if (read(uart->in_fd, &value, 1) < 0)
        fprintf(stderr, "failed to read UART input: %s\n", strerror(errno));

    struct pollfd pfd = { uart->in_fd, POLLIN, 0 };
    poll(&pfd, 1, 0);
    if (!(pfd.revents & POLLIN)) {
        uart->in_ready = false;
        uart->need_more_data(uart->cookie);
    }
    return value;
}

#define __u8250_body(R, W) \
    switch (addr) { \
        case 0: \
            if (uart->lcr & (1 << 7)) /* DLAB */ \
                { REG_LVALUE(R, W, uart->dll); break; } \
            W(u8250_handle_out(uart, value);) \
            W(uart->pending_ints |= 1 << U8250_INT_THRE;) \
            R(*value = u8250_handle_in(uart);) \
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
        REG_CASE_RO(R, W, 5, 0x60 | (uart->in_ready ? 1 : 0)) /* LSR = no error, TX done & ready */ \
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


// VIRTIO-NET (no extra features. just the basics)

typedef struct {
    uint32_t QueueNum;
    uint32_t QueueDesc;
    uint32_t QueueAvail;
    uint32_t QueueUsed;
    uint16_t last_avail;
    bool ready;
    bool fd_ready;
} virtionet_queue_t;

typedef struct {
    // feature negotiation
    uint32_t DeviceFeaturesSel;
    uint32_t DriverFeatures;
    uint32_t DriverFeaturesSel;
    // queue config
    uint32_t QueueSel;
    virtionet_queue_t queues [2];
    // status
    uint32_t Status;
    uint32_t InterruptStatus;
    // supplied by environment
    int tap_fd;
    uint32_t* ram;
    void (*need_more_data)(int queue_idx, void *cookie);
    void *cookie;
} virtionet_state_t;

#define __VNET_FEATURES_0 0
#define __VNET_FEATURES_1 1 // VIRTIO_F_VERSION_1
#define __VNET_QUEUE_NUM_MAX 1024
#define __VNET_QUEUE (vnet->queues[vnet->QueueSel])

#define __VNET_PREPROCESS_ADDR(addr) ((addr) < RAM_SIZE && !((addr) & 0b11) ? ((addr) >> 2) : (virtionet_set_fail(vnet), 0))

void virtionet_set_fail(virtionet_state_t* vnet) {
    fprintf(stderr, "[VNET] device fail\n");
    vnet->Status |= VIRTIO_STATUS__DEVICE_NEEDS_RESET;
    if (vnet->Status & VIRTIO_STATUS__DRIVER_OK)
        vnet->InterruptStatus |= VIRTIO_INT__CONF_CHANGE;
}

void virtionet_update_status(virtionet_state_t* vnet, uint32_t status) {
    vnet->Status |= status;
    if (!status) { // reset
        virtionet_state_t old_vnet = *vnet;
        memset(vnet, 0, sizeof(*vnet));
        vnet->tap_fd = old_vnet.tap_fd;
        vnet->ram = old_vnet.ram;
        vnet->need_more_data = old_vnet.need_more_data;
        vnet->cookie = old_vnet.cookie;
        vnet->queues[0].fd_ready = old_vnet.queues[0].fd_ready;
        vnet->queues[1].fd_ready = old_vnet.queues[1].fd_ready;
    }
    fprintf(stderr, "[VNET] status: %s\n", virtio_status_to_string(vnet->Status));
}

bool __vnet_iovec_write(struct iovec** vecs, size_t* nvecs, const uint8_t* src, size_t n) {
    while (n && *nvecs) {
        if (n < (*vecs)->iov_len) {
            memcpy((*vecs)->iov_base, src, n);
            (*vecs)->iov_base = ((char*)(*vecs)->iov_base) + n;
            (*vecs)->iov_len -= n;
            return true;
        }
        memcpy((*vecs)->iov_base, src, (*vecs)->iov_len);
        src += (*vecs)->iov_len; n -= (*vecs)->iov_len;
        (*vecs)++; (*nvecs)--;
    }
    return n && !*nvecs;
}

bool __vnet_iovec_read(struct iovec** vecs, size_t* nvecs, uint8_t* dst, size_t n) {
    while (n && *nvecs) {
        if (n < (*vecs)->iov_len) {
            memcpy(dst, (*vecs)->iov_base, n);
            (*vecs)->iov_base = ((char*)(*vecs)->iov_base) + n;
            (*vecs)->iov_len -= n;
            return true;
        }
        memcpy(dst, (*vecs)->iov_base, (*vecs)->iov_len);
        dst += (*vecs)->iov_len; n -= (*vecs)->iov_len;
        (*vecs)++; (*nvecs)--;
    }
    return n && !*nvecs;
}

// requires existing 'desc_idx' to use as iteration variable, and input 'buffer_idx'.
#define __VNET_ITERATE_BUFFER(checked, body) \
    desc_idx = buffer_idx; \
    while (1) { \
        if (checked && desc_idx >= queue->QueueNum) \
            return virtionet_set_fail(vnet); \
        uint32_t* desc = &ram[queue->QueueDesc + desc_idx * 4]; \
        uint16_t desc_flags = desc[3]; \
        body \
        if (!(desc_flags & VIRTQ_DESC_F_NEXT)) break; \
        desc_idx = desc[3] >> 16; \
    }

// input 'buffer_idx'. output 'buffer_niovs' and 'buffer_iovs'
#define __VNET_BUFFER_TO_IOV(expect_readable) \
    uint16_t desc_idx; \
    /* do a first pass to validate flags and count buffers */ \
    size_t buffer_niovs = 0; \
    __VNET_ITERATE_BUFFER(true, \
        if ((!!(desc_flags & VIRTQ_DESC_F_WRITE)) != (expect_readable)) \
            return virtionet_set_fail(vnet); \
        buffer_niovs++; \
    ) \
    /* convert to iov */ \
    struct iovec buffer_iovs [buffer_niovs]; \
    buffer_niovs = 0; \
    __VNET_ITERATE_BUFFER(false, \
        uint32_t desc_addr = desc[0]; uint32_t desc_len = desc[2]; \
        buffer_iovs[buffer_niovs].iov_base = ((char*)ram) + desc_addr; \
        buffer_iovs[buffer_niovs].iov_len = desc_len; \
        buffer_niovs++; \
    )

#define __VNET_GENERATE_QUEUE_HANDLER(NAME_SUFFIX, VERB, QUEUE_IDX, READ) \
    void __virtionet_try_##NAME_SUFFIX(virtionet_state_t* vnet) { \
        uint32_t* ram = vnet->ram; \
        virtionet_queue_t* queue = &vnet->queues[QUEUE_IDX]; \
        if (!(vnet->Status & VIRTIO_STATUS__DRIVER_OK) || (vnet->Status & VIRTIO_STATUS__DEVICE_NEEDS_RESET) || !queue->fd_ready) \
            return; \
        if (!queue->ready) return virtionet_set_fail(vnet); \
        \
        /* check for new buffers */ \
        uint16_t new_avail = ram[queue->QueueAvail] >> 16; \
        if (new_avail - queue->last_avail > (uint16_t)queue->QueueNum) \
            return (fprintf(stderr, "size check fail\n"), virtionet_set_fail(vnet)); \
        if (queue->last_avail == new_avail) \
            return; \
        \
        /* process them */ \
        uint16_t new_used = ram[queue->QueueUsed] >> 16; \
        while (queue->last_avail != new_avail) { \
            uint16_t queue_idx = queue->last_avail % queue->QueueNum; \
            uint16_t buffer_idx = ram[queue->QueueAvail + 1 + queue_idx / 2] >> (16 * (queue_idx % 2)); \
            __VNET_BUFFER_TO_IOV(READ) \
            struct iovec* buffer_iovs_cursor = buffer_iovs; \
            uint8_t virtio_header [12]; \
            if (READ) {\
                memset(virtio_header, 0, sizeof(virtio_header)); \
                virtio_header[10] = 1; \
                __vnet_iovec_write(&buffer_iovs_cursor, &buffer_niovs, virtio_header, sizeof(virtio_header)); \
            } else { \
                __vnet_iovec_read(&buffer_iovs_cursor, &buffer_niovs, virtio_header, sizeof(virtio_header)); \
            } \
            \
            ssize_t plen = VERB##v(vnet->tap_fd, buffer_iovs_cursor, buffer_niovs); \
            if (plen < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) { \
                queue->fd_ready = false; \
                vnet->need_more_data(QUEUE_IDX, vnet->cookie); \
                break; \
            } \
            if (plen < 0) { \
                plen = 0; \
                fprintf(stderr, "[VNET] could not " #VERB " packet: %s\n", strerror(errno)); \
            } \
            \
            /* consume from available queue, write to used queue */ \
            queue->last_avail++; \
            ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2] = buffer_idx; \
            ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2 + 1] = READ ? (plen + sizeof(virtio_header)) : 0; \
            new_used++; \
        } \
        vnet->ram[queue->QueueUsed] &= MASK(16); \
        vnet->ram[queue->QueueUsed] |= ((uint32_t)new_used) << 16; \
        \
        /* send interrupt, unless VIRTQ_AVAIL_F_NO_INTERRUPT is set */ \
        if (!(ram[queue->QueueAvail] & 1)) \
            vnet->InterruptStatus |= VIRTIO_INT__USED_RING; \
    }

__VNET_GENERATE_QUEUE_HANDLER(rx, read, __VNET_QUEUE_RX, true)
__VNET_GENERATE_QUEUE_HANDLER(tx, write, __VNET_QUEUE_TX, false)

void virtionet_notify_queue(virtionet_state_t* vnet, uint32_t queueIdx) {
    switch (queueIdx) {
        case __VNET_QUEUE_RX: return __virtionet_try_rx(vnet);
        case __VNET_QUEUE_TX: return __virtionet_try_tx(vnet);
    }
}

#define __virtionet_body(R, W) \
    switch (addr) { \
        R(case 0: /* MagicValue (R) */ \
            *value = __VIRTIO_MAGIC; return true;) \
        R(case 1: /* Version (R) */ \
            *value = __VIRTIO_ID_NET; return true;) \
        R(case 2: /* DeviceID (R) */ \
            *value = 1; return true;) \
        R(case 3: /* VendorID (R) */ \
            *value = VIRTIO_VENDOR_ID; return true;) \
        \
        R(case 4: /* DeviceFeatures (R) */ \
            *value = \
                vnet->DeviceFeaturesSel == 0 ? __VNET_FEATURES_0 : \
                vnet->DeviceFeaturesSel == 1 ? __VNET_FEATURES_1 : \
                0; return true;) \
        W(case 5: /* DeviceFeaturesSel (W) */ \
            vnet->DeviceFeaturesSel = value; return true;) \
        W(case 8: /* DriverFeatures (W) */ \
            vnet->DriverFeaturesSel == 0 ? (vnet->DriverFeatures = value) : 0; return true;) \
        W(case 9: /* DriverFeaturesSel (W) */ \
            vnet->DriverFeaturesSel = value; return true;) \
        \
        W(case 12: /* QueueSel (W) */ \
            if (value < (sizeof(vnet->queues) / sizeof(*(vnet->queues)))) \
                vnet->QueueSel = value; \
            else \
                virtionet_set_fail(vnet); \
            return true;) \
        R(case 13: /* QueueNumMax (R) */ \
            *value = __VNET_QUEUE_NUM_MAX; return true;) \
        W(case 14: /* QueueNum (W) */ \
            if (value > 0 && value <= __VNET_QUEUE_NUM_MAX) \
                __VNET_QUEUE.QueueNum = value; \
            else \
                virtionet_set_fail(vnet); \
            return true;) \
        case 17: /* QueueReady (RW) */ \
            R(*value = __VNET_QUEUE.ready ? 1 : 0;) \
            W(__VNET_QUEUE.ready = value & 1;) \
            W( \
                if (value & 1) \
                    __VNET_QUEUE.last_avail = vnet->ram[__VNET_QUEUE.QueueAvail] >> 16; \
            ) \
            return true; \
        W(case 32: /* QueueDescLow (W) */ \
            __VNET_QUEUE.QueueDesc = __VNET_PREPROCESS_ADDR(value); return true;) \
        W(case 33: /* QueueDescHigh (W) */ \
            if (value) virtionet_set_fail(vnet); return true;) \
        W(case 36: /* QueueAvailLow (W) */ \
            __VNET_QUEUE.QueueAvail = __VNET_PREPROCESS_ADDR(value); return true;) \
        W(case 37: /* QueueAvailHigh (W) */ \
            if (value) virtionet_set_fail(vnet); return true;) \
        W(case 40: /* QueueUsedLow (W) */ \
            __VNET_QUEUE.QueueUsed = __VNET_PREPROCESS_ADDR(value); return true;) \
        W(case 41: /* QueueUsedHigh (W) */ \
            if (value) virtionet_set_fail(vnet); return true;) \
        \
        W(case 20: /* QueueNotify (W) */ \
            if (value < (sizeof(vnet->queues) / sizeof(*(vnet->queues)))) \
                virtionet_notify_queue(vnet, value); \
            else \
                virtionet_set_fail(vnet); \
            return true;) \
        R(case 24: /* InterruptStatus (R) */ \
            *value = vnet->InterruptStatus; return true;) \
        W(case 25: /* InterruptACK (W) */ \
            vnet->InterruptStatus &= ~value; return true;) \
        case 28: /* Status (RW) */ \
            R(*value = vnet->Status;) \
            W(virtionet_update_status(vnet, value);) \
            W(__virtionet_try_rx(vnet);) \
            W(__virtionet_try_tx(vnet);) \
            return true; \
        \
        R(case 63: /* ConfigGeneration (R) */ \
            *value = 0; return true;) \
        /* FIXME: we should also expose MAC address (even if with invalid value) under 8bit accesses */ \
        default: return false; \
    }

REG_FUNCTIONS(bool virtionet_reg, (virtionet_state_t *vnet, uint32_t addr), uint32_t, __virtionet_body)

// we still need a wrapper for memory accesses
#define __virtionet_wrap_body(R, W) \
    switch (width) { \
        case R(RISCV_MEM_LW) W(RISCV_MEM_SW): \
            if (!REG_FUNC(R, W, virtionet_reg)(vnet, addr >> 2, value)) \
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

REG_FUNCTIONS(void virtionet_wrap_mem, (core_t *core, virtionet_state_t *vnet, uint32_t addr, uint8_t width), uint32_t, __virtionet_wrap_body)


// VIRTIO-GPU

#if USE_VIRGLRENDERER

// the protocols we offer
static const uint32_t __VGPU_CAPSETS [] = {
   VIRTIO_GPU_CAPSET_VIRGL,
   VIRTIO_GPU_CAPSET_VIRGL2,
   // VIRTIO_GPU_CAPSET_VENUS,  // even though virglrenderer implements this, we're going to start slow
};

#define __VGPU_NUM_SCANOUTS 1
#define __VGPU_NUM_CAPSETS (sizeof(__VGPU_CAPSETS) / sizeof(*__VGPU_CAPSETS))

typedef struct {
    uint32_t QueueNum;
    uint32_t QueueDesc;
    uint32_t QueueAvail;
    uint32_t QueueUsed;
    uint16_t last_avail;
    bool ready;
    bool fd_ready;
} virtiogpu_queue_t;

typedef struct virtiogpu_request_list_t {
    uint32_t fence_id;
    uint32_t buffer_idx;
    uint32_t len;
    struct virtiogpu_request_list_t *prev;
    struct virtiogpu_request_list_t *next;
} virtiogpu_request_list_t;

typedef struct {
    // feature negotiation
    uint32_t DeviceFeaturesSel;
    uint32_t DriverFeatures;
    uint32_t DriverFeaturesSel;
    // queue config
    uint32_t QueueSel;
    virtiogpu_queue_t queues [2];
    // status
    uint32_t Status;
    uint32_t InterruptStatus;
    // device specific config
    uint32_t pending_events;
    uint32_t next_fence_id;
    virtiogpu_request_list_t *fenced_cmds_head;
    virtiogpu_request_list_t *fenced_cmds_tail;
    // supplied by environment
    uint32_t* ram;
} virtiogpu_state_t;

#define __VGPU_FEATURES_0 ((1 << VIRTIO_GPU_F_VIRGL) | (1 << VIRTIO_GPU_F_CONTEXT_INIT) | (1 << VIRTIO_GPU_F_RESOURCE_UUID))
#define __VGPU_FEATURES_1 1 // VIRTIO_F_VERSION_1
#define __VGPU_QUEUE_NUM_MAX 1024
#define __VGPU_QUEUE (vgpu->queues[vgpu->QueueSel])

#define __VGPU_PREPROCESS_ADDR(addr) ((addr) < RAM_SIZE && !((addr) & 0b11) ? ((addr) >> 2) : (virtiogpu_set_fail(vgpu), 0))

void virtiogpu_set_fail(virtiogpu_state_t* vgpu) {
    fprintf(stderr, "[VGPU] device fail\n");
    vgpu->Status |= VIRTIO_STATUS__DEVICE_NEEDS_RESET;
    if (vgpu->Status & VIRTIO_STATUS__DRIVER_OK)
        vgpu->InterruptStatus |= VIRTIO_INT__CONF_CHANGE;
}

void virtiogpu_update_status(virtiogpu_state_t* vgpu, uint32_t status) {
    vgpu->Status |= status;
    if (!status) { // reset
        while (vgpu->fenced_cmds_head) {
            virtiogpu_request_list_t *next = vgpu->fenced_cmds_head->next;
            free(vgpu->fenced_cmds_head);
            vgpu->fenced_cmds_head = next;
        }
        uint32_t* ram = vgpu->ram;
        memset(vgpu, 0, sizeof(*vgpu));
        vgpu->ram = ram;
        virgl_renderer_reset();
    }
    fprintf(stderr, "[VGPU] status: %s\n", virtio_status_to_string(vgpu->Status));
}

// requires existing 'desc_idx' to use as iteration variable, and input 'buffer_idx'.
#define __VGPU_ITERATE_BUFFER(checked, body) \
    desc_idx = buffer_idx; \
    while (1) { \
        if (checked && desc_idx >= queue->QueueNum) \
            return virtiogpu_set_fail(vgpu); \
        uint32_t* desc = &ram[queue->QueueDesc + desc_idx * 4]; \
        uint16_t desc_flags = desc[3]; \
        body \
        if (!(desc_flags & VIRTQ_DESC_F_NEXT)) break; \
        desc_idx = desc[3] >> 16; \
    }

#define __vgpu_assert_cond(COND, MESSAGE) \
    if (!(COND)) { \
        fprintf(stderr, "[VGPU] " MESSAGE ", failing\n"); \
        return -1; \
    }

#define __vgpu_safe_cast(VAR, TYPE) \
    TYPE *__tmp_ ## VAR = (TYPE *)VAR; \
    TYPE *VAR = __tmp_ ## VAR; \
    __vgpu_assert_cond(VAR ## _len >= sizeof(TYPE), #VAR " cannot hold " #TYPE);

#define __vgpu_check_ret(RESP, VALUE) { \
    int __err = (VALUE); \
    if (!__err) { \
        (RESP).type = VIRTIO_GPU_RESP_OK_NODATA; \
    } else if (__err == EINVAL) { \
        (RESP).type = VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER; \
        return sizeof(RESP); \
    } else if (__err == ENOMEM) { \
        (RESP).type = VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY; \
        return sizeof(RESP); \
    } else if (__err) { \
        fprintf(stderr, "[VGPU] unknown code %s, failing\n", strerror(__err)); \
        return -1; \
    } \
}

int virtiogpu_is_supported_capset(uint32_t capset) {
    for (size_t i = 0; i < __VGPU_NUM_CAPSETS; i++)
        if (capset == __VGPU_CAPSETS[i])
            return true;
    return false;
}

int virtiogpu_process_control_cmd(virtiogpu_state_t* vgpu, const struct virtio_gpu_ctrl_hdr *cmd, uint32_t cmd_len, const void *data, uint32_t data_len, struct virtio_gpu_ctrl_hdr *resp, uint32_t resp_len) {
    if (cmd->type == VIRTIO_GPU_CMD_GET_CAPSET_INFO) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_get_capset_info);
        __vgpu_assert_cond(cmd->capset_index < __VGPU_NUM_CAPSETS, "invalid capset index");
        uint32_t capset = __VGPU_CAPSETS[cmd->capset_index];
        __vgpu_safe_cast(resp, struct virtio_gpu_resp_capset_info);
        resp->hdr.type = VIRTIO_GPU_RESP_OK_CAPSET_INFO;

        uint32_t max_ver, max_size;
        virgl_renderer_get_cap_set(capset, &max_ver, &max_size);
        resp->capset_id = capset;
        resp->capset_max_version = max_ver;
        resp->capset_max_size = max_size;
        resp->padding = 0;
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_GET_CAPSET) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_get_capset);
        uint32_t capset = cmd->capset_id;
        __vgpu_assert_cond(virtiogpu_is_supported_capset(capset), "invalid capset");
        __vgpu_safe_cast(resp, struct virtio_gpu_resp_capset);
        resp->hdr.type = VIRTIO_GPU_RESP_OK_CAPSET;

        uint32_t max_ver, max_size;
        virgl_renderer_get_cap_set(capset, &max_ver, &max_size);
        void* caps = ((uint8_t*)resp) + sizeof(*resp);
        uint32_t caps_len = resp_len - sizeof(*resp);
        __vgpu_assert_cond(caps_len >= max_size, "out payload < max_size");
        __vgpu_assert_cond(cmd->capset_version <= max_ver, "unsupported version");
        memset(caps, 0, caps_len);
        virgl_renderer_fill_caps(capset, cmd->capset_version, caps);
        return resp_len;
    }

    if (cmd->type == VIRTIO_GPU_CMD_GET_DISPLAY_INFO) {
        __vgpu_safe_cast(resp, struct virtio_gpu_resp_display_info);
        memset(resp, 0, sizeof(*resp));
        resp->hdr.type = VIRTIO_GPU_RESP_OK_CAPSET;
        resp->pmodes[0].enabled = 0;
        resp->pmodes[0].r.x = 0;
        resp->pmodes[0].r.y = 0;
        resp->pmodes[0].r.width = 800;
        resp->pmodes[0].r.height = 600;
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_CTX_CREATE) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_ctx_create);
        __vgpu_assert_cond(cmd->nlen < sizeof(cmd->debug_name), "debugging name does not fit");
        __vgpu_check_ret(*resp, virgl_renderer_context_create_with_flags(cmd->hdr.ctx_id, cmd->context_init, cmd->nlen, cmd->debug_name));
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_CTX_DESTROY) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_ctx_destroy);
        virgl_renderer_context_destroy(cmd->hdr.ctx_id);
        resp->type = VIRTIO_GPU_RESP_OK_NODATA;
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_SUBMIT_3D) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_cmd_submit);
        uint32_t size = cmd->size;
        __vgpu_assert_cond(cmd_len == sizeof(*cmd) && data && data_len >= size, "unexpected layout...");
        __vgpu_check_ret(*resp, virgl_renderer_submit_cmd(data, cmd->hdr.ctx_id, size / sizeof(uint32_t)));
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_RESOURCE_CREATE_3D) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_resource_create_3d);
        __vgpu_check_ret(*resp, virgl_renderer_resource_create((struct virgl_renderer_resource_create_args *)&cmd->resource_id, NULL, 0));
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_resource_attach_backing);
        // check for data, allocate space for iov list
        const struct virtio_gpu_mem_entry *iovs_src = (struct virtio_gpu_mem_entry *)data;
        __vgpu_assert_cond(data_len >= cmd->nr_entries * sizeof(*iovs_src), "message too short");
        struct iovec *iovs = (struct iovec *) malloc(cmd->nr_entries * sizeof(*iovs));
        if (!iovs) {
            resp->type = VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY;
            return sizeof(*resp);
        }
        // convert iovs
        for (size_t i = 0; i < cmd->nr_entries; i++) {
            iovs[i].iov_base = &vgpu->ram[__VGPU_PREPROCESS_ADDR(iovs_src[i].addr)];
            iovs[i].iov_len = iovs_src[i].length;
        }
        // attach them
        int ret = virgl_renderer_resource_attach_iov(cmd->resource_id, iovs, cmd->nr_entries);
        if (ret) free(iovs);
        __vgpu_check_ret(*resp, ret);
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_resource_detach_backing);
        struct iovec *iovs = NULL;
        virgl_renderer_resource_detach_iov(cmd->resource_id, &iovs, NULL);
        free(iovs);
        resp->type = VIRTIO_GPU_RESP_OK_NODATA;
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_RESOURCE_UNREF) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_resource_unref);
        struct iovec *iovs = NULL;
        virgl_renderer_resource_detach_iov(cmd->resource_id, &iovs, NULL);
        free(iovs);
        virgl_renderer_resource_unref(cmd->resource_id);
        resp->type = VIRTIO_GPU_RESP_OK_NODATA;
        return sizeof(*resp);
    }

    if (cmd->type == VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE || cmd->type == VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE) {
        __vgpu_safe_cast(cmd, const struct virtio_gpu_ctx_resource);
        (cmd->hdr.type == VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE ? virgl_renderer_ctx_attach_resource : virgl_renderer_ctx_detach_resource)(cmd->hdr.ctx_id, cmd->resource_id);
        resp->type = VIRTIO_GPU_RESP_OK_NODATA;
        return sizeof(*resp);
    }

    return -1;
}

/*int virtiogpu_process_cursor_cmd(virtiogpu_state_t* vgpu, const struct virtio_gpu_ctrl_hdr *cmd, uint32_t cmd_len, const void *data, uint32_t data_len, struct virtio_gpu_ctrl_hdr *resp, uint32_t resp_len) {
    // FIXME

    return -1;
}*/

// returns written length, or -1 to set fail status, or -2 to just skip returning buffer
int virtiogpu_process_buffer(virtiogpu_state_t* vgpu, uint32_t queue_idx, uint16_t buffer_idx) {
    uint32_t* ram = vgpu->ram;
    virtiogpu_queue_t* queue = &vgpu->queues[queue_idx];
    uint32_t desc_idx = buffer_idx;

    // assume command and response have exactly 1 buffer descriptor each,
    // SG not supported by now (not sure if we are supposed to)
    __vgpu_assert_cond(desc_idx < queue->QueueNum, "descriptor 1 bad addr");
    const struct virtq_desc *cmd_desc = (struct virtq_desc*) &ram[queue->QueueDesc + desc_idx * 4];
    __vgpu_assert_cond(cmd_desc->flags & VIRTQ_DESC_F_NEXT, "less than 2 descriptors");;
    desc_idx = cmd_desc->next;
    __vgpu_assert_cond(desc_idx < queue->QueueNum, "descriptor 2 bad addr");
    const struct virtq_desc *resp_desc = (struct virtq_desc*) &ram[queue->QueueDesc + desc_idx * 4];
    const struct virtq_desc *data_desc = NULL;
    if (resp_desc->flags & VIRTQ_DESC_F_NEXT) {
        data_desc = resp_desc;
        desc_idx = data_desc->next;
        resp_desc = (struct virtq_desc*) &ram[queue->QueueDesc + desc_idx * 4];
        __vgpu_assert_cond(!(data_desc->flags & VIRTQ_DESC_F_WRITE), "middle descriptor is not READ");
    }
    __vgpu_assert_cond(!(resp_desc->flags & VIRTQ_DESC_F_NEXT), "more than 2 descriptors");

    __vgpu_assert_cond(!(cmd_desc->flags & VIRTQ_DESC_F_WRITE), "descriptor 1 is not READ");
    __vgpu_assert_cond(resp_desc->flags & VIRTQ_DESC_F_WRITE, "descriptor 2 is not WRITE");
    __vgpu_assert_cond(cmd_desc->len >= sizeof(struct virtio_gpu_ctrl_hdr), "descriptor 1 too short");
    __vgpu_assert_cond(resp_desc->len >= sizeof(struct virtio_gpu_ctrl_hdr), "descriptor 2 too short");

    const struct virtio_gpu_ctrl_hdr *cmd = (struct virtio_gpu_ctrl_hdr *) &vgpu->ram[__VGPU_PREPROCESS_ADDR(cmd_desc->addr)];
    struct virtio_gpu_ctrl_hdr *resp = (struct virtio_gpu_ctrl_hdr *) &vgpu->ram[__VGPU_PREPROCESS_ADDR(resp_desc->addr)];
    const void* data = data_desc ? &vgpu->ram[__VGPU_PREPROCESS_ADDR(data_desc->addr)] : NULL;
    uint32_t data_len = data_desc ? data_desc->len : 0;

    fprintf(stderr, "[VGPU] [%u] %s [flags: %u, fence: %lu, ctx: %u] payload: %lu\n", queue_idx, virtio_gpu_ctrl_type_to_string(cmd->type), cmd->flags, cmd->fence_id, cmd->ctx_id, cmd_desc->len - sizeof(struct virtio_gpu_ctrl_hdr));

    memset(resp, 0, sizeof(*resp));
    resp->ctx_id = cmd->ctx_id;

    int ret = -1;
    if (queue_idx == __VGPU_QUEUE_CONTROL)
        ret = virtiogpu_process_control_cmd(vgpu, cmd, cmd_desc->len, data, data_len, resp, resp_desc->len);
    // if (queue_idx == __VGPU_QUEUE_CURSOR)
    //     ret = virtiogpu_process_cursor_cmd(vgpu, cmd, cmd_desc->len, data, data_len, resp, resp_desc->len);

    assert(ret < 0 || ret >= (int)(sizeof(struct virtio_gpu_ctrl_hdr)));
    if (ret >= 0)
        fprintf(stderr, "[VGPU]   -> %s [flags: %u, fence: %lu, ctx: %u] payload: %lu\n", virtio_gpu_ctrl_type_to_string(resp->type), resp->flags, resp->fence_id, resp->ctx_id, ret - sizeof(struct virtio_gpu_ctrl_hdr));
    fflush(stderr);

    if (ret >= 0 && (cmd->flags & VIRTIO_GPU_FLAG_FENCE) && resp->type == VIRTIO_GPU_RESP_OK_NODATA) {
        __vgpu_assert_cond(queue_idx == __VGPU_QUEUE_CONTROL, "fences not implemented for cursor ops");
        // copy fence to response
        resp->flags |= VIRTIO_GPU_FLAG_FENCE;
        resp->fence_id = cmd->fence_id;
        // create fence
        uint32_t fence_id = vgpu->next_fence_id++;
        __vgpu_check_ret(*resp, virgl_renderer_create_fence((int)fence_id, 0));
        // allocate & initialize queue item
        virtiogpu_request_list_t *item = malloc(sizeof(*item));
        if (!item) {
            resp->type = VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY;
            return sizeof(*resp);
        }
        item->buffer_idx = buffer_idx;
        item->len = ret;
        item->fence_id = fence_id;
        // add it to queue
        item->prev = NULL;
        item->next = vgpu->fenced_cmds_head;
        *(vgpu->fenced_cmds_head ? &vgpu->fenced_cmds_head->prev : &vgpu->fenced_cmds_tail) = item;
        vgpu->fenced_cmds_head = item;
        // return -2 to tell queue handler not to return buffer yet
        return -2;
    }
    return ret;
}

#define __VGPU_GENERATE_QUEUE_HANDLER(NAME_SUFFIX, QUEUE_IDX) \
    void __virtiogpu_try_##NAME_SUFFIX(virtiogpu_state_t* vgpu) { \
        uint32_t* ram = vgpu->ram; \
        virtiogpu_queue_t* queue = &vgpu->queues[QUEUE_IDX]; \
        if ((vgpu->Status & VIRTIO_STATUS__DEVICE_NEEDS_RESET)) \
            return; \
        if (!( (vgpu->Status & VIRTIO_STATUS__DRIVER_OK) && queue->ready )) \
            return virtiogpu_set_fail(vgpu); \
        \
        /* check for new buffers */ \
        uint16_t new_avail = ram[queue->QueueAvail] >> 16; \
        if (new_avail - queue->last_avail > (uint16_t)queue->QueueNum) \
            return (fprintf(stderr, "size check fail\n"), virtiogpu_set_fail(vgpu)); \
        if (queue->last_avail == new_avail) \
            return; \
        \
        /* process them */ \
        uint16_t new_used = ram[queue->QueueUsed] >> 16; \
        while (queue->last_avail != new_avail) { \
            uint16_t queue_idx = queue->last_avail % queue->QueueNum; \
            uint16_t buffer_idx = ram[queue->QueueAvail + 1 + queue_idx / 2] >> (16 * (queue_idx % 2)); \
            int resp_len = virtiogpu_process_buffer(vgpu, QUEUE_IDX, buffer_idx); \
            if (resp_len == -1) return virtiogpu_set_fail(vgpu); \
            /* consume from available queue, write to used queue */ \
            queue->last_avail++; \
            if (resp_len < 0) continue; \
            ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2] = buffer_idx; \
            ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2 + 1] = resp_len; \
            new_used++; \
        } \
        vgpu->ram[queue->QueueUsed] &= MASK(16); \
        vgpu->ram[queue->QueueUsed] |= ((uint32_t)new_used) << 16; \
        \
        /* send interrupt, unless VIRTQ_AVAIL_F_NO_INTERRUPT is set */ \
        if (!(ram[queue->QueueAvail] & 1)) \
            vgpu->InterruptStatus |= VIRTIO_INT__USED_RING; \
    }

__VGPU_GENERATE_QUEUE_HANDLER(control, __VGPU_QUEUE_CONTROL)
__VGPU_GENERATE_QUEUE_HANDLER(cursor, __VGPU_QUEUE_CURSOR)

void virtiogpu_notify_queue(virtiogpu_state_t* vgpu, uint32_t queueIdx) {
    switch (queueIdx) {
        case __VGPU_QUEUE_CONTROL: return __virtiogpu_try_control(vgpu);
        case __VGPU_QUEUE_CURSOR: return __virtiogpu_try_cursor(vgpu);
    }
}

void virtiogpu_cb_write_fence(void *cookie, uint32_t fence) {
    virtiogpu_state_t* vgpu = (virtiogpu_state_t*) cookie;
    if (!(vgpu->Status & VIRTIO_STATUS__DRIVER_OK) || (vgpu->Status & VIRTIO_STATUS__DEVICE_NEEDS_RESET))
        return;

    uint32_t QUEUE_IDX = __VGPU_QUEUE_CONTROL;
    uint32_t* ram = vgpu->ram;
    virtiogpu_queue_t* queue = &vgpu->queues[QUEUE_IDX];
    uint16_t new_used = ram[queue->QueueUsed] >> 16;

    while (true) {
        // check we have an item to dequeue and it's not past our target
        virtiogpu_request_list_t *item = vgpu->fenced_cmds_tail;
        int32_t diff;
        assert(item && (diff = (int32_t)(fence - item->fence_id)) >= 0);
        // return buffer
        ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2] = item->buffer_idx;
        ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2 + 1] = item->len;
        new_used++;
        // dequeue it
        *(item->prev ? &item->prev->next : &vgpu->fenced_cmds_head) = NULL;
        vgpu->fenced_cmds_tail = item->prev;
        free(item);

        if (diff == 0) break;
    }

    // update ringbuffer
    vgpu->ram[queue->QueueUsed] &= MASK(16);
    vgpu->ram[queue->QueueUsed] |= ((uint32_t)new_used) << 16;
    /* send interrupt, unless VIRTQ_AVAIL_F_NO_INTERRUPT is set */
    if (!(ram[queue->QueueAvail] & 1))
        vgpu->InterruptStatus |= VIRTIO_INT__USED_RING;
}

#define __virtiogpu_body(R, W) \
    switch (addr) { \
        R(case 0: /* MagicValue (R) */ \
            *value = __VIRTIO_MAGIC; return true;) \
        R(case 1: /* Version (R) */ \
            *value = 2; return true;) \
        R(case 2: /* DeviceID (R) */ \
            *value = __VIRTIO_ID_GPU; return true;) \
        R(case 3: /* VendorID (R) */ \
            *value = VIRTIO_VENDOR_ID; return true;) \
        \
        R(case 4: /* DeviceFeatures (R) */ \
            *value = \
                vgpu->DeviceFeaturesSel == 0 ? __VGPU_FEATURES_0 : \
                vgpu->DeviceFeaturesSel == 1 ? __VGPU_FEATURES_1 : \
                0; return true;) \
        W(case 5: /* DeviceFeaturesSel (W) */ \
            vgpu->DeviceFeaturesSel = value; return true;) \
        W(case 8: /* DriverFeatures (W) */ \
            vgpu->DriverFeaturesSel == 0 ? (vgpu->DriverFeatures = value) : 0; return true;) \
        W(case 9: /* DriverFeaturesSel (W) */ \
            vgpu->DriverFeaturesSel = value; return true;) \
        \
        W(case 12: /* QueueSel (W) */ \
            if (value < (sizeof(vgpu->queues) / sizeof(*(vgpu->queues)))) \
                vgpu->QueueSel = value; \
            else \
                virtiogpu_set_fail(vgpu); \
            return true;) \
        R(case 13: /* QueueNumMax (R) */ \
            *value = __VGPU_QUEUE_NUM_MAX; return true;) \
        W(case 14: /* QueueNum (W) */ \
            if (value > 0 && value <= __VGPU_QUEUE_NUM_MAX) \
                __VGPU_QUEUE.QueueNum = value; \
            else \
                virtiogpu_set_fail(vgpu); \
            return true;) \
        case 17: /* QueueReady (RW) */ \
            R(*value = __VGPU_QUEUE.ready ? 1 : 0;) \
            W(__VGPU_QUEUE.ready = value & 1;) \
            W( \
                if (value & 1) \
                    __VGPU_QUEUE.last_avail = vgpu->ram[__VGPU_QUEUE.QueueAvail] >> 16; \
            ) \
            return true; \
        W(case 32: /* QueueDescLow (W) */ \
            __VGPU_QUEUE.QueueDesc = __VGPU_PREPROCESS_ADDR(value); return true;) \
        W(case 33: /* QueueDescHigh (W) */ \
            if (value) virtiogpu_set_fail(vgpu); return true;) \
        W(case 36: /* QueueAvailLow (W) */ \
            __VGPU_QUEUE.QueueAvail = __VGPU_PREPROCESS_ADDR(value); return true;) \
        W(case 37: /* QueueAvailHigh (W) */ \
            if (value) virtiogpu_set_fail(vgpu); return true;) \
        W(case 40: /* QueueUsedLow (W) */ \
            __VGPU_QUEUE.QueueUsed = __VGPU_PREPROCESS_ADDR(value); return true;) \
        W(case 41: /* QueueUsedHigh (W) */ \
            if (value) virtiogpu_set_fail(vgpu); return true;) \
        \
        W(case 20: /* QueueNotify (W) */ \
            if (value < (sizeof(vgpu->queues) / sizeof(*(vgpu->queues)))) \
                virtiogpu_notify_queue(vgpu, value); \
            else \
                virtiogpu_set_fail(vgpu); \
            return true;) \
        R(case 24: /* InterruptStatus (R) */ \
            *value = vgpu->InterruptStatus; return true;) \
        W(case 25: /* InterruptACK (W) */ \
            vgpu->InterruptStatus &= ~value; return true;) \
        case 28: /* Status (RW) */ \
            R(*value = vgpu->Status;) \
            W(virtiogpu_update_status(vgpu, value);) \
            return true; \
        \
        W(case 43: /* SHMSel (W) */ \
            return true;) \
        R(case 44: case 45: /* SHMLenLow / SHMLenHigh (R) */ \
            *value = -1; /* signal SHM not implemented */ return true; ) \
        R(case 46: case 47: /* SHMBaseLow / SHMBaseHigh (R) */ \
            *value = -1; /* signal SHM not implemented */ return true; ) \
        \
        /* case 48: */ /* QueueReset (RW) */ \
        \
        R(case 63: /* ConfigGeneration (R) */ \
            *value = 0; return true;) \
        \
        /* device-specific configuration space */ \
        R(case 64: /* events_read */ \
            *value = vgpu->pending_events; return true;) \
        W(case 65: /* events_clear */ \
            vgpu->pending_events &= ~value; return true;) \
        R(case 66: /* num_scanouts */ \
            *value = __VGPU_NUM_SCANOUTS; return true;) \
        R(case 67: /* num_capsets */ \
            *value = __VGPU_NUM_CAPSETS; return true;) \
        default: return false; \
    }

REG_FUNCTIONS(bool virtiogpu_reg, (virtiogpu_state_t *vgpu, uint32_t addr), uint32_t, __virtiogpu_body)

// we still need a wrapper for memory accesses
#define __virtiogpu_wrap_body(R, W) \
    switch (width) { \
        case R(RISCV_MEM_LW) W(RISCV_MEM_SW): \
            if (!REG_FUNC(R, W, virtiogpu_reg)(vgpu, addr >> 2, value)) \
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

REG_FUNCTIONS(void virtiogpu_wrap_mem, (core_t *core, virtiogpu_state_t *vgpu, uint32_t addr, uint8_t width), uint32_t, __virtiogpu_wrap_body)

#endif


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

#define IRQ_UART 1
#define IRQ_UART_BIT (1 << IRQ_UART)
#define IRQ_VNET 2
#define IRQ_VNET_BIT (1 << IRQ_VNET)
#define IRQ_VGPU 3
#define IRQ_VGPU_BIT (1 << IRQ_VGPU)

// keep queue capacity over number of items here
typedef enum {
    IO_EVENT_STOP = 0,
    IO_EVENT_UART_RX,
    IO_EVENT_VNET_RX,
    IO_EVENT_VNET_TX,
    IO_EVENT_VGPU,
} io_event_t;

typedef struct {
    bool stopped;
    uint32_t *ram;
    plic_state_t plic;
    u8250_state_t uart;
    virtionet_state_t vnet;
#ifdef USE_VIRGLRENDERER
    virtiogpu_state_t gpu;
#endif
    int virglrenderer_fd;
    uint64_t timer;

    // these queues are used for communication between the I/O and main
    // threads. when I/O gets a poll condition on one of the FDs, it writes
    // the corresponding event ID into the io2main queue and removes it from
    // subsequent polls. when main has handled that poll condition and wants
    // polls to include it again, it writes the event ID back into the
    // main2io queue. the special event ID 0 is a stop condition, it is sent
    // by main telling I/O thread loop to exit, or by I/O thread indicating
    // it has found an irrecoverable error and exited.
    spsc_queue_t io2main;
    spsc_queue_t main2io;
} emu_state_t;

// we define fetch separately because it's much simpler (width is fixed,
// alignment already checked, only main RAM is executable)
static void mem_fetch(core_t* core, uint32_t page_num, uint32_t** page_addr) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    if (unlikely(page_num >= RAM_SIZE / RISCV_PAGE_SIZE)) {
        // FIXME: check for other regions
        core_set_exception(core, RISCV_EXC_FETCH_FAULT, core->exc_val);
        return;
    }
    *page_addr = &data->ram[page_num << (RISCV_PAGE_SHIFT - 2)];
}

// similarly only main memory pages can be used as page_tables
static uint32_t* mem_page_table(const core_t* core, uint32_t ppn) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    if (ppn < (RAM_SIZE / RISCV_PAGE_SIZE))
        return &data->ram[ppn << (RISCV_PAGE_SHIFT - 2)];
    return NULL;
}

void emulator_update_uart_interrupts(core_t* core) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    u8250_update_interrupts(&data->uart);
    data->uart.pending_ints ? (data->plic.active |= IRQ_UART_BIT) : (data->plic.active &= ~IRQ_UART_BIT);
    plic_update_interrupts(core, &data->plic);
}

void emulator_update_vnet_interrupts(core_t* core) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    data->vnet.InterruptStatus ? (data->plic.active |= IRQ_VNET_BIT) : (data->plic.active &= ~IRQ_VNET_BIT);
    plic_update_interrupts(core, &data->plic);
}

#ifdef USE_VIRGLRENDERER
void emulator_update_vgpu_interrupts(core_t* core) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    data->gpu.InterruptStatus ? (data->plic.active |= IRQ_VGPU_BIT) : (data->plic.active &= ~IRQ_VGPU_BIT);
    plic_update_interrupts(core, &data->plic);
}

#define __VGPU_MEM_BODY(R, W) \
            case 0x42: /* VIRTIO-GPU */ \
                REG_FUNC(R, W, virtiogpu_wrap_mem)(core, &data->gpu, addr & 0xFFFFF, width, value); \
                virgl_renderer_poll(); \
                emulator_update_vgpu_interrupts(core); \
                return;
#else
#define __VGPU_MEM_BODY(R, W)
#endif

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
                emulator_update_uart_interrupts(core); \
                return; \
            case 0x41: /* VIRTIO-NET */ \
                REG_FUNC(R, W, virtionet_wrap_mem)(core, &data->vnet, addr & 0xFFFFF, width, value); \
                emulator_update_vnet_interrupts(core); \
                return; \
            __VGPU_MEM_BODY(R, W) \
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
            data->timer = (((uint64_t)core->x_regs[RISCV_R_A1]) << 32) | (uint64_t)(core->x_regs[RISCV_R_A0]);
            return (sbiret_t){ SBI_SUCCESS, 0 };
    }
    return (sbiret_t){ SBI_ERR_NOT_SUPPORTED, 0 };
}

sbiret_t handle_sbi_ecall_RST(core_t* core, int32_t fid) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    switch (fid) {
        case SBI_RST__SYSTEM_RESET:
            fprintf(stderr, "system reset: type=%u, reason=%u\n", core->x_regs[RISCV_R_A0], core->x_regs[RISCV_R_A1]);
            data->stopped = true;
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
            bool available = eid == SBI_EID_BASE || eid == SBI_EID_TIMER || eid == SBI_EID_RST;
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
        __SBI_CASE(RST)
        default:
            ret = (sbiret_t){ SBI_ERR_NOT_SUPPORTED, 0 };
    }
    core->x_regs[RISCV_R_A0] = (uint32_t)ret.error;
    core->x_regs[RISCV_R_A1] = (uint32_t)ret.value;
    // clear error to allow execution to continue
    core->error = ERR_NONE;
}


// I/O thread

typedef struct {
    struct pollfd *pfd;
    bool stop_requested;
} io_thread_handler_data_t;

static void io_thread_handler(uint8_t event, void *__arg);

#define __IO_THREAD_HANDLE_EVENT(n, poll_event, event_id) \
    if (pfd[n].revents & poll_event) { \
        spsc_queue_write(&data->io2main, event_id); \
        spsc_queue_commit(&data->io2main); \
        pfd[n].events &= ~poll_event; \
    }

#define __IO_THREAD_EVENTS(macro) \
    macro(1, POLLIN, IO_EVENT_UART_RX) \
    macro(2, POLLIN, IO_EVENT_VNET_RX) \
    macro(2, POLLOUT, IO_EVENT_VNET_TX) \
    macro(3, POLLIN, IO_EVENT_VGPU) \
    //

void *io_thread(void *__arg) {
    emu_state_t *data = (emu_state_t *) __arg;
    struct pollfd pfd [] = {
        { data->main2io.eventfd, POLLIN, 0 },
        { data->uart.in_fd, POLLIN, 0 },
        { data->vnet.tap_fd, POLLIN | POLLOUT, 0 },
        { data->virglrenderer_fd, POLLIN, 0 },
    };

    while (1) {
        __checkerrno(poll(pfd, sizeof(pfd) / sizeof(*pfd), -1) < 0, "I/O poll");

        if (pfd[0].revents & POLLIN) {
            read_eventfd(pfd[0].fd);
            io_thread_handler_data_t hdata = { pfd, false };
            spsc_queue_read_all(&data->main2io, io_thread_handler, &hdata);
            if (hdata.stop_requested)
                break;
        }

        __IO_THREAD_EVENTS(__IO_THREAD_HANDLE_EVENT)
    }
    return NULL;
}

#define __IO_THREAD_HANDLE_EVENT_BACK(n, poll_event, event_id) \
    case event_id: \
        hdata->pfd[n].events |= poll_event; \
        break;

void io_thread_handler(uint8_t event, void *__arg) {
    io_thread_handler_data_t *hdata = (io_thread_handler_data_t *) __arg;
    switch (event) {
        case IO_EVENT_STOP:
            hdata->stop_requested = true;
            break;
        __IO_THREAD_EVENTS(__IO_THREAD_HANDLE_EVENT_BACK)
        default:
            abort();
    }
}


// main emulation

#define __checkerrno_file(expr, callname) \
    __checkerrno(expr, callname " \"%s\"", name)

void read_file_into_ram(char** ram_cursor, const char* name) {
    // FIXME: map instead of reading
    int fd = open(name, O_RDONLY);
    __checkerrno_file(fd < 0, "open");

    while (true) {
        int ret = read(fd, *ram_cursor, 1024 * 1024);
        __checkerrno_file(ret < 0, "read");
        *ram_cursor += ret;
        if (ret == 0) break;
    }

    __checkerrno_file(close(fd) < 0, "close");
}

void main_io_handler(uint8_t event, void *__arg) {
    core_t *core = (core_t *) __arg;
    emu_state_t *data = (emu_state_t *) core->user_data;
    switch (event) {
        case IO_EVENT_UART_RX:
            data->uart.in_ready = true;
            emulator_update_uart_interrupts(core);
            break;
        case IO_EVENT_VNET_RX:
            data->vnet.queues[__VNET_QUEUE_RX].fd_ready = true;
            __virtionet_try_rx(&data->vnet);
            emulator_update_vnet_interrupts(core);
            break;
        case IO_EVENT_VNET_TX:
            data->vnet.queues[__VNET_QUEUE_TX].fd_ready = true;
            __virtionet_try_tx(&data->vnet);
            emulator_update_vnet_interrupts(core);
            break;
#ifdef USE_VIRGLRENDERER
        case IO_EVENT_VGPU:
            virgl_renderer_poll();
            spsc_queue_write(&data->main2io, IO_EVENT_VGPU);
            spsc_queue_commit(&data->main2io);
            if (data->gpu.InterruptStatus)
                emulator_update_vgpu_interrupts(core);
            break;
#endif
        default:
            abort();
    }
}

void uart_need_more_data(void *__arg) {
    emu_state_t *data = (emu_state_t *) __arg;
    spsc_queue_write(&data->main2io, IO_EVENT_UART_RX);
    spsc_queue_commit(&data->main2io);
}

void vnet_need_more_data(int queue_idx, void *__arg) {
    emu_state_t *data = (emu_state_t *) __arg;
    spsc_queue_write(&data->main2io,
        queue_idx == __VNET_QUEUE_RX ? IO_EVENT_VNET_RX :
        queue_idx == __VNET_QUEUE_TX ? IO_EVENT_VNET_TX :
        (abort(), 0));
    spsc_queue_commit(&data->main2io);
}

int main() {
    int ret;

    // initialize emulator
    emu_state_t data;
    memset(&data, 0, sizeof(data));
    core_t core;
    core_init(&core);
    core.user_data = &data;
    core.mem_fetch = mem_fetch;
    core.mem_load = mem_read;
    core.mem_store = mem_write;
    core.mem_page_table = mem_page_table;

    // set up RAM
    data.ram = mmap(NULL, RAM_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    __checkerrno(data.ram == MAP_FAILED, "mmap guest RAM");
    assert(!(((uintptr_t)data.ram) & 0b11));

    char* ram_cursor = (char*) data.ram;
    read_file_into_ram(&ram_cursor, "linux/arch/riscv/boot/Image");
    // load at last MB to prevent kernel / initrd from overwriting it
    uint32_t dtb_addr = RAM_SIZE - 1024 * 1024;
    ram_cursor = ((char*)data.ram) + dtb_addr;
    read_file_into_ram(&ram_cursor, "linux_dtb");

    data.timer = 0xFFFFFFFFFFFFFFFF;
    core.s_mode = true;
    core.x_regs[RISCV_R_A0] = 0; // hart ID (cpuid)
    core.x_regs[RISCV_R_A1] = dtb_addr;

    // set up peripherals

    data.uart.in_fd = 0;
    data.uart.out_fd = 1;
    data.uart.need_more_data = uart_need_more_data;
    data.uart.cookie = &data;

    data.vnet.tap_fd = open("/dev/net/tun", O_RDWR);
    __checkerrno(data.vnet.tap_fd < 0, "open TAP device");
    struct ifreq ifreq;
    memset(&ifreq, 0, sizeof(ifreq));
    // iproute2 sets PI, let's make it easier for folks who want to use persistent taps
    ifreq.ifr_flags = IFF_TAP | IFF_NO_PI;
    strncpy(ifreq.ifr_name, TAP_INTERFACE, sizeof(ifreq.ifr_name));
    __checkerrno(ioctl(data.vnet.tap_fd, TUNSETIFF, &ifreq) < 0, "TUNSETIFF");
    fprintf(stderr, "allocated TAP interface: %s\n", ifreq.ifr_name);
    assert(fcntl(data.vnet.tap_fd, F_SETFL, fcntl(data.vnet.tap_fd, F_GETFL, 0) | O_NONBLOCK) >= 0);
    data.vnet.ram = data.ram;
    data.vnet.need_more_data = vnet_need_more_data;
    data.vnet.cookie = &data;

    data.virglrenderer_fd = -1;
#ifdef USE_VIRGLRENDERER
    int virgl_flags = VIRGL_RENDERER_THREAD_SYNC | VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_SURFACELESS /* | VIRGL_RENDERER_USE_EXTERNAL_BLOB */;
    struct virgl_renderer_callbacks virgl_cbs;
    memset(&virgl_cbs, 0, sizeof(virgl_cbs));
    virgl_cbs.version = VIRGL_RENDERER_CALLBACKS_VERSION;
    virgl_cbs.write_fence = virtiogpu_cb_write_fence;
    if ((ret = virgl_renderer_init(&data.gpu, virgl_flags, &virgl_cbs))) {
        fprintf(stderr, "failed to initialize virgl renderer\n");
        return 2;
    }
    data.gpu.ram = data.ram;
    if ((data.virglrenderer_fd = virgl_renderer_get_poll_fd()) < 0) {
        fprintf(stderr, "virgl renderer thread sync not enabled\n");
        return 2;
    }
#endif

    // start I/O thread
    spsc_queue_init(&data.io2main);
    spsc_queue_init(&data.main2io);
    pthread_t io_thread_id;
    __checkerrno((ret = pthread_create(&io_thread_id, NULL, io_thread, &data)), "spawn I/O thread");

    // emulate!
    int cycle_count_fd = simple_perf_counter(PERF_COUNT_HW_CPU_CYCLES);
    int instr_count_fd = simple_perf_counter(PERF_COUNT_HW_INSTRUCTIONS);
    struct timespec start, end;

    int res = clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    ioctl(cycle_count_fd, PERF_EVENT_IOC_ENABLE, 0);
    ioctl(instr_count_fd, PERF_EVENT_IOC_ENABLE, 0);

    uint32_t peripheral_update_ctr = 0;
    while (!data.stopped) {
        //printf("pc: %#08x, sp: %#08x\n", core.pc, core.x_regs[RISCV_R_SP]);

        if (peripheral_update_ctr-- == 0) {
            peripheral_update_ctr = 64;
            spsc_queue_read_all(&data.io2main, main_io_handler, &core);
        }

        (core.instr_count > data.timer) ? (core.sip |= RISCV_INT_STI_BIT) : (core.sip &= ~RISCV_INT_STI_BIT);

        core_step(&core);
        if (likely(!core.error)) continue;

        if (core.error == ERR_EXCEPTION && core.exc_cause == RISCV_EXC_ECALL_S) {
            handle_sbi_ecall(&core);
            continue;
        }

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
    fprintf(stderr, "executed %lu instructions in %.3f ms, %lld instructions, %lld cycles\n",
        core.instr_count, elapsed / 1e6, host_instr_count, host_cycle_count);
    fprintf(stderr, "stats: %.3f c/i, %.1f i/I, %.1f c/I, %.1f MIps\n",
        ((double)host_cycle_count) / ((double)host_instr_count),
        ((double)host_instr_count) / ((double)core.instr_count),
        ((double)host_cycle_count) / ((double)core.instr_count),
        core.instr_count / (double)elapsed * 1e3);

    // stop I/O thread
    spsc_queue_write(&data.main2io, IO_EVENT_STOP);
    spsc_queue_commit(&data.main2io);
    __checkerrno((ret = pthread_join(io_thread_id, NULL)), "join I/O thread");
}
