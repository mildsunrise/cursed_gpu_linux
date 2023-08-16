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
#include <linux/if.h>
#include <linux/if_tun.h>
#include <sys/uio.h>
#include <virglrenderer.h>
#include "measure.c"

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

void u8250_check_ready(u8250_state_t* uart) {
    if (uart->in_ready) return; // no need to check
    struct pollfd pfd = { uart->in_fd, POLLIN, 0 };
    poll(&pfd, 1, 0);
    if (pfd.revents & POLLIN) uart->in_ready = true;
}

void u8250_handle_out(u8250_state_t* uart, uint8_t value) {
    if (write(uart->out_fd, &value, 1) < 1)
        fprintf(stderr, "failed to write UART output: %s\n", strerror(errno));
}

uint8_t u8250_handle_in(u8250_state_t* uart) {
    uint8_t value = 0;
    u8250_check_ready(uart);
    if (!uart->in_ready) return value;
    if (read(uart->in_fd, &value, 1) < 0)
        fprintf(stderr, "failed to read UART input: %s\n", strerror(errno));
    uart->in_ready = false;
    u8250_check_ready(uart);
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
        int tap_fd = vnet->tap_fd;
        uint32_t* ram = vnet->ram;
        memset(vnet, 0, sizeof(*vnet));
        vnet->tap_fd = tap_fd;
        vnet->ram = ram;
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
        if ((vnet->Status & VIRTIO_STATUS__DEVICE_NEEDS_RESET) || !queue->fd_ready) \
            return; \
        if (!( (vnet->Status & VIRTIO_STATUS__DRIVER_OK) && queue->ready )) \
            return virtionet_set_fail(vnet); \
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

void virtionet_refresh_queue(virtionet_state_t* vnet) {
    if (!(vnet->Status & VIRTIO_STATUS__DRIVER_OK) || (vnet->Status & VIRTIO_STATUS__DEVICE_NEEDS_RESET))
        return;
    struct pollfd pfd = { vnet->tap_fd, POLLIN | POLLOUT, 0 };
    poll(&pfd, 1, 0);
    if (pfd.revents & POLLIN)
        vnet->queues[__VNET_QUEUE_RX].fd_ready = true, __virtionet_try_rx(vnet);
    if (pfd.revents & POLLOUT)
        vnet->queues[__VNET_QUEUE_TX].fd_ready = true, __virtionet_try_tx(vnet);
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
                if (vnet->QueueSel == __VNET_QUEUE_RX) \
                    vnet->ram[__VNET_QUEUE.QueueAvail] |= 1; /* set VIRTQ_AVAIL_F_NO_INTERRUPT */ \
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

typedef struct {
    uint32_t QueueNum;
    uint32_t QueueDesc;
    uint32_t QueueAvail;
    uint32_t QueueUsed;
    uint16_t last_avail;
    bool ready;
    bool fd_ready;
} virtiogpu_queue_t;

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
    // supplied by environment
    uint32_t* ram;
} virtiogpu_state_t;

#define __VGPU_FEATURES_0 ((1 << VIRTIO_GPU_F_VIRGL) /*| (1 << VIRTIO_GPU_F_RESOURCE_SHARED)*/)
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
        //int tap_fd = vgpu->tap_fd;
        uint32_t* ram = vgpu->ram;
        memset(vgpu, 0, sizeof(*vgpu));
        //vgpu->tap_fd = tap_fd;
        vgpu->ram = ram;
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

#define __VGPU_GENERATE_QUEUE_HANDLER(NAME_SUFFIX, QUEUE_IDX) \
    void __virtiogpu_try_##NAME_SUFFIX(virtiogpu_state_t* vgpu) { \
        uint32_t* ram = vgpu->ram; \
        virtiogpu_queue_t* queue = &vgpu->queues[QUEUE_IDX]; \
        if ((vgpu->Status & VIRTIO_STATUS__DEVICE_NEEDS_RESET) || !queue->fd_ready) \
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
            printf("[BUFFER]\n"); \
            uint16_t desc_idx; __VGPU_ITERATE_BUFFER(true, \
                printf("desc %#x: flags %u, len %u, %s\n", desc_idx, desc_flags & ~(VIRTQ_DESC_F_WRITE | VIRTQ_DESC_F_NEXT), desc[2], (desc_flags & VIRTQ_DESC_F_WRITE) ? "WRITE" : "READ"); \
                if (!(desc_flags & VIRTQ_DESC_F_WRITE)) { \
                    if (desc[2] < sizeof(struct virtio_gpu_ctrl_hdr)) \
                        return virtiogpu_set_fail(vgpu); \
                    struct virtio_gpu_ctrl_hdr *hdr = (struct virtio_gpu_ctrl_hdr *) &vgpu->ram[desc[0] >> 2]; \
                    printf("  type: %32s [flags: %u, fence: %lu, ctx: %u] payload: %lu\n", virtio_gpu_ctrl_type_to_string(hdr->type), hdr->flags, hdr->fence_id, hdr->ctx_id, desc[2] - sizeof(struct virtio_gpu_ctrl_hdr)); \
                } \
            ) \
            /* consume from available queue, write to used queue */ \
            queue->last_avail++; \
            ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2] = buffer_idx; \
            ram[queue->QueueUsed + 1 + (new_used % queue->QueueNum) * 2 + 1] = 0 /*FIXME*/; \
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

void virtiogpu_refresh_queue(virtiogpu_state_t* vgpu) {
    if (!(vgpu->Status & VIRTIO_STATUS__DRIVER_OK) || (vgpu->Status & VIRTIO_STATUS__DEVICE_NEEDS_RESET))
        return;
    // FIXME
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
                vgpu->ram[__VGPU_QUEUE.QueueAvail] |= 1; /* set VIRTQ_AVAIL_F_NO_INTERRUPT */ \
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
        R(case 63: /* ConfigGeneration (R) */ \
            *value = 0; return true;) \
        /* device-specific configuration space */ \
        R(case 64: /* events_read */ \
            *value = vgpu->pending_events; return true;) \
        W(case 65: /* events_clear */ \
            vgpu->pending_events &= ~value; return true;) \
        R(case 66: /* num_scanouts */ \
            *value = 1; return true;) \
        R(case 67: /* reserved */ \
            *value = 0; return true;) \
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

typedef struct {
    bool stopped;
    uint32_t *ram;
    plic_state_t plic;
    u8250_state_t uart;
    virtionet_state_t vnet;
    virtiogpu_state_t gpu;
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

void emulator_update_vgpu_interrupts(core_t* core) {
    emu_state_t *data = (emu_state_t *)core->user_data;
    data->gpu.InterruptStatus ? (data->plic.active |= IRQ_VGPU_BIT) : (data->plic.active &= ~IRQ_VGPU_BIT);
    plic_update_interrupts(core, &data->plic);
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
                emulator_update_uart_interrupts(core); \
                return; \
            case 0x41: /* VIRTIO-NET */ \
                REG_FUNC(R, W, virtionet_wrap_mem)(core, &data->vnet, addr & 0xFFFFF, width, value); \
                emulator_update_vnet_interrupts(core); \
                return; \
            case 0x42: /* VIRTIO-GPU */ \
                REG_FUNC(R, W, virtiogpu_wrap_mem)(core, &data->gpu, addr & 0xFFFFF, width, value); \
                emulator_update_vgpu_interrupts(core); \
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
    core.x_regs[RISCV_R_A1] = dtb_addr;

    // set up peripherals

    data.uart.in_fd = 0;
    data.uart.out_fd = 1;

    data.vnet.tap_fd = open("/dev/net/tun", O_RDWR);
    if (data.vnet.tap_fd < 0) {
        fprintf(stderr, "failed to open TAP device: %s\n", strerror(errno));
        return 2;
    }
    struct ifreq ifreq;
    memset(&ifreq, 0, sizeof(ifreq));
    // iproute2 sets PI, let's make it easier for folks who want to use persistent taps
    ifreq.ifr_flags = IFF_TAP | IFF_NO_PI;
    strncpy(ifreq.ifr_name, TAP_INTERFACE, sizeof(ifreq.ifr_name));
    if (ioctl(data.vnet.tap_fd, TUNSETIFF, &ifreq) < 0) {
        fprintf(stderr, "failed to allocate TAP device: %s\n", strerror(errno));
        return 2;
    }
    fprintf(stderr, "allocated TAP interface: %s\n", ifreq.ifr_name);
    assert(fcntl(data.vnet.tap_fd, F_SETFL, fcntl(data.vnet.tap_fd, F_GETFL, 0) | O_NONBLOCK) >= 0);
    data.vnet.ram = data.ram;

    int virgl_flags = VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_SURFACELESS /* | VIRGL_RENDERER_USE_EXTERNAL_BLOB */;
    struct virgl_renderer_callbacks virgl_cbs;
    memset(&virgl_cbs, 0, sizeof(virgl_cbs));
    virgl_cbs.version = VIRGL_RENDERER_CALLBACKS_VERSION;
    // TODO: write_fence cb?
    if (virgl_renderer_init(&data, virgl_flags, &virgl_cbs)) {
        fprintf(stderr, "failed to initialize virgl renderer\n");
        return 2;
    }

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

            u8250_check_ready(&data.uart);
            if (data.uart.in_ready)
                emulator_update_uart_interrupts(&core);

            virtionet_refresh_queue(&data.vnet);
            if (data.vnet.InterruptStatus)
                emulator_update_vnet_interrupts(&core);

            virtiogpu_refresh_queue(&data.gpu);
            if (data.gpu.InterruptStatus)
                emulator_update_vgpu_interrupts(&core);
        }

        bool timer_active = core.instr_count_h > data.timer_h || (core.instr_count_h == data.timer_h && core.instr_count > data.timer_l);
        timer_active ? (core.sip |= RISCV_INT_STI_BIT) : (core.sip &= ~RISCV_INT_STI_BIT);

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
    fprintf(stderr, "executed %u instructions in %.3f ms, %lld instructions, %lld cycles\n",
        core.instr_count, elapsed / 1e6, host_instr_count, host_cycle_count);
    fprintf(stderr, "stats: %.3f c/i, %.1f i/I, %.1f c/I, %.1f MIps\n",
        ((double)host_cycle_count) / ((double)host_instr_count),
        ((double)host_instr_count) / ((double)core.instr_count),
        ((double)host_cycle_count) / ((double)core.instr_count),
        core.instr_count / (double)elapsed * 1e3);
}
