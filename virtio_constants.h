// VIRTQUEUE

/* An interface for efficient virtio implementation.
 *
 * This header is BSD licensed so anyone can use the definitions
 * to implement compatible drivers/servers.
 *
 * Copyright 2007, 2009, IBM Corporation
 * Copyright 2011, Red Hat, Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of IBM nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ‘‘AS IS’’ AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL IBM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <stdint.h>

/* This marks a buffer as continuing via the next field. */
#define VIRTQ_DESC_F_NEXT       1
/* This marks a buffer as write-only (otherwise read-only). */
#define VIRTQ_DESC_F_WRITE      2
/* This means the buffer contains a list of buffer descriptors. */
#define VIRTQ_DESC_F_INDIRECT   4

/* The device uses this in used->flags to advise the driver: don’t kick me
 * when you add a buffer.  It’s unreliable, so it’s simply an
 * optimization. */
#define VIRTQ_USED_F_NO_NOTIFY  1
/* The driver uses this in avail->flags to advise the device: don’t
 * interrupt me when you consume a buffer.  It’s unreliable, so it’s
 * simply an optimization.  */
#define VIRTQ_AVAIL_F_NO_INTERRUPT      1

/* Support for indirect descriptors */
#define VIRTIO_F_INDIRECT_DESC    28

/* Support for avail_event and used_event fields */
#define VIRTIO_F_EVENT_IDX        29

/* Arbitrary descriptor layouts. */
#define VIRTIO_F_ANY_LAYOUT       27

/* Virtqueue descriptors: 16 bytes.
* These can chain together via "next". */
struct virtq_desc {
    /* Address (guest-physical). */
    uint64_t addr;
    /* Length. */
    uint32_t len;
    /* The flags as indicated above. */
    uint16_t flags;
    /* We chain unused descriptors via this, too */
    uint16_t next;
};

struct virtq_avail {
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[];
    /* Only if VIRTIO_F_EVENT_IDX: uint16_t used_event; */
};

/* uint32_t is used here for ids for padding reasons. */
struct virtq_used_elem {
    /* Index of start of used descriptor chain. */
    uint32_t id;
    /* Total length of the descriptor chain which was written to. */
    uint32_t len;
};

struct virtq_used {
    uint16_t flags;
    uint16_t idx;
    struct virtq_used_elem ring[];
    /* Only if VIRTIO_F_EVENT_IDX: uint16_t avail_event; */
};

struct virtq {
    unsigned int num;

    struct virtq_desc *desc;
    struct virtq_avail *avail;
    struct virtq_used *used;
};

static inline int virtq_need_event(uint16_t event_idx, uint16_t new_idx, uint16_t old_idx) {
    return (uint16_t)(new_idx - event_idx - 1) < (uint16_t)(new_idx - old_idx);
}

/* Get location of event indices (only with VIRTIO_F_EVENT_IDX) */
static inline uint16_t *virtq_used_event(struct virtq *vq) {
    /* For backwards compat, used event index is at *end* of avail ring. */
    return &vq->avail->ring[vq->num];
}

static inline uint16_t *virtq_avail_event(struct virtq *vq) {
    /* For backwards compat, avail event index is at *end* of used ring. */
    return (uint16_t *)&vq->used->ring[vq->num];
}

// VIRTIO CORE

#define __VIRTIO_MAGIC 0x74726976

#define VIRTIO_STATUS__ACKNOWLEDGE           1
#define VIRTIO_STATUS__DRIVER                2
#define VIRTIO_STATUS__FAILED              128
#define VIRTIO_STATUS__FEATURES_OK           8
#define VIRTIO_STATUS__DRIVER_OK             4
#define VIRTIO_STATUS__DEVICE_NEEDS_RESET   64

#define VIRTIO_INT__USED_RING   1
#define VIRTIO_INT__CONF_CHANGE 2

// VIRTIO-NET

#define __VIRTIO_ID_NET 2

#define __VNET_QUEUE_RX 0
#define __VNET_QUEUE_TX 1

// VIRTIO-GPU

#define __VIRTIO_ID_GPU 16

#define __VGPU_QUEUE_CONTROL 0
#define __VGPU_QUEUE_CURSOR 1

#define VIRTIO_GPU_F_VIRGL (0) // virgl 3D mode is supported.
#define VIRTIO_GPU_F_EDID (1) // EDID is supported.
#define VIRTIO_GPU_F_RESOURCE_SHARED (2) // shared resources are supported.

#define VIRTIO_GPU_EVENT_DISPLAY (1 << 0)

enum virtio_gpu_ctrl_type {
    /* 2d commands */
    VIRTIO_GPU_CMD_GET_DISPLAY_INFO = 0x0100,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_2D,
    VIRTIO_GPU_CMD_RESOURCE_UNREF,
    VIRTIO_GPU_CMD_SET_SCANOUT,
    VIRTIO_GPU_CMD_RESOURCE_FLUSH,
    VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D,
    VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING,
    VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING,
    VIRTIO_GPU_CMD_GET_CAPSET_INFO,
    VIRTIO_GPU_CMD_GET_CAPSET,
    VIRTIO_GPU_CMD_GET_EDID,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_2D_SHARED,

    /* 3d commands (OpenGL) */
    VIRTIO_GPU_CMD_CTX_CREATE = 0x0200,
    VIRTIO_GPU_CMD_CTX_DESTROY,
    VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE,
    VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_3D,
    VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D,
    VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D,
    VIRTIO_GPU_CMD_SUBMIT_3D,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_3D_SHARED,

    /* cursor commands */
    VIRTIO_GPU_CMD_UPDATE_CURSOR = 0x0300,
    VIRTIO_GPU_CMD_MOVE_CURSOR,

    /* success responses */
    VIRTIO_GPU_RESP_OK_NODATA = 0x1100,
    VIRTIO_GPU_RESP_OK_DISPLAY_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET,
    VIRTIO_GPU_RESP_OK_EDID,

    /* error responses */
    VIRTIO_GPU_RESP_ERR_UNSPEC = 0x1200,
    VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY,
    VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER,
    VIRTIO_GPU_RESP_ERR_NO_BACKING_STORAGE,
};

#include <stdio.h>

const char *virtio_gpu_ctrl_type_to_string(uint32_t type) {
    static struct { uint32_t value; const char *label; } NAMES [] = {
        { VIRTIO_GPU_CMD_GET_DISPLAY_INFO, "CMD_GET_DISPLAY_INFO" },
        { VIRTIO_GPU_CMD_RESOURCE_CREATE_2D, "CMD_RESOURCE_CREATE_2D" },
        { VIRTIO_GPU_CMD_RESOURCE_UNREF, "CMD_RESOURCE_UNREF" },
        { VIRTIO_GPU_CMD_SET_SCANOUT, "CMD_SET_SCANOUT" },
        { VIRTIO_GPU_CMD_RESOURCE_FLUSH, "CMD_RESOURCE_FLUSH" },
        { VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D, "CMD_TRANSFER_TO_HOST_2D" },
        { VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING, "CMD_RESOURCE_ATTACH_BACKING" },
        { VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING, "CMD_RESOURCE_DETACH_BACKING" },
        { VIRTIO_GPU_CMD_GET_CAPSET_INFO, "CMD_GET_CAPSET_INFO" },
        { VIRTIO_GPU_CMD_GET_CAPSET, "CMD_GET_CAPSET" },
        { VIRTIO_GPU_CMD_GET_EDID, "CMD_GET_EDID" },
        { VIRTIO_GPU_CMD_RESOURCE_CREATE_2D_SHARED, "CMD_RESOURCE_CREATE_2D_SHARED" },
        { VIRTIO_GPU_CMD_CTX_CREATE, "CMD_CTX_CREATE" },
        { VIRTIO_GPU_CMD_CTX_DESTROY, "CMD_CTX_DESTROY" },
        { VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE, "CMD_CTX_ATTACH_RESOURCE" },
        { VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE, "CMD_CTX_DETACH_RESOURCE" },
        { VIRTIO_GPU_CMD_RESOURCE_CREATE_3D, "CMD_RESOURCE_CREATE_3D" },
        { VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D, "CMD_TRANSFER_TO_HOST_3D" },
        { VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D, "CMD_TRANSFER_FROM_HOST_3D" },
        { VIRTIO_GPU_CMD_SUBMIT_3D, "CMD_SUBMIT_3D" },
        { VIRTIO_GPU_CMD_RESOURCE_CREATE_3D_SHARED, "CMD_RESOURCE_CREATE_3D_SHARED" },
        { VIRTIO_GPU_CMD_UPDATE_CURSOR, "CMD_UPDATE_CURSOR" },
        { VIRTIO_GPU_CMD_MOVE_CURSOR, "CMD_MOVE_CURSOR" },
        { VIRTIO_GPU_RESP_OK_NODATA, "RESP_OK_NODATA" },
        { VIRTIO_GPU_RESP_OK_DISPLAY_INFO, "RESP_OK_DISPLAY_INFO" },
        { VIRTIO_GPU_RESP_OK_CAPSET_INFO, "RESP_OK_CAPSET_INFO" },
        { VIRTIO_GPU_RESP_OK_CAPSET, "RESP_OK_CAPSET" },
        { VIRTIO_GPU_RESP_OK_EDID, "RESP_OK_EDID" },
        { VIRTIO_GPU_RESP_ERR_UNSPEC, "RESP_ERR_UNSPEC" },
        { VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY, "RESP_ERR_OUT_OF_MEMORY" },
        { VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID, "RESP_ERR_INVALID_SCANOUT_ID" },
        { VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, "RESP_ERR_INVALID_RESOURCE_ID" },
        { VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID, "RESP_ERR_INVALID_CONTEXT_ID" },
        { VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, "RESP_ERR_INVALID_PARAMETER" },
        { VIRTIO_GPU_RESP_ERR_NO_BACKING_STORAGE, "RESP_ERR_NO_BACKING_STORAGE" },
    };
    static char unknown_buf [32];
    for (size_t i = 0; i < sizeof(NAMES) / sizeof(*NAMES); i++) {
        if (NAMES[i].value == type)
            return NAMES[i].label;
    }
    snprintf(unknown_buf, sizeof(unknown_buf), "0x%04x", type);
    return unknown_buf;
}

#define VIRTIO_GPU_FLAG_FENCE (1 << 0)

// On success the device will return VIRTIO_GPU_RESP_OK_NODATA in case there is no payload. Otherwise the type field will indicate the kind of payload.
//
// On error the device will return one of the VIRTIO_GPU_RESP_ERR_* error codes.
struct virtio_gpu_ctrl_hdr {
    // specifies the type of the driver request (VIRTIO_GPU_CMD_*) or device response (VIRTIO_GPU_RESP_*).
    uint32_t type;
    // request / response flags.
    uint32_t flags;
    // If the driver sets the VIRTIO_GPU_FLAG_FENCE bit in the request flags field the device MUST:
    //  - set VIRTIO_GPU_FLAG_FENCE bit in the response,
    //  - copy the content of the fence_id field from the request to the response, and
    //  - send the response only after command processing is complete.
    uint64_t fence_id;
    // Rendering context (used in 3D mode only).
    uint32_t ctx_id;

    uint32_t __padding;
};
