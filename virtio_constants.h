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

#include <string.h>
#include <stdio.h>

static inline const char *virtio_status_to_string(uint32_t type) {
    static struct { uint32_t value; const char *label; } NAMES [] = {
        { VIRTIO_STATUS__ACKNOWLEDGE, "ACKNOWLEDGE" },
        { VIRTIO_STATUS__DRIVER, "DRIVER" },
        { VIRTIO_STATUS__FAILED, "FAILED" },
        { VIRTIO_STATUS__FEATURES_OK, "FEATURES_OK" },
        { VIRTIO_STATUS__DRIVER_OK, "DRIVER_OK" },
        { VIRTIO_STATUS__DEVICE_NEEDS_RESET, "DEVICE_NEEDS_RESET" },
    };
    static char result_buf [128];
    char *result = result_buf;
    *result = 0;
    for (size_t i = 0; i < sizeof(NAMES) / sizeof(*NAMES); i++) {
        if (!(type & NAMES[i].value))
            continue;
        if (result != result_buf)
            result = stpcpy(result, " | ");
        result = stpcpy(result, NAMES[i].label);
        type &= ~NAMES[i].value;
    }
    if (type) {
        if (result != result_buf)
            result = stpcpy(result, " | ");
        sprintf(result, "%#x", type);
    }
    if (result == result_buf)
        return "none";
    return result_buf;
}

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

/*
 * virgl 3D mode is supported.
 * VIRTIO_GPU_CMD_CTX_*
 * VIRTIO_GPU_CMD_*_3D
 */
#define VIRTIO_GPU_F_VIRGL               0

/*
 * EDID is supported.
 * VIRTIO_GPU_CMD_GET_EDID
 */
#define VIRTIO_GPU_F_EDID                1
/*
 * assigning resources UUIDs for export to other virtio devices is supported.
 * VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID
 */
#define VIRTIO_GPU_F_RESOURCE_UUID       2

/*
 * creating and using size-based blob resources is supported.
 * VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB
 */
#define VIRTIO_GPU_F_RESOURCE_BLOB       3
/*
 * multiple context types and synchronization timelines supported. Requires VIRTIO_GPU_F_VIRGL.
 * VIRTIO_GPU_CMD_CREATE_CONTEXT with
 * context_init and multiple timelines
 */
#define VIRTIO_GPU_F_CONTEXT_INIT        4

#define VIRTIO_GPU_EVENT_DISPLAY (1 << 0)

enum virtio_gpu_ctrl_type {
    VIRTIO_GPU_UNDEFINED = 0,

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
    VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB,
    VIRTIO_GPU_CMD_SET_SCANOUT_BLOB,

    /* 3d commands */
    VIRTIO_GPU_CMD_CTX_CREATE = 0x0200,
    VIRTIO_GPU_CMD_CTX_DESTROY,
    VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE,
    VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_3D,
    VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D,
    VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D,
    VIRTIO_GPU_CMD_SUBMIT_3D,
    VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB,
    VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB,

    /* cursor commands */
    VIRTIO_GPU_CMD_UPDATE_CURSOR = 0x0300,
    VIRTIO_GPU_CMD_MOVE_CURSOR,

    /* success responses */
    VIRTIO_GPU_RESP_OK_NODATA = 0x1100,
    VIRTIO_GPU_RESP_OK_DISPLAY_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET,
    VIRTIO_GPU_RESP_OK_EDID,
    VIRTIO_GPU_RESP_OK_RESOURCE_UUID,
    VIRTIO_GPU_RESP_OK_MAP_INFO,

    /* error responses */
    VIRTIO_GPU_RESP_ERR_UNSPEC = 0x1200,
    VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY,
    VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER,
};

#include <stdio.h>

static inline const char *virtio_gpu_ctrl_type_to_string(uint32_t type) {
    static struct { uint32_t value; const char *label; } NAMES [] = {
        { VIRTIO_GPU_UNDEFINED, "UNDEFINED" },
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
        { VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID, "CMD_RESOURCE_ASSIGN_UUID" },
        { VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB, "CMD_RESOURCE_CREATE_BLOB" },
        { VIRTIO_GPU_CMD_SET_SCANOUT_BLOB, "CMD_SET_SCANOUT_BLOB" },
        { VIRTIO_GPU_CMD_CTX_CREATE, "CMD_CTX_CREATE" },
        { VIRTIO_GPU_CMD_CTX_DESTROY, "CMD_CTX_DESTROY" },
        { VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE, "CMD_CTX_ATTACH_RESOURCE" },
        { VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE, "CMD_CTX_DETACH_RESOURCE" },
        { VIRTIO_GPU_CMD_RESOURCE_CREATE_3D, "CMD_RESOURCE_CREATE_3D" },
        { VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D, "CMD_TRANSFER_TO_HOST_3D" },
        { VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D, "CMD_TRANSFER_FROM_HOST_3D" },
        { VIRTIO_GPU_CMD_SUBMIT_3D, "CMD_SUBMIT_3D" },
        { VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB, "CMD_RESOURCE_MAP_BLOB" },
        { VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB, "CMD_RESOURCE_UNMAP_BLOB" },
        { VIRTIO_GPU_CMD_UPDATE_CURSOR, "CMD_UPDATE_CURSOR" },
        { VIRTIO_GPU_CMD_MOVE_CURSOR, "CMD_MOVE_CURSOR" },
        { VIRTIO_GPU_RESP_OK_NODATA, "RESP_OK_NODATA" },
        { VIRTIO_GPU_RESP_OK_DISPLAY_INFO, "RESP_OK_DISPLAY_INFO" },
        { VIRTIO_GPU_RESP_OK_CAPSET_INFO, "RESP_OK_CAPSET_INFO" },
        { VIRTIO_GPU_RESP_OK_CAPSET, "RESP_OK_CAPSET" },
        { VIRTIO_GPU_RESP_OK_EDID, "RESP_OK_EDID" },
        { VIRTIO_GPU_RESP_OK_RESOURCE_UUID, "RESP_OK_RESOURCE_UUID" },
        { VIRTIO_GPU_RESP_OK_MAP_INFO, "RESP_OK_MAP_INFO" },
        { VIRTIO_GPU_RESP_ERR_UNSPEC, "RESP_ERR_UNSPEC" },
        { VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY, "RESP_ERR_OUT_OF_MEMORY" },
        { VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID, "RESP_ERR_INVALID_SCANOUT_ID" },
        { VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID, "RESP_ERR_INVALID_RESOURCE_ID" },
        { VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID, "RESP_ERR_INVALID_CONTEXT_ID" },
        { VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER, "RESP_ERR_INVALID_PARAMETER" },
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
/*
 * If the following flag is set, then ring_idx contains the index
 * of the command ring that needs to used when creating the fence
 */
#define VIRTIO_GPU_FLAG_INFO_RING_IDX (1 << 1)

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

    // If VIRTIO_GPU_F_CONTEXT_INIT is supported, then the driver MAY set VIRTIO_GPU_FLAG_INFO_RING_IDX bit in the request flags. In that case:
    //  - ring_idx indicates the value of a context-specific ring index. The minimum value is 0 and maximum value is 63 (inclusive).
    //  - If VIRTIO_GPU_FLAG_FENCE is set, fence_id acts as a sequence number on the synchronization timeline defined by ctx_idx and the ring index.
    //  - If VIRTIO_GPU_FLAG_FENCE is set and when the command associated with fence_id is complete, the device MUST send a response for all outstanding commands with a sequence number less than or equal to fence_id on the same synchronization timeline.
    uint8_t ring_idx;
    uint8_t padding[3];
};

/* data passed in the cursor vq */

struct virtio_gpu_cursor_pos {
    uint32_t scanout_id;
    uint32_t x;
    uint32_t y;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_UPDATE_CURSOR, VIRTIO_GPU_CMD_MOVE_CURSOR */
struct virtio_gpu_update_cursor {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_cursor_pos pos;  /* update & move */
    uint32_t resource_id;           /* update only */
    uint32_t hot_x;                 /* update only */
    uint32_t hot_y;                 /* update only */
    uint32_t padding;
};

/* data passed in the control vq, 2d related */

struct virtio_gpu_rect {
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
};

/* VIRTIO_GPU_CMD_RESOURCE_UNREF */
struct virtio_gpu_resource_unref {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_RESOURCE_CREATE_2D: create a 2d resource with a format */
struct virtio_gpu_resource_create_2d {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t format;
    uint32_t width;
    uint32_t height;
};

/* VIRTIO_GPU_CMD_SET_SCANOUT */
struct virtio_gpu_set_scanout {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_rect r;
    uint32_t scanout_id;
    uint32_t resource_id;
};

/* VIRTIO_GPU_CMD_RESOURCE_FLUSH */
struct virtio_gpu_resource_flush {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_rect r;
    uint32_t resource_id;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D: simple transfer to_host */
struct virtio_gpu_transfer_to_host_2d {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_rect r;
    uint64_t offset;
    uint32_t resource_id;
    uint32_t padding;
};

struct virtio_gpu_mem_entry {
    uint64_t addr;
    uint32_t length;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING */
struct virtio_gpu_resource_attach_backing {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t nr_entries;
};

/* VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING */
struct virtio_gpu_resource_detach_backing {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t padding;
};

/* VIRTIO_GPU_RESP_OK_DISPLAY_INFO */
#define VIRTIO_GPU_MAX_SCANOUTS 16
struct virtio_gpu_resp_display_info {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_display_one {
        struct virtio_gpu_rect r;
        uint32_t enabled;
        uint32_t flags;
    } pmodes[VIRTIO_GPU_MAX_SCANOUTS];
};

/* data passed in the control vq, 3d related */

struct virtio_gpu_box {
    uint32_t x, y, z;
    uint32_t w, h, d;
};

/* VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D, VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D */
struct virtio_gpu_transfer_host_3d {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_box box;
    uint64_t offset;
    uint32_t resource_id;
    uint32_t level;
    uint32_t stride;
    uint32_t layer_stride;
};

/* VIRTIO_GPU_CMD_RESOURCE_CREATE_3D */
#define VIRTIO_GPU_RESOURCE_FLAG_Y_0_TOP (1 << 0)
struct virtio_gpu_resource_create_3d {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t target;
    uint32_t format;
    uint32_t bind;
    uint32_t width;
    uint32_t height;
    uint32_t depth;
    uint32_t array_size;
    uint32_t last_level;
    uint32_t nr_samples;
    uint32_t flags;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_CTX_CREATE */
#define VIRTIO_GPU_CONTEXT_INIT_CAPSET_ID_MASK 0x000000ff
struct virtio_gpu_ctx_create {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t nlen;
    uint32_t context_init;
    char debug_name[64];
};

/* VIRTIO_GPU_CMD_CTX_DESTROY */
struct virtio_gpu_ctx_destroy {
    struct virtio_gpu_ctrl_hdr hdr;
};

/* VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE, VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE */
struct virtio_gpu_ctx_resource {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_SUBMIT_3D */
struct virtio_gpu_cmd_submit {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t size;
    uint32_t padding;
};

#define VIRTIO_GPU_CAPSET_VIRGL 1
#define VIRTIO_GPU_CAPSET_VIRGL2 2
#define VIRTIO_GPU_CAPSET_GFXSTREAM 3
#define VIRTIO_GPU_CAPSET_VENUS 4
#define VIRTIO_GPU_CAPSET_CROSS_DOMAIN 5

/* VIRTIO_GPU_CMD_GET_CAPSET_INFO */
struct virtio_gpu_get_capset_info {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t capset_index;
    uint32_t padding;
};

/* VIRTIO_GPU_RESP_OK_CAPSET_INFO */
struct virtio_gpu_resp_capset_info {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t capset_id;
    uint32_t capset_max_version;
    uint32_t capset_max_size;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_GET_CAPSET */
struct virtio_gpu_get_capset {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t capset_id;
    uint32_t capset_version;
};

/* VIRTIO_GPU_RESP_OK_CAPSET */
struct virtio_gpu_resp_capset {
    struct virtio_gpu_ctrl_hdr hdr;
    uint8_t capset_data[];
};

/* VIRTIO_GPU_CMD_GET_EDID */
struct virtio_gpu_cmd_get_edid {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t scanout;
    uint32_t padding;
};

/* VIRTIO_GPU_RESP_OK_EDID */
struct virtio_gpu_resp_edid {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t size;
    uint32_t padding;
    uint8_t edid[1024];
};

#define VIRTIO_GPU_EVENT_DISPLAY (1 << 0)

struct virtio_gpu_config {
    uint32_t events_read;
    uint32_t events_clear;
    uint32_t num_scanouts;
    uint32_t num_capsets;
};

/* simple formats for fbcon/X use */
enum virtio_gpu_formats {
    VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM  = 1,
    VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM  = 2,
    VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM  = 3,
    VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM  = 4,

    VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM  = 67,
    VIRTIO_GPU_FORMAT_X8B8G8R8_UNORM  = 68,

    VIRTIO_GPU_FORMAT_A8B8G8R8_UNORM  = 121,
    VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM  = 134,
};

/* VIRTIO_GPU_CMD_RESOURCE_ASSIGN_UUID */
struct virtio_gpu_resource_assign_uuid {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t padding;
};

/* VIRTIO_GPU_RESP_OK_RESOURCE_UUID */
struct virtio_gpu_resp_resource_uuid {
    struct virtio_gpu_ctrl_hdr hdr;
    uint8_t uuid[16];
};

/* VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB */
struct virtio_gpu_resource_create_blob {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
#define VIRTIO_GPU_BLOB_MEM_GUEST             0x0001
#define VIRTIO_GPU_BLOB_MEM_HOST3D            0x0002
#define VIRTIO_GPU_BLOB_MEM_HOST3D_GUEST      0x0003

#define VIRTIO_GPU_BLOB_FLAG_USE_MAPPABLE     0x0001
#define VIRTIO_GPU_BLOB_FLAG_USE_SHAREABLE    0x0002
#define VIRTIO_GPU_BLOB_FLAG_USE_CROSS_DEVICE 0x0004
    /* zero is invalid blob mem */
    uint32_t blob_mem;
    uint32_t blob_flags;
    uint32_t nr_entries;
    uint64_t blob_id;
    uint64_t size;
    /*
     * sizeof(nr_entries * virtio_gpu_mem_entry) bytes follow
     */
};

/* VIRTIO_GPU_CMD_SET_SCANOUT_BLOB */
struct virtio_gpu_set_scanout_blob {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_rect r;
    uint32_t scanout_id;
    uint32_t resource_id;
    uint32_t width;
    uint32_t height;
    uint32_t format;
    uint32_t padding;
    uint32_t strides[4];
    uint32_t offsets[4];
};

/* VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB */
struct virtio_gpu_resource_map_blob {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t padding;
    uint64_t offset;
};

/* VIRTIO_GPU_RESP_OK_MAP_INFO */
#define VIRTIO_GPU_MAP_CACHE_MASK     0x0f
#define VIRTIO_GPU_MAP_CACHE_NONE     0x00
#define VIRTIO_GPU_MAP_CACHE_CACHED   0x01
#define VIRTIO_GPU_MAP_CACHE_UNCACHED 0x02
#define VIRTIO_GPU_MAP_CACHE_WC       0x03
struct virtio_gpu_resp_map_info {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t map_info;
    uint32_t padding;
};

/* VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB */
struct virtio_gpu_resource_unmap_blob {
    struct virtio_gpu_ctrl_hdr hdr;
    uint32_t resource_id;
    uint32_t padding;
};
