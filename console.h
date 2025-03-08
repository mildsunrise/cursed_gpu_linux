#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct console_t console_t;

// to be called from console thread
// --------------------------------

// returns NULL on failure
console_t* console_new();

// returns FD to poll for POLLIN. lives forever
int console_get_poll_fd(console_t* con);

// make the console's OpenGL context current. this needs to be called
// at least once before console_poll(), and possibly more times if
// something else makes another context current between calls.
void console_make_current(console_t* con);

// call on activity on the poll FD
void console_poll(console_t* con);

// set user data passed to callbacks
void console_set_cb_data(console_t* con, void* data);

// called from console_poll() when window is closed (can be called multiple times)
typedef void(*console_stop_cb)(void* data);
void console_set_stop_cb(console_t* con, console_stop_cb cb);

// to be called from emulation thread
// ----------------------------------

typedef struct {
    uint32_t drm_format;
    uint64_t drm_modifiers;
    uint32_t width;
    uint32_t height;
    int num_planes;
    /* transfer */ int fds [4];
    int strides [4];
    int offsets [4];
} console_buffer_import_data_t;

typedef struct console_buffer_t console_buffer_t;

/* transfer */ console_buffer_t* console_import_buffer(console_t* con, console_buffer_import_data_t* buffer);

void console_free_buffer(console_t* con, /* transfer */ console_buffer_t* buf);

typedef struct { uint32_t x, y, w, h; } console_rectangle_t;

typedef struct {
    // buffer (NULL -> no scanout). no transfer
    console_buffer_t* buf;
    // rectangle of 'buf' to show (passed in SET_VIEWPORT)
    console_rectangle_t viewport;
    // damage rectangle since last scanout flush
    console_rectangle_t damage;
} console_scanout_flush_t;

void console_update_scanout(console_t* con, /* no transfer */ console_scanout_flush_t* sc);
