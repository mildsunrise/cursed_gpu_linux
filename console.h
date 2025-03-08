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

// called from console_poll() when suggested scanout size has changed.
// this won't fire before console_get_scanout_size()'s first call,
// and when fired, it won't fire again until console_get_scanout_size() is called.
typedef void(*console_new_scanout_size_cb)(void* data);
void console_set_new_scanout_size_cb(console_t* con, console_new_scanout_size_cb cb);

// to be called from emulation thread
// ----------------------------------

typedef struct {
    uint32_t width;
    uint32_t height;
} console_scanout_size_t;

// get current suggested scanout size. after the first call, this must ONLY
// be called in response to new_scanout_size firing
void console_get_scanout_size(console_t* con, /* no transfer */ console_scanout_size_t* ssize);

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
