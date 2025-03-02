#include "console.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>
#include <dlfcn.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <stdatomic.h>
#include <wayland-client.h>
#include <wayland-egl.h>
#include "wl_protocols/stable/xdg-shell/xdg-shell.h"
#include "wl_protocols/unstable/xdg-decoration/xdg-decoration-unstable-v1.h"
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <GL/glext.h>

#define __checkerrno(expr, fmt, ...) \
    if (expr) { \
        fprintf(stderr, fmt " failed: %s\n", __VA_ARGS__ __VA_OPT__(,) strerror(errno)); \
        exit(2); \
    }

#define ARRAY_SIZE(x) ((sizeof x) / (sizeof *x))

struct console_buffer_t {
    _Atomic uint8_t nrefs;
    bool is_bound;
    GLuint tex_id;
    uint32_t width, height;
    EGLImage egl_image;
};

typedef struct {
    _Atomic uint8_t nrefs;
    console_scanout_flush_t flush;
} flush_request_t;

#define WL_GLOBALS(MACRO) \
    MACRO(wl_xdg, xdg_wm_base, 1) \
    MACRO(wl_xdg_decoration, zxdg_decoration_manager_v1, 1) \
    MACRO(wl_compositor, wl_compositor, 4)

struct console_t {
    int poll_fd;
    int event_fd;
    bool wl_waiting_for_write;

    // pools to avoid frequent malloc
    console_buffer_t bufs [4];
    flush_request_t flushes [4];

    _Atomic (flush_request_t*) pending_flush;
    console_scanout_flush_t current_flush;

    EGLDisplay egl_display;
    EGLContext egl_context;
    struct wl_egl_window* wl_egl_window;
    EGLSurface egl_surface;

    struct wl_display* wl_display;
    struct wl_registry* wl_registry;
#define WL_DECLARE_GLOBAL(NAME, INTERFACE, VERSION) \
    struct INTERFACE* NAME; \
    uint32_t __##NAME##_name;
WL_GLOBALS(WL_DECLARE_GLOBAL)
#undef WL_DECLARE_GLOBAL
    struct wl_surface* wl_surface;
    struct xdg_surface* wl_xdg_surface;
    struct xdg_toplevel* wl_toplevel;
    struct zxdg_toplevel_decoration_v1* wl_decoration;
    bool xdg_decoration_configure_done;
};

// EVENT LOOP

static inline bool _eventfd_read(int fd, uint64_t* n) {
    int nread = read(fd, n, sizeof(*n));
    if (nread == -1 && errno == EWOULDBLOCK) return false;
    __checkerrno(nread < 0, "read eventfd");
    assert(nread == sizeof(*n));
    return true;
}

static inline bool _eventfd_write(int fd, uint64_t n) {
    int nwrite = write(fd, &n, sizeof(n));
    if (nwrite == -1 && errno == EWOULDBLOCK) return false;
    __checkerrno(nwrite < 0, "read eventfd");
    assert(nwrite == sizeof(n));
    return true;
}

static void poll_eventfd(console_t* con, uint64_t n);

static void poll_event(console_t* con, struct epoll_event ev) {
    switch (ev.data.u32) {
        case 0: {
            uint64_t n;
            if (_eventfd_read(con->event_fd, &n))
                poll_eventfd(con, n);
            break;
        }
        case 1:
            break; // no need to do anything, a read/flush is done at the start/end of poll
        default: abort();
    }
}

static void init_wayland(console_t* con);
static void init_egl(console_t* con);
static void init_gl(console_t* con);

static void wayland_pre_poll(console_t* con);

console_t* console_new() {
    console_t* con = calloc(1, sizeof(console_t));
    if (!con) {
        fprintf(stderr, "failed to allocate console state\n");
        return NULL;
    }

    __checkerrno((con->poll_fd = epoll_create(2)) <= 0, "create epoll");
    __checkerrno((con->event_fd = eventfd(0, EFD_NONBLOCK)) <= 0, "create eventfd");
    struct epoll_event epoll_ev;
    epoll_ev.events = EPOLLIN;
    epoll_ev.data.u32 = 0;
    __checkerrno(epoll_ctl(con->poll_fd, EPOLL_CTL_ADD, con->event_fd, &epoll_ev), "add eventfd to epoll");

    __checkerrno(!(con->wl_display = wl_display_connect(NULL)), "connect to wayland server");
    epoll_ev.events = EPOLLIN | (con->wl_waiting_for_write ? EPOLLOUT : 0);
    epoll_ev.data.u32 = 1;
    __checkerrno(epoll_ctl(con->poll_fd, EPOLL_CTL_ADD, wl_display_get_fd(con->wl_display), &epoll_ev), "add wayland display to epoll");

    init_wayland(con);
    init_egl(con);
    console_make_current(con);
    init_gl(con);
    wayland_pre_poll(con);
    return con;
}

int console_get_poll_fd(console_t* con) {
    return con->poll_fd;
}

void console_poll(console_t* con) {
    struct epoll_event evs [4];
    int n_evs = epoll_wait(con->poll_fd, evs, ARRAY_SIZE(evs), 0);
    __checkerrno(n_evs < 0, "epoll_wait");
    assert((size_t)n_evs < ARRAY_SIZE(evs));

    __checkerrno(wl_display_read_events(con->wl_display) < 0, "wl_display_read_events");

    for (int i = 0; i < n_evs; i++)
        poll_event(con, evs[i]);

    wayland_pre_poll(con);
}

static void wayland_pre_poll(console_t* con) {
    while (wl_display_prepare_read(con->wl_display) != 0)
        __checkerrno(wl_display_dispatch_pending(con->wl_display) < 0, "wl_display_dispatch_pending");

    int err = wl_display_flush(con->wl_display);
    __checkerrno(err < 0 && errno != EAGAIN, "wl_display_flush");
    if ((err < 0) != con->wl_waiting_for_write) {
        con->wl_waiting_for_write = err < 0;
        struct epoll_event epoll_ev;
        epoll_ev.events = EPOLLIN | (con->wl_waiting_for_write ? EPOLLOUT : 0);
        epoll_ev.data.u32 = 1;
        __checkerrno(epoll_ctl(con->poll_fd, EPOLL_CTL_MOD, wl_display_get_fd(con->wl_display), &epoll_ev), "mod wayland display in epoll");
    }
}

// WAYLAND SET UP

static void draw_frame(console_t* con);

static void registry_global(
    void *__data,
    struct wl_registry *wl_registry,
    uint32_t name,
    const char *interface,
    uint32_t /*version*/
) {
    console_t* con = (console_t*) __data;
#define WL_BIND_GLOBAL(NAME, INTERFACE, VERSION) \
    if (strcmp(interface, (INTERFACE##_interface).name) == 0) { \
        assert(!con->NAME); \
        con->__##NAME##_name = name; \
        con->NAME = wl_registry_bind(wl_registry, name, & INTERFACE##_interface, VERSION); \
        assert(con->NAME); \
        return; \
    }
WL_GLOBALS(WL_BIND_GLOBAL)
#undef WL_BIND_GLOBAL
}

static void registry_global_remove(
    void *__data,
    struct wl_registry* /*wl_registry*/,
    uint32_t name
) {
    console_t* con = (console_t*) __data;
#define WL_UNBIND_GLOBAL(NAME, INTERFACE, VERSION) \
    if (name == con->__##NAME##_name && con->NAME) { \
        fprintf(stderr, "compositor tried to remove global" #INTERFACE " of name=%u\n", name); \
        abort(); \
    }
WL_GLOBALS(WL_UNBIND_GLOBAL)
#undef WL_UNBIND_GLOBAL
}

static const struct wl_registry_listener registry_listener = {
    .global = registry_global,
    .global_remove = registry_global_remove,
};

static void xdg_ping(void * /*__data*/, struct xdg_wm_base *xdg_wm_base, uint32_t serial) {
    xdg_wm_base_pong(xdg_wm_base, serial);
}

static const struct xdg_wm_base_listener xdg_listener = {
    .ping = xdg_ping,
};

static void xdg_configure(void * /*__data*/, struct xdg_surface *xdg_surface, uint32_t serial) {
    //console_t* con = (console_t*) __data;
    xdg_surface_ack_configure(xdg_surface, serial);
}

static const struct xdg_surface_listener xdg_surface_listener = {
    .configure = xdg_configure,
};

static void xdg_close(void * /*__data*/, struct xdg_toplevel *) {
    fprintf(stderr, "window closed, exiting...\n");
    exit(0);
}

static void xdg_tl_configure(
    void */*data*/,
    struct xdg_toplevel */*xdg_toplevel*/,
    int32_t /*width*/,
    int32_t /*height*/,
    struct wl_array */*states*/
) {}

static void xdg_tl_configure_bounds(
    void */*data*/,
    struct xdg_toplevel */*xdg_toplevel*/,
    int32_t /*width*/,
    int32_t /*height*/
) {}

static void xdg_wm_capabilities(
    void */*data*/,
    struct xdg_toplevel */*xdg_toplevel*/,
    struct wl_array */*capabilities*/
) {}

static const struct xdg_toplevel_listener xdg_toplevel_listener = {
    .close = xdg_close,
    .configure = xdg_tl_configure,
    .configure_bounds = xdg_tl_configure_bounds,
    .wm_capabilities = xdg_wm_capabilities,
};

static void xdg_decoration_configure(void *__data, struct zxdg_toplevel_decoration_v1 *, uint32_t mode) {
    console_t* con = (console_t*) __data;
    if (mode != ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE)
        fprintf(stderr, "Compositor rejected request to use server-side decoration. You will thus have no decorations. Sorry.\n");
    con->xdg_decoration_configure_done = true;
}

static const struct zxdg_toplevel_decoration_v1_listener xdg_decoration_listener = {
    .configure = xdg_decoration_configure,
};

static void init_wayland(console_t* con) {
    con->wl_registry = wl_display_get_registry(con->wl_display);
    assert(con->wl_registry);
    wl_registry_add_listener(con->wl_registry, &registry_listener, con);
    __checkerrno(wl_display_roundtrip(con->wl_display) < 0, "wl_display_roundtrip");

    assert(con->wl_compositor && con->wl_xdg);
    xdg_wm_base_add_listener(con->wl_xdg, &xdg_listener, con);

    con->wl_surface = wl_compositor_create_surface(con->wl_compositor);
    assert(con->wl_surface);

    con->wl_xdg_surface = xdg_wm_base_get_xdg_surface(con->wl_xdg, con->wl_surface);
    assert(con->wl_xdg_surface);
    xdg_surface_add_listener(con->wl_xdg_surface, &xdg_surface_listener, con);
    con->wl_toplevel = xdg_surface_get_toplevel(con->wl_xdg_surface);
    assert(con->wl_toplevel);
    xdg_toplevel_add_listener(con->wl_toplevel, &xdg_toplevel_listener, con);

    xdg_toplevel_set_title(con->wl_toplevel, "Emulator console");
    xdg_toplevel_set_app_id(con->wl_toplevel, "sh.alba.cursed_gpu_linux");
    xdg_toplevel_set_min_size(con->wl_toplevel, 800, 600);
    xdg_toplevel_set_max_size(con->wl_toplevel, 800, 600);
    struct wl_region* region = wl_compositor_create_region(con->wl_compositor);
    wl_region_add(region, 0, 0, 800, 600);
    wl_surface_set_input_region(con->wl_surface, region);
    wl_surface_set_opaque_region(con->wl_surface, region);
    con->wl_egl_window = wl_egl_window_create(con->wl_surface, 800, 600);
    assert(con->wl_egl_window);

    if (con->wl_xdg_decoration) {
        con->wl_decoration = zxdg_decoration_manager_v1_get_toplevel_decoration(con->wl_xdg_decoration, con->wl_toplevel);
        assert(con->wl_decoration);
        zxdg_toplevel_decoration_v1_add_listener(con->wl_decoration, &xdg_decoration_listener, con);
        zxdg_toplevel_decoration_v1_set_mode(con->wl_decoration, ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE);
        while (!con->xdg_decoration_configure_done)
            __checkerrno(wl_display_roundtrip(con->wl_display) < 0, "wl_display_roundtrip");
    } else {
        fprintf(stderr,
            "You are using one of the only compositors that don't implement server-side decorations, "
            "likely because it refuses to do so. You will thus have no decorations. Sorry.\n");
    }
}

// EGL SET UP

static void check_egl_error(const char* msg) {
    EGLint error = eglGetError();
    if (error != EGL_SUCCESS) {
        fprintf(stderr, "EGL error on %s: %#x\n", msg, error);
        exit(2);
    }
}

#define DEFINE_EGL_ATTRS(TYPE, NAME, CAPACITY) \
    TYPE NAME [(CAPACITY)*2+1]; \
    size_t __##NAME##_n = 0; \
    NAME[__##NAME##_n] = EGL_NONE;
#define ADD_EGL_ATTR(NAME, KEY, VAL) \
    assert(__##NAME##_n % 2 == 0 && (__##NAME##_n + 2) < ARRAY_SIZE(NAME)); \
    NAME[__##NAME##_n++] = (KEY); \
    NAME[__##NAME##_n++] = (VAL); \
    NAME[__##NAME##_n] = EGL_NONE;

static PFNEGLQUERYDMABUFFORMATSEXTPROC eglQueryDmaBufFormatsEXT;
static PFNEGLQUERYDMABUFMODIFIERSEXTPROC eglQueryDmaBufModifiersEXT;

static void load_egl_extensions(console_t* con) {
    // FIXME: use EGL_EXT_platform_base and EGL_KHR_image_base rather than assuming EGL >=1.5
    const char* exts = eglQueryString(con->egl_display, EGL_EXTENSIONS);
    check_egl_error("eglQueryString(EXTENSIONS)");
    if (!strstr(exts, "EGL_EXT_image_dma_buf_import_modifiers")) {
        // FIXME: maybe try to fall back to EGL_EXT_image_dma_buf_import
        fprintf(stderr, "DMA-BUF import extension not found\n");
        exit(1);
    }

    bool ok = true
        && (eglQueryDmaBufFormatsEXT = (PFNEGLQUERYDMABUFFORMATSEXTPROC) eglGetProcAddress("eglQueryDmaBufFormatsEXT"))
        && (eglQueryDmaBufModifiersEXT = (PFNEGLQUERYDMABUFMODIFIERSEXTPROC) eglGetProcAddress("eglQueryDmaBufModifiersEXT"))
    ;
    assert(ok);
}

void console_make_current(console_t* con) {
    eglMakeCurrent(con->egl_display, con->egl_surface, con->egl_surface, con->egl_context);
    check_egl_error("eglMakeCurrent");
}

static void init_egl(console_t* con) {
    con->egl_display = eglGetPlatformDisplay(EGL_PLATFORM_WAYLAND_KHR, con->wl_display, NULL);
    check_egl_error("eglGetPlatformDisplay");
    eglInitialize(con->egl_display, NULL, NULL);
    check_egl_error("eglInitialize");
    load_egl_extensions(con);
    DEFINE_EGL_ATTRS(EGLint, config_attrs, 1);
    ADD_EGL_ATTR(config_attrs, EGL_SURFACE_TYPE, EGL_WINDOW_BIT);
    EGLConfig configs [1];
    EGLint num_configs;
    eglChooseConfig(con->egl_display, config_attrs, configs, ARRAY_SIZE(configs), &num_configs);
    check_egl_error("eglChooseConfig");
    if (!num_configs) {
        fprintf(stderr, "failed to find a suitable EGL config\n");
        exit(2);
    }

    con->egl_surface = eglCreatePlatformWindowSurface(con->egl_display, configs[0], con->wl_egl_window, NULL);
    check_egl_error("eglCreatePlatformWindowSurface");
    eglBindAPI(EGL_OPENGL_API);
    check_egl_error("eglBindAPI");
    DEFINE_EGL_ATTRS(EGLint, context_attrs, 2);
    ADD_EGL_ATTR(context_attrs, EGL_CONTEXT_MAJOR_VERSION, 3);
    ADD_EGL_ATTR(context_attrs, EGL_CONTEXT_MINOR_VERSION, 2);
    con->egl_context = eglCreateContext(con->egl_display, configs[0], EGL_NO_CONTEXT, context_attrs);
    check_egl_error("eglCreateContext");
}

// GL SETUP + IMPORTING + THREAD SYNC

static void check_gl_error(const char* msg) {
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        fprintf(stderr, "OpenGL error on %s: %#x\n", msg, error);
        exit(2);
    }
}

#define GL_TEXTURE_EXTERNAL_OES                                0x8D65

static PFNGLGETSTRINGIPROC glGetStringi;
static PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;

static bool has_gl_extension(const char* ext_name) {
    GLint n_exts = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &n_exts);
    check_gl_error("glGetIntegerv(GL_NUM_EXTENSIONS)");
    for (GLint i = 0; i < n_exts; i++) {
        const char* name = (const char*) glGetStringi(GL_EXTENSIONS, i);
        check_gl_error("glGetStringi(GL_EXTENSIONS)");
        if (strcmp(name, ext_name) == 0)
            return true;
    }
    return false;
}

static void load_gl_extensions() {
    // until EGL 1.5, non-extension functions would fail
    if (!(glGetStringi = (PFNGLGETSTRINGIPROC) eglGetProcAddress("glGetStringi"))
        && !(glGetStringi = dlsym(RTLD_DEFAULT, "glGetStringi"))) {
        abort(); // should never happen i think, we specified OpenGL >= 3 when creating the context
    }

    if (!has_gl_extension("GL_OES_EGL_image_external")) {
        // FIXME: maybe try to fall back to GL_OES_EGL_image
        fprintf(stderr, "GL_OES_EGL_image_external extension not found\n");
        exit(1);
    }

    bool ok = true
        && (glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC) eglGetProcAddress("glEGLImageTargetTexture2DOES"))
    ;
    assert(ok);
}

static void init_gl(console_t* con) {
    load_gl_extensions();

    eglSwapInterval(con->egl_display, 0);
    check_egl_error("eglSwapInterval(0)");

    GLuint buf_tex_ids [ARRAY_SIZE(con->bufs)];
    glGenTextures(ARRAY_SIZE(buf_tex_ids), buf_tex_ids);
    for (size_t i = 0; i < ARRAY_SIZE(buf_tex_ids); i++)
        con->bufs[i].tex_id = buf_tex_ids[i];

    draw_frame(con);
}

static console_buffer_t* get_buffer(console_t* con) {
    for (size_t i = 0; i < ARRAY_SIZE(con->bufs); i++) {
        uint8_t nrefs = 0;
        if (atomic_compare_exchange_strong(&con->bufs[i].nrefs, &nrefs, 1))
            return &con->bufs[i];
    }
    // should never happen; we (emulator thread) are not keeping any buffers
    // alive at this time, and the other side is keeping at most 2 buffers alive
    // at all times; the other side never requests buffers,
    // and our operations are sequentially consistent
    fprintf(stderr, "assertion failed: no free buffers to take\n");
    abort();
}

#define IMPORT_PLANE(I) \
    if (I < buffer->num_planes) { \
        ADD_EGL_ATTR(image_attrs, EGL_DMA_BUF_PLANE##I##_FD_EXT, buffer->fds[I]); \
        ADD_EGL_ATTR(image_attrs, EGL_DMA_BUF_PLANE##I##_OFFSET_EXT, buffer->offsets[I]); \
        ADD_EGL_ATTR(image_attrs, EGL_DMA_BUF_PLANE##I##_PITCH_EXT, buffer->strides[I]); \
        ADD_EGL_ATTR(image_attrs, EGL_DMA_BUF_PLANE##I##_MODIFIER_LO_EXT, buffer->drm_modifiers & (0xFFFFFFFF)); \
        ADD_EGL_ATTR(image_attrs, EGL_DMA_BUF_PLANE##I##_MODIFIER_HI_EXT, buffer->drm_modifiers >> 32); \
    }

console_buffer_t* console_import_buffer(console_t* con, console_buffer_import_data_t* buffer) {
    console_buffer_t* buf = get_buffer(con);
    buf->is_bound = false;
    buf->width = buffer->width;
    buf->height = buffer->height;
    DEFINE_EGL_ATTRS(EGLAttrib, image_attrs, 4+4*5);
    ADD_EGL_ATTR(image_attrs, EGL_IMAGE_PRESERVED, EGL_TRUE);
    ADD_EGL_ATTR(image_attrs, EGL_WIDTH, buffer->width);
    ADD_EGL_ATTR(image_attrs, EGL_HEIGHT, buffer->height);
    ADD_EGL_ATTR(image_attrs, EGL_LINUX_DRM_FOURCC_EXT, buffer->drm_format);
    IMPORT_PLANE(0);
    IMPORT_PLANE(1);
    IMPORT_PLANE(2);
    IMPORT_PLANE(3);
    buf->egl_image = eglCreateImage(con->egl_display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, image_attrs);
    check_egl_error("eglCreateImage");
    for (int i = 0; i < buffer->num_planes; i++)
        __checkerrno(close(buffer->fds[i]), "closing plane %d", i);
    return buf;
}

#undef IMPORT_PLANE

// careful, we call this from both threads
void console_free_buffer(console_t* con, console_buffer_t* buf) {
    EGLImage img = buf->egl_image;
    uint8_t nrefs = atomic_fetch_sub(&buf->nrefs, 1);
    assert(nrefs != 0);
    if (nrefs == 1) {
        eglDestroyImage(con->egl_display, img);
        check_egl_error("eglDestroyImage");
    }
}

static flush_request_t* get_flush_request(console_t* con) {
    for (size_t i = 0; i < ARRAY_SIZE(con->flushes); i++) {
        uint8_t nrefs = 0;
        if (atomic_compare_exchange_strong(&con->flushes[i].nrefs, &nrefs, 1))
            return &con->flushes[i];
    }
    fprintf(stderr, "assertion failed: no free flush requests to take\n");
    abort();
}

// careful, we call this from both threads
// keep updated with poll_eventfd below
static void free_flush_request(console_t* con, flush_request_t* req) {
    console_buffer_t* buf = req->flush.buf;
    uint8_t nrefs = atomic_fetch_sub(&req->nrefs, 1);
    assert(nrefs != 0);
    if (nrefs == 1) {
        if (buf)
            console_free_buffer(con, buf);
    }
}

void console_update_scanout(console_t* con, console_scanout_flush_t* sc) {
    flush_request_t* req = get_flush_request(con);
    req->flush = *sc;
    if (req->flush.buf) {
        uint8_t nrefs = atomic_fetch_add(&req->flush.buf->nrefs, 1);
        assert(nrefs != (uint8_t)-1);
    }
    req = atomic_exchange(&con->pending_flush, req);
    bool ok = _eventfd_write(con->event_fd, 1);
    assert(ok);

    if (req)
        free_flush_request(con, req);
}

static void poll_eventfd(console_t* con, uint64_t /*n*/) {
    flush_request_t* req = atomic_exchange(&con->pending_flush, NULL);
    if (!req) return;
    console_scanout_flush_t* sc = &con->current_flush;
    if (sc->buf)
        console_free_buffer(con, sc->buf);
    *sc = req->flush;
    uint8_t nrefs = atomic_fetch_sub(&req->nrefs, 1);
    assert(nrefs != 0);

    if (sc->buf && !sc->buf->is_bound) {
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, sc->buf->tex_id);
        check_gl_error("glBindTexture");
        glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, sc->buf->egl_image);
        check_gl_error("glEGLImageTargetTexture2DOES");
    }

    draw_frame(con);
}

static void draw_frame(console_t* con) {
    console_scanout_flush_t* sc = &con->current_flush;

    if (!sc->buf) {
        xdg_toplevel_set_title(con->wl_toplevel, "Emulator (no output)");
    } else {
        char buf [128];
        snprintf(buf, sizeof(buf), "Emulator (%u×%u)", sc->viewport.w, sc->viewport.h);
        xdg_toplevel_set_title(con->wl_toplevel, buf);
    }

    glClearColor(0.1, 0.1, 0.1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    // TODO: draw

    eglSwapBuffers(con->egl_display, con->egl_surface);
    check_egl_error("eglSwapBuffers");
}
