#include "console.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>
#include <dlfcn.h>
#include <math.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <stdatomic.h>
#include <wayland-client.h>
#include <wayland-egl.h>
#include "wl_protocols/stable/xdg-shell/xdg-shell.h"
#include "wl_protocols/unstable/xdg-decoration/xdg-decoration-unstable-v1.h"
#include "wl_protocols/stable/viewporter/viewporter.h"
#include "wl_protocols/staging/fractional-scale/fractional-scale-v1.h"

#define GLAD_EGL_IMPLEMENTATION
#include "glad/egl.h"
#define GLAD_GL_IMPLEMENTATION
#include "glad/gl.h"

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

// holds double-buffered commit state, i.e. the kind of state that should
// only enter into effect atomically when xdg_surface.configure is called
typedef struct {
    // window geometry suggested in xdg_toplevel.configure
    int32_t width, height;

    // window geometry bounds in xdg_toplevel.configure_bounds
    int32_t bounds_width, bounds_height;

    enum wl_output_transform transform;

    // holds the preferred_scale if the fractional scale protocol is supported,
    // or the preferred_buffer_scale if not. always units of x120
    uint32_t scale;
} window_state_t;

static const window_state_t INITIAL_WINDOW_STATE = {
    .width = 0, .height = 0,
    .bounds_width = 0, .bounds_height = 0,
    .transform = WL_OUTPUT_TRANSFORM_NORMAL,
    .scale = 120,
};

typedef struct {
    _Atomic uint8_t nrefs;
    console_scanout_flush_t flush;
} flush_request_t;

#define WL_GLOBALS(MACRO) \
    MACRO(wl_xdg, xdg_wm_base, 1) \
    MACRO(wl_xdg_decoration, zxdg_decoration_manager_v1, 1) \
    MACRO(wl_viewporter, wp_viewporter, 1) \
    MACRO(wl_fractional_scale_manager, wp_fractional_scale_manager_v1, 1) \
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

    GLint _prog_blit;
    GLint _attr_blit_model_mat;
    GLint _attr_blit_texture_mat;
    GLint _prog_bg;
    GLint _attr_bg_cell_size;
    GLint _attr_bg_cell_offset;

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
    struct wp_viewport* wl_viewport;
    struct wp_fractional_scale_v1* wl_fractional_scale;
    struct zxdg_toplevel_decoration_v1* wl_decoration;
    bool xdg_decoration_configure_done;
    bool xdg_configure_done;
    window_state_t configured_state;
    window_state_t configuring_state;
    // used only during start-up to ensure correct ordering
    bool do_draw_frames;
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

static void xdg_configure(void * __data, struct xdg_surface *xdg_surface, uint32_t serial) {
    console_t* con = (console_t*) __data;
    xdg_surface_ack_configure(xdg_surface, serial);
    con->xdg_configure_done = true;
    con->configured_state = con->configuring_state;
    if (con->do_draw_frames)
        draw_frame(con);
}

static const struct xdg_surface_listener xdg_surface_listener = {
    .configure = xdg_configure,
};

static void xdg_close(void * /*__data*/, struct xdg_toplevel *) {
    fprintf(stderr, "window closed, exiting...\n");
    exit(0);
}

static void xdg_tl_configure(
    void *__data,
    struct xdg_toplevel */*xdg_toplevel*/,
    int32_t width,
    int32_t height,
    struct wl_array */*states*/
) {
    console_t* con = (console_t*) __data;
    con->configuring_state.width = width;
    con->configuring_state.height = height;
}

static void xdg_tl_configure_bounds(
    void *__data,
    struct xdg_toplevel */*xdg_toplevel*/,
    int32_t width,
    int32_t height
) {
    console_t* con = (console_t*) __data;
    con->configuring_state.bounds_width = width;
    con->configuring_state.bounds_height = height;
}

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

static void wl_surface_enter(
    void */*__data*/,
    struct wl_surface */*wl_surface*/,
    struct wl_output */*output*/
) {}

static void wl_surface_leave(
    void */*__data*/,
    struct wl_surface */*wl_surface*/,
    struct wl_output */*output*/
) {}

static void wl_surface_preferred_buffer_scale(
    void *__data,
    struct wl_surface */*wl_surface*/,
    int32_t factor
) {
    console_t* con = (console_t*) __data;
    assert(factor > 0);
    if (con->wl_fractional_scale) {
        if (factor != 1)
            fprintf(stderr, "Warning: compositor reported buffer scale %d while also supporting the fractional scale protocol... ignoring buffer scale.\n", factor);
    } else {
        con->configuring_state.scale = factor * 120;
    }
}

static void wl_surface_preferred_buffer_transform(
    void *__data,
    struct wl_surface */*wl_surface*/,
    uint32_t transform
) {
    console_t* con = (console_t*) __data;
    con->configuring_state.transform = transform;
}

static const struct wl_surface_listener wl_surface_listener = {
    .enter = wl_surface_enter,
    .leave = wl_surface_leave,
    .preferred_buffer_scale = wl_surface_preferred_buffer_scale,
    .preferred_buffer_transform = wl_surface_preferred_buffer_transform,
};

static void wl_preferred_fractional_scale(
    void *__data,
    struct wp_fractional_scale_v1 */*wp_fractional_scale_v1*/,
    uint32_t scale
) {
    console_t* con = (console_t*) __data;
    assert(scale > 0);
    con->configuring_state.scale = scale;
}

static const struct wp_fractional_scale_v1_listener wp_fractional_scale_listener = {
    .preferred_scale = wl_preferred_fractional_scale,
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

static const int32_t MIN_SIZE [2] = { 100, 100 };
static const int32_t INITIAL_SIZE [2] = { 800, 600 };

static void init_wayland(console_t* con) {
    con->configuring_state = INITIAL_WINDOW_STATE;

    con->wl_registry = wl_display_get_registry(con->wl_display);
    assert(con->wl_registry);
    wl_registry_add_listener(con->wl_registry, &registry_listener, con);
    __checkerrno(wl_display_roundtrip(con->wl_display) < 0, "wl_display_roundtrip");

    assert(con->wl_compositor && con->wl_xdg);
    xdg_wm_base_add_listener(con->wl_xdg, &xdg_listener, con);

    con->wl_surface = wl_compositor_create_surface(con->wl_compositor);
    assert(con->wl_surface);
    wl_surface_add_listener(con->wl_surface, &wl_surface_listener, con);

    con->wl_xdg_surface = xdg_wm_base_get_xdg_surface(con->wl_xdg, con->wl_surface);
    assert(con->wl_xdg_surface);
    xdg_surface_add_listener(con->wl_xdg_surface, &xdg_surface_listener, con);
    con->wl_toplevel = xdg_surface_get_toplevel(con->wl_xdg_surface);
    assert(con->wl_toplevel);
    xdg_toplevel_add_listener(con->wl_toplevel, &xdg_toplevel_listener, con);

    xdg_toplevel_set_title(con->wl_toplevel, "Emulator console");
    xdg_toplevel_set_app_id(con->wl_toplevel, "sh.alba.cursed_gpu_linux");
    xdg_toplevel_set_min_size(con->wl_toplevel, MIN_SIZE[0], MIN_SIZE[1]);
    con->wl_egl_window = wl_egl_window_create(con->wl_surface, INITIAL_SIZE[0], INITIAL_SIZE[1]);
    assert(con->wl_egl_window);

    if (con->wl_fractional_scale_manager && con->wl_viewporter) {
        con->wl_fractional_scale = wp_fractional_scale_manager_v1_get_fractional_scale(con->wl_fractional_scale_manager, con->wl_surface);
        assert(con->wl_fractional_scale);
        wp_fractional_scale_v1_add_listener(con->wl_fractional_scale, &wp_fractional_scale_listener, con);
    }

    if (con->wl_viewporter) {
        con->wl_viewport = wp_viewporter_get_viewport(con->wl_viewporter, con->wl_surface);
        assert(con->wl_viewport);
    }

    if (con->wl_xdg_decoration) {
        con->wl_decoration = zxdg_decoration_manager_v1_get_toplevel_decoration(con->wl_xdg_decoration, con->wl_toplevel);
        assert(con->wl_decoration);
        zxdg_toplevel_decoration_v1_add_listener(con->wl_decoration, &xdg_decoration_listener, con);
        zxdg_toplevel_decoration_v1_set_mode(con->wl_decoration, ZXDG_TOPLEVEL_DECORATION_V1_MODE_SERVER_SIDE);
    } else {
        fprintf(stderr,
            "You are using one of the only compositors that don't implement server-side decorations, "
            "likely because it refuses to do so. You will thus have no decorations. Sorry.\n");
    }

    wl_surface_commit(con->wl_surface);
    while (!con->xdg_configure_done || !(!con->wl_xdg_decoration || con->xdg_decoration_configure_done))
        __checkerrno(wl_display_roundtrip(con->wl_display) < 0, "wl_display_roundtrip");
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

void console_make_current(console_t* con) {
    eglMakeCurrent(con->egl_display, con->egl_surface, con->egl_surface, con->egl_context);
    check_egl_error("eglMakeCurrent");
}

static void init_egl(console_t* con) {
    gladLoaderLoadEGL(EGL_NO_DISPLAY);
    assert(GLAD_EGL_VERSION_1_0);
    assert(GLAD_EGL_EXT_platform_base);
    assert(GLAD_EGL_EXT_platform_wayland || GLAD_EGL_KHR_platform_wayland);

    con->egl_display = eglGetPlatformDisplayEXT(EGL_PLATFORM_WAYLAND_KHR, con->wl_display, NULL);
    check_egl_error("eglGetPlatformDisplayEXT");
    eglInitialize(con->egl_display, NULL, NULL);
    check_egl_error("eglInitialize");

    gladLoaderLoadEGL(con->egl_display);
    assert(GLAD_EGL_VERSION_1_4);
    assert(GLAD_EGL_KHR_create_context);
    assert(GLAD_EGL_KHR_image || GLAD_EGL_KHR_image_base);
    assert(GLAD_EGL_EXT_image_dma_buf_import);
    assert(GLAD_EGL_EXT_image_dma_buf_import_modifiers);

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

    con->egl_surface = eglCreatePlatformWindowSurfaceEXT(con->egl_display, configs[0], con->wl_egl_window, NULL);
    check_egl_error("eglCreatePlatformWindowSurfaceEXT");
    eglBindAPI(EGL_OPENGL_API);
    check_egl_error("eglBindAPI");
    DEFINE_EGL_ATTRS(EGLint, context_attrs, 2);
    ADD_EGL_ATTR(context_attrs, EGL_CONTEXT_MAJOR_VERSION_KHR, 3);
    ADD_EGL_ATTR(context_attrs, EGL_CONTEXT_MINOR_VERSION_KHR, 2);
    con->egl_context = eglCreateContext(con->egl_display, configs[0], EGL_NO_CONTEXT, context_attrs);
    check_egl_error("eglCreateContext");
}

// GL SETUP + IMPORTING + THREAD SYNC

#define IDENTITY_3x2 { \
    1,0,0, \
    0,1,0, }

const char vertex_shader_src[] = {
#embed "shaders/quad.glsl"
};
const char blit_fragment_shader_src[] = {
#embed "shaders/blit.glsl"
};
const char bg_fragment_shader_src[] = {
#embed "shaders/bg.glsl"
};

static void check_gl_error(const char* msg) {
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        fprintf(stderr, "OpenGL error on %s: %#x\n", msg, error);
        exit(2);
    }
}

#define COMPILE_SHADER(VAR, TYPE) compile_shader(#VAR, VAR, sizeof(VAR), TYPE)

static GLuint compile_shader(const char* name, const char* src, GLint src_len, GLenum type) {
    GLuint shader = glCreateShader(type);
    check_gl_error("glCreateShader");
    glShaderSource(shader, 1, &src, &src_len);
    check_gl_error("glShaderSource");
    glCompileShader(shader);
    check_gl_error("glCompileShader");

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        GLsizei log_size;
        glGetShaderInfoLog(shader, sizeof(log), &log_size, log);
        check_gl_error("glGetShaderInfoLog");
        fprintf(stderr, "Shader %s compilation failed. Log:\n", name);
        fwrite(log, sizeof(*log), log_size, stdout);
        exit(2);
    }
    return shader;
}

static GLuint link_program(GLuint vertex_shader, GLuint fragment_shader) {
    GLuint program = glCreateProgram();
    check_gl_error("glCreateProgram");
    glAttachShader(program, vertex_shader);
    check_gl_error("glAttachShader vertex");
    glAttachShader(program, fragment_shader);
    check_gl_error("glAttachShader fragment");
    glLinkProgram(program);
    check_gl_error("glLinkProgram");

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        GLsizei log_size;
        glGetProgramInfoLog(program, sizeof(log), &log_size, log);
        check_gl_error("glGetProgramInfoLog");
        fprintf(stderr, "Program link failed. Log:\n");
        fwrite(log, sizeof(*log), log_size, stdout);
        exit(2);
    }
    return program;
}

static inline GLint get_uniform_location(GLuint prog, const char *name) {
    GLint res = glGetUniformLocation(prog, name);
    check_gl_error("glGetUniformLocation");
    assert(res != -1);
    return res;
}

static void init_gl(console_t* con) {
    gladLoaderLoadGL();
    assert(GLAD_GL_VERSION_3_2);
    assert(GLAD_GL_EXT_EGL_image_storage);

    eglSwapInterval(con->egl_display, 0);
    check_egl_error("eglSwapInterval(0)");

    GLuint v_shader = COMPILE_SHADER(vertex_shader_src, GL_VERTEX_SHADER);

    con->_prog_blit = link_program(v_shader, COMPILE_SHADER(blit_fragment_shader_src, GL_FRAGMENT_SHADER));
    glUseProgram(con->_prog_blit);
    check_gl_error("glUseProgram");
    con->_attr_blit_model_mat = get_uniform_location(con->_prog_blit, "modelMat");
    con->_attr_blit_texture_mat = get_uniform_location(con->_prog_blit, "textureMat");
    glUniform1i(get_uniform_location(con->_prog_blit, "_texture"), 0);
    check_gl_error("glUniform1i");
    glActiveTexture(GL_TEXTURE0);
    check_gl_error("glActiveTexture(GL_TEXTURE0)");

    con->_prog_bg = link_program(v_shader, COMPILE_SHADER(bg_fragment_shader_src, GL_FRAGMENT_SHADER));
    glUseProgram(con->_prog_bg);
    check_gl_error("glUseProgram");
    con->_attr_bg_cell_size = get_uniform_location(con->_prog_bg, "bgCellSize");
    con->_attr_bg_cell_offset = get_uniform_location(con->_prog_bg, "bgCellOffset");
    GLfloat model_mat [3*2] = IDENTITY_3x2;
    glUniformMatrix3x2fv(get_uniform_location(con->_prog_bg, "modelMat"), 1, GL_TRUE, model_mat);
    check_gl_error("glUniformMatrix3x2fv");

    // bind a dummy VAO for drawing
    GLuint vao;
    glGenVertexArrays(1, &vao);
    check_gl_error("glGenVertexArrays");
    glBindVertexArray(vao);
    check_gl_error("glBindVertexArray");

    con->do_draw_frames = true;
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
    DEFINE_EGL_ATTRS(EGLint, image_attrs, 4+4*5);
    ADD_EGL_ATTR(image_attrs, EGL_WIDTH, buffer->width);
    ADD_EGL_ATTR(image_attrs, EGL_HEIGHT, buffer->height);
    ADD_EGL_ATTR(image_attrs, EGL_LINUX_DRM_FOURCC_EXT, buffer->drm_format);
    IMPORT_PLANE(0);
    IMPORT_PLANE(1);
    IMPORT_PLANE(2);
    IMPORT_PLANE(3);
    buf->egl_image = eglCreateImageKHR(con->egl_display, EGL_NO_CONTEXT, EGL_LINUX_DMA_BUF_EXT, NULL, image_attrs);
    check_egl_error("eglCreateImageKHR");
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
    if (nrefs == 1 && img) {
        eglDestroyImageKHR(con->egl_display, img);
        check_egl_error("eglDestroyImageKHR");
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
    // FIXME: hack: this should be in emulator.c but i don't want to load GL there
    glFlush();

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
        if (sc->buf->tex_id) {
            glDeleteTextures(1, &sc->buf->tex_id);
            check_gl_error("glDeleteTextures");
        }
        glGenTextures(1, &sc->buf->tex_id);
        check_gl_error("glGenTextures");
        glBindTexture(GL_TEXTURE_2D, sc->buf->tex_id);
        check_gl_error("glBindTexture");

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        check_gl_error("glTexParameteri(GL_TEXTURE_MIN_FILTER, GL_TEXTURE_WRAP_S)");
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        check_gl_error("glTexParameteri(GL_TEXTURE_MIN_FILTER, GL_TEXTURE_WRAP_T)");

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        check_gl_error("glTexParameteri(GL_TEXTURE_MIN_FILTER)");
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        check_gl_error("glTexParameteri(GL_TEXTURE_MAG_FILTER)");

        glEGLImageTargetTexStorageEXT(GL_TEXTURE_2D, sc->buf->egl_image, NULL);
        check_gl_error("glEGLImageTargetTexStorageEXT");

        eglDestroyImageKHR(con->egl_display, sc->buf->egl_image);
        check_egl_error("eglDestroyImageKHR");
        sc->buf->egl_image = NULL;
        sc->buf->is_bound = true;
    }

    draw_frame(con);
}

static void draw_frame(console_t* con) {
    console_scanout_flush_t* sc = &con->current_flush;
    window_state_t* st = &con->configured_state;

    // window size (surface coordinates)
    int32_t window_w =
        st->width ? st->width :
        st->bounds_width && st->bounds_width < INITIAL_SIZE[0] ? st->bounds_width :
        INITIAL_SIZE[0];
    int32_t window_h =
        st->height ? st->height :
        st->bounds_height && st->bounds_height < INITIAL_SIZE[1] ? st->bounds_height :
        INITIAL_SIZE[1];
    assert(window_w > 0 && window_h > 0);

    // render size (FIXME: check correct rounding according to fractional scaling protocol)
    double pixel_scale = st->scale / 120.;
    size_t w = round(window_w * pixel_scale);
    size_t h = round(window_h * pixel_scale);
    wl_egl_window_resize(con->wl_egl_window, w, h, 0, 0);
    glViewport(0, 0, w, h);

    // communicate window size to compositor
    if (con->wl_fractional_scale)
        wp_viewport_set_destination(con->wl_viewport, window_w, window_h);
    else
        wl_surface_set_buffer_scale(con->wl_surface, st->scale / 120);
    struct wl_region* region = wl_compositor_create_region(con->wl_compositor);
    assert(region);
    wl_region_add(region, 0, 0, window_w, window_h);
    wl_surface_set_input_region(con->wl_surface, region);
    wl_surface_set_opaque_region(con->wl_surface, region);
    wl_region_destroy(region);

    glUseProgram(con->_prog_bg);
    check_gl_error("glUseProgram");
    glUniform1f(con->_attr_bg_cell_size, 11 * pixel_scale);
    check_gl_error("glUniform1f");
    glUniform2f(con->_attr_bg_cell_offset, w/2., h/2.);
    check_gl_error("glUniform2f");
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    check_gl_error("glDrawArrays");

    if (!sc->buf) {
        xdg_toplevel_set_title(con->wl_toplevel, "Emulator (no output)");
        eglSwapBuffers(con->egl_display, con->egl_surface);
        check_egl_error("eglSwapBuffers");
        return;
    }

    GLfloat model_mat [3*2] = IDENTITY_3x2;
    GLfloat texture_mat [3*2] = IDENTITY_3x2;
    GLfloat buf_w = sc->buf->width, buf_h = sc->buf->height;
    texture_mat[0] = sc->viewport.w / buf_w;
    texture_mat[2] = sc->viewport.x / buf_w;
    texture_mat[4] = sc->viewport.h / buf_h;
    texture_mat[5] = sc->viewport.y / buf_h;
    double scale_fit = fmin(w / (double)sc->viewport.w, h / (double)sc->viewport.h);
    double scale = fmin(scale_fit, pixel_scale);
    model_mat[0] = sc->viewport.w * scale / w;
    model_mat[4] = sc->viewport.h * scale / h;
    model_mat[2] = (1 - model_mat[0]) / 2;
    model_mat[5] = (1 - model_mat[4]) / 2;
    for (size_t i = 0; i < 3; i++) model_mat[3+i] *= -1;
    model_mat[3+2] += 1;

    glUseProgram(con->_prog_blit);
    check_gl_error("glUseProgram");
    glUniformMatrix3x2fv(con->_attr_blit_model_mat, 1, GL_TRUE, model_mat);
    check_gl_error("glUniformMatrix4fv");
    glUniformMatrix3x2fv(con->_attr_blit_texture_mat, 1, GL_TRUE, texture_mat);
    check_gl_error("glUniformMatrix3x2fv");
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    check_gl_error("glDrawArrays");

    char buf [128];
    snprintf(buf, sizeof(buf), "Emulator (%u×%u, %d%%)", sc->viewport.w, sc->viewport.h, (int)(scale * 100));
    xdg_toplevel_set_title(con->wl_toplevel, buf);
    eglSwapBuffers(con->egl_display, con->egl_surface);
    check_egl_error("eglSwapBuffers");
}
