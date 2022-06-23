#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h> /* Definition of HW_* constants */
#include <sys/syscall.h>         /* Definition of SYS_* constants */
#include <unistd.h>
#include <sys/ioctl.h>

static long perf_event_open(struct perf_event_attr *hw_event, pid_t pid, int cpu, int group_fd, unsigned long flags) {
    return syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
}

static int simple_perf_counter(long long config) {
    struct perf_event_attr pe;
    memset(&pe, 0, sizeof(struct perf_event_attr));
    pe.size = sizeof(struct perf_event_attr);
    pe.type = PERF_TYPE_HARDWARE;
    pe.config = config;
    pe.disabled = 1;
    pe.exclude_kernel = 1;
    pe.exclude_hv = 1;

    int fd = perf_event_open(&pe, 0, -1, -1, 0);
    assert(fd >= 0);
    int res = ioctl(fd, PERF_EVENT_IOC_RESET, 0);
    assert(res >= 0);
    return fd;
}

static long long simple_perf_read(int fd) {
    long long count;
    int res = read(fd, &count, sizeof(count));
    assert(res == sizeof(count));
    return count;
}
