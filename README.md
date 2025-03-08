# Standalone RISC-V emulator for Linux

This used to be an attempt to run Linux in a GLSL shader, but as I was first writing it in C (to then port to GLSL) I found that implementing paravirtualized peripherals was more fun for me.

The core emulator is now mature enough to emulate a full startup of Linux, including the userspace.
Specs:

 - RV32IMA
 - S & U modes
 - Rv32 MMU
 - On x64, it performs at about 165 instructions, 60 cycles (per emulated instruction).

The full emulator exposes a few peripherals:

 - 512MiB main memory
 - Standard RISC-V PLIC (32 interrupts, no priority support)
 - Standard SBI, with timer & reset extensions
 - 8250/16550 UART for console, mapped as stdin/stdout
 - virtio-net v2 interface, mapped as TAP interface (no extra features)
 - virtio-gpu supporting only VirGL (through virglrenderer), with a [virtual output](https://tech.lgbt/@mildsunrise/114106697163003943)

[![Interacting with the emulated machine through the console (recording)](https://asciinema.org/a/IFZeNgyV6Glf3TBrc5Nn08QjL.svg)](https://asciinema.org/a/IFZeNgyV6Glf3TBrc5Nn08QjL)

> **Note**: the emulator currently assumes a little-endian host.

## Why

No reason in particular, I just thought it would be rad as hell.
And also let me learn a bit more about emulation, ISAs and kernels in general.

## Structure

 - `core.c`, `core.h`: Part of the emulator that implements the RISC-V core itself, including the MMU.

 - `emulator.c`: Part of the emulator that provides the platform (environment), including memory mapping & peripherals.
   Sets up state, memory and enters into the I/O + emulation loop.
   Calls into `core.c` for the emulation, handling any exceptions, memory accesses or interrupts.

 - `console.c`, `console.h`: UI for the emulator's console. This is run by `emulator.c` in the I/O thread as a sub-eventloop.

 - `linux_config`: Config for the kernel. Nothing too exotic, provided mostly for reference.

 - `buildroot_config`: Buildroot config to build a minimal rootfs for the kernel.
   You can use whatever userspace you like, just specify it when building the kernel.

 - `linux_dts`: Source code for the Device Tree describing the hardware exposed by the emulator.
   It's compiled into `linux_dtb`, which is loaded into memory by the emulator and passed to the kernel.

 - `test.c`: Simple test environment that emulates a user-mode ELF (loads it into memory, provides a stack region) using `core.c`.
   I used this in early stages of the project to verify that the core was working properly.

 - `Makefile`: By any means a serious buildsystem, more like a set of common commands I wanted to automate or note somewhere.

## Usage

To execute the C emulator, you first need a kernel.
And to build the kernel, you first need a userspace and a toolchain.
I did that using [Buildroot](https://buildroot.org), so clone it under `buildroot` in the repo:

~~~ bash
git clone https://github.com/buildroot/buildroot.git -b 2022.05 --depth=1
~~~

Then build using the provided `buildroot_config`:

~~~ bash
cd buildroot
cp ../buildroot_config .config
make
cd ..
~~~

Once the rootfs is built, clone Linux under `linux` in the repo:

~~~ bash
git clone https://github.com/torvalds/linux.git -b v6.13 --depth=1
~~~

Then build using the provided `linux_config` and the toolchain produced by Buildroot:

~~~ bash
PATH="$PWD/buildroot/output/host/bin:$PATH"
export CROSS_COMPILE=riscv32-buildroot-linux-gnu- ARCH=riscv
cd linux
cp ../linux_config .config
make
cd ..
~~~

Then either build the virglrenderer dependency or disable VGPU, as indicated in [Virtual GPU](#virtual-gpu).

Now build the DTB and the emulator:

~~~ bash
make
~~~

And run the emulator (for now you'll need to either run it as root, or set up a TAP interface for guest network beforehand, see [Virtual network](#virtual-network)):

~~~ bash
sudo -E ./emulator
~~~

The path to the kernel is hardcoded in `emulator.c`, change it if needed.
You should see the Linux UART output appear on stdout, and emulator messages will be printed to stderr.

### Virtual UART

To properly interact with the UART, you can expose the emulator's stdin/stdout on a socket with socat:

~~~ bash
sudo -E socat UNIX-LISTEN:emulator-console EXEC:./emulator
~~~

And then in another terminal, attach your favorite terminal emulator to that socket:

~~~ bash
minicom -D unix#emulator-console
~~~

### Virtual network

The emulator exposes a virtual network interface to the guest, backed by a TAP interface in the host.
This cannot be disabled; the emulator will attempt to open a `tap0` interface (the name is also hardcoded,
see `TAP_INTERFACE` in `emulator.c`) so if you wish to run the emulator without elevated privileges,
you'll need to create the TAP interface beforehand with appropriate ownership:

~~~ bash
sudo ip tuntap add tap0 mode tap user "$(id -nu)" group "$(id -ng)"
~~~

Even if you don't plan to actually set up and use this interface, you should at least put it UP so that
packets can flow and the guest is happy (newer Linux versions print repeated warnings when this isn't the case):

~~~ bash
sudo ip link set tap0 up
~~~

### Virtual GPU

This emulator now exposes a virtual GPU to the guest, which means it depends on EGL/OpenGL.

The virtual GPU also has a virtual output, and it uses Wayland + Mesa to present a console window
that shows the contents of the virtual screen, as well as forwarding input events on the window
back to the guest as an input device. This means if you're running headless, with X11 or with
a Nvidia GPU, you're out of luck for now :(

The code is in theory prepared to allow leaving the VGPU out of the build (apply
[this patch](https://github.com/mildsunrise/cursed_gpu_linux/commit/no_graphics_example) to
the `Makefile`). This will also leave out the console, leaving the emulator basically
free of dependencies. However I don't test this very often so possibly other fixes would
be needed...

If you don't do this, you should clone and build the virglrenderer submodule:

~~~ bash
git submodule update --init virglrenderer && cd virlgrenderer
meson setup build
ninja -C build
~~~

The window is currently fixed at 800×600, and will display whatever is output by the guest (which may choose to disobey the recommended size) scaled up or down, without preserving aspect ratio. The window starts displaying a checkerboard pattern when no output is being displayed by the guest (either because it didn't yet set it up, or because it cleared it). It currently does not support client-side decorations, so if you are in Mutter (which refuses to implement server-side decorations) you'll just see a bare window.
