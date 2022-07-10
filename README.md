# Linux in the GPU

This is an attempt at emulating a Linux kernel inside a GLSL shader.

Read the [threads](https://twitter.com/mild_sunrise/status/1538447372392488968) for the full (and very long) story.
I've chosen the RISC-V architecture, 32-bit, without an FPU or even M-mode.
The plan is to first write a C version, but trying to stick only to features that are present in GLSL ES 3.0, to facilitate the port afterwards.
We need GLSL ES 3.0 for its integer support, and also because it has switches.

The core emulator is now mature enough to emulate a full startup of Linux, including the userspace.
Specs:

 - RV32IMA
 - S & U modes
 - Rv32 MMU
 - On x64, it performs at about 165 instructions, 60 cycles (per emulated instruction).

The full emulator exposes a few peripherals:

 - 512MiB main memory
 - Standard RISC-V PLIC (32 interrupts, no priority support)
 - Standard SBI, with the timer extension
 - 8250/16550 UART for console, mapped as stdin/stdout
 - virtio-net v2 interface, mapped as TAP interface (no extra features)

[![Interacting with the emulated machine through the console (recording)](https://asciinema.org/a/IFZeNgyV6Glf3TBrc5Nn08QjL.svg)](https://asciinema.org/a/IFZeNgyV6Glf3TBrc5Nn08QjL)

## Why

No reason in particular, I just thought it would be rad as hell.
And also let me learn a bit more about emulation, ISAs and kernels in general.

## Structure

 - `core.c`, `core.h`: Part of the emulator that implements the RISC-V core itself, including the MMU.

 - `emulator.c`: Part of the emulator that provides the platform (environment), including memory mapping & peripherals.
   Sets up state, memory and enters into the I/O + emulation loop.
   Calls into `core.c` for the emulation, handling any exceptions, memory accesses or interrupts.

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
git clone https://github.com/torvalds/linux.git -b v5.19-rc5 --depth=1
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

Now build the DTB and the emulator:

~~~ bash
make
~~~

And run the emulator:

~~~ bash
./emulator
~~~

The paths to the DTB and the kernel are hardcoded in `emulator.c`, change them if needed.
You should see the Linux UART output appear.
