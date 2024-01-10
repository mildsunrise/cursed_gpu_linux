all: linux_dtb emulator

# emulator

DT_CFLAGS = -DCLOCK_FREQ=45000000
CFLAGS = -flto -O3 -g -Wall -Wextra
LDFLAGS =

# use VirGL Renderer to expose a VGPU
DT_CFLAGS += -DUSE_VIRGLRENDERER
CFLAGS += -Ivirglrenderer/src
LDFLAGS += -Lvirglrenderer/build/src -lvirglrenderer

CFLAGS += $(DT_CFLAGS)

core.o: core.c core.h riscv_constants.h
	gcc $(CFLAGS) -c $< -o $@
emulator.o: emulator.c core.h measure.c reg_macros.h riscv_constants.h virtio_constants.h
	gcc $(CFLAGS) -c $< -o $@
emulator: core.o emulator.o
	gcc $(CFLAGS) $^ $(LDFLAGS) -o $@

core_test: core.c core.h test.c measure.c reg_macros.h riscv_constants.h
	gcc $(CFLAGS) core.c test.c -o $@

clean:
	rm -f *.o linux_dtb{,.*} emulator core_test

# kernel

LINUX_DIR=linux
LINUX_IMAGE=$(LINUX_DIR)/arch/riscv/boot/Image
LINUX_DTC=$(LINUX_DIR)/scripts/dtc/dtc

$(LINUX_IMAGE): linux_config
	cp linux_config linux/.config
	cd linux && CROSS_COMPILE=riscv32-unknown-linux-gnu- ARCH=riscv make Image

# dtb

LINUX_DTC=$(LINUX_DIR)/scripts/dtc/dtc

$(LINUX_DTC):
	make scripts_dtc

DTC_FLAGS=-Wno-interrupt_provider
# extra warnings
DTC_FLAGS += -Wno-unit_address_vs_reg \
        -Wno-avoid_unnecessary_addr_size \
        -Wno-alias_paths \
        -Wno-graph_child_address \
        -Wno-simple_bus_reg \
        -Wno-unique_unit_address
DTC_FLAGS += -Wnode_name_chars_strict \
        -Wproperty_name_chars_strict \
        -Winterrupt_provider

DTC_INCLUDE=$(LINUX_DIR)/scripts/dtc/include-prefixes $(LINUX_DIR)/arch/riscv/boot/dts/canaan

linux_dtb: linux_dts $(LINUX_DTC)
	cc -E -Wp,-MMD,$@.dep.cpp -undef -D__DTS__ -nostdinc -x assembler-with-cpp \
		$(addprefix -I,$(DTC_INCLUDE)) $(DT_CFLAGS) \
		-o $@.tmp $<
	$(LINUX_DTC) -b 0 \
		$(addprefix -i,$(DTC_INCLUDE)) \
		$(DTC_FLAGS) -d $@.dep.dtc -o $@ $@.tmp
	cat $@.dep.cpp $@.dep.dtc > $@.dep


# (make sure to have the riscv toolchain from
# https://github.com/riscv-collab/riscv-gnu-toolchain/releases
# in path)
