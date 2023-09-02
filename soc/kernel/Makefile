CROSS := aarch64-linux-
CONFIG := rockchip_linux_defconfig rockchip_linux_bifrost.config

SOURCE_DIR=$(shell pwd)/../src

all:
	${MAKE} -C ../src ARCH=arm64 $(CONFIG) CROSS_COMPILE=$(CROSS)
	${MAKE} -C ../src ARCH=arm64 rk3568-evb1-ddr4-v10-linux.img CROSS_COMPILE=$(CROSS)

	cp -f ../src/zboot.img ../bin/uImage
clean:
	@$(MAKE) -C $(SOURCE_DIR)/ distclean
	-@rm ../bin/* -rf
	-@rm ../src/arch/arm/boot/Image* -rf
	-@rm ../src/arch/arm/boot/uImage* -rf
