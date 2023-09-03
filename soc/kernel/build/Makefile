CROSS := aarch64-linux-gnu-
CONFIG := lubancat2_defconfig

SOURCE_DIR=$(shell pwd)/../src

all:
	${MAKE} -C ../src ARCH=arm64 $(CONFIG) CROSS_COMPILE=$(CROSS)
	${MAKE} -C ../src ARCH=arm64 rk356x-lubancat-rk_series.img CROSS_COMPILE=$(CROSS)

clean:
	@$(MAKE) -C $(SOURCE_DIR)/ distclean
	-@rm ../bin/* -rf
	-@rm ../src/arch/arm/boot/Image* -rf
	-@rm ../src/arch/arm/boot/uImage* -rf
