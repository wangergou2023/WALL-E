export RK_ARCH :=arm64
export RK_CROSS := aarch64-linux-gnu-
export RK_CONFIG := lubancat2_defconfig
export RK_KERNEL_DTS :=rk356x-lubancat-rk_series
export RK_SOURCE_DIR=$(shell pwd)/../src


all:
	# ${MAKE} -C ../src ARCH=$(RK_ARCH) CROSS_COMPILE=$(RK_CROSS) $(RK_CONFIG)
	# ${MAKE} -C ../src ARCH=$(RK_ARCH) CROSS_COMPILE=$(RK_CROSS) bindeb-pkg RK_KERNEL_DTS=$(RK_KERNEL_DTS)
	# ${MAKE} -C ../src ARCH=$(RK_ARCH) CROSS_COMPILE=$(RK_CROSS) $(RK_KERNEL_DTS).img
	# ${MAKE} -C ../src ARCH=$(RK_ARCH) CROSS_COMPILE=$(RK_CROSS) dtbs
	chmod 777 $(RK_SOURCE_DIR)/extboot.sh
	cd $(RK_SOURCE_DIR);./extboot.sh 

clean:
	@$(MAKE) -C $(RK_SOURCE_DIR)/ distclean
	-@rm ../bin/* -rf
	-@rm ../src/arch/arm/boot/Image* -rf
	-@rm ../src/arch/arm/boot/uImage* -rf
	-@rm ../linux-*.buildinfo ../linux-*.changes -rf 
	-@rm ../linux-headers-*.deb ../linux-image-*.deb ../linux-libc-dev*.deb -rf
