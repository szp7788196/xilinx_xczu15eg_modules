KERN_DIR := /home/szp/work/xilinix/kernel_xilinix

obj-m := reserved-device.o

all:
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C $(KERN_DIR) M=`pwd` modules

clean:
	make -C $(KERN_DIR) M=`pwd` clean
