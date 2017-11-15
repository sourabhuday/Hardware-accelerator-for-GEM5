ARCH=arm
COMPILER=arm-linux-gnueabi-
obj-m:= sam_dev.o
KERNELDIR := /home/sbetiger/LAB3/linux-4.13.9
PWD:= $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) clean
