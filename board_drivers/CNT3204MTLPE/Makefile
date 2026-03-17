MAKE+=--no-print-directory
obj-m += CNT3204MT.o
DIR:=$(shell pwd)
EXTRA_CFLAGS += -O -Wall -DMODULE -D__KERNEL__ -I/usr/realtime/include

all:
	@$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(DIR) modules

clean:
	@$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(DIR) clean
