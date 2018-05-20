#ARCH = arm-xilinx-eabi
ARCH = /opt/Xilinx2015/SDK/2015.3/gnu/arm/lin/bin/arm-xilinx-eabi
CC = $(ARCH)-gcc
OBJCOPY = ${ARCH}-objcopy
CFLAGS = -c -g -mcpu=cortex-a9

LD = $(ARCH)-ld

CASM = $(ARCH)-asm
CASM_FLAGS = -g -mcpu=cortex-a9

INCLUDES = -I inc
 
all: boot vectors exceptions mmu cache misc zynq_uart string printk main executable binary
	
boot:
	$(CC) $(CFLAGS) ${INCLUDES} src/boot.S -o src/boot.o
	
vectors:
	$(CC) $(CFLAGS) ${INCLUDES} src/vectors.S -o src/vectors.o
	
exceptions:
	$(CC) $(CFLAGS) ${INCLUDES} src/exceptions.S -o src/exceptions.o
	
mmu:
	$(CC) $(CFLAGS) ${INCLUDES} src/mmu.S -o src/mmu.o

cache:
	$(CC) $(CFLAGS) ${INCLUDES} src/cache.S -o src/cache.o
			
misc:
	$(CC) $(CFLAGS) ${INCLUDES} src/misc.S -o src/misc.o
	
zynq_uart:
	$(CC) $(CFLAGS) ${INCLUDES} src/zynq_uart.c -o src/zynq_uart.o

string:
	$(CC) $(CFLAGS) ${INCLUDES} src/string.c -o src/string.o

printk:
	$(CC) $(CFLAGS) ${INCLUDES} src/printk.c -o src/printk.o
	
main:
	$(CC) $(CFLAGS) ${INCLUDES} src/main.c -o src/main.o
	

executable:
	$(CC) -T src/lscript.ld \
	src/boot.o \
	src/vectors.o \
	src/exceptions.o \
	src/mmu.o \
	src/cache.o \
	src/misc.o \
	src/zynq_uart.o \
	src/string.o \
	src/printk.o \
	src/main.o \
	-o ns_guest.elf
	@echo 'Finished object files'
	rm src/*.o
	@echo 'Finished building'

binary:
	$(OBJCOPY) -O binary ns_guest.elf ns_guest.bin
	
	
clean:
	rm src/*.o *.elf
	@echo 'Finished cleaning'
	







