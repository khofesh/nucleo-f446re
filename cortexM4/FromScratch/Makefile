CC = arm-none-eabi-gcc
MACHINE = cortex-m4
CFLAGS = -c -mcpu=$(MACHINE) -mthumb -std=gnu11 -Wall -O0
OBJDUMP = arm-none-eabi-objdump
LDFLAGS = -nostdlib -T stm32_ls.ld -Wl,-Map=final.map

all: main.o led.o stm32_startup.o final.elf

dump-elf: final.elf
	$(OBJDUMP) -h $^

dump-startup: stm32_startup.o
	$(OBJDUMP) -h $^

final.elf: main.o led.o stm32_startup.o
	$(CC) $(LDFLAGS) *.o -o $@

main.o: main.c
	$(CC) $(CFLAGS) $^ -o $@

led.o: led.c
	$(CC) $(CFLAGS) $^ -o $@

stm32_startup.o: stm32_startup.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf *.o *elf