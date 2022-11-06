CC = arm-none-eabi-gcc
MACHINE = cortex-m4
CFLAGS = -c -mcpu=$(MACHINE) -mthumb -std=gnu11 -Wall -O0

all: main.o led.o stm32_startup.o

main.o: main.c
	$(CC) $(CFLAGS) $^ -o $@

led.o: led.c
	$(CC) $(CFLAGS) $^ -o $@

stm32_startup.o: stm32_startup.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm *.o *.elf