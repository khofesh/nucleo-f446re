CC = arm-none-eabi-gcc
MACHINE = cortex-m4
CFLAGS = -c -mcpu=$(MACHINE) -mthumb -std=gnu11 -Wall -O0
OBJDUMP = arm-none-eabi-objdump

all: main.o led.o stm32_startup.o

dump-startup: stm32_startup.o
	$(OBJDUMP) -h $^

main.o: main.c
	$(CC) $(CFLAGS) $^ -o $@

led.o: led.c
	$(CC) $(CFLAGS) $^ -o $@

stm32_startup.o: stm32_startup.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm *.o