CC = arm-none-eabi-gcc
MACHINE = cortex-m4
CFLAGS = -c -mcpu=$(MACHINE) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -O0
OBJDUMP = arm-none-eabi-objdump
LDFLAGS = -mcpu=$(MACHINE) -mthumb -mfloat-abi=soft --specs=nano.specs -T stm32_ls.ld -Wl,-Map=final.map
# semi-hosting
LDFLAGS_SH = -mcpu=$(MACHINE) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T stm32_ls.ld -Wl,-Map=final.map

all: main.o led.o stm32_startup.o syscalls.o final.elf

semi: main.o led.o stm32_startup.o syscalls.o final_sh.elf

list-ld-symbols: final.elf
	arm-none-eabi-nm $^

dump-elf: final.elf
	$(OBJDUMP) -h $^

dump-startup: stm32_startup.o
	$(OBJDUMP) -h $^

final.elf: main.o led.o stm32_startup.o syscalls.o
	$(CC) $(LDFLAGS) $^ -o $@

final_sh.elf: main.o led.o stm32_startup.o
	$(CC) $(LDFLAGS_SH) $^ -o $@

main.o: main.c
	$(CC) $(CFLAGS) $^ -o $@

led.o: led.c
	$(CC) $(CFLAGS) $^ -o $@

syscalls.o: syscalls.c
	$(CC) $(CFLAGS) $^ -o $@

stm32_startup.o: stm32_startup.c
	$(CC) $(CFLAGS) $^ -o $@

flash:	
	openocd -f /home/fahmad/GitHub/OpenOCD/tcl/board/st_nucleo_f4.cfg

clean:
	rm -rf *.o *elf