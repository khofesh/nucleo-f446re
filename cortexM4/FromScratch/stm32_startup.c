#include <stdint.h>

#define SRAM_START 0x20000000U
#define SRAM_SIZE (128 * 1024) // 128KB
#define SRAM_END ((SRAM_START) + (SRAM_SIZE))

#define STACK_START SRAM_END

void Reset_handler();

uint32_t vectors[] = {
    STACK_START,
    (uint32_t)&Reset_handler,
};

void Reset_handler()
{
}
