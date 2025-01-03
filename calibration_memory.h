#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>

#include "hardware/flash.h"

typedef struct
{
    uint32_t high_range_offset;
    uint32_t high_range_gain_offset;
    uint32_t low_range_offset;
    uint32_t low_range_gain_offset;
} Calibration_Offsets;

#define BUFFER_LENGTH FLASH_PAGE_SIZE/sizeof(uint32_t)
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

void write_calibration_offsets(Calibration_Offsets calibration_offsets);
Calibration_Offsets read_calibration_offsets(void);

#endif