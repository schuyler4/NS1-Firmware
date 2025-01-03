#include "hardware/flash.h"
#include "hardware/sync.h"

#include "calibration_memory.h"

// TODO: There may be a better way to convert between an array and a struct

void write_calibration_offsets(Calibration_Offsets calibration_offsets)
{
    uint32_t buffer[FLASH_PAGE_SIZE/sizeof(uint32_t)];
    buffer[0] = calibration_offsets.high_range_offset; 
    buffer[1] = calibration_offsets.high_range_gain_offset;
    buffer[2] = calibration_offsets.low_range_offset;
    buffer[3] = calibration_offsets.low_range_gain_offset;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase((PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE);
    flash_range_program((PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE), (uint8_t*)buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

Calibration_Offsets read_calibration_offsets(void)
{
    uint32_t* address = (uint32_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    Calibration_Offsets calibration_offsets;
    calibration_offsets.high_range_offset = *(address + 0);
    calibration_offsets.high_range_gain_offset = *(address + 1);
    calibration_offsets.low_range_offset = *(address + 2);
    calibration_offsets.low_range_gain_offset = *(address + 3);
    return calibration_offsets;
}