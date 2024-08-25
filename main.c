#include <pico/stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "main.h"
#include "serial_protocol.h"
#include "calibration_memory.h"
#include "mcp41010.h"

#include "sampler.pio.h"

#define TEST_PIN 15

static uint8_t trigger_flag = 0;
TriggerType trigger_type = RISING_EDGE;
uint8_t force_trigger = 0;

static volatile uint8_t trigger_vector_available = 0;

static Sampler normal_sampler;
static Sampler force_sampler;

uint clk_div;

int main(void)
{
    stdio_init_all();

    setup_IO();
    setup_SPI();
    setup_cal_pin();

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
    clock_gpio_init(CLOCK_PIN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 2); 

    sampler_init(&normal_sampler, 0, pio0);
    sampler_init(&force_sampler, 0, pio0);
    clk_div = 1;

    while(1)
    {
        if(trigger_vector_available)
        {
            if(force_trigger)
            {
                print_samples(force_sampler.capture_buffer, SAMPLE_COUNT, force_trigger);
                free(force_sampler.capture_buffer);
            }
            else
            {
                print_samples(normal_sampler.capture_buffer, SAMPLE_COUNT, force_trigger);
                free(normal_sampler.capture_buffer);
                pio_sm_set_enabled(normal_sampler.pio, normal_sampler.sm, false);
                pio_remove_program(normal_sampler.pio, &normal_trigger_positive_program, normal_sampler.offset);
                normal_sampler.created = 0;
            }
            trigger_vector_available = 0;
        }

        if(trigger_flag && !gpio_get(TRIGGER_PIN))
        {
            trigger(&force_sampler, &normal_sampler, force_trigger);
            trigger_flag = 0;
        }

        char command = (char)getchar_timeout_us(CHARACTER_TIMEOUT);
        switch(command)
        {
            case HIGH_RANGE_COMMAND:
                gpio_put(RANGE_PIN, 0);
                break;
            case LOW_RANGE_COMMAND:
                gpio_put(RANGE_PIN, 1);
                break;
            case RISING_EDGE_TRIGGER_COMMAND:
                trigger_type = RISING_EDGE;
                break;
            case FALLING_EDGE_TRIGGER_COMMAND:
                trigger_type = FALLING_EDGE;
                break;
            case TRIGGER_COMMAND:
                reset_triggers();
                run_trigger();
                break;
            case FORCE_TRIGGER_COMMAND:
                {
                    reset_triggers();
                    force_trigger = 1;
                    trigger(&force_sampler, &normal_sampler, force_trigger);
                    break;
                }
            case TRIGGER_LEVEL_COMMAND:
                {
                    char code_string[MAX_STRING_LENGTH];
                    get_string(code_string);
                    uint8_t pot_code = atoi(code_string);
                    if(pot_code <= MAX_POT_CODE && pot_code >= MIN_POT_CODE) write_pot_code(&pot_code);
                    break;
                }
            case CLOCK_DIV_COMMAND:
                update_clock(force_sampler, normal_sampler); 
                break;
            case STOP_COMMAND:
                {
                    uint32_t interrupt_state = save_and_disable_interrupts();
                    if(force_trigger)
                    {
                        pio_sm_set_enabled(force_sampler.pio, force_sampler.sm, false);
                        dma_channel_abort(force_sampler.dma_channel);
                    } 
                    else
                    {
                        pio_sm_set_enabled(normal_sampler.pio, normal_sampler.sm, false);
                        dma_channel_abort(normal_sampler.dma_channel);
                    }
                    irq_set_enabled(DMA_IRQ_0, false);
                    reset_triggers();
                    restore_interrupts(interrupt_state);
                    break;
                }
            case SET_CAL:
                {
                    char cal_string[MAX_STRING_LENGTH];
                    get_string(cal_string);
                    char* high_range_cal_string = malloc(4*sizeof(char));   
                    char* low_range_cal_string = malloc(4*sizeof(char));
                    memcpy(high_range_cal_string, cal_string, 4*sizeof(char));
                    memcpy(low_range_cal_string, cal_string+4, 4*sizeof(char));
                    uint16_t high_range_cal = atoi(high_range_cal_string);
                    uint16_t low_range_cal = atoi(low_range_cal_string);
                    Calibration_Offsets calibration_offsets; 
                    calibration_offsets.high_range_offset = high_range_cal;
                    calibration_offsets.low_range_offset = low_range_cal;
                    write_calibration_offsets(calibration_offsets);
                    free(high_range_cal_string);
                    free(low_range_cal_string);
                    break;
                }
            case READ_CAL:
                {
                    Calibration_Offsets calibration_offsets = read_calibration_offsets();
                    uint8_t low_high_range_byte = (uint8_t)(calibration_offsets.high_range_offset & 0xFF);
                    uint8_t high_high_range_byte = (uint8_t)((calibration_offsets.high_range_offset >> 8) & 0xFF);
                    uint8_t low_low_range_byte = (uint8_t)(calibration_offsets.low_range_offset & 0xFF);
                    uint8_t high_low_range_byte = (uint8_t)((calibration_offsets.low_range_offset >> 8) & 0xFF);
                    write(1, &low_high_range_byte, sizeof(uint8_t));
                    write(1, &high_high_range_byte, sizeof(uint8_t));
                    write(1, &low_low_range_byte, sizeof(uint8_t));
                    write(1, &high_low_range_byte, sizeof(uint8_t)); 
                    break;
                }
            default:
                // Do nothing
                break;
        }
    }

    // The program should never return. 
    return 1;
}

void sampler_init(Sampler* sampler, uint8_t sampler_number, PIO pio_module)
{
    sampler->created = 0;
    sampler->dma_channel = sampler_number;
    sampler->pio = pio_module;
    sampler->sm = sampler_number;
    sampler->c = malloc(sizeof(pio_sm_config));
    *(sampler->c) = pio_get_default_sm_config();
}

void reset_triggers(void)
{
    trigger_vector_available = 0;
    trigger_flag = 0;
    if(force_trigger)
        dma_hw->ints0 = 1 << force_sampler.dma_channel;
    else
        dma_hw->ints0 = 1 << normal_sampler.dma_channel;
}

void run_trigger(void)
{
    force_trigger = 0;
    if(!gpio_get(TRIGGER_PIN))
        trigger(&force_sampler, &normal_sampler, force_trigger);
    else
        if(!trigger_flag) 
            trigger_flag = 1;
}

void get_string(char* str)
{
    uint8_t i;
    for(i = 0; i < MAX_STRING_LENGTH; i++)
    {
        *(str + i) = (char)getchar();
        if(*(str + i) == '\0') return;
    }
}

void setup_IO(void)
{
    gpio_init(PS_SET_PIN);
    gpio_init(RANGE_PIN);
    gpio_init(CS_PIN);
    gpio_init(TRIGGER_PIN);
    
    gpio_set_dir(PS_SET_PIN, GPIO_OUT);
    gpio_set_dir(RANGE_PIN, GPIO_OUT);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_set_dir(TRIGGER_PIN, GPIO_IN);

    gpio_put(PS_SET_PIN, 1); 
    gpio_put(RANGE_PIN, 0);
    gpio_put(CS_PIN, 1);

    gpio_set_function(CAL_PIN, GPIO_FUNC_PWM);
}

void setup_SPI(void)
{
    spi_init(spi1, SPI_SCK_FREQUENCY);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RX, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX, GPIO_FUNC_SPI);
}

void setup_cal_pin(void)
{
    uint8_t slice_number = pwm_gpio_to_slice_num(CAL_PIN);
    pwm_set_chan_level(slice_number, PWM_CHAN_A, PWM_HIGH_COUNT);
    pwm_set_clkdiv(slice_number, PWM_CLK_DIV);
    pwm_set_enabled(slice_number, 1);
}

void dma_complete_handler(void)
{
    trigger_vector_available = 1; 
    if(force_trigger)
        dma_hw->ints0 = 1 << force_sampler.dma_channel;
    else
        dma_hw->ints0 = 1 << normal_sampler.dma_channel;
}

void arm_sampler(Sampler sampler, uint trigger_pin, bool trigger_level, uint8_t force_trigger)
{
    pio_sm_set_enabled(sampler.pio, sampler.sm, false);
    pio_sm_clear_fifos(sampler.pio, sampler.sm);
    pio_sm_restart(sampler.pio, sampler.sm);

    dma_channel_config c = dma_channel_get_default_config(sampler.dma_channel);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(sampler.pio, sampler.sm, false));

    dma_channel_configure(sampler.dma_channel, &c, sampler.capture_buffer, &sampler.pio->rxf[sampler.sm], SAMPLE_COUNT/4, 1);
    dma_channel_set_irq0_enabled(sampler.dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    
    pio_sm_set_enabled(sampler.pio, sampler.sm, true);
}

void update_clock(Sampler force_sampler, Sampler normal_sampler)
{
    char code_string[MAX_STRING_LENGTH];
    get_string(code_string);
    uint16_t commanded_clock_div = atoi(code_string);   
    if(commanded_clock_div > 0)
    {
        clk_div = commanded_clock_div;
        if(force_sampler.created)
        {
            sm_config_set_clkdiv(force_sampler.c, clk_div);
            pio_sm_set_config(force_sampler.pio, force_sampler.sm, force_sampler.c);
        }
        if(normal_sampler.created)
        {
            sm_config_set_clkdiv(normal_sampler.c, clk_div);
            pio_sm_set_config(normal_sampler.pio, normal_sampler.sm, normal_sampler.c);
        }
    }
}

void trigger(Sampler* force_sampler, Sampler* normal_sampler, uint8_t forced)
{
    uint total_sample_bits = SAMPLE_COUNT*FORCE_TRIGGER_PIN_COUNT;
    int buffer_size_words = total_sample_bits/FIFO_REGISTER_WIDTH;
    if(forced)
    {
        if(normal_sampler->created)
        {
            pio_sm_set_enabled(normal_sampler->pio, normal_sampler->sm, false);
            pio_remove_program(normal_sampler->pio, &normal_trigger_positive_program, normal_sampler->offset);
            normal_sampler->created = 0;
        }
        force_sampler->capture_buffer = malloc(SAMPLE_COUNT);
        if(!force_sampler->created)
        {
            force_sampler->offset = pio_add_program(force_sampler->pio, &force_trigger_program);
            force_trigger_sampler_init(*force_sampler, PIN_BASE, clk_div);
            force_sampler->created = 1;
        }
        arm_sampler(*force_sampler, PIN_BASE, trigger_type, forced);
    }
    else
    {
        if(force_sampler->created)
        {
            pio_sm_set_enabled(force_sampler->pio, force_sampler->sm, false);
            pio_remove_program(force_sampler->pio, &force_trigger_program, force_sampler->offset);
            force_sampler->created = 0;
        }
        normal_sampler->capture_buffer = malloc(SAMPLE_COUNT);
        if(!normal_sampler->created)
        {
            normal_sampler->offset = pio_add_program(force_sampler->pio, &normal_trigger_positive_program);
            normal_trigger_sampler_init(*normal_sampler, PIN_BASE, TRIGGER_PIN, clk_div);
            normal_sampler->created = 1;
        }
        arm_sampler(*normal_sampler, PIN_BASE, trigger_type, forced);
    }
}

void print_samples(uint8_t* sample_buffer, uint sample_buffer_length, uint8_t force_trigger)
{
    char samples[SAMPLE_COUNT];
    uint32_t j;
    for(j = 0; j < SAMPLE_COUNT; j++) samples[j] = 0; 
    for(j = 0; j < FORCE_TRIGGER_PIN_COUNT; j++)
    {
        uint32_t i;
        for(i = 0; i < sample_buffer_length; i++)
        {
            uint bit_index = j + i * FORCE_TRIGGER_PIN_COUNT;
            uint byte_index = bit_index / WORD_SIZE;
            uint byte_mask = 1 << bit_index % WORD_SIZE;
            uint bit = (sample_buffer[byte_index] & byte_mask) != 0;
            samples[i] |= (bit << j);
        } 
    }
    write(1, (char*)sample_buffer, sizeof(sample_buffer));
}