#ifndef MAIN_H
#define MAIN_H

#define WORD_SIZE 32

#define TRIGGER_PIN_COUNT 9
#define FORCE_TRIGGER_PIN_COUNT 8
#define FIFO_REGISTER_WIDTH 32
#define PIN_BASE 0
#define SAMPLE_BIT_COUNT 8
#define SAMPLE_BIT_MASK 0xFF
#define SAMPLE_COUNT 16384

#define CLOCK_PIN 21
#define PS_NOISE_SET_PIN 23
#define RANGE_PIN 27 
#define GAIN_PIN 28
#define TRIGGER_PIN 8

#define SIMU_WAVEFORM_POINTS 50

#define SPI_SCK_FREQUENCY 10000
#define SPI_SCK 10
#define SPI_RX 12
#define SPI_TX 11
#define CS_PIN 13
#define CAL_PIN 14
#define TRIGGER_ENABLE_PIN 12

#define MAX_STRING_LENGTH 100

#define PWM_HIGH_COUNT 32770
#define PWM_CLK_DIV 2

#define CHARACTER_TIMEOUT 100

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"

typedef enum
{
    FALLING_EDGE,
    RISING_EDGE
} TriggerType;

typedef struct
{
    uint8_t created;
    uint dma_channel;
    uint second_dma_channel;
    PIO pio;
    uint sm;
    pio_sm_config *c;
    uint8_t* capture_buffer;
    uint offset;
    uint clock_div;
    TriggerType trigger_type;
} Sampler;

void setup_IO(void);
void setup_SPI(void);
void sampler_init(Sampler* sampler, uint8_t sampler_number, PIO pio_module);
void update_clock(Sampler force_sampler, Sampler normal_sampler);
uint8_t sampler_pio_init(Sampler sampler, uint8_t pin_base);
uint16_t get_dma_last_index(Sampler sampler);
void arm_sampler(Sampler sampler, uint trigger_pin, uint8_t force_trigger);
void trigger(Sampler* force_sampler, Sampler* normal_sampler, uint8_t forced);
void trigger_callback(uint gpio, uint32_t event_mask);
void transmit_vector(uint16_t* vector, uint16_t point_count);
void get_string(char* str);
void setup_cal_pin(void);
void run_trigger(void);
void initialize_peripherals(void);
void set_cal_command(void);
void read_cal_command(void);
void stop_capture(void);
bool record_callback(struct repeating_timer *t);
void stop_trigger(void);

#endif
