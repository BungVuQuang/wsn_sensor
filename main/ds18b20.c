#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

void ds18b20_init(void)
{
    gpio_pad_select_gpio(GPIO_NUM_32);
}

uint8_t DS18B20_Start(void)
{
    uint8_t Response = 0;
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT); // set the pin as output
    gpio_set_level(GPIO_NUM_32, 0);                    // pull the pin low
    ets_delay_us(500);                                 // delay according to datasheet
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);  // set the pin as input
    ets_delay_us(80);                                  // delay according to datasheet

    if (!(gpio_get_level(GPIO_NUM_32)))
        Response = 1; // if the pin is low i.e the presence pulse is detected
    else
        Response = -1;

    ets_delay_us(500); // 480 us delay totally.
    return Response;
}

void DS18B20_Write(uint8_t data)
{
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT); // set the pin as output
    for (int i = 0; i < 8; i++)
    {

        if ((data & (1 << i)) != 0) // if the bit is high
        {
            // write 1
            gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT); // set the pin as output
            gpio_set_level(GPIO_NUM_32, 0);                    // pull the pin low
            ets_delay_us(2);                                   // wait for 1 us
            gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);  // set the pin as input
            ets_delay_us(60);                                  // wait for 60 us
        }

        else // if the bit is low
        {
            // write 0
            gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT); // set the pin as output
            gpio_set_level(GPIO_NUM_32, 0);                    // pull the pin low
            ets_delay_us(60);                                  // wait for 60 us
            gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);  // set the pin as input
        }
    }
}

uint8_t DS18B20_Read(void)
{
    uint8_t value = 0;
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT); // set the pin as input
    for (int i = 0; i < 8; i++)
    {
        gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT); // set the pin as output
        gpio_set_level(GPIO_NUM_32, 0);                    // pull the pin lowull the data pin LOW
        ets_delay_us(2);                                   // wait for > 1us
        gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);  // set the pin as input
        if ((gpio_get_level(GPIO_NUM_32)))                 // if the pin is HIGH
        {
            value |= 1 << i; // read = 1
        }
        ets_delay_us(50); // wait for 60 us
    }
    return value;
}
