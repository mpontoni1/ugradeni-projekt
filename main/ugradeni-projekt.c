#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "dht11.h"

// delay funkcija

void delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void app_main(void)
{
// MQ135 digital pin
#define MQ_digital GPIO_NUM_13
// MQ135 analog pin
#define MQ_analog ADC1_CHANNEL_3

    int humidity = 0;
    int temperature = 0;

    // DHT11 pin
    DHT11_init(GPIO_NUM_4);

    // Postavi MQ135 digital pin kao input
    gpio_set_direction(MQ_digital, GPIO_MODE_INPUT);

    // Ocitavaj analognu vrijednost MQ135 u rasponu od 0 do 4096
    adc1_config_width(ADC_WIDTH_BIT_12);
    // Ocitavaj raspon od 0 do 3.3v na analognom ulazu od MQ135
    adc1_config_channel_atten(MQ_analog, ADC_ATTEN_DB_11);

    while (1)
    {
        // Dohvati trenutnu analognu vrijednost od MQ135
        int MQreadAnalog = adc1_get_raw(MQ_analog);
        // Dohvati trenutnu digitalnu vrijednost od MQ135
        int MQreadDigital = gpio_get_level(MQ_digital);
        // Dohvati trenutnu vrijednost temperature i vlage sa DHT11
        temperature = DHT11_read().temperature;
        humidity = DHT11_read().humidity;

        printf("%d %d %d\n", temperature, humidity, MQreadAnalog);
        delay(500);
    }
}