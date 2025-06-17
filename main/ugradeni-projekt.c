#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "dht11.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

// MQ135 digital pin
#define MQ_digital GPIO_NUM_13
// MQ135 analog pin
#define MQ_analog ADC1_CHANNEL_3

// I2C prvi uredaj -> zato sto nema niti jednog drugog onda je on prvi
#define I2C_PORT I2C_NUM_0
// Data i clock pin-ovi
#define I2C_SDA 21
#define I2C_SCL 22
// Frekvencija komunikacije
#define I2C_FREQ 100000
// Adresa display-a, defaultna = 0x27
#define LCD_ADDR 0x27

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_RS 0x01

#define SERVO_PIN GPIO_NUM_18
#define SERVO_MIN 1000
#define SERVO_MAX 2000
#define SERVO_TIMER LEDC_TIMER_0
#define SERVO_CHANNEL LEDC_CHANNEL_0

// delay funkcija

void delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void lcd_write(uint8_t data)
{
    // Uspostavi i2c vezu
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Zapocni zapis u cmd sekvencu
    i2c_master_start(cmd);
    // Shiftaj adresu lcd-a u lijevo i zapisi (0 je write bit), true -> ocekuj odgovor od uredaja
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | 0, true);
    // Zapisi potreban data na uredaj
    i2c_master_write_byte(cmd, data, true);
    // Zaustavi zapis u cmd sekvencu
    i2c_master_stop(cmd);
    // Zapocni komunikaciju
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    // Oslobodi memoriju nakon zavrsene komunikacije
    i2c_cmd_link_delete(cmd);
}

// slanje 4 bita
void lcd_send_nibble(uint8_t nib, uint8_t mode)
{
    uint8_t data = (nib & 0xF0) | mode | LCD_BACKLIGHT;
    lcd_write(data | LCD_ENABLE);
    esp_rom_delay_us(1);
    lcd_write(data & ~LCD_ENABLE);
    esp_rom_delay_us(50);
}

// slanje 8 bita
void lcd_send_byte(uint8_t val, uint8_t mode)
{
    lcd_send_nibble(val & 0xF0, mode);
    lcd_send_nibble((val << 4) & 0xF0, mode);
}

void lcd_cmd(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
}

void lcd_char(char c)
{
    lcd_send_byte(c, LCD_RS);
}

void lcd_init()
{
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_nibble(0x30, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x30, 0);
    esp_rom_delay_us(150);
    lcd_send_nibble(0x30, 0);
    lcd_send_nibble(0x20, 0); // 4-bit mode

    lcd_cmd(0x28); // 4-bit, 2-line
    lcd_cmd(0x0C); // display ON
    lcd_cmd(0x06); // entry mode
    lcd_cmd(0x01); // clear
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    const uint8_t offsets[] = {0x00, 0x40};
    lcd_cmd(0x80 | (col + offsets[row]));
}

void lcd_print(const char *s)
{
    while (*s)
        lcd_char(*s++);
}

void servoWrite(int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    int pulse_width = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * angle) / 180;
    int duty = (pulse_width * 65535) / 20000;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL);
}

void app_main(void)
{

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

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = I2C_FREQ,
    };

    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0);

    lcd_init();

    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};

    ledc_timer_config(&timer_config);

    ledc_channel_config_t channel_config = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0};

    ledc_channel_config(&channel_config);

    while (1)
    {
        // Dohvati trenutnu analognu vrijednost od MQ135
        int MQreadAnalog = adc1_get_raw(MQ_analog);
        // Dohvati trenutnu digitalnu vrijednost od MQ135
        int MQreadDigital = gpio_get_level(MQ_digital);
        // Dohvati trenutnu vrijednost temperature i vlage sa DHT11
        temperature = DHT11_read().temperature;
        humidity = DHT11_read().humidity;
        int airQ = (MQreadAnalog * 100) / 4096;
        char tempStr[3];
        char humStr[2];
        char airqStr[10];

        sprintf(tempStr, "%d", temperature);
        sprintf(humStr, "%d", humidity);
        sprintf(airqStr, "%d", airQ);

        printf("%s\n", airqStr);

        lcd_set_cursor(0, 0);
        lcd_print("Temp:");
        lcd_print(tempStr);
        lcd_set_cursor(9, 0);
        lcd_print("Hum:");
        lcd_print(humStr);
        lcd_set_cursor(0, 1);
        lcd_print("Air Q: ");
        lcd_set_cursor(7, 1);
        lcd_print(airqStr);
        lcd_print("%");

        servoWrite(180);

        delay(500);
    }
}