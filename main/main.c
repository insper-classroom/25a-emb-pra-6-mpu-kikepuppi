#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

QueueHandle_t xQueuePos;
typedef struct {
    int id;    // (0 = yaw, 1 = roll, 2 = click)
    int dados; // Valor do dado
} adc_t;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;
    float acceleration_antiga = 0.0f, yaw_antigo = 0.0f, roll_antigo = 0.0f, first_yaw = 0.0f;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    const float CLICK_BASE = 0.6f;

    while (true) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        float acceleration_total = sqrt((accelerometer.axis.x * accelerometer.axis.x) + (accelerometer.axis.y * accelerometer.axis.y) + (accelerometer.axis.z * accelerometer.axis.z));

        bool click = acceleration_total - acceleration_antiga > CLICK_BASE;

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));      

        if (yaw_antigo == 0) {
            first_yaw = euler.angle.yaw;
        }

        adc_t yaw_data = { 
            .id = 0, 
            .dados = (int)((euler.angle.yaw-first_yaw)*-3) 
        };


        if (fabs(euler.angle.yaw - yaw_antigo) > 0.01) {xQueueSend(xQueuePos, &yaw_data, 1);}
        yaw_antigo = euler.angle.yaw;

        adc_t roll_data = { 
            .id = 1, 
            .dados = (int)(euler.angle.roll) 
        };
        if (fabs(euler.angle.roll - roll_antigo) > 0.01) {xQueueSend(xQueuePos, &roll_data, 1);}
        roll_antigo = euler.angle.roll;

        adc_t click_data = { 
            .id = 2, 
            .dados = click ? 1 : 0 
        };
        if (click) {xQueueSend(xQueuePos, &click_data, 1);}

        acceleration_antiga = acceleration_total;

        vTaskDelay(pdMS_TO_TICKS(10));

    }
}

void inicializar_hardware(void) {
    stdio_init_all();
    uart_init(uart_default, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}

void uart_task(void *parametros) {
    adc_t dados_recebidos;

    while (1) {
        if (xQueueReceive(xQueuePos, &dados_recebidos, 100)) {
            uint8_t byte_1 = ((uint16_t)dados_recebidos.dados) >> 8 & 0xFF;
            uint8_t byte_0 = (uint8_t)dados_recebidos.dados & 0xFF;

            uart_putc_raw(uart_default, dados_recebidos.id);
            uart_putc_raw(uart_default, byte_0);
            uart_putc_raw(uart_default, byte_1);
            uart_putc_raw(uart_default, 0xFF);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {

    inicializar_hardware();

    xQueuePos = xQueueCreate(3, sizeof(adc_t));

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_Task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
