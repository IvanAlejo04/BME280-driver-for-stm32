#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "BME_Driver.h"

// ==== MACROs ====//
#define BME_I2C (&hi2c1) // modify this
#define BME_TIM TIM7
#define BME_ADDR 0x77

// ==== temp ==== /
uint16_t dig_T1; // first param
int16_t dig_T2;  // second param
int16_t dig_T3;  // third param

// ==== pres ==== /
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

// ==== humi ==== //
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

// ==== global var ==== //

// ==== dont change ==== //
int32_t t_fine; // fine resolution temp value

// ==== flag ==== //
volatile bool dma_i2c_complete = false;
volatile bool dma_rearm = false;
volatile bool safe_to_rearm = false;
volatile bool proceed_process = false;
volatile bool bme_read = false;

int32_t temperature_raw;
int32_t pressure_raw;
int32_t humidity_raw;

int32_t temperature_final = 0;
int32_t pressure_final = 0;
int32_t humidity_final = 0;

// ==== struct ==== //
// ==== data
typedef struct
{
    uint8_t pres_msb;
    uint8_t pres_lsb;
    uint8_t pres_xlsb;

    uint8_t temp_msb;
    uint8_t temp_lsb;
    uint8_t temp_xlsb;

    uint8_t hum_msb;
    uint8_t hum_lsb;
} __attribute__((packed)) dta;
dta data;

// ==== calibration
typedef struct
{
    // temp
    uint16_t t1;
    int16_t t2;
    int16_t t3;

    // pre
    uint16_t p1;
    int16_t p2;
    int16_t p3;
    int16_t p4;
    int16_t p5;
    int16_t p6;
    int16_t p7;
    int16_t p8;
    int16_t p9;

    uint8_t unused; // 0xA0, the gap
    // hum
    uint8_t h1;
    int16_t h2;
    uint8_t h3;

    int16_t h4_msb : 8;
    uint16_t h4_lsb : 4;

    uint16_t h5_lsb : 4;
    int16_t h5_msb : 8;

    int8_t h6;

} __attribute__((packed)) clbrtn;
clbrtn calb;
_Static_assert(sizeof(clbrtn) == 33, "calibration struct must be 33 bytes");
// ==== buffer ==== //

// ==== data
uint8_t frnt_bfr[8]; // 0xF7 - 0xFE
// ==== calibration
uint8_t cal_bfr[33]; // [0x88 - 0xA1]   [0xE1 - 0xE7] -- two DMA burst from index [0] to [24] then [25] to [33]

// ==== formula ==== //

int32_t BME280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2;
    int32_t T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)(p >> 8);
}

uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                       ((int32_t)dig_H2) +
                   8192) >>
                  14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);
}

void BME280_start()
{

    uint8_t hum_data = 0x01;
    uint8_t meas_data = 0x27;
    uint8_t ID;

    while (HAL_I2C_Mem_Read(BME_I2C, (BME_ADDR << 1), 0xD0, I2C_MEMADD_SIZE_8BIT, &ID, 1, 100) != HAL_OK)
        ;

    if (ID == 0x60) // check if it is BME or BMP
    {
        while (HAL_I2C_Mem_Write(BME_I2C, (BME_ADDR << 1), 0xF2, I2C_MEMADD_SIZE_8BIT, &hum_data, 1, HAL_MAX_DELAY) != HAL_OK)
            ;
        while (HAL_I2C_Mem_Write(BME_I2C, (BME_ADDR << 1), 0xF4, I2C_MEMADD_SIZE_8BIT, &meas_data, 1, HAL_MAX_DELAY) != HAL_OK)
            ;

        HAL_I2C_Mem_Read_DMA(BME_I2C, (BME_ADDR << 1), 0x88, I2C_MEMADD_SIZE_8BIT, cal_bfr, 26);
        while (dma_i2c_complete != true)
            ;
        dma_i2c_complete = false;
        HAL_I2C_Mem_Read_DMA(BME_I2C, (BME_ADDR << 1), 0xE1, I2C_MEMADD_SIZE_8BIT, &cal_bfr[26], 7);
        while (dma_i2c_complete != true)
            ;
        dma_i2c_complete = false;
        memcpy(&calb, cal_bfr, sizeof(cal_bfr)); // calibration buffer transfer to the struct already

        // transfer from struct to variables that is used for compensation math
        dig_T1 = calb.t1;
        dig_T2 = calb.t2;
        dig_T3 = calb.t3;

        dig_P1 = calb.p1;
        dig_P2 = calb.p2;
        dig_P3 = calb.p3;
        dig_P4 = calb.p4;
        dig_P5 = calb.p5;
        dig_P6 = calb.p6;
        dig_P7 = calb.p7;
        dig_P8 = calb.p8;
        dig_P9 = calb.p9;

        dig_H1 = calb.h1;
        dig_H2 = calb.h2;
        dig_H3 = calb.h3;
        dig_H4 = (calb.h4_msb << 4) | calb.h4_lsb;
        dig_H5 = (calb.h5_msb << 4) | calb.h5_lsb;
        dig_H6 = calb.h6;

        // arm DMA to start data transfer
        HAL_Delay(100);
        dma_rearm = true;
        HAL_I2C_Mem_Read_DMA(BME_I2C, (BME_ADDR << 1), 0xF7, I2C_MEMADD_SIZE_8BIT, frnt_bfr, 8);
    }
    else
    {
        while (true)
            ;
    }
}

// CallBack functions:
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == BME_I2C)
    {
        dma_i2c_complete = true;

        if (dma_rearm)
        {
            proceed_process = true;
        }
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BME_TIM)
    {
        if (safe_to_rearm)
        {
            bme_read = true;
        }
    }
}

void BME280_process()
{
    if (proceed_process)
    {
        memcpy(&data, frnt_bfr, sizeof(frnt_bfr)); // copy data

        temperature_raw = (data.temp_msb << 12) | (data.temp_lsb << 4) | (data.temp_xlsb >> 4); // from struct to variable can be put to process
        pressure_raw = (data.pres_msb << 12) | (data.pres_lsb << 4) | (data.pres_xlsb >> 4);    // from struct to variable can be put to process
        humidity_raw = (data.hum_msb << 8) | data.hum_lsb;                                      // from struct to variable can be put to process

        temperature_final = BME280_compensate_T_int32(temperature_raw);
        pressure_final = BME280_compensate_P_int64(pressure_raw);
        humidity_final = bme280_compensate_H_int32(humidity_raw);

        safe_to_rearm = true;
        proceed_process = false;
    }
}

void BME280_read()
{
    if (bme_read)
    {

        if (HAL_I2C_Mem_Read_DMA(BME_I2C, (BME_ADDR << 1), 0xF7, I2C_MEMADD_SIZE_8BIT, frnt_bfr, 8) == HAL_OK)
        {
            safe_to_rearm = false;
        }
        else
        {
            safe_to_rearm = true;
        }
        bme_read = false;
    }
}

int main(void)
{

    BME280_start(); // START TIMER AFTER THIS

    while (true)
    {
        BME280_process(); // process the data you can use global variable to send data or do anything abt it
        BME280_read();
        /*

        you can now use:
        int32_t temperature_final;
        int32_t pressure_final;
        int32_t humidity_final;

        *aslong as you call the process the data will be processed again and again.
        *on the other hand the data will be taken from the sensor based on your sampling rate.
        *the data will arrive even if you dont process it.
        *the safe_to_rearm flag tell the timer that it is safe to initiate new DMA transfer

        */
    }

    return 0;
}
