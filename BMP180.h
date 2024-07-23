#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include <stdbool.h>

#define BMP180_I2C_ADDR 0x77 // 7-bit
#define BMP180_MAX_I2C_FREQ (3400 * 1000) // 3.4MHz
#define BMP180_STARTUP_TIME_US (10 * 1000) // 10ms
#define BMP180_CHIP_ID 0x55

#define BMP180_TEMP_CONV_TIME_US 4500 // 4.5ms

enum BMP180_OversamplingSetting {
    BMP180_OSS_ULP_4_5ms = 0,
    BMP180_OSS_STD_7_5ms = 1,
    BMP180_OSS_HR_13_5ms = 2,
    BMP180_OSS_XHR_25_5ms = 3
};

struct BMP180_Platform {
    int (*i2cWriteReg)(uint8_t addr7bit, uint8_t regNum, const uint8_t *data, uint8_t length, uint8_t wait);
    int (*i2cReadReg)(uint8_t addr7bit, uint8_t regNum, uint8_t *data, uint8_t length, int timeout);

    void (*delayUs)(int us);
    void (*debugPrint)(const char *fmt, ...);
};

void BMP180_Init(const struct BMP180_Platform *platform);

bool BMP180_SoftwareReset(void);
bool BMP180_ReadCalibrationData(void);
bool BMP180_ReadChipId(uint8_t *chipId);
bool BMP180_CheckChipId(void);

bool BMP180_ReadTemperaturePressure(enum BMP180_OversamplingSetting pressureOss, float *temp, float *pressure);

float BMP180_CalcAltitude(float pressure);
float BMP180_CalcPressureSeaLevel(float altitude, float pressure);

const char *BMP180_UnitTest(void);

#endif // BMP180_H
