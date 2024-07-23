#include <stddef.h>
#include <math.h>
#include "MinUnit.h"
#include "BMP180.h"

#define OUT_XLSB_REG 0xF8
#define OUT_LSB_REG 0xF7
#define OUT_MSB_REG 0xF6
#define CTRL_MEAS_REG 0xF4
#define SOFT_RESET_REG 0xE0
#define ID_REG 0xD0

#define CALIB_AC1_REG_MSB 0xAA
#define CALIB_AC1_REG_LSB 0xAB
#define CALIB_AC2_REG_MSB 0xAC
#define CALIB_AC2_REG_LSB 0xAD
#define CALIB_AC3_REG_MSB 0xAE
#define CALIB_AC3_REG_LSB 0xAF
#define CALIB_AC4_REG_MSB 0xB0
#define CALIB_AC4_REG_LSB 0xB1
#define CALIB_AC5_REG_MSB 0xB2
#define CALIB_AC5_REG_LSB 0xB3
#define CALIB_AC6_REG_MSB 0xB4
#define CALIB_AC6_REG_LSB 0xB5
#define CALIB_B1_REG_MSB 0xB6
#define CALIB_B1_REG_LSB 0xB7
#define CALIB_B2_REG_MSB 0xB8
#define CALIB_B2_REG_LSB 0xB9
#define CALIB_MB_REG_MSB 0xBA
#define CALIB_MB_REG_LSB 0xBB
#define CALIB_MC_REG_MSB 0xBC
#define CALIB_MC_REG_LSB 0xBD
#define CALIB_MD_REG_MSB 0xBE
#define CALIB_MD_REG_LSB 0xBF

#define SOFT_RESET_VALUE 0xB6

#define MEAS_CTRL_SCO_TEMP 0x2E // start of conversion - temperature
#define MEAS_CTRL_SCO_PRESS 0x34 // start of conversion - pressure

// Calibration values, should be read from chip using BMP180_ReadCalibration() function
static short ac1;
static short ac2;
static short ac3;
static unsigned short ac4;
static unsigned short ac5;
static unsigned short ac6;
static short b1;
static short b2;
static short mb;
static short mc;
static short md;

static struct BMP180_Platform platform = { 0 };

static bool readCalibReg(uint8_t regMsb, uint8_t regLsb, short *valueShort, unsigned short *valueUShort) {
    int ret;

    // Read MSB
    uint8_t msb[1];
    ret = platform.i2cReadReg(BMP180_I2C_ADDR, regMsb, msb, sizeof(msb), 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 calibration register MSB: %d\r\n", -ret);
        return false;
    }

    // Read LSB
    uint8_t lsb[1];
    ret = platform.i2cReadReg(BMP180_I2C_ADDR, regLsb, lsb, sizeof(lsb), 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 calibration register LSB: %d\r\n", -ret);
        return false;
    }

    // Combine MSB and LSB
    if (valueShort) {
        *valueShort = (short)((msb[0] << 8) | lsb[0]);
    }

    if (valueUShort) {
        *valueUShort = (unsigned short)((msb[0] << 8) | lsb[0]);
    }

    return true;
}

static bool readUncompensatedTemp(long *ut) {
    int ret;

    // Read MSB
    uint8_t msb[1];
    ret = platform.i2cReadReg(BMP180_I2C_ADDR, OUT_MSB_REG, msb, sizeof(msb), 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 uncompensated temperature MSB: %d\r\n", -ret);
        return false;
    }

    // Read LSB
    uint8_t lsb[1];
    ret = platform.i2cReadReg(BMP180_I2C_ADDR, OUT_LSB_REG, lsb, sizeof(lsb), 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 uncompensated temperature LSB: %d\r\n", -ret);
        return false;
    }

    *ut = (msb[0] << 8) | lsb[0];

    return true;
}

static bool readUncompensatedPressure(long *up, int oss) {
    int ret;

    // Read MSB
    uint8_t msb[1];
    ret = platform.i2cReadReg(BMP180_I2C_ADDR, OUT_MSB_REG, msb, sizeof(msb), 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 uncompensated pressure MSB: %d\r\n", -ret);
        return false;
    }

    // Read LSB
    uint8_t lsb[1];
    ret = platform.i2cReadReg(BMP180_I2C_ADDR, OUT_LSB_REG, lsb, sizeof(lsb), 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 uncompensated pressure LSB: %d\r\n", -ret);
        return false;
    }

    // Read XLSB
    uint8_t xlsb[1];
    ret = platform.i2cReadReg(BMP180_I2C_ADDR, OUT_XLSB_REG, xlsb, sizeof(xlsb), 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 uncompensated pressure XLSB: %d\r\n", -ret);
        return false;
    }

    *up = ((msb[0] << 16) | (lsb[0] << 8) | xlsb[0]) >> (8 - oss);

    return true;
}

static void calcTrueTemperaturePressure(long ut, long up, int oss, float *temperature, float *pressure) {
    // Calculate true temperature
    long x1 = ((ut - ac6) * ac5) >> 15;
    long x2 = (mc << 11) / (x1 + md);
    long b5 = x1 + x2;
    long t = (b5 + 8) >> 4; // temp in 0.1 deg C
    
    *temperature = t / 10.0f; // convert to deg C

    // Calculate true pressure
    long b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    long x3 = x1 + x2;
    long b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    unsigned long b4 = ac4 * (unsigned long)(x3 + 32768) >> 15;
    unsigned long b7 = ((unsigned long)up - b3) * (50000 >> oss);

    long p;
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4); // pressure in Pa

    *pressure = p / 100.0f; // convert to mbar
}

void BMP180_Init(const struct BMP180_Platform *platformPtr) {
    platform = *platformPtr;
}

bool BMP180_SoftwareReset(void) {
    uint8_t reg[1] = { SOFT_RESET_VALUE };
    int ret = platform.i2cWriteReg(BMP180_I2C_ADDR, SOFT_RESET_REG, reg, sizeof(reg), 1);
    if (ret < 0) {
        platform.debugPrint("Error writing BMP180 software reset register: %d\r\n", -ret);
        return false;
    }

    // will perform the same sequence as power on reset
    platform.delayUs(BMP180_STARTUP_TIME_US);
    platform.debugPrint("BMP180 software reset complete\r\n");

    return true;
}

bool BMP180_ReadCalibrationData(void) {
    bool ok = true;

    // Read all calibration registers
    ok &= readCalibReg(CALIB_AC1_REG_MSB, CALIB_AC1_REG_LSB, &ac1, NULL);
    ok &= readCalibReg(CALIB_AC2_REG_MSB, CALIB_AC2_REG_LSB, &ac2, NULL);
    ok &= readCalibReg(CALIB_AC3_REG_MSB, CALIB_AC3_REG_LSB, &ac3, NULL);
    ok &= readCalibReg(CALIB_AC4_REG_MSB, CALIB_AC4_REG_LSB, NULL, &ac4);
    ok &= readCalibReg(CALIB_AC5_REG_MSB, CALIB_AC5_REG_LSB, NULL, &ac5);
    ok &= readCalibReg(CALIB_AC6_REG_MSB, CALIB_AC6_REG_LSB, NULL, &ac6);
    ok &= readCalibReg(CALIB_B1_REG_MSB, CALIB_B1_REG_LSB, NULL, &b1);
    ok &= readCalibReg(CALIB_B2_REG_MSB, CALIB_B2_REG_LSB, NULL, &b2);
    ok &= readCalibReg(CALIB_MB_REG_MSB, CALIB_MB_REG_LSB, NULL, &mb);
    ok &= readCalibReg(CALIB_MC_REG_MSB, CALIB_MC_REG_LSB, NULL, &mc);
    ok &= readCalibReg(CALIB_MD_REG_MSB, CALIB_MD_REG_LSB, NULL, &md);

    return ok;
}

bool BMP180_ReadChipId(uint8_t *chipId) {
    int ret = platform.i2cReadReg(BMP180_I2C_ADDR, ID_REG, chipId, 1, 1);
    if (ret < 0) {
        platform.debugPrint("Error reading BMP180 chip ID: %d\r\n", -ret);
        return false;
    }

    return true;
}

bool BMP180_CheckChipId(void) {
    uint8_t chipId;
    bool ok = BMP180_ReadChipId(&chipId);
    if (!ok) {
        return false;
    }

    if (chipId != BMP180_CHIP_ID) {
        platform.debugPrint("Invalid BMP180 chip ID: %02X, expected: %02X\r\n", chipId, BMP180_CHIP_ID);
        return false;
    }

    return true;
}

bool BMP180_ReadTemperaturePressure(enum BMP180_OversamplingSetting pressureOss, float *temp, float *pressure) {
    uint8_t ctrlMeas[1];
    int ret;

    // Start of conversion for temperature
    uint8_t sco = 1;
    ctrlMeas[0] = MEAS_CTRL_SCO_TEMP; // sco=1 -> start of conversion
    ret = platform.i2cWriteReg(BMP180_I2C_ADDR, CTRL_MEAS_REG, ctrlMeas, sizeof(ctrlMeas), 1);
    if (ret < 0) {
        platform.debugPrint("Error starting BMP180 temperature conversion: %d\r\n", -ret);
        return false;
    }

    // Wait
    platform.delayUs(BMP180_TEMP_CONV_TIME_US);

    // Read uncompensated temperature
    long ut;
    ret = readUncompensatedTemp(&ut);
    if (!ret) {
        return false;
    }

    // Start of conversion for pressure
    ctrlMeas[0] = MEAS_CTRL_SCO_PRESS | ((int)pressureOss << 6); // sco=1 -> start of conversion
    ret = platform.i2cWriteReg(BMP180_I2C_ADDR, CTRL_MEAS_REG, ctrlMeas, sizeof(ctrlMeas), 1);
    if (ret < 0) {
        platform.debugPrint("Error starting BMP180 pressure conversion: %d\r\n", -ret);
        return false;
    }

    // Wait
    switch (pressureOss) {
    case BMP180_OSS_ULP_4_5ms:
        platform.delayUs(4500);
        break;
    case BMP180_OSS_STD_7_5ms:
        platform.delayUs(7500);
        break;
    case BMP180_OSS_HR_13_5ms:
        platform.delayUs(13500);
        break;
    case BMP180_OSS_XHR_25_5ms:
        platform.delayUs(25500);
        break;
    }

    // Read uncompensated pressure
    long up;
    ret = readUncompensatedPressure(&up, (int)pressureOss);
    if (!ret) {
        return false;
    }

    // Calculate true temperature and pressure
    calcTrueTemperaturePressure(ut, up, (int)pressureOss, temp, pressure);

    return true;
}

float BMP180_CalcAltitude(float pressure) {
    float p0 = 1013.25f; // pressure at sea level
    return 44330.0f * (1.0f - powf(pressure / p0, 1 / 5.255f));
}

float BMP180_CalcPressureSeaLevel(float altitude, float pressure) {
    return pressure / powf(1.0f - altitude / 44330.0f, 5.255f);
}

static bool approxEqual(float a, float b, float epsilon) {
    return fabsf(a - b) < epsilon;
}

const char *BMP180_UnitTest(void) {
    // Setup test calibration values
    ac1 = 408;
    ac2 = -72;
    ac3 = -14383;
    ac4 = 32741;
    ac5 = 32757;
    ac6 = 23153;
    b1 = 6190;
    b2 = 4;
    mb = -32768;
    mc = -8711;
    md = 2868;

    // Test temperature and pressure calculation
    long ut = 27898;
    int oss = (int)BMP180_OSS_ULP_4_5ms;
    long up = 23843;

    float temperature, pressure;
    calcTrueTemperaturePressure(ut, up, oss, &temperature, &pressure);
    mu_assert("Bad temperature calculation", approxEqual(temperature, 15.0f, 0.1f));
    mu_assert("Bad pressure calculation", approxEqual(pressure, 699.64f, 0.1f));

    return NULL;
}
