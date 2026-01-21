/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_INA226

#include <stdbool.h>
#include <stdint.h>

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus_i2c.h"
#include "drivers/time.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "ina226.h"

#define INA226_REG_CONFIG            0x00
#define INA226_REG_SHUNT_VOLTAGE     0x01
#define INA226_REG_BUS_VOLTAGE       0x02
#define INA226_REG_CURRENT           0x04
#define INA226_REG_CALIBRATION       0x05
#define INA226_REG_MANUFACTURER_ID   0xFE
#define INA226_REG_DIE_ID            0xFF

#define INA226_MANUFACTURER_ID       0x5449
#define INA226_DIE_ID                0x2260

#define INA226_CONFIG_RESET          0x8000
#define INA226_CONFIG_AVG_MASK       0x0E00
#define INA226_CONFIG_VBUSCT_MASK    0x01C0
#define INA226_CONFIG_VSHCT_MASK     0x0038
#define INA226_CONFIG_MODE_MASK      0x0007

#define INA226_CONFIG_MODE_CONTINUOUS_SHUNT_BUS 0x0007

#define INA226_SHUNT_VOLTAGE_LSB_UV  25   // 2.5uV * 10 for integer math
#define INA226_BUS_VOLTAGE_LSB_UV    1250 // 1.25mV
#define INA226_SHUNT_VOLTAGE_MAX_UV  81920

#define INA226_MIN_POLL_INTERVAL_US  40000

#ifndef INA226_DEFAULT_SHUNT_UOHM
#define INA226_DEFAULT_SHUNT_UOHM    10000
#endif

#ifndef INA226_DEFAULT_AVG_SAMPLES
#define INA226_DEFAULT_AVG_SAMPLES   16
#endif

#ifndef INA226_DEFAULT_CONV_TIME_US
#define INA226_DEFAULT_CONV_TIME_US  1100
#endif

#ifndef INA226_DEFAULT_MAX_CURRENT_A
#define INA226_DEFAULT_MAX_CURRENT_A 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(ina226Config_t, ina226Config, PG_INA226_CONFIG, 0);

PG_RESET_TEMPLATE(ina226Config_t, ina226Config,
    .i2cAddress = INA226_I2C_DEFAULT_ADDRESS,
    .shuntMicroOhm = INA226_DEFAULT_SHUNT_UOHM,
    .maxCurrentA = INA226_DEFAULT_MAX_CURRENT_A,
    .avgSamples = INA226_DEFAULT_AVG_SAMPLES,
    .convTimeUs = INA226_DEFAULT_CONV_TIME_US
);

typedef struct ina226ConfigEntry_s {
    uint16_t value;
    uint16_t configBits;
} ina226ConfigEntry_t;

static const ina226ConfigEntry_t ina226AvgTable[] = {
    { 1, 0x0000 },
    { 4, 0x0200 },
    { 16, 0x0400 },
    { 64, 0x0600 },
    { 128, 0x0800 },
    { 256, 0x0A00 },
    { 512, 0x0C00 },
    { 1024, 0x0E00 },
};

static const ina226ConfigEntry_t ina226ConvTimeTable[] = {
    { 140, 0x0000 },
    { 204, 0x0040 },
    { 332, 0x0080 },
    { 588, 0x00C0 },
    { 1100, 0x0100 },
    { 2116, 0x0140 },
    { 4156, 0x0180 },
    { 8244, 0x01C0 },
};

typedef struct ina226State_s {
    bool present;
    bool dataValid;
    bool initialized;
    i2cDevice_e i2cDevice;
    uint8_t i2cAddress;
    timeUs_t lastUpdateUs;
    uint32_t updateIntervalUs;
    uint32_t currentLsbMicroamps;
    uint16_t calibration;
    uint16_t busVoltageCentiV;
    int32_t currentCentiAmps;
    int32_t shuntMicrovolts;
} ina226State_t;

static ina226State_t ina226State;

static i2cDevice_e ina226SelectI2cDevice(void)
{
#if defined(I2C_DEVICE)
    if (I2C_DEVICE != I2CINVALID) {
        return I2C_DEVICE;
    }
#endif
#ifdef USE_I2C_DEVICE_1
    return I2CDEV_1;
#elif defined(USE_I2C_DEVICE_0)
    return I2CDEV_0;
#elif defined(USE_I2C_DEVICE_2)
    return I2CDEV_2;
#elif defined(USE_I2C_DEVICE_3)
    return I2CDEV_3;
#else
    return I2CINVALID;
#endif
}

static bool ina226ReadRegister16(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    if (!i2cReadBuffer(ina226State.i2cDevice, ina226State.i2cAddress, reg, sizeof(buf), buf)) {
        return false;
    }
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

static bool ina226WriteRegister16(uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)value };
    return i2cWriteBuffer(ina226State.i2cDevice, ina226State.i2cAddress, reg, sizeof(buf), buf);
}

static const ina226ConfigEntry_t *ina226FindNearestConfig(const ina226ConfigEntry_t *table, size_t entries, uint16_t requested)
{
    const ina226ConfigEntry_t *best = &table[0];
    uint16_t bestDiff = ABS((int32_t)requested - (int32_t)best->value);

    for (size_t i = 1; i < entries; i++) {
        uint16_t diff = ABS((int32_t)requested - (int32_t)table[i].value);
        if (diff < bestDiff) {
            bestDiff = diff;
            best = &table[i];
        }
    }

    return best;
}

static void ina226ComputeCalibration(uint32_t shuntMicroOhm, uint16_t maxCurrentA, uint32_t *currentLsbMicroamps, uint16_t *calibration)
{
    if (shuntMicroOhm == 0) {
        shuntMicroOhm = INA226_DEFAULT_SHUNT_UOHM;
    }

    const uint64_t maxCurrentFromShuntMicroamps = ((uint64_t)INA226_SHUNT_VOLTAGE_MAX_UV * 1000000ULL) / shuntMicroOhm;
    uint64_t maxCurrentMicroamps = (maxCurrentA > 0) ? (uint64_t)maxCurrentA * 1000000ULL : maxCurrentFromShuntMicroamps;

    if (maxCurrentMicroamps > maxCurrentFromShuntMicroamps) {
        maxCurrentMicroamps = maxCurrentFromShuntMicroamps;
    }

    uint32_t lsbMicroamps = (uint32_t)((maxCurrentMicroamps + 32767ULL) / 32768ULL);
    if (lsbMicroamps == 0) {
        lsbMicroamps = 1;
    }

    uint64_t cal = 5120000000ULL / ((uint64_t)lsbMicroamps * shuntMicroOhm);
    if (cal == 0) {
        cal = 1;
    }
    if (cal > 0xFFFF) {
        cal = 0xFFFF;
    }

    *currentLsbMicroamps = lsbMicroamps;
    *calibration = (uint16_t)cal;
}

static bool ina226Detect(void)
{
    uint16_t manufacturer = 0;
    uint16_t dieId = 0;

    if (!ina226ReadRegister16(INA226_REG_MANUFACTURER_ID, &manufacturer)) {
        return false;
    }

    if (!ina226ReadRegister16(INA226_REG_DIE_ID, &dieId)) {
        return false;
    }

    return (manufacturer == INA226_MANUFACTURER_ID) && ((dieId & 0xFFF0) == INA226_DIE_ID);
}

static uint16_t ina226BuildConfig(uint16_t *avgSamples, uint16_t *convTimeUs)
{
    const ina226ConfigEntry_t *avgEntry = ina226FindNearestConfig(ina226AvgTable, ARRAYLEN(ina226AvgTable), *avgSamples);
    const ina226ConfigEntry_t *convEntry = ina226FindNearestConfig(ina226ConvTimeTable, ARRAYLEN(ina226ConvTimeTable), *convTimeUs);

    *avgSamples = avgEntry->value;
    *convTimeUs = convEntry->value;

    const uint16_t vbusBits = convEntry->configBits & INA226_CONFIG_VBUSCT_MASK;
    const uint16_t shuntBits = (convEntry->configBits >> 3) & INA226_CONFIG_VSHCT_MASK;

    return (avgEntry->configBits & INA226_CONFIG_AVG_MASK)
        | vbusBits
        | shuntBits
        | INA226_CONFIG_MODE_CONTINUOUS_SHUNT_BUS;
}

bool ina226Init(void)
{
    if (ina226State.initialized) {
        return ina226State.present;
    }

    ina226State = (ina226State_t){
        .present = false,
        .dataValid = false,
        .initialized = true,
        .i2cDevice = ina226SelectI2cDevice(),
        .i2cAddress = ina226Config()->i2cAddress,
        .lastUpdateUs = 0,
        .updateIntervalUs = INA226_MIN_POLL_INTERVAL_US,
        .currentLsbMicroamps = 0,
        .calibration = 0,
        .busVoltageCentiV = 0,
        .currentCentiAmps = 0,
        .shuntMicrovolts = 0,
    };

    if (ina226State.i2cDevice == I2CINVALID) {
        return false;
    }

    if (!ina226Detect()) {
        return false;
    }

    uint16_t avgSamples = ina226Config()->avgSamples;
    uint16_t convTimeUs = ina226Config()->convTimeUs;
    const uint16_t config = ina226BuildConfig(&avgSamples, &convTimeUs);

    ina226ComputeCalibration(ina226Config()->shuntMicroOhm, ina226Config()->maxCurrentA,
        &ina226State.currentLsbMicroamps, &ina226State.calibration);

    bool ok = true;
    ok = ok && ina226WriteRegister16(INA226_REG_CALIBRATION, ina226State.calibration);
    ok = ok && ina226WriteRegister16(INA226_REG_CONFIG, config & ~INA226_CONFIG_RESET);

    if (!ok) {
        return false;
    }

    ina226State.updateIntervalUs = MAX(INA226_MIN_POLL_INTERVAL_US, (uint32_t)convTimeUs * 2U * avgSamples);
    ina226State.present = true;
    return true;
}

bool ina226IsPresent(void)
{
    return ina226State.present;
}

void ina226Update(timeUs_t currentTimeUs)
{
    if (!ina226State.present) {
        return;
    }

    if (cmp32(currentTimeUs, ina226State.lastUpdateUs) < (int32_t)ina226State.updateIntervalUs) {
        return;
    }

    ina226State.lastUpdateUs = currentTimeUs;

    uint16_t busRaw = 0;
    uint16_t shuntRaw = 0;
    uint16_t currentRaw = 0;

    if (!ina226ReadRegister16(INA226_REG_BUS_VOLTAGE, &busRaw)
        || !ina226ReadRegister16(INA226_REG_SHUNT_VOLTAGE, &shuntRaw)
        || !ina226ReadRegister16(INA226_REG_CURRENT, &currentRaw)) {
        ina226State.dataValid = false;
        return;
    }

    const int16_t shuntSigned = (int16_t)shuntRaw;
    const int16_t currentSigned = (int16_t)currentRaw;

    ina226State.busVoltageCentiV = (uint16_t)(((uint32_t)busRaw * INA226_BUS_VOLTAGE_LSB_UV + 5000U) / 10000U);
    ina226State.shuntMicrovolts = (int32_t)shuntSigned * INA226_SHUNT_VOLTAGE_LSB_UV / 10;
    ina226State.currentCentiAmps = (int32_t)((int64_t)currentSigned * ina226State.currentLsbMicroamps / 10000);

    ina226State.dataValid = true;
}

bool ina226GetBusVoltageCentiV(uint16_t *voltageCentiV)
{
    if (!ina226State.present || !ina226State.dataValid) {
        return false;
    }

    *voltageCentiV = ina226State.busVoltageCentiV;
    return true;
}

bool ina226GetCurrentCentiAmps(int32_t *currentCentiAmps)
{
    if (!ina226State.present || !ina226State.dataValid) {
        return false;
    }

    *currentCentiAmps = ina226State.currentCentiAmps;
    return true;
}

int32_t ina226GetShuntMicrovolts(void)
{
    return ina226State.shuntMicrovolts;
}

uint32_t ina226GetCurrentLsbMicroamps(void)
{
    return ina226State.currentLsbMicroamps;
}

#endif
