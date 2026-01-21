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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"

#ifdef USE_INA226

#define INA226_I2C_DEFAULT_ADDRESS 0x40

typedef struct ina226Config_s {
    uint8_t i2cAddress;
    uint32_t shuntMicroOhm;
    uint16_t maxCurrentA;
    uint16_t avgSamples;
    uint16_t convTimeUs;
} ina226Config_t;

PG_DECLARE(ina226Config_t, ina226Config);

bool ina226Init(void);
bool ina226IsPresent(void);
void ina226Update(timeUs_t currentTimeUs);
bool ina226GetBusVoltageCentiV(uint16_t *voltageCentiV);
bool ina226GetCurrentCentiAmps(int32_t *currentCentiAmps);
int32_t ina226GetShuntMicrovolts(void);
uint32_t ina226GetCurrentLsbMicroamps(void);

#endif
