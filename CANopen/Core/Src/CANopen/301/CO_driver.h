/*
 * CAN driver for CANopenNode.
 *
 * @file        CO_driver.h
 * @author      Janez Paternoster
 * @copyright   2010 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CO_DRIVER_H
#define CO_DRIVER_H

#include "CO_driver_target.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CAN module functions */
void CO_CANsetConfigurationMode(void* CANptr);
void CO_CANsetNormalMode(CO_CANmodule_t* CANmodule);
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate);
void CO_CANmodule_disable(CO_CANmodule_t* CANmodule);
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object, void (*CANrx_callback)(void* object, void* message));
CO_ReturnError_t CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag);
CO_ReturnError_t CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer);
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule);
void CO_CANmodule_process(CO_CANmodule_t* CANmodule);

#ifdef __cplusplus
}
#endif

#endif /* CO_DRIVER_H */ 