/*
 * Configuration file for CANopenNode.
 *
 * @file        CO_config.h
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

#ifndef CO_CONFIG_H
#define CO_CONFIG_H

/* Stack configuration override default values.
 * For more information see file CO_driver_target.h. */

/* Basic stack configuration */
#define CO_CONFIG_NMT 1
#define CO_CONFIG_HB_CONS 1
#define CO_CONFIG_EM 1
#define CO_CONFIG_SDO_SRV 1
#define CO_CONFIG_SDO_CLI 0
#define CO_CONFIG_SYNC 1
#define CO_CONFIG_TIME 0
#define CO_CONFIG_PDO 1
#define CO_CONFIG_LEDS 1
#define CO_CONFIG_LSS 1

/* Enable specific features */
#define CO_CONFIG_NMT_ENABLE 1
#define CO_CONFIG_HB_CONS_ENABLE 1
#define CO_CONFIG_EM_ENABLE 1
#define CO_CONFIG_SDO_SRV_ENABLE 1
#define CO_CONFIG_SDO_CLI_ENABLE 0
#define CO_CONFIG_SYNC_ENABLE 1
#define CO_CONFIG_TIME_ENABLE 0
#define CO_CONFIG_RPDO_ENABLE 1
#define CO_CONFIG_TPDO_ENABLE 1
#define CO_CONFIG_LEDS_ENABLE 1
#define CO_CONFIG_LSS_SLAVE 1
#define CO_CONFIG_LSS_MASTER 0

/* FreeRTOS specific */
#define CO_CONFIG_FREERTOS 1

#endif /* CO_CONFIG_H */ 