/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) CO LTD
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems
 *    integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and
 *    the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "sdkconfig.h"
#include "hal/gpio_types.h"
#include "hal/uart_types.h"
#include "driver/uart.h"

#define ZB_ESP
#define ZB_CONFIG_ESP
#if CONFIG_ZB_RADIO_MACSPLIT_UART
#define ZB_PLATFORM_SOC
#endif

#define ZB_LEDS_MASK
#define ZB_USE_BUTTONS
#define ZB_SOFT_SECURITY

#define MAC_TRANSPORT_USES_SELECT

#define ZB_LITTLE_ENDIAN
#define ZB_NEED_ALIGN

/* our MAC */
#define ZB_MANUAL_ACK
#define ZB_AUTO_ACK_TX
#define ZB_MAC_AUTO_ACK_RECV
#define ZB_MAC_RX_QUEUE_CAP 20
#define ZB_MAC_TIMESTAMP_IN_PKT
#define MAC_AUTO_DELAY_IN_MAC_GP_SEND

/* MAC Pending  Bit Matching */
#define ZB_MAC_PENDING_BIT_SOURCE_MATCHING
#define ZB_MAC_HARDWARE_PB_MATCHING

#define ZB_GPD_TX_OFFSET_US                                   \
  (ZB_MILLISECONDS_TO_USEC(ZB_GPD_RX_OFFSET_MS) -             \
   2 * ZB_MAC_A_UNIT_BACKOFF_PERIOD * ZB_SYMBOL_DURATION_USEC \
   - 900)

typedef enum {
    RADIO_MODE_INIT_UART = 0x0,
    HOST_MODE_INIT_UART  = 0x1,
} zb_esp_uart_init_mode;

typedef enum {
    RADIO_MODE_NATIVE   = 0x0,      /*!< Use the native 15.4 radio */
    RADIO_MODE_UART_RCP = 0x1,      /*!< UART connection to a 15.4 capable radio co - processor (RCP) */
    RADIO_MODE_SPI_RCP  = 0x2,      /*!< SPI connection to a 15.4 capable radio co - processor (RCP) */
} esp_zb_radio_mode_t;

typedef enum {
    HOST_CONNECTION_MODE_NONE       = 0x0, /*!< Disable host connection */
    HOST_CONNECTION_MODE_CLI_UART   = 0x1, /*!< CLI UART connection to the host */
    HOST_CONNECTION_MODE_RCP_UART   = 0x2, /*!< RCP UART connection to the host */
} esp_zb_host_connection_mode_t;

typedef enum {
    SERIAL_MODE_DISABLE   = 0x0,      /*!< Disable osif serial mode  */
    SERIAL_MODE_UART      = 0x1,      /*!< osif serial mode through uart */
} esp_zb_serial_mode_t;

typedef struct {
    uart_port_t port;               /*!< UART port number */
    uart_config_t uart_config;      /*!< UART configuration, see uart_config_t docs */
    gpio_num_t rx_pin;              /*!< UART RX pin */
    gpio_num_t tx_pin;              /*!< UART TX pin */
} esp_zb_uart_config_t;

typedef struct {
    esp_zb_serial_mode_t             serial_mode;                   /*!< The osif serial connection mode */
    esp_zb_uart_config_t             osif_serial_uart_config;       /*!< The uart configuration to osif serial */
} esp_zb_serial_config_t;

typedef struct {
    esp_zb_radio_mode_t     radio_mode;         /*!< The radio mode */
    esp_zb_uart_config_t    radio_uart_config;  /*!< The uart configuration to RCP */
} esp_zb_radio_config_t;

typedef struct {
    esp_zb_host_connection_mode_t   host_connection_mode;   /*!< The host connection mode */
    esp_zb_uart_config_t            host_uart_config;       /*!< The uart configuration to host */
} esp_zb_host_config_t;

typedef struct {
    esp_zb_radio_config_t               radio_config;   /*!< The radio configuration */
    esp_zb_host_config_t                host_config;    /*!< The host connection configuration */
}  esp_zb_platform_config_t;

typedef void (*esp_rcp_failure_callback_t)(uint8_t param);

/**
 * @brief  Set the espressif soc platform config
 *
 * @param[in] config - pointer to platform configuration @ref esp_zb_platform_config_t
 *
 * @return - ESP_OK on success
 *
 */
esp_err_t esp_zb_platform_config(esp_zb_platform_config_t *config);
esp_err_t esp_zb_platform_serial_config_set(esp_zb_serial_config_t *serial_config);

/**
 * @brief  Get the rcp Zigbee rcp version string
 *
 * @return string of rcp Zigbee stack version string
 *
 */
char* esp_zb_macsplit_get_version();

/**
 * @brief  Set the rcp Zigbee rcp version string
 *
 */
void esp_zb_macsplit_set_version(char *ver_string);

/**
 * @brief  Reset the rcp through UART reset command from host (gateway)
 *
 */
void esp_zb_macsplit_host_reset_rcp();

void esp_zb_add_rcp_failure_cb(esp_rcp_failure_callback_t cb);

void esp_zb_macsplit_uart_deinit();

/**
 * @brief  Set zigbee default long poll interval
 *
 * @note  Used internally by the Zigbee SDK, no user intervention is required.
 *
 * @param[in] milliseconds - default long_poll_interval
 */
void esp_zb_set_default_long_poll_interval(uint32_t milliseconds);

/**
 * @brief  Get the zigbee default long poll interval
 *
 * @note  Used internally by the Zigbee SDK, no user intervention is required.
 *
 * @return zigbee default long poll interval
 *
 */
uint32_t esp_zb_get_default_long_poll_interval(void);

#define ESP_ZB_PIM_DEFAULT_LONG_POLL_INTERVAL  esp_zb_get_default_long_poll_interval()
