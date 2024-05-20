// Copyright 2023 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates simple Zigbee light bulb.
 *
 * The example demonstrates how to use ESP Zigbee stack to create a end device light bulb.
 * The light bulb is a Zigbee end device, which is controlled by a Zigbee coordinator.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 */

#if (!PLATFORMIO)
  // Enable Arduino-ESP32 logging in Arduino IDE
  #ifdef CORE_DEBUG_LEVEL
    #undef CORE_DEBUG_LEVEL
  #endif
  #ifdef LOG_LOCAL_LEVEL
    #undef LOG_LOCAL_LEVEL
  #endif

  #define CORE_DEBUG_LEVEL 4
  #define LOG_LOCAL_LEVEL CORE_DEBUG_LEVEL
#endif


#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "driver/gpio.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

#define LED_PIN RGB_BUILTIN

static const char *TAG = "ESP_ZB_ON_OFF_LIGHT";

/* Default End Device config */
#define ESP_ZB_ZED_CONFIG()                                                                 \
  {                                                                                         \
    .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
    .nwk_cfg = {                                                                            \
      .zed_cfg =                                                                            \
        {                                                                                   \
          .ed_timeout = ED_AGING_TIMEOUT,                                                   \
          .keep_alive = ED_KEEP_ALIVE,                                                      \
        },                                                                                  \
    },                                                                                      \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
  { .radio_mode = RADIO_MODE_NATIVE, }

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
  { .host_connection_mode = HOST_CONNECTION_MODE_NONE, }

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE   false /* enable the install code policy for security */
#define ED_AGING_TIMEOUT            ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE               3000                                 /* 3000 millisecond */
#define HA_ESP_LIGHT_ENDPOINT       10                                   /* esp light bulb device endpoint, used to process light controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/********************* Zigbee functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
  esp_err_t ret = ESP_OK;
  bool light_state = 0;

  if (!message) {
    ESP_LOGE(TAG, "Empty message\n");
  }
  if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
    ESP_LOGE(TAG, "Received message: error status(%d)\n", message->info.status);
  }

  ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)\n", message->info.dst_endpoint, message->info.cluster, message->attribute.id,
    message->attribute.data.size
  );
  if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
      if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
        light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
        ESP_LOGI(TAG, "Light sets to %s\n", light_state ? "On" : "Off");
        // neopixelWrite(LED_PIN, 255 * light_state, 255 * light_state, 255 * light_state);  // Toggle light
        gpio_set_level(GPIO_NUM_15, !light_state);
      }
    }
  }
  return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "Zigbee stack initialized\n");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        ESP_LOGI(TAG, "Start network steering\n");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
      } else {
        /* commissioning failed */
        ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)\n", esp_err_to_name(err_status));
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
          extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4], extended_pan_id[3], extended_pan_id[2], extended_pan_id[1],
          extended_pan_id[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address()
        );
      } else {
        ESP_LOGI(TAG, "Network steering was not successful (status: %s)\n", esp_err_to_name(err_status));
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
    default: ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s\n", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status)); break;
  }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
  }
  return ret;
}

char modelid[] = {13, 'E', 'S', 'P', '3', '2', 'C', '6', '.', 'L', 'i', 'g', 'h', 't'};
char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee end-device config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    // set to try all channels
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    /* set the on-off light device config */
    uint8_t test_attr, test_attr2;

    test_attr = 0;
    test_attr2 = 4;

    /* basic cluster create with fully customized */
    // create attribute list for cluster id basic
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    // add zcl attr version id 0
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    // add zcl attr power source id 4 (?)
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr2);
    // change zcl attr version id to 4
    esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr2);
    // add zcl attr basic model id "ESP32C6.Light"
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);
    // add zcl attr basic manufacturer name "Espressif"
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);

    /* identify cluster create with fully customized */
    // create attribute list for cluster id identify
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);

    /* group cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &test_attr);

    /* scenes cluster create with standard cluster + customized */
    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(NULL);
    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);

    /* on-off cluster create with standard cluster config*/
    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    /* create cluster list for this endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    // add basic cluster and set server role (it is a lamp server)
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* update basic cluster in the existed cluster list */
    //esp_zb_cluster_list_update_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    // the clusters to the list
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list, esp_zb_scenes_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // create an endpoint list
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    /* add created endpoint (cluster_list) to endpoint list */
    // add an endpoint to the list with the cluster list and define as a light endpoint, HA profile, HA on-off output device
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, HA_ESP_LIGHT_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID);
    esp_zb_device_register(esp_zb_ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);

    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_main_loop_iteration();
}
/* Handle the light attribute */

void app_main(void)
 {
  ESP_LOGI(TAG, "setup start\n");
  ESP_ERROR_CHECK(nvs_flash_init());

  // Init Zigbee
  esp_zb_platform_config_t config = {
    .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  // Init RMT and leave light OFF
  // neopixelWrite(LED_PIN, 0, 0, 0);
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(GPIO_NUM_15, 1);

  // Start Zigbee task
  xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
