#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "mqtt_client.h"

// ==========================================
// --- 1. CONFIGURATION (EDIT THIS SECTION) ---
// ==========================================

// WiFi Credentials
#define WIFI_SSID "Harun_A55" // <--- ENTER WIFI NAME
#define WIFI_PASS "123456789" // <--- ENTER WIFI PASSWORD

// MQTT Broker Settings (HiveMQ Public)
#define MQTT_BROKER_URI "mqtt://broker.hivemq.com:1883"
#define MQTT_USERNAME "Krc6266" // Client ID / Username
#define MQTT_TOPIC "Krc6266/dem_esp_freertos"

// UART 0 (PC Debug Console)
#define PC_UART_PORT UART_NUM_0
#define PC_UART_BAUD 115200

// UART 2 (STM32 Communication)
#define STM_UART_PORT UART_NUM_2
#define STM_TX_PIN 25 // Not used for TX, but required by driver
#define STM_RX_PIN 26 // Connect STM32 TX to this PIN (GPIO 26)
#define STM_UART_BAUD 115200
#define RX_BUF_SIZE 1024

// Status LED
#define STATUS_LED_PIN 33

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ==========================================
// --- 2. GLOBAL VARIABLES (SHARED MEMORY) ---
// ==========================================
static const char *TAG = "MAIN_APP";

// Sensor data from STM32 (Shared between RX Task and MQTT Task)
volatile int g_pitch = 0;
volatile int g_roll = 0;
volatile int g_temp = 0;
volatile int g_light = 0;
volatile bool is_mqtt_connected = false;
// MQTT Client Handle
esp_mqtt_client_handle_t mqtt_client = NULL;

// ==========================================
// --- 3. WIFI & MQTT HANDLERS ---
// ==========================================

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "WiFi disconnected, retrying...");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "WiFi Connected! IP:" IPSTR, IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT Connected to Broker!");
    is_mqtt_connected = true;
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGW(TAG, "MQTT Disconnected");
    is_mqtt_connected = false;
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGE(TAG, "MQTT Error");
    break;
  default:
    break;
  }
}

void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, NULL, NULL);

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();
}

void mqtt_app_start(void) {
  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = MQTT_BROKER_URI,
      .credentials.username = MQTT_USERNAME,
  };

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqtt_client);
}

// ==========================================
// --- 4. UART & TASKS ---
// ==========================================

void init_dual_uart(void) {
  // --- 1. Setup UART0 (PC Debugging) ---
  const uart_config_t uart0_config = {
      .baud_rate = PC_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  // No buffer needed for UART0 usually, but we install driver to be safe
  uart_driver_install(PC_UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0);
  uart_param_config(PC_UART_PORT, &uart0_config);
  uart_set_pin(PC_UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // --- 2. Setup UART2 (STM32 Data) ---
  const uart_config_t uart2_config = {
      .baud_rate = STM_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  // Install driver with RX buffer
  uart_driver_install(STM_UART_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(STM_UART_PORT, &uart2_config);
  // Assign pins for STM32
  uart_set_pin(STM_UART_PORT, STM_TX_PIN, STM_RX_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);

  // LED Init
  gpio_reset_pin(STATUS_LED_PIN);
  gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);

  printf("UART Init Complete: UART0(PC) & UART2(STM32) Ready.\n");
}

// Task: Read data from STM32, Parse it, Update globals
static void rx_task_stm32(void *arg) {
  uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
  int p, r, t, l;

  ESP_LOGI(TAG, "RX Task Started - Listening to STM32...");

  while (1) {
    // Read from UART2 (STM_UART_PORT)
    const int rxBytes = uart_read_bytes(STM_UART_PORT, data, RX_BUF_SIZE,
                                        100 / portTICK_PERIOD_MS);

    if (rxBytes > 0) {
      data[rxBytes] = 0; // Null terminate

      // Parse format: "P:10 R:-5 T:25 L:40"
      int items = sscanf((char *)data, "P:%d R:%d T:%d L:%d", &p, &r, &t, &l);

      if (items == 4) {
        // Update Global Variables
        g_pitch = p;
        g_roll = r;
        g_temp = t;
        g_light = l;

        // Blink LED briefly indicating valid data
        gpio_set_level(STATUS_LED_PIN, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(STATUS_LED_PIN, 0);

        // Debug print (optional)
        // printf("PARSED: Pitch:%d Roll:%d Temp:%d Light:%d\n", p, r, t, l);
      } else {
        ESP_LOGW(TAG, "Parsing Error: %s", data);
      }
    }
  }
  free(data);
}

// Task: Create JSON from globals and Publish to MQTT
static void mqtt_publish_task(void *arg) {
  char json_payload[128];

  // Wait a bit for network
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  while (1) {
    // Format JSON using latest global data
    sprintf(json_payload,
            "{\"pitch\":%d, \"roll\":%d, \"temp\":%d, \"light\":%d}", g_pitch,
            g_roll, g_temp, g_light);

    if (mqtt_client != NULL && is_mqtt_connected) {
      int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC,
                                           json_payload, 0, 1, 0);

      if (msg_id != -1) {
        printf("MQTT Sent to %s: %s\n", MQTT_TOPIC, json_payload);
      } else {
        ESP_LOGE(TAG, "MQTT Publish Failed");
      }
    } else {
      ESP_LOGW(TAG, "MQTT Not Connected. Skipping publish...");
    }

    // Publish rate: 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// --- 5. MAIN APPLICATION ---
// ==========================================
void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // 1. Init Hardware (UARTs)
  init_dual_uart();

  // 2. Init WiFi
  ESP_LOGI(TAG, "Starting WiFi...");
  wifi_init_sta();

  ESP_LOGI(TAG, "Waiting for IP address...");
  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                      portMAX_DELAY);
  ESP_LOGI(TAG, "IP obtained, starting MQTT...");

  // 3. Init MQTT
  ESP_LOGI(TAG, "Starting MQTT...");
  mqtt_app_start();

  // 4. Create Tasks
  // RX Task (High Priority)
  xTaskCreate(rx_task_stm32, "stm32_rx_task", 4096, NULL, 10, NULL);

  // MQTT Publish Task (Normal Priority)
  xTaskCreate(mqtt_publish_task, "mqtt_pub_task", 4096, NULL, 5, NULL);
}