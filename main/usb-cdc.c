/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define EXAMPLE_USB_HOST_PRIORITY 20
#define EXAMPLE_USB_DEVICE_VID    0x1eab  // 0x303A:0x4001 (TinyUSB CDC device)
#define EXAMPLE_USB_DEVICE_PID    0x1a06  // Change this to 0x4002 for dual CDC device



static const char *TAG = "USB-CDC";
static const char *DATA_TAG = "QR-DATA";


/* ------------------------------- Callbacks -------------------------------- */
static void handle_rx(uint8_t *data, size_t data_len, void *arg)
{
    ESP_LOGI(TAG, "%d", data_len);
    ESP_LOGI(DATA_TAG, "%s", data);
    //ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
    uart_write_bytes(UART_NUM_1, data, data_len);
}

void usb_lib_task(void *arg)
{
    while (1) {
        //Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "All clients deregistered");
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            break;
        }
    }

    //Clean up USB Host
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

void UartInit(void)
{
    
    esp_log_level_set("UART", ESP_LOG_INFO);
    const int uart_buffer_size = (1024 * 2);
 
    /* Configure parameters of an UART driver, communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
 
    //Set UART pins (using UART0 default pins ie no changes.)
    ESP_ERROR_CHECK( uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) );
   
    //Install UART driver, and get the queue.
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 20, NULL, 0) );
 

}

/* ---------------------------------- Main ---------------------------------- */
void app_main(void)
{

    UartInit();

    //Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    xTaskCreate(usb_lib_task, "usb_lib", 4096, xTaskGetCurrentTaskHandle(), EXAMPLE_USB_HOST_PRIORITY, NULL);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    ESP_LOGI(TAG, "Opening CDC ACM device 0x%04X:0x%04X", EXAMPLE_USB_DEVICE_VID, EXAMPLE_USB_DEVICE_PID);
    cdc_acm_dev_hdl_t cdc_dev;
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 5000,
        .out_buffer_size = 64,
        .user_arg = NULL,
        .event_cb = NULL,
        .data_cb = handle_rx
    };
    
    ESP_ERROR_CHECK(cdc_acm_host_open(EXAMPLE_USB_DEVICE_VID, EXAMPLE_USB_DEVICE_PID, 0, &dev_config, &cdc_dev));
    assert(cdc_dev);
    
    //cdc_acm_host_desc_print(cdc_dev);
    
    vTaskDelay(100);

    // Test sending and receiving: Send AT commands, responses are handled in handle_rx callback
    //static char text1[] = "AT\r";
    //ESP_ERROR_CHECK(cdc_acm_host_data_tx_blocking(cdc_dev, (uint8_t *)text1, strlen(text1), 1000));
    //vTaskDelay(100);

    //static char text2[] = "AT+GSN\r";
    //ESP_ERROR_CHECK(cdc_acm_host_data_tx_blocking(cdc_dev, (uint8_t *)text2, strlen(text2), 1000));
    //vTaskDelay(100);

    // Test Line Coding commands: Get current line coding, change it 9600 7N1 and read again
    //ESP_LOGI(TAG, "Setting up line coding");

    cdc_acm_line_coding_t line_coding;
    //ESP_ERROR_CHECK(cdc_acm_host_line_coding_get(cdc_dev, &line_coding));
    //ESP_LOGI(TAG, "Line Get: Rate: %d, Stop bits: %d, Parity: %d, Databits: %d", line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

    line_coding.dwDTERate = 9600;
    line_coding.bDataBits = 8;
    line_coding.bParityType = 0;
    line_coding.bCharFormat = 1;
 
    ESP_ERROR_CHECK(cdc_acm_host_line_coding_set(cdc_dev, &line_coding));
    ESP_LOGI(TAG, "Line Set: Rate: %d, Stop bits: %d, Parity: %d, Databits: %d", line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

    ESP_ERROR_CHECK(cdc_acm_host_line_coding_get(cdc_dev, &line_coding));
    ESP_LOGI(TAG, "Line Get: Rate: %d, Stop bits: %d, Parity: %d, Databits: %d", line_coding.dwDTERate, line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

    ESP_ERROR_CHECK(cdc_acm_host_set_control_line_state(cdc_dev, true, false));

    //ESP_LOGI(TAG, "Example finished successfully! WOOHOO");
}





