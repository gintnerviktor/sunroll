/*
 *
 */
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include <driver/gpio.h>

static const char *TAG = "gardenlight-lightbulb";

#define LEDCLOSE GPIO_NUM_21
#define LEDOPEN GPIO_NUM_22
#define RELAYCLOSE GPIO_NUM_33
#define RELAYOPEN GPIO_NUM_25

bool relayOpen_status = false;
bool relayClose_status = false;

/**
 * @brief initialize the lightbulb lowlevel module
 */
void relays_init(void)
{
    gpio_set_direction(LEDOPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDCLOSE, GPIO_MODE_OUTPUT);

    gpio_set_direction(RELAYOPEN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RELAYCLOSE, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "Initilalization done");
}

/**
 * @brief turn on/off the lowlevel relayOpen
 */
int relay1_set_on(bool value){
    ESP_LOGI(TAG, "relayOpen_set_on : %s", value == true ? "true" : "false");
    if ( value){
        gpio_set_level(LEDOPEN, 1);
        gpio_set_level(RELAYOPEN, 1);
        relayOpen_status = true;
    } else {
        gpio_set_level(LEDOPEN, 0);
        gpio_set_level(RELAYOPEN, 0);
        relayOpen_status = false;
    }
    return 0;
}

/**
 * @brief turn on/off the lowlevel relayClose
 */
int relay2_set_on(bool value)
{
    ESP_LOGI(TAG, "relayClose_set_on : %s", value == true ? "true" : "false");

    if ( value){
        gpio_set_level(LEDCLOSE, 1);
        gpio_set_level(RELAYCLOSE, 1);
        relayClose_status = true;
    } else {
        gpio_set_level(LEDCLOSE, 0);
        gpio_set_level(RELAYCLOSE, 0);
        relayClose_status = false;
    }
    return 0;
}

/**
 * @brief get relayOpen status
 * @return
 */
bool get_relayOpen_status(){
    return relayOpen_status;
}

/**
 * @brief get relayClose status
 * @return
 */
bool get_relayClose_status(){
    return relayClose_status;
}