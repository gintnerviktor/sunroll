/*
 *
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <hap_fw_upgrade.h>

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include "relays.h"

#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"

#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"

/* Comment out the below line to disable Firmware Upgrades */
#define CONFIG_FIRMWARE_SERVICE

static const char *TAG = "HAP sunroll";

#define SUNROLL_TASK_PRIORITY  1
#define SUNROLL_TASK_STACKSIZE 4 * 1024
#define SUNROLL_TASK_NAME      "hap_sunroll"

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        2

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0
#define LED GPIO_NUM_2

#define ADC_MOTOR_OPEN              ADC1_CHANNEL_4
#define ADC_MOTOR_CLOSE             ADC1_CHANNEL_3
#define ADC_KEY_OPEN                ADC1_CHANNEL_2
#define ADC_KEY_CLOSE               ADC1_CHANNEL_1

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

//ADC Calibration
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
static esp_adc_cal_characteristics_t adc1_chars;

#define ADC_UP_LIMIT           2500
#define ADC_DOWN_LIMIT         1500
#define MOTOR_DIRECTION_UP_COUNTER_LIMIT  5
#define MOTOR_DIRECTION_DOWN_COUNTER_LIMIT 10
#define KEY_UP_COUNTER_LIMIT  2
#define KEY_DOWN_COUNTER_LIMIT 2

#define TIME_TO_GO      600


hap_char_t *cpValueHC;
hap_char_t *psValueHC;
int timerCount = 0;
static bool timerChanged = false;
int ledOutput = 0;
int currentPosition;
int targetPosition;
int notifyTimerCount = 0;
int motorDirectionOpenUpCounter = 0;
int motorDirectionOpenDownCounter = 0;
int motorDirectionCloseUpCounter = 0;
int motorDirectionCloseDownCounter = 0;
int openKeyUpCounter = 0;
int openKeyDownCounter = 0;
int closeKeyUpCounter = 0;
int closeKeyDownCounter = 0;

typedef enum {
    STATUS_OFF = 0,
    STATUS_ON = 1,
} status;
static status openMotorStatus = STATUS_OFF;
static status closeMotorStatus = STATUS_OFF;
static status openKeyStatus = STATUS_OFF;
static status closeKeyStatus = STATUS_OFF;

typedef enum {
    PS_GOINGTO_MINIMUM = 0,
    PS_GOINGTO_MAXIMUM = 1,
    PS_STOPPED = 2,
} psState;
static psState positionState = PS_STOPPED;

/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(uint32_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int sunroll_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/*
 * An optional HomeKit Event handler which can be used to track HomeKit
 * specific events.
 */
static void fan_hap_event_handler(void* arg, esp_event_base_t event_base, int32_t event, void *data)
{
    switch(event) {
        case HAP_EVENT_PAIRING_STARTED :
            ESP_LOGI(TAG, "Pairing Started");
            break;
        case HAP_EVENT_PAIRING_ABORTED :
            ESP_LOGI(TAG, "Pairing Aborted");
            break;
        case HAP_EVENT_CTRL_PAIRED :
            ESP_LOGI(TAG, "Controller %s Paired. Controller count: %d",
                     (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_UNPAIRED :
            ESP_LOGI(TAG, "Controller %s Removed. Controller count: %d",
                     (char *)data, hap_get_paired_controller_count());
            break;
        case HAP_EVENT_CTRL_CONNECTED :
            ESP_LOGI(TAG, "Controller %s Connected", (char *)data);
            break;
        case HAP_EVENT_CTRL_DISCONNECTED :
            ESP_LOGI(TAG, "Controller %s Disconnected", (char *)data);
            break;
        case HAP_EVENT_ACC_REBOOTING : {
            char *reason = (char *)data;
            ESP_LOGI(TAG, "Accessory Rebooting (Reason: %s)",  reason ? reason : "null");
            break;
            case HAP_EVENT_PAIRING_MODE_TIMED_OUT :
                ESP_LOGI(TAG, "Pairing Mode timed out. Please reboot the device.");
        }
        default:
            /* Silently ignore unknown events */
            break;
    }
}

/* Callback for handling writes on the Light Bulb Service
 */
static int sunroll_write(hap_write_data_t write_data[], int count,
        void *serv_priv, void *write_priv)
{
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        /* Setting a default error value */
        *(write->status) = HAP_STATUS_VAL_INVALID;
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_TARGET_POSITION)) {
            ESP_LOGI(TAG, "Target position write: %d", write->val.i);
            targetPosition = write->val.i;

            *(write->status) = HAP_STATUS_SUCCESS;
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
        /* If the characteristic write was successful, update it in hap core
         */
        if (*(write->status) == HAP_STATUS_SUCCESS) {
            hap_char_update_val(write->hc, &(write->val));
        } else {
            /* Else, set the return value appropriately to report error */
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static int sunroll_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv)
{
    if (hap_req_get_ctrl_id(read_priv)) {
        ESP_LOGI(TAG, "Received read from %s", hap_req_get_ctrl_id(read_priv));
    }

    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_POSITION)) {

        ESP_LOGI(TAG, "Current position lekérdezés: %d", currentPosition);
        hap_val_t new_val;
        new_val.i = currentPosition;

        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;

        const hap_val_t *val = hap_char_get_val(hc);

        cpValueHC = hc;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_POSITION_STATE)) {

        ESP_LOGI(TAG, "Position state lekérdezés: %d", positionState);
        hap_val_t new_val;
        new_val.i = positionState;

        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;

        const hap_val_t *val = hap_char_get_val(hc);

        psValueHC = hc;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_TARGET_POSITION)) {

        ESP_LOGI(TAG, "Target position lekérdezés: %d", targetPosition);
        hap_val_t new_val;
        new_val.i = targetPosition;

        hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;

        const hap_val_t *val = hap_char_get_val(hc);

        psValueHC = hc;
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_NAME)) {
        ESP_LOGI(TAG, "Név lekérdezése");
        hap_val_t new_val;
        new_val.s = "sun_roll";
        //hap_char_update_val(hc, &new_val);
        *status_code = HAP_STATUS_SUCCESS;
    } else {
        ESP_LOGI(TAG, "Ismeretlen lekérdezés %s", hap_char_get_type_uuid(hc));
        *status_code = HAP_STATUS_SUCCESS;
    }
    return HAP_SUCCESS;
}

/*The main thread for handling the Light Bulb Accessory */
static void sunroll_thread_entry(void *arg)
{
    hap_acc_t *accessory;
    hap_serv_t *service;

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "Sunroll",
        .manufacturer = "DigitInvent kft.",
        .model = "Sunroll01",
        .serial_num = "10000001",
        .fw_rev = "0.9.0",
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .identify_routine = sunroll_identify,
        .cid = HAP_CID_LIGHTING,
    };

    /* Create accessory object */
    accessory = hap_acc_create(&cfg);
    if (!accessory) {
        ESP_LOGE(TAG, "Failed to create accessory");
        goto light_err;
    }

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* Create the window_covering Service. Include the "name" since this is a user visible service  */
    service = hap_serv_window_covering_create(0,0,0);
    if (!service) {
        ESP_LOGE(TAG, "Failed to create window_covering Service");
        goto light_err;
    }

    /* Add the optional characteristic to the Light Bulb Service */
    int ret = hap_serv_add_char(service, hap_char_name_create("Sunroll"));
    
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to sunroll");
        goto light_err;
    }
    /* Set the write callback for the service */
    hap_serv_set_write_cb(service, sunroll_write);

    /* Set the read callback for the service */
    hap_serv_set_read_cb(service, sunroll_read);
    
    /* Add the window_covering Service to the Accessory Object */
    hap_acc_add_serv(accessory, service);

#ifdef CONFIG_FIRMWARE_SERVICE
    /*  Required for server verification during OTA, PEM format as string  */
    static char server_cert[] = {};
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem = server_cert,
    };
    /* Create and add the Firmware Upgrade Service, if enabled.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    service = hap_serv_fw_upgrade_create(&ota_config);
    if (!service) {
        ESP_LOGE(TAG, "Failed to create Firmware Upgrade Service");
        goto light_err;
    }
    hap_acc_add_serv(accessory, service);
#endif

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Initialize the relays Hardware */
    relays_init();

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* TODO: Do the actual hardware initialization here */

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, &fan_hap_event_handler, NULL);

    /* After all the initializations are done, start the HAP core */
    hap_start();
    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);

light_err:
    hap_acc_delete(accessory);
    vTaskDelete(NULL);
}

typedef struct {
    uint64_t event_count;
} example_queue_element_t;

static bool IRAM_ATTR example_timer_on_alarm_cb_v1(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;

    timerChanged = true;
    timerCount++;

    if ( timerCount == 10){
        timerCount = 0;
        if (ledOutput == 0){
            gpio_set_level(LED, 1);
            ledOutput = 1;
        } else {
            gpio_set_level(LED, 0);
            ledOutput = 0;
        }
    }

// Retrieve count value and send to queue
example_queue_element_t ele = {
        .event_count = edata->count_value
};
xQueueSendFromISR(queue, &ele, &high_task_awoken);
// return whether we need to yield at the end of ISR
return (high_task_awoken == pdTRUE);
}


static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

void app_main()
{
    gptimer_handle_t gptimer = NULL;
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    xTaskCreate(sunroll_thread_entry, SUNROLL_TASK_NAME, SUNROLL_TASK_STACKSIZE,
            NULL, SUNROLL_TASK_PRIORITY, NULL);

    example_queue_element_t ele;
    QueueHandle_t queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!queue) {
        ESP_LOGE(TAG, "Creating queue failed");
        return;
    }

    ESP_LOGI(TAG, "Create timer1 handle");

    gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
            .on_alarm = example_timer_on_alarm_cb_v1,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));

    gptimer_alarm_config_t alarm_config1 = {
            .reload_count = 0,
            .alarm_count = 100000, // period = 0.1s
            .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));

    if ( gptimer != NULL){
        ESP_ERROR_CHECK(gptimer_start(gptimer));
        ESP_LOGI(TAG, "Timer elindítva");
    }

    /*
     * Analog csatornák inicializálása
     */
    esp_err_t ret = ESP_OK;
    uint32_t voltage = 0;
    bool cali_enable = adc_calibration_init();

    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_MOTOR_OPEN, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_MOTOR_CLOSE, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_KEY_OPEN, ADC_EXAMPLE_ATTEN));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_KEY_CLOSE, ADC_EXAMPLE_ATTEN));

    while (1){
        if ( timerChanged){
            timerChanged = false;

            /*
             * Vizsgáljuk a nyitás irányú motort
             * Elvégezzük az analog konverziót
             */
            int adcUpValue = adc1_get_raw(ADC_MOTOR_OPEN);
            if ( adcUpValue < ADC_DOWN_LIMIT || adcUpValue >= ADC_UP_LIMIT){
                // Határértékeken kívül van, valószínűleg áram alatt.
                motorDirectionOpenDownCounter = 0;
                motorDirectionOpenUpCounter++;
                if ( motorDirectionOpenUpCounter >= MOTOR_DIRECTION_UP_COUNTER_LIMIT){
                    motorDirectionOpenUpCounter = MOTOR_DIRECTION_UP_COUNTER_LIMIT;
                    if ( openMotorStatus == STATUS_OFF){
                        openMotorStatus = STATUS_ON;
                        ESP_LOGI(TAG, "Openmotor ON");
                    }

                }
            } else {
                // Határértékeken belül van, valószínűleg áll.
                motorDirectionOpenUpCounter = 0;
                motorDirectionOpenDownCounter++;
                if ( motorDirectionOpenDownCounter >= MOTOR_DIRECTION_DOWN_COUNTER_LIMIT){
                    motorDirectionOpenDownCounter = MOTOR_DIRECTION_DOWN_COUNTER_LIMIT;
                    if ( openMotorStatus == STATUS_ON){
                        openMotorStatus = STATUS_OFF;
                        ESP_LOGI(TAG, "Openmotor OFF");
                    }
                }
            }

            /*
             * Vizsgáljuk a zárás irányú motort
             * Elvégezzük az analog konverziót
             */
            int adcDownValue = adc1_get_raw(ADC_MOTOR_CLOSE);
            if ( adcDownValue < ADC_DOWN_LIMIT || adcDownValue >= ADC_UP_LIMIT){
                // Határértékeken kívül van, valószínűleg áram alatt.
                motorDirectionCloseDownCounter = 0;
                motorDirectionCloseUpCounter++;
                if ( motorDirectionCloseUpCounter >= MOTOR_DIRECTION_UP_COUNTER_LIMIT){
                    motorDirectionCloseUpCounter = MOTOR_DIRECTION_UP_COUNTER_LIMIT;
                    if ( closeMotorStatus == STATUS_OFF){
                        closeMotorStatus = STATUS_ON;
                        ESP_LOGI(TAG, "Closemotor ON");
                    }

                }
            } else {
                // Határértékeken belül van, valószínűleg áll.
                motorDirectionCloseUpCounter = 0;
                motorDirectionCloseDownCounter++;
                if ( motorDirectionCloseDownCounter >= MOTOR_DIRECTION_DOWN_COUNTER_LIMIT){
                    motorDirectionCloseDownCounter = MOTOR_DIRECTION_DOWN_COUNTER_LIMIT;
                    if ( closeMotorStatus == STATUS_ON){
                        closeMotorStatus = STATUS_OFF;
                        ESP_LOGI(TAG, "Closemotor OFF");
                    }
                }
            }

            /*
             * Vizsgáljuk az openKey-t
             * Elvégezzük az analog konverziót
             */
            int adcOpenKeyValue = adc1_get_raw(ADC_KEY_OPEN);
            if ( adcOpenKeyValue >= ADC_UP_LIMIT){
                // Határértékeken kívül van, valószínűleg áram alatt.
                openKeyDownCounter = 0;
                openKeyUpCounter++;
                if ( openKeyUpCounter >= KEY_UP_COUNTER_LIMIT){
                    openKeyUpCounter = KEY_UP_COUNTER_LIMIT;
                    if ( openKeyStatus == STATUS_OFF){
                        openKeyStatus = STATUS_ON;
                        ESP_LOGI(TAG, "Openkey ON");
                    }

                }
            } else {
                // Határértékeken belül van, nincs feszültség alatt
                openKeyUpCounter = 0;
                openKeyDownCounter++;
                if ( openKeyDownCounter >= KEY_DOWN_COUNTER_LIMIT){
                    openKeyDownCounter = KEY_DOWN_COUNTER_LIMIT;
                    if ( openKeyStatus == STATUS_OFF){
                        openKeyStatus = STATUS_ON;
                        ESP_LOGI(TAG, "Openkey OFF");
                    }
                }
            }

            /*
             * Vizsgáljuk a closeKey-t
             * Elvégezzük az analog konverziót
             */
            int adcCloseKeyValue = adc1_get_raw(ADC_KEY_CLOSE);
            if ( adcCloseKeyValue >= ADC_UP_LIMIT){
                // Határértékeken kívül van, valószínűleg áram alatt.
                closeKeyDownCounter = 0;
                closeKeyUpCounter++;
                if ( closeKeyUpCounter >= KEY_UP_COUNTER_LIMIT){
                    closeKeyUpCounter = KEY_UP_COUNTER_LIMIT;
                    if ( closeKeyStatus == STATUS_OFF){
                        closeKeyStatus = STATUS_ON;
                        ESP_LOGI(TAG, "Closekey ON");
                    }

                }
            } else {
                // Határértékeken belül van, nincs feszültség alatt
                closeKeyUpCounter = 0;
                closeKeyDownCounter++;
                if ( closeKeyDownCounter >= KEY_DOWN_COUNTER_LIMIT){
                    closeKeyDownCounter = KEY_DOWN_COUNTER_LIMIT;
                    if ( closeKeyStatus == STATUS_OFF){
                        closeKeyStatus = STATUS_ON;
                        ESP_LOGI(TAG, "Closekey OFF");
                    }
                }
            }

            //ESP_LOGI(TAG, "Current sensor value: %d", adcValue);

            notifyTimerCount++;
            if ( notifyTimerCount >= 50){
                //ESP_LOGI(TAG, "It's time to notify...");
                hap_val_t new_val;
                new_val.i = currentPosition;
                hap_char_update_val(cpValueHC, &new_val);

                notifyTimerCount = 0;
            }

            /*
             * Pozíció kalkuláció
             */
        }
    }
}
