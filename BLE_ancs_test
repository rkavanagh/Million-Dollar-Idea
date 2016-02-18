
// This is the draft of code for the million dollar file. 





//#include <app_timer.h>
//#include <app_timer_appsh.h>
#include <app_error.h>
#include <app_util.h>
#include <app_util_bds.h>
#include <app_util_platform.h>
#include <common.h>
#include <nordic_common.h>
#include <nrf_assert.h>
#include <nrf_log.h>
#include <sdk_common.h>
#include <sdk_errors.h>
#include <sdk_mapped_flags.h>
#include <sdk_os.h>
#include <sdk_resources.h>
//#include <ANCS.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "ble_ancs_c.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "nrf_gpio.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "nrf51_bitfields.h"
#include "device_manager.h"
#include "ble_err.h"
//#include "app_gpiote.h"
//#include "app_button.h"
//#include "app_timer.h"
#include "nrf_assert.h"
#include "pstorage.h"
#include "nrf_soc.h"
//#include "bsp.h"
#include "softdevice_handler.h"
#include "app_trace.h"
#include "ble_types.h"

#define BLE_UUID_UNKNOWN                              0x0000 /**< Reserved UUID. */
#define BLE_UUID_SERVICE_PRIMARY                      0x2800 /**< Primary Service. */
#define BLE_UUID_SERVICE_SECONDARY                    0x2801 /**< Secondary Service. */
#define BLE_UUID_SERVICE_INCLUDE                      0x2802 /**< Include. */
#define BLE_UUID_CHARACTERISTIC                       0x2803 /**< Characteristic. */
#define BLE_UUID_DESCRIPTOR_CHAR_EXT_PROP             0x2900 /**< Characteristic Extended Properties Descriptor. */
#define BLE_UUID_DESCRIPTOR_CHAR_USER_DESC            0x2901 /**< Characteristic User Description Descriptor. */
#define BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG        0x2902 /**< Client Characteristic Configuration Descriptor. */
#define BLE_UUID_DESCRIPTOR_SERVER_CHAR_CONFIG        0x2903 /**< Server Characteristic Configuration Descriptor. */
#define BLE_UUID_DESCRIPTOR_CHAR_PRESENTATION_FORMAT  0x2904 /**< Characteristic Presentation Format Descriptor. */
#define BLE_UUID_DESCRIPTOR_CHAR_AGGREGATE_FORMAT     0x2905 /**< Characteristic Aggregate Format Descriptor. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                                    /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define MAX_CHARACTERS_PER_LINE         16                                                   /**< Maximum characters that are visible in one line on nRF6350 display */
#define DISPLAY_BUFFER_SIZE             ANCS_ATTRIBUTE_DATA_MAX                              /**< Size of LCD display buffer */

//#define BOND_DELETE_ALL_WAKEUP_BUTTON_ID 0                                             /**< Button used for deleting all bonded masters/services during startup and to wake up */
//#define DISPLAY_MESSAGE_BUTTON_ID        1                                             /**< Button used to display message contents */

#define DEVICE_NAME                     "BLE NANO"                                               /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL                40                                                   /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_INTERVAL_SLOW           3200                                                 /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                                  /**< The advertising timeout in units of seconds. */
#define ADV_INTERVAL_FAST_PERIOD        30                                                   /**< The duration of the fast advertising period (in seconds). */

#define APP_TIMER_PRESCALER             0                                                    /**< Value of the RTC1 PRESCALER register. */
//#define APP_TIMER_MAX_TIMERS            (2+BSP_APP_TIMERS_NUMBER)                            /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                    /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)                     /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)                    /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)                      /**< Connection supervisory timeout (4 seconds). */

//#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)           /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
//#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)          /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                                    /**< Number of attempts before giving up the connection parameter negotiation. */

#define MESSAGE_BUFFER_SIZE             18                                                   /**< Size of buffer holding optional messages in notifications. */

#define APP_GPIOTE_MAX_USERS            1                                                    /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)             /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                                   /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                                    /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                                    /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                                 /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                                    /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                                    /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                                   /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                           /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

typedef enum
{
    BLE_NO_ADVERTISING,                                                                      /**< No advertising running. */
    BLE_SLOW_ADVERTISING,                                                                    /**< Slow advertising running. */
    BLE_FAST_ADVERTISING                                                                     /**< Fast advertising running. */
} ble_advertising_mode_t;


static const char *lit_catid[] =
{
    "Other",
    "IncomingCall",
    "MissedCall",
    "VoiceMail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "HealthAndFitness",
    "BusinessAndFinance",
    "Location",
    "Entertainment"
};

static const char *lit_eventid[] =
{
    "Added",
    "Modified",
    "Removed"
};

static ble_ancs_c_t                     m_ancs_c;                                            /**< Structure used to identify the Apple Notification Service Client. */
static uint8_t                          m_apple_message_buffer[MESSAGE_BUFFER_SIZE];         /**< Message buffer for optional notify messages. */

static ble_gap_adv_params_t             m_adv_params;                                        /**< Parameters to be passed to the stack when starting advertising. */
static ble_advertising_mode_t           m_advertising_mode;                                  /**< Variable to keep track of when we are advertising. */



static char                             display_title[DISPLAY_BUFFER_SIZE];
static char                             display_message[DISPLAY_BUFFER_SIZE];
static uint8_t                          m_ancs_uuid_type;
static dm_application_instance_t        m_app_handle;                                        /**< Application identifier allocated by device manager. */
static dm_handle_t                      m_peer_handle;                                       /**< Identifes the peer that is currently connected. */


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    if (m_advertising_mode == BLE_NO_ADVERTISING)
    {
        m_advertising_mode = BLE_FAST_ADVERTISING;
    }
    else
    {
        m_advertising_mode = BLE_SLOW_ADVERTISING;
    }

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;

    if (m_advertising_mode == BLE_FAST_ADVERTISING)
    {
        m_adv_params.interval = APP_ADV_INTERVAL;
        m_adv_params.timeout  = ADV_INTERVAL_FAST_PERIOD;
    }
    else
    {
        m_adv_params.interval = APP_ADV_INTERVAL_SLOW;
        m_adv_params.timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
    }

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, 
                                          (const uint8_t *)DEVICE_NAME, 
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    ble_uuid_t    ancs_uuid;
    
    ancs_uuid.uuid = ((ble_ancs_base_uuid128.uuid128[12]) | (ble_ancs_base_uuid128.uuid128[13] << 8));
    ancs_uuid.type = m_ancs_uuid_type;    
    
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
   // advdata.flags.size             = sizeof(flags); // .size before, idk what it actually 
    //advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = 0;
    advdata.uuids_complete.p_uuids  = NULL;
    advdata.uuids_solicited.uuid_cnt = 1;
    advdata.uuids_solicited.p_uuids  = &ancs_uuid;    
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Display an iOS notification
 *
 *
 * **********************THIS IS WHAT WE CHANGE *************************
 *
 *
 *
 */
void evt_ios_notification(ble_ancs_c_evt_ios_notification_t *p_notice)
{
//    char buffer[DISPLAY_BUFFER_SIZE];
//
//    printf("Category: %s\n",lit_catid[p_notice->category_id]);
//    
//    sprintf(buffer, "%s %02x%02x%02x%02x",
//        lit_eventid[p_notice->event_id], 
//        p_notice->notification_uid[0], p_notice->notification_uid[1], 
//        p_notice->notification_uid[2], p_notice->notification_uid[3]);
//
//    printf("Notification: %s\n",buffer);
}




static void evt_notif_attribute(ble_ancs_c_evt_notif_attribute_t *p_attr)
{
    if (p_attr->attribute_id == BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_TITLE)
    {
        memcpy(display_title, p_attr->data, p_attr->attribute_len);
    }
    else if (p_attr->attribute_id == BLE_ANCS_NOTIFICATION_ATTRIBUTE_ID_MESSAGE)
    {
        memcpy(display_message, p_attr->data, p_attr->attribute_len);
    }
}


/**@brief Function for handling the Apple Notification Service Client.
 *
 * @details This function will be called for all events in the Apple Notification Client which
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Apple Notification Service Client.
 * 
 * Used by service_add()
 */
static void on_ancs_c_evt(ble_ancs_c_evt_t * p_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_evt->evt_type)
    {
        case BLE_ANCS_C_EVT_DISCOVER_COMPLETE:
            err_code = dm_security_setup_req(&m_peer_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ANCS_C_EVT_IOS_NOTIFICATION:
            evt_ios_notification(&p_evt->data.notification);
            break;

        case BLE_ANCS_C_EVT_NOTIF_ATTRIBUTE:
            evt_notif_attribute(&p_evt->data.attribute);
            break;

        default:
            //No implementation needed
            break;
    }
}






/**@brief Function for handling the Apple Notification Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 * 
 * Called in service_add()
 * 
 */
static void apple_notification_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void service_add(void)
{
    ble_ancs_c_init_t ancs_init_obj;
    ble_uuid_t        service_uuid;
    uint32_t          err_code;
    bool              services_delete;

    //add the ANCS GATT profiles

    err_code = sd_ble_uuid_vs_add(&ble_ancs_base_uuid128, &m_ancs_uuid_type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_cp_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_ns_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_uuid_vs_add(&ble_ancs_ds_base_uuid128, &service_uuid.type);
    APP_ERROR_CHECK(err_code);

    memset(&ancs_init_obj, 0, sizeof(ancs_init_obj));
    memset(m_apple_message_buffer, 0, MESSAGE_BUFFER_SIZE);

    ancs_init_obj.evt_handler         = on_ancs_c_evt;
    ancs_init_obj.message_buffer_size = MESSAGE_BUFFER_SIZE;
    ancs_init_obj.p_message_buffer    = m_apple_message_buffer;
    ancs_init_obj.error_handler       = apple_notification_error_handler;

    err_code = ble_ancs_c_init(&m_ancs_c, &ancs_init_obj);
    APP_ERROR_CHECK(err_code);
    
    // Clear all discovered and stored services if the  "delete all bonds" button is pushed.
   // services_delete = bsp_buttons_state_get() & (1 << BOND_DELETE_ALL_WAKEUP_BUTTON_ID);

    if (services_delete)
    {
        err_code = ble_ans_c_service_delete();
        APP_ERROR_CHECK(err_code);
    }

    err_code = ble_ans_c_service_load(&m_ancs_c);
    APP_ERROR_CHECK(err_code);    
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = NULL;
    cp_init.next_conn_params_update_delay  = NULL;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}



void loop() {
  // put your main code here, to run repeatedly:

    uint32_t err_code;
    // Initialize.
    gap_params_init(); // done
    service_add(); 
    advertising_init();
    conn_params_init();

    // Start execution.
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }

}
