
#include <stdio.h>
#include <string.h>
#include "esp_spiffs.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "freertos/event_groups.h"
#include "ds18b20.c"
#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#define TAG "WSN_SENSOR"
#define TAG_BACKUP "BACKUP_DATA"
static char *MOUNT_POINT = "/vendor_server";
static uint8_t temp_check_ack = 0;
#define CID_ESP 0x02E5
// D:/esp/esp-idf-ftpClient-master

// ===================== SPIFFS VARIABLE ====================================
esp_err_t ret;
esp_err_t err;
nvs_handle_t my_handle;
static char srcFileName[64];
static char dstFileName[64];
float thresh_temper = 0.0;
int red_rx = 0;
int yel_rx = 0;
int gre_rx = 0;
// =======================  SHOW BACKUP DATA VARIABLE =============================
#define INPUT_PIN 0
int state = 0;
xQueueHandle interputQueue;

// ======================  ADC BATTERY VARIABLE  =================================
#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64 // Multisampling
int Battery_value;
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC1_CHANNEL_0; // GPIO
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11; // 3.3V range

// ======================  SLEEP MODE VARIABLE  =================================
#define uS_TO_S_FACTOR 1000000 // Conversion microseconds to seconds
#define TIME_TO_SLEEP 19       // Time ESP32 will go to sleep (seconds)
// ======================  BLE VARIABLE  =================================
static EventGroupHandle_t rx_task;
#define START_CONNECTED_BIT BIT0
#define START_RETRANSMIT_BIT BIT1
#define TEMPERATURE_GROUP_ADDRESS 0xC001
#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT 0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};
#define SENSOR_PROPERTY_ID_0 0x0056 /* Present Indoor Ambient Temperature */
#define SENSOR_PROPERTY_ID_1 0x005B /* Present Outdoor Ambient Temperature */
static uint8_t net_buf_data_sensor_data_0[10];
static uint8_t net_buf_data_sensor_data_1[10];

static struct net_buf_simple sensor_data_0 = {
    .data = net_buf_data_sensor_data_0,
    .len = 0,
    .size = 10,
    .__buf = net_buf_data_sensor_data_0,
};

static struct net_buf_simple sensor_data_1 = {
    .data = net_buf_data_sensor_data_1,
    .len = 0,
    .size = 10,
    .__buf = net_buf_data_sensor_data_1,
};
#define SENSOR_POSITIVE_TOLERANCE ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL
static esp_ble_mesh_sensor_state_t sensor_states[2] = {

    [0] = {

        .sensor_property_id = SENSOR_PROPERTY_ID_0,
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 0, /* 0 represents the length is 1 */
        .sensor_data.raw_value = &sensor_data_0,
    },
    [1] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_1,
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 0, /* 0 represents the length is 1 */
        .sensor_data.raw_value = &sensor_data_1,
    },
};

/* 20 octets is large enough to hold two Sensor Descriptor state values. */
static uint8_t net_buf_data_bt_mesh_pub_msg_sensor_pub[20];
static struct net_buf_simple bt_mesh_pub_msg_sensor_pub = {
    .data = net_buf_data_bt_mesh_pub_msg_sensor_pub,
    .len = 10,
    .size = 20,
    .__buf = net_buf_data_bt_mesh_pub_msg_sensor_pub,
};
static esp_ble_mesh_model_pub_t sensor_pub = {
    .update = (uint32_t)NULL,
    .msg = &bt_mesh_pub_msg_sensor_pub,
    .dev_role = ROLE_NODE,
};
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 20, ROLE_NODE);
static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

static esp_ble_mesh_model_t root_models[] = {
    // model sensor server
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    // model vendor sever
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
                              vnd_op, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static struct example_info_store
{
    uint16_t net_idx;                    /* NetKey Index */
    uint16_t app_idx; /* AppKey Index */ /* Remote OnOff */
    uint8_t task_send_check;
    esp_ble_mesh_model_cb_param_t param_provison;
    esp_ble_mesh_sensor_server_cb_param_t param_po;
    uint8_t ack_check; // biến báo ack từ gw
    float red;         // ngưỡng đỏ
    float yellow;      // ngưỡng vàng
    float green;       // ngưỡng xanh
    uint8_t tid;       /* Message TID */
} __attribute__((packed)) store = {
    .net_idx = ESP_BLE_MESH_KEY_UNUSED,
    .app_idx = ESP_BLE_MESH_KEY_UNUSED,
    .task_send_check = 0,
    .ack_check = 1,
    .red = 15.0,
    .yellow = 10.0,
    .green = 5.0,
    .tid = 0x0,
};
static nvs_handle_t NVS_HANDLE;
static const char *NVS_KEY = "Node1";

static void IRAM_ATTR gpio_interrupt_handler(void *args) // hàm ISR ngắt ngoài
{
    int pinNumber = (int)args;
    xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

/**
 *  @brief Hàm này lưu lại thông tin và trạng thái của sensor node
 *
 *  @return None
 */
static void mesh_example_info_store(void)
{
    ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
}
/**
 *  @brief Hàm này khôi phục lại thông tin và trạng thái của sensor node
 *
 *  @return None
 */
static void mesh_example_info_restore(void)
{
    esp_err_t err = ESP_OK;
    bool exist = false;

    err = ble_mesh_nvs_restore(NVS_HANDLE, NVS_KEY, &store, sizeof(store), &exist);
    if (err != ESP_OK)
    {
        return;
    }
    if (store.task_send_check == 1)
    {
        xEventGroupSetBits(rx_task, START_CONNECTED_BIT);
    }
    if (exist)
    {
        ESP_LOGI(TAG, "Restore, net_idx 0x%04x, app_idx 0x%04x, tid 0x%02x",
                 store.net_idx, store.app_idx, store.tid);
    }
}
/**
 *  @brief Hàm này khởi tạo thông tin về SPIFFS
 *
 *  @param[in] partition_label tên vùng trên SPIFFS
 *  @param[in] mount_point tên file
 *  @return None
 */
esp_err_t mountSPIFFS(char *partition_label, char *mount_point)
{
    ESP_LOGI(TAG, "Initializing SPIFFS file system");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = mount_point,
        .partition_label = partition_label,
        .max_files = 5,
        .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
        ESP_LOGI(TAG, "Mount SPIFFS filesystem on %s", mount_point);
    }
    // ESP_LOGI(TAG, "Mount SPIFFS filesystem");
    return ret;
}
/**
 *  @brief Config SPIFFS
 *
 *  @return None
 */
void SPIFFS_Config(void)
{
    char *partition_label = "storage0";
    ret = mountSPIFFS(partition_label, MOUNT_POINT);
}

/**
 *  @brief Hàm này lưu lại dữ liệu backup vào file trong SPIFFS
 *
 *  @param[in] data Dữ liệu cần lưu
 *  @return None
 */
static FILE *file;
void Backup_data_handler(char *data)
{
    sprintf(srcFileName, "%s/backupdata.txt", MOUNT_POINT);
    file = fopen(srcFileName, "a"); // tạo file
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return 0;
    }
    fprintf(file, "%s  ", data); // lưu dữ liệu backup vào file
    fclose(file);                // đóng file
}
/**
 *  @brief Hàm hiện dữ liệu backup lên monitor
 *
 *  @return None
 */
void show_backup_Data(void)
{
    char line[500];
    file = fopen("/vendor_server/backupdata.txt", "r"); // mở file backupdata.txt trong SPIFFS
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    fgets(line, sizeof(line), file); // lấy dữ liệu trong file
    file = fopen(srcFileName, "w");  // xoá dữ liệu trong file
    fclose(file);
    ESP_LOGI(TAG_BACKUP, "%s ", line); // hiện lên monitor
}
/**
 *  @brief Task này thực hiện gửi backup data qua serial monitor
 *
 *  @param[in] params Tham số truyền vào khi tạo Task
 *  @return None
 */
void Show_Backup_Task(void *params)
{
    int pinNumber, count = 0;
    while (true)
    {
        if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY)) // chờ có items
        {
            printf("Du lieu khi mat gateway: \n");
            show_backup_Data(); // show dữ liệu backup
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

char data_temper[15];
char data_backup[15];
/**
 *  @brief Task này thực hiện đọc dữ liệu DS18B20 và truyền bản tin đến gateway chu kỳ 20s
 *
 *  @param[in] pvParameter Tham số truyền vào khi tạo Task
 *  @return None
 */
void DS18B20_Task(void *pvParameter)
{
    char data[10];
    float Temperature = 0;
    uint8_t Presence = 0;
    uint8_t Temp_byte1, Temp_byte2;
    uint16_t TEMP;
    while (1)
    {
        xEventGroupWaitBits(rx_task, START_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY); // Chờ Bits START_CONNECTED_BIT
        Presence = DS18B20_Start();
        vTaskDelay(1 / (portTICK_RATE_MS));
        DS18B20_Write(0xCC);                  // skip ROM
        DS18B20_Write(0x44);                  // convert t
        vTaskDelay(750 / (portTICK_RATE_MS)); // wait convert complete

        Presence = DS18B20_Start();
        vTaskDelay(1 / (portTICK_RATE_MS));
        DS18B20_Write(0xCC); // skip ROM
        DS18B20_Write(0xBE); // Read Scratch-pad

        Temp_byte1 = DS18B20_Read(); // read 8 bit dau
        Temp_byte2 = DS18B20_Read(); // read 8 bit sau
        TEMP = (Temp_byte2 << 8) | Temp_byte1;
        Temperature = (float)TEMP / 16;
        if (Temperature < store.green) // nếu nhiệt độ hiện tại nhỏ hơn ngưỡng xanh
        {
            printf("Temperature < store.green\n");
            gpio_set_level(25, 1);
            gpio_set_level(26, 1);
            gpio_set_level(27, 0);
        }
        else if (Temperature > store.green && Temperature < store.yellow)
        {
            printf("Temperature > store.green && Temperature < store.yellow\n");
            gpio_set_level(25, 1);
            gpio_set_level(26, 0);
            gpio_set_level(27, 1);
        }
        else if (Temperature > store.yellow)
        {
            printf("Temperature > store.yellow\n");
            gpio_set_level(25, 0);
            gpio_set_level(26, 1);
            gpio_set_level(27, 1);
        }
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            Battery_value += adc1_get_raw((adc1_channel_t)channel);
        }
        Battery_value /= 64;
        float VBAT = (200.0f / 100.0f) * 3.30f * (Battery_value) / 4096.0f; // LiPo battery
        float percentBattery = ((VBAT - 2.5) / (1.70)) * 100;               // phần trăm pin
        printf("Dien ap cua Battery la: %f %.1f\n", VBAT, percentBattery + 20);
        sprintf(data_temper, "1 %.1f %.0f", Temperature, percentBattery + 20);
        if (percentBattery < 20.0)
        {
            gpio_set_level(22, 0); // bật led báo sắp hết pin
        }
        else
        {
            gpio_set_level(22, 1);
        }

        if (Temperature < 125.0)
        {
            if (store.ack_check == 0) // nếu gửi bản tin không thành công thì backup dữ liệu
            {
                xEventGroupClearBits(rx_task, START_RETRANSMIT_BIT);
                sprintf(data_backup, "%.1f ", Temperature);
                Backup_data_handler(data_backup);
            }
            store.ack_check = 0;
            // gửi dữ liệu nhiệt độ và phần trăm pin đến gw
            esp_ble_mesh_server_model_send_msg(store.param_po.model, &store.param_po.ctx,
                                               ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, strlen(data_temper), (uint8_t *)data_temper);
        }

        vTaskDelay(20000 / portTICK_PERIOD_MS); // Blocked trong 20s
    }
    vTaskDelete(NULL);
}
int cout = 0;
/**
 *  @brief Task này thực hiện truyền lại bản tin dữ liệu đến Gateway
 *
 *  @param[in] pvParameter Tham số truyền vào khi tạo Task
 *  @return None
 */
void ReTransmit_Task(void *pvParameter)
{
    while (1)
    {
        xEventGroupWaitBits(rx_task, START_RETRANSMIT_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        if (cout >= 1)
        {
            // gửi bản tin theo sensor model
            esp_ble_mesh_server_model_send_msg(store.param_po.model, &store.param_po.ctx,
                                               ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, strlen(data_temper), (uint8_t *)data_temper);
        }
        if (cout >= 3)
        { // gửi lại tối đa 3 lần xong xoá Task
            vTaskDelete(NULL);
        }
        cout++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
}
/**
 *  @brief Hàm này được gọi lại KHI Gateway gửi yêu cầu cấp phép cho Sensor NODE
 *
 *  @param[in] event Event của callBack
 *  @param[in] param Thông tin của sensor node
 *  @return None
 */
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT: // Event khi đăng ký callback xong
        rx_task = xEventGroupCreate();
        mesh_example_info_restore();                                               // Khôi phục lại thông tin của gateway
        xTaskCreate(&ReTransmit_Task, "ReTransmit_Task", 2516 * 2, NULL, 5, NULL); // task thực hiện gửi lại bản tin nếu thất bại
        xTaskCreate(&DS18B20_Task, "nhan du lieu", 2516, NULL, 5, NULL);           // task đọc dữ liệu và gửi lên cho gateway
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT: // Event báo đã phát bản tin quảng cáo
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT: // Event báo Gateway đã Scan được ADV
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT: // Đóng phát bản tin ADV
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT: // Hoàn thành việc cấp phép
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}
/**
 *  @brief Hàm này được gọi lại nếu Sensor node Bind App key với các Model thành công
 *
 *  @param[in] event Event của callBack
 *  @param[in] param Thông tin của sensor node
 *  @return None
 */
static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT)
    {
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                     param->value.state_change.appkey_add.net_idx,
                     param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_app_bind.element_addr,
                     param->value.state_change.mod_app_bind.app_idx,
                     param->value.state_change.mod_app_bind.company_id,
                     param->value.state_change.mod_app_bind.model_id);
            store.app_idx = param->value.state_change.mod_app_bind.app_idx;
            mesh_example_info_store();                                                 /* Store proper mesh example info */
            rx_task = xEventGroupCreate();                                             // tạo eventGroup
            xTaskCreate(&ReTransmit_Task, "ReTransmit_Task", 2516 * 2, NULL, 5, NULL); // task thực hiện gửi lại bản tin nếu thất bại
            xTaskCreate(&DS18B20_Task, "nhan du lieu", 2516, NULL, 5, NULL);           // task đọc dữ liệu và gửi lên cho gateway
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_PUB_SET");
            ESP_LOGI(TAG, "elem_addr 0x%04x, pub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                     param->value.state_change.mod_pub_set.element_addr,
                     param->value.state_change.mod_pub_set.pub_addr,
                     param->value.state_change.mod_pub_set.company_id,
                     param->value.state_change.mod_pub_set.model_id);
            break;
        default:
            break;
        }
    }
}
/**
 *  @brief Hàm này được gọi lại nếu như Gateway request GetData Sensor Model đến Node
 *
 *  @param[in] event Event của callBack
 *  @param[in] param Thông tin của Gateway
 *  @return None
 */
static void example_ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                              esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
             event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);
    esp_ble_mesh_sensor_server_cb_param_t *param_temp = (esp_ble_mesh_sensor_server_cb_param_t *)param; // trở tới dữ liệu của gateway
    switch (event)
    {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT: // Event nhận thông tin model từ gateway
        switch (param->ctx.recv_op)
        {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "Gui data lan thu ESP_BLE_MESH_MODEL_OP_SENSOR_GET");
            store.param_po = *param_temp;
            store.task_send_check = 1;
            mesh_example_info_store();                        // lưu lại cờ check đã nhận thông tin lần đầu của gateway
            xEventGroupSetBits(rx_task, START_CONNECTED_BIT); // xét EventBit START_CONNECTED_BIT để Task DS18B20 UNBLOCKED
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04x", param->ctx.recv_op);
            return;
        }
        break;
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        switch (param->ctx.recv_op)
        {
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04x", param->ctx.recv_op);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Server event %d", event);
        break;
    }
}

/**
 *  @brief Hàm này được gọi lại nếu như có Event liên quan đến truyền nhận tin nhắn giữa các Node và Gateway
 *
 *  @param[in] event Event của callBack
 *  @param[in] param Thông tin của Gateway
 *  @return None
 */
static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    esp_ble_mesh_model_cb_param_t *param_temp = (esp_ble_mesh_model_cb_param_t *)param;
    char temp[5];
    char *ptr;
    switch (event)
    {

    case ESP_BLE_MESH_MODEL_OPERATION_EVT: // Event nhận được tin nhắn từ Gateway
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND)
        {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            char data_rx_1[50];
            sprintf(data_rx_1, "%.*s", param->model_operation.length, (char *)(param->model_operation.msg));
            ESP_LOGI(TAG, "Sensor Data: %.*s\n", param->model_operation.length, (char *)(param->model_operation.msg));
            if (strstr(data_rx_1, "Red") != NULL)
            {
                ESP_LOGI(TAG, "Sensor Data: %.*s\n", param->model_operation.length, (char *)(param->model_operation.msg));
                red_rx = 1;
                strtok(data_rx_1, " ");
                ptr = strtok(NULL, " ");
                sprintf(temp, "%s", ptr);
                thresh_temper = atof(temp);
                printf("%f\n", thresh_temper);
                store.red = thresh_temper;
                mesh_example_info_store();
                esp_restart();
            }
            else if (strstr(data_rx_1, "Yellow") != NULL)
            {
                ESP_LOGI(TAG, "Sensor Data: %.*s\n", param->model_operation.length, (char *)(param->model_operation.msg));
                yel_rx = 1;
                strtok(data_rx_1, " ");
                ptr = strtok(NULL, " ");
                sprintf(temp, "%s", ptr);
                thresh_temper = atof(temp);
                store.yellow = thresh_temper;
                printf("%f\n", thresh_temper);
                mesh_example_info_store();
                esp_restart();
            }
            else if (strstr(data_rx_1, "Green") != NULL)
            {
                ESP_LOGI(TAG, "Sensor Data: %.*s\n", param->model_operation.length, (char *)(param->model_operation.msg));
                gre_rx = 1;
                strtok(data_rx_1, " ");
                ptr = strtok(NULL, " ");
                sprintf(temp, "%s", ptr);
                thresh_temper = atof(temp);
                printf("%f\n", thresh_temper);
                store.green = thresh_temper;
                mesh_example_info_store();
                esp_restart();
            }
            else if (strstr(data_rx_1, "ACK") != NULL)
            {
                ESP_LOGI(TAG, "Sensor Data: %.*s\n", param->model_operation.length, (char *)(param->model_operation.msg));
                store.ack_check = 1;
                mesh_example_info_store();
                esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
                ESP_LOGI(TAG, "Entering deep sleep \n");
                gpio_hold_dis(GPIO_NUM_22);
                gpio_hold_dis(GPIO_NUM_25);
                gpio_hold_dis(GPIO_NUM_26);
                gpio_hold_dis(GPIO_NUM_27);
                gpio_hold_en(GPIO_NUM_25);
                gpio_hold_en(GPIO_NUM_26);
                gpio_hold_en(GPIO_NUM_22);
                gpio_hold_en(GPIO_NUM_27);
                gpio_deep_sleep_hold_en();
                esp_deep_sleep_start();
            }
            else
            {
                ESP_LOGI(TAG, "Mat ket noi voi Cluster\n");
            }
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT: // Send message xong sẽ vào đây
        xEventGroupSetBits(rx_task, START_RETRANSMIT_BIT);
        if (param->model_send_comp.err_code)
        {
            ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
            break;
        }
        break;
    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
    {
        xEventGroupSetBits(rx_task, START_RETRANSMIT_BIT);
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT, err_code %d",
                 param->model_publish_comp.err_code);
        if (param->model_publish_comp.err_code)
        {
            ESP_LOGE(TAG, "Failed to publish message 0x%06x", param->model_publish_comp.err_code);
            break;
        }
        break;
    }
    default:
        break;
    }
}

/**
 *  @brief Khởi tạo BLE Mesh module và đăng ký các hàm callback xử lý event
 *
 *  @return None
 */
static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);           // function callback xử lý event cấp phép của Node
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb); // Register BLE Mesh Config Server Model callback.
    esp_ble_mesh_register_sensor_server_callback(example_ble_mesh_sensor_server_cb); // Register BLE Mesh Sensor Server Model callback
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);   // function callback được gọi khi

    esp_ble_mesh_init(&provision, &composition); // Initialize BLE Mesh module
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT); // Phát các gói tin quảng cáo
    return ESP_OK;
}
/**
 *  @brief Config các chân đầu ra
 *
 *  @return None
 */
void output_create(int pin)
{
    gpio_config_t io_conf;
    // disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << pin);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(pin, 1);
}
/**
 *  @brief Config các chân đầu vào
 *
 *  @return None
 */
void input_create(int pin)
{
    gpio_config_t io_conf;
    // disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << pin);
    // disable pull-down mode
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
}

void app_main(void)
{
    esp_err_t err;
    /*Khởi tạo GPIO, ADC, NVS_Flash*/
    ESP_LOGI(TAG, "Initializing...");
    adc1_config_width(width);                  // adc 12bit
    adc1_config_channel_atten(channel, atten); // Range 3.3V
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    ds18b20_init(); // Khởi tạo DS18B20
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    SPIFFS_Config();
    input_create(0);
    output_create(22);
    output_create(25);
    output_create(26);
    output_create(27);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);                            // Cấu hình ngắt ngoài cho nút IO0
    interputQueue = xQueueCreate(10, sizeof(int));                               // sử dụng queue cho ngắt
    xTaskCreate(&Show_Backup_Task, "Show_Backup_Task", 2048 * 5, NULL, 6, NULL); // task thực hiện ngắt ngoài
    gpio_install_isr_service(0);                                                 // khởi tạo ISR Service cho ngắt
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);  // đăng ký hàm handler cho ngắt ngoài

    /*Khởi tạo Blutooth Controller*/
    err = bluetooth_init();
    if (err)
    {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }
    err = ble_mesh_nvs_open(&NVS_HANDLE); // lấy lại thông tin cấp phép nếu có
    if (err)
    {
        return;
    }
    ble_mesh_get_dev_uuid(dev_uuid); // tạo UUID cho sensor node

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err)
    {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}
