/**********************************************************************
* - Description:		esp32-mpu6050
* - File:				main.c
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Shaurya Chandra
* - Target:				ESP32
* - Created:			17-01-2021
* - Last changed:		17-01-2021
*
**********************************************************************/

#include "AppConfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "i2c_application.h"
#include "i2c_driver.h"
#include "nvs_flash.h"
#include "mpu6050_application.h"

static const char *TAG = "i2c-example";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 3000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
//LED Config
#define BLINK_LED 2
bool Led_Status = 0;
// Debounced button task

#define BUTTONPIN 0
#define DEBOUNCETIME 10
#define ESP_INTR_FLAG_DEFAULT 0
bool Button_Status = 0;
volatile int numberOfButtonInterrupts = 0;
volatile bool lastState;
volatile uint32_t debounceTimeout = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR handleButtonInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  numberOfButtonInterrupts++;
  lastState = gpio_get_level(BUTTONPIN);
  debounceTimeout = xTaskGetTickCount(); //version of millis() that works from interrupt
  portEXIT_CRITICAL_ISR(&mux);
}




SemaphoreHandle_t print_mux = NULL;

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


static esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR,
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}


static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

static void i2c_task(void *arg)
{
    int i = 0;
    int ret;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
    while (1) {
        data[0] = Led_Status;
        xSemaphoreTake(print_mux, portMAX_DELAY);
        size_t d_size = i2c_slave_write_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
        if (d_size == 0) {
            ESP_LOGW(TAG, "i2c slave tx buffer full");
            ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);
        } else {
            ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, RW_TEST_LENGTH);
        }

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER READ FROM SLAVE\n", task_idx);
            printf("*******************\n");
            // printf("====TASK[%d] Slave buffer data ====\n", task_idx);
            // disp_buf(data, d_size);
            // printf("====TASK[%d] Master read ====\n", task_idx);
            // disp_buf(data_rd, d_size);
            printf("Led Status is : %d\n",data_rd[0]);
        } else {
            ESP_LOGW(TAG, "TASK[%d] %s: Master read slave error, IO not connected...\n",
                     task_idx, esp_err_to_name(ret));
        }
        xSemaphoreGive(print_mux);
        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);    
        
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}


void taskButtonRead( void *arg)
{
  printf("Debounced ButtonRead Task running on core [%d]",xPortGetCoreID());

  uint32_t saveDebounceTimeout;
  bool saveLastState;
  int save;
  while (1)
  {

    portENTER_CRITICAL_ISR(&mux); 
    save  = numberOfButtonInterrupts;
    saveDebounceTimeout = debounceTimeout;
    saveLastState  = lastState;
    portEXIT_CRITICAL_ISR(&mux); 

    bool currentState = gpio_get_level(BUTTONPIN);
    if ((save != 0) 
        && (currentState == saveLastState) 
        && (xTaskGetTickCount() - saveDebounceTimeout > DEBOUNCETIME ))
    { 
      
    if (currentState == 0)
    {
        Button_Status = !Button_Status;
        printf("Button is pressed and debounced, current tick=%d, Button_Status= %d\n", xTaskGetTickCount(),Button_Status); 
    }
    else
    {
        Button_Status = !Button_Status;
        printf("Button is released and debounced, current tick=%d , Button_Status= %d\n", xTaskGetTickCount(),Button_Status); 
    }
      
    printf("Button Interrupt Triggered %d times, current State=%u, time since last trigger %dms\n", save, currentState, xTaskGetTickCount() - saveDebounceTimeout);
      
    portENTER_CRITICAL_ISR(&mux); 
    numberOfButtonInterrupts = 0; 
    portEXIT_CRITICAL_ISR(&mux);

    int ret;
    uint32_t task_idx = (uint32_t)arg;
    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);

        if(Button_Status){
            data_wr[0] = 1;
        }
        else{
            data_wr[0] = 0;
        }
        int size;            
        
    
        ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, RW_TEST_LENGTH);
        if (ret == ESP_OK) {
            size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
        }
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf("TASK[%d]  MASTER WRITE TO SLAVE\n", task_idx);
            printf("*******************\n");
            // printf("----TASK[%d] Master write ----\n", task_idx);
            // disp_buf(data_wr, RW_TEST_LENGTH);
            // printf("----TASK[%d] Slave read: [%d] bytes ----\n", task_idx, size);
            // disp_buf(data, size);
            if(data[0]!= Led_Status){ 
                Led_Status = !Led_Status;               
                printf("LED STATUS %d\n",Led_Status);
                gpio_set_level(BLINK_LED, Led_Status);
            }
        } else {
            ESP_LOGW(TAG, "TASK[%d] %s: Master write slave error, IO not connected....\n",task_idx, esp_err_to_name(ret));
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    //vTaskDelete(NULL);
    vTaskDelay(10 / portTICK_PERIOD_MS);

  }
}

#define STACK_SIZE_2048 2048

/* Structure that will hold the TCB of the task being created. */
StaticTask_t xI2CWriteTaskBuffer;
StaticTask_t xI2CReadTaskBuffer;
StaticTask_t xMPU6050TaskBuffer;
/* Buffer that the task being created will use as its stack.  Note this is
    an array of StackType_t variables.  The size of StackType_t is dependent on
    the RTOS port. */
StackType_t xStack_I2C_Write[ STACK_SIZE_2048 ];
StackType_t xStack_I2C_Read[ STACK_SIZE_2048 ];
StackType_t xStack_MPU6050Task[ STACK_SIZE_2048 ];

void app_main(void)
{
    esp_err_t ret;
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initializations
	vI2CInit();

    xTaskCreateStaticPinnedToCore(vI2CWrite,            /* Function that implements the task. */
                                  "vI2CWrite",          /* Text name for the task. */
                                  STACK_SIZE_2048,      /* Number of indexes in the xStack array. */
                                  NULL,                 /* Parameter passed into the task. */
                                  osPriorityHigh,       /* Priority at which the task is created. */
                                  xStack_I2C_Write,     /* Array to use as the task's stack. */
                                  &xI2CWriteTaskBuffer, /* Variable to hold the task's data structure. */
                                  0                     /*  0 for PRO_CPU, 1 for APP_CPU, or tskNO_AFFINITY which allows the task to run on both */
                                  );

    xTaskCreateStaticPinnedToCore(vI2CRead,            /* Function that implements the task. */
                                  "vI2CRead",          /* Text name for the task. */
                                  STACK_SIZE_2048,     /* Number of indexes in the xStack array. */
                                  NULL,                /* Parameter passed into the task. */
                                  osPriorityHigh,      /* Priority at which the task is created. */
                                  xStack_I2C_Read,     /* Array to use as the task's stack. */
                                  &xI2CReadTaskBuffer, /* Variable to hold the task's data structure. */
                                  0                    /*  0 for PRO_CPU, 1 for APP_CPU, or tskNO_AFFINITY which allows the task to run on both */
                                  );

    xTaskCreateStaticPinnedToCore(vMPU6050Task,        /* Function that implements the task. */
                                  "vMPU6050Task",      /* Text name for the task. */
                                  STACK_SIZE_2048,     /* Number of indexes in the xStack array. */
                                  NULL,                /* Parameter passed into the task. */
                                  osPriorityNormal,    /* Priority at which the task is created. */
                                  xStack_MPU6050Task,  /* Array to use as the task's stack. */
                                  &xMPU6050TaskBuffer, /* Variable to hold the task's data structure. */
                                  0                    /*  0 for PRO_CPU, 1 for APP_CPU, or tskNO_AFFINITY which allows the task to run on both */
                                  );

    //Button Config
    gpio_pad_select_gpio(BUTTONPIN);
    gpio_set_direction(BUTTONPIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(BUTTONPIN, GPIO_INTR_NEGEDGE);

    xTaskCreate(taskButtonRead, "button_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(BUTTONPIN, handleButtonInterrupt, NULL);

    //Led Config
    gpio_pad_select_gpio(BLINK_LED);
    gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);

    //I2C communication
    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_slave_init());
    //ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_task, "i2c_task", 1024 * 2, (void *)0, 10, NULL);

}
