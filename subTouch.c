/*

 * This module implements a sliding window of touch samples
 * We're using touch to switch the ESP32 on/off and we don't want this 
 * happening accidentally.
 * It detects a user touching the touch area at 1Hz for about ten seconds
 * 
 * The difference between each sample and the previous sample is buffered
 * It turns out that the touch input is quite slow - it's sampled at 10Hz and
 * the resulting differences look like a ragged sine curve.
 * 
Touch channels are hard-wired in the processor
#define TOUCH_PAD_GPIO4_CHANNEL     0
#define TOUCH_PAD_NUM0_GPIO_NUM     4 conflicts with wifi

#define TOUCH_PAD_GPIO0_CHANNEL     1
#define TOUCH_PAD_NUM1_GPIO_NUM     0 conflicts with wifi

#define TOUCH_PAD_GPIO2_CHANNEL     2
#define TOUCH_PAD_NUM2_GPIO_NUM     2 conflicts with wifi

#define TOUCH_PAD_GPIO15_CHANNEL    3
#define TOUCH_PAD_NUM3_GPIO_NUM     15 conflicts with wifi

#define TOUCH_PAD_GPIO13_CHANNEL    4
#define TOUCH_PAD_NUM4_GPIO_NUM     13 conflicts with wifi

#define TOUCH_PAD_GPIO12_CHANNEL    5
#define TOUCH_PAD_NUM5_GPIO_NUM     12 conflicts with wifi

#define TOUCH_PAD_GPIO14_CHANNEL    6
#define TOUCH_PAD_NUM6_GPIO_NUM     14

#define TOUCH_PAD_GPIO27_CHANNEL    7
#define TOUCH_PAD_NUM7_GPIO_NUM     27 conflicts with wifi

#define TOUCH_PAD_GPIO33_CHANNEL    8
#define TOUCH_PAD_NUM8_GPIO_NUM     33

#define TOUCH_PAD_GPIO32_CHANNEL    9
#define TOUCH_PAD_NUM9_GPIO_NUM     32
 
 */


#include <stdio.h>
#include <string.h>

#include "driver/touch_sens.h"
#include "touch_sens_example_config.h"
#include <math.h>
#include "esp_sleep.h"

#include "driver/uart.h"


#include "esp_log.h"

static char *TAG = "subTouch";

#define SUB_TOUCH_BUFFER_SIZE 100
#define SUB_TOUCH_HERZ 10
#define SUB_TOUCH_PAD_NUMBER 6 // Pin 32

#define SUB_TOUCH_START_CONFIRMED 1
#define SUB_TOUCH_SECONDS_TO_WAIT_FOR_CONFIRM 30

// THE 10HZ POWER NEEDED TO TRIGGER ON OR OFF. DETERMINED BY INSPECTION.
#define SUB_TOUCH_THRESHOLD 500

static uint16_t subTouchBuffer[SUB_TOUCH_BUFFER_SIZE];
static float subTouchWindow[SUB_TOUCH_BUFFER_SIZE];
static float subTouchSin[SUB_TOUCH_HERZ];
static float subTouchCos[SUB_TOUCH_HERZ];

static TaskHandle_t subTouchTaskHandle;

static touch_channel_handle_t touch_channel_handle = NULL;
static touch_sensor_handle_t touch_sensor_handle = NULL;

static void subTouchDoInitialScanning(void)
{
	static float s_thresh2bm_ratio = 0.02f; // Equivalent to 2% - apparently

	// Enable the touch sensor to do the initial scanning, so that to initialize the channel data
	ESP_ERROR_CHECK(touch_sensor_enable(touch_sensor_handle));

	// Scan the enabled touch channels several times, to make sure the initial channel data is stable
	// I've no idea why we want to do this - can find nothing in the docs
	for (int i = 0; i < 3; i++)
	{
		ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(touch_sensor_handle, 2000));
	}

	/* Disable the touch channel to rollback the state */
	ESP_ERROR_CHECK(touch_sensor_disable(touch_sensor_handle));

	// (Optional) Read the initial channel benchmark and reconfigure the channel 
	// active threshold accordingly
	// I think this sets the threshold for the touch wake-up
	ESP_LOGI(TAG, "Initial benchmark and new threshold are as follows");
	/* Read the initial benchmark of the touch channel */
	uint32_t benchmark;
#if SOC_TOUCH_SUPPORT_BENCHMARK
	ESP_ERROR_CHECK(touch_channel_read_data(touch_channel_handle, TOUCH_CHAN_DATA_TYPE_BENCHMARK, &benchmark));
#else
	/* Read smooth data instead if the hardware does not support benchmark */
	ESP_ERROR_CHECK(touch_channel_read_data(touch_channel_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, &benchmark));
#endif  // SOC_TOUCH_SUPPORT_BENCHMARK
	/* Calculate the proper active thresholds regarding the initial benchmark */
	ESP_LOGI(TAG, "Touch [CH %d]", SUB_TOUCH_PAD_NUMBER);
	/* Generate the default channel configuration and then update the active threshold based on the real benchmark */
	touch_channel_config_t touch_channel_config = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();

#if SOC_TOUCH_SENSOR_VERSION == 1
	// Touch V1 (ESP32) uses absolute threshold.
	touch_channel_config.abs_active_thresh[0] = (uint32_t) (benchmark * (1 - s_thresh2bm_ratio));
	ESP_LOGI(TAG, " %d: %"PRIu32", %"PRIu32"\t", 0, benchmark, touch_channel_config.abs_active_thresh[0]);
#else
	touch_channel_config.active_thresh[0] = (uint32_t) (benchmark * s_thresh2bm_ratio);
	ESP_LOGI(TAG, " %d: %"PRIu32", %"PRIu32"\t", 0, benchmark, touch_channel_config.active_thresh[0]);
#endif  // SOC_TOUCH_SENSOR_VERSION == 1

	/* Update the channel configuration */
	ESP_ERROR_CHECK(touch_sensor_reconfig_channel(touch_channel_handle, &touch_channel_config));
}

void subTouchInit(void)
{
	// Some target has multiple sets of sample configuration can be set, here take one for example
#define SAMPLE_NUM 1
	// sample configuration

	// Create a new touch sensor controller handle with default sample configuration
	touch_sensor_sample_config_t touch_sensor_sample_config[TOUCH_SAMPLE_CFG_NUM] = EXAMPLE_TOUCH_SAMPLE_CFG_DEFAULT();
	touch_sensor_config_t touch_sensor_config = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(SAMPLE_NUM, touch_sensor_sample_config);
	
	// Allocate a new touch sensor controller handle
	ESP_ERROR_CHECK(touch_sensor_new_controller(&touch_sensor_config, &touch_sensor_handle));
	
	touch_channel_config_t touch_channel_config = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
	int channel_id = SUB_TOUCH_PAD_NUMBER;

	// Allocate a new touch sensor controller handle
	ESP_ERROR_CHECK(touch_sensor_new_channel(touch_sensor_handle, channel_id, &touch_channel_config, &touch_channel_handle));

	touch_chan_info_t touch_chan_info = {};
	ESP_ERROR_CHECK(touch_sensor_get_channel_info(touch_channel_handle, &touch_chan_info));
	
	ESP_LOGI(TAG, "Touch [CH %d] enabled on GPIO%d", channel_id, touch_chan_info.chan_gpio);
	
	touch_sensor_filter_config_t touch_sensor_filter_config = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
	ESP_ERROR_CHECK(touch_sensor_config_filter(touch_sensor_handle, &touch_sensor_filter_config));
	
	// Implement a Hann window on the hundred samples
	float f = 2 * M_PI / SUB_TOUCH_BUFFER_SIZE;
	for (int i = 0; i < SUB_TOUCH_BUFFER_SIZE; i ++)
	{
		subTouchWindow[i] = 0.5 - 0.5 * cos(f * i);
	}
	
	f = 2 * M_PI / SUB_TOUCH_HERZ;
	for (int i = 0; i < SUB_TOUCH_HERZ; i ++)
	{
		subTouchSin[i] = sin(f * i);
		subTouchCos[i] = cos(f * i);
	}
}

void subTouchDeinit(void)
{
	ESP_ERROR_CHECK(touch_sensor_config_filter(touch_sensor_handle, NULL));
	
	ESP_ERROR_CHECK(touch_sensor_del_channel(touch_channel_handle));
	
	touch_sensor_del_controller(touch_sensor_handle);
}


void subTouchDeepSleep(void)
{
	subTouchDeinit();
	ESP_LOGI(TAG, "Deep Sleep");
	
	subTouchInit();
	
	subTouchDoInitialScanning();

	touch_sleep_config_t touch_sleep_config = TOUCH_SENSOR_DEFAULT_DSLP_CONFIG();

	ESP_ERROR_CHECK(touch_sensor_config_sleep_wakeup(touch_sensor_handle, &touch_sleep_config));

	/* Step 5: Enable touch sensor controller and start continuous scanning before entering light sleep */
	ESP_ERROR_CHECK(touch_sensor_enable(touch_sensor_handle));
	ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(touch_sensor_handle));

	vTaskDelay(pdMS_TO_TICKS(1000));

	
	ESP_LOGI(TAG, "touch wakeup source is ready");
	
	vTaskDelay(pdMS_TO_TICKS(1000));
	
	/* Wait the UART to finish printing the log */
	uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);
	/* Enter the deep sleep */
	esp_deep_sleep_start();
}

static void subTouchTask(void *pvParameters)
{
	/*
	// Enable the touch sensor to do the initial scanning, so that to initialize the channel data
	ESP_ERROR_CHECK(touch_sensor_enable(touch_sensor_handle));

	// Scan the enabled touch channels several times, to make sure the initial channel data is stable
	// I've no idea why we need to do this - can find nothing in the docs
	for (int i = 0; i < 3; i++)
	{
		ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(touch_sensor_handle, 2000));
	}

	// Disable the touch channel to rollback the state
	ESP_ERROR_CHECK(touch_sensor_disable(touch_sensor_handle));
	
	*/

	// Enable touch sensor
	ESP_ERROR_CHECK(touch_sensor_enable(touch_sensor_handle));
	
	// Start continuous scan
	ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(touch_sensor_handle));

	// Set the previous value in the right ballpark
	static uint32_t uint32Prev = 0;
	ESP_ERROR_CHECK(touch_channel_read_data(touch_channel_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, &uint32Prev));
	
	int writeIndex = 0;
	int scanCount = 0;
	int status = 0;
	int stopped = false;

	while(!stopped)
	{
		uint32_t uint32;
		ESP_ERROR_CHECK(touch_channel_read_data(touch_channel_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, &uint32));
		subTouchBuffer[writeIndex ++] = (int16_t)((uint32 - uint32Prev) & 0xffff);
		writeIndex %= 100;

		if ((writeIndex % SUB_TOUCH_HERZ) == 0)
		{
			// Carry out the single line of the DFT that would calculate the
			// target frequency
			float real = 0;
			float imag = 0;
			int index = writeIndex;
			for (int j = 0; j < SUB_TOUCH_BUFFER_SIZE; j++)
			{
				float f = subTouchWindow[j] * subTouchBuffer[index];
				float c = subTouchCos[j % SUB_TOUCH_HERZ];
				float s = subTouchSin[j % SUB_TOUCH_HERZ];
				real += c * f;
				imag -= s * f;
				index++;
				index %= SUB_TOUCH_BUFFER_SIZE;
			}

			float power = round(sqrt(real * real + imag * imag));
			int pow = power;
			//int r = round(sqrt(real * real));
			//int j = round(sqrt(imag * imag));
			//		ESP_LOGI("t", "%d\t%d\t%d\t%d\n", writeIndex, i, r, j);
				
			scanCount ++;
			if (scanCount < SUB_TOUCH_HERZ)
			{
				// Do nothing - buffer is not full yet
			}
			else if (power > SUB_TOUCH_THRESHOLD)
			{
				printf("%d %d----------\n", writeIndex, pow);
				if (scanCount < (SUB_TOUCH_SECONDS_TO_WAIT_FOR_CONFIRM))
				{
					printf("Start confirmed\n");
					status = SUB_TOUCH_START_CONFIRMED;
				}
				else if (status == SUB_TOUCH_START_CONFIRMED)
				{
					// Time is up. Start confirmation (10Hz tapping) not detected
					stopped = true;
				}
			}
			else
			{
				printf("%d %d %d\n", writeIndex, pow, scanCount);
				if (scanCount > (SUB_TOUCH_SECONDS_TO_WAIT_FOR_CONFIRM))
				{
					if (status != SUB_TOUCH_START_CONFIRMED)
					{
						// Time is up. Start confirmation (10Hz tapping) not detected
						stopped = true;
					}
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	ESP_ERROR_CHECK(touch_sensor_disable(touch_sensor_handle));
	ESP_ERROR_CHECK(touch_sensor_stop_continuous_scanning(touch_sensor_handle));
	subTouchDeepSleep();
}



// Scan will start at power-on or wake from deep sleep
// If a 10Hz signal is not detected within 30 seconds go back to sleep
// Otherwise continue scanning and if a 10Hz signal is detected go to sleep

void subTouchStartScan(void)
{
	xTaskCreate(subTouchTask, "subTouchTask", 4096, NULL, tskIDLE_PRIORITY, &subTouchTaskHandle);
}


/*
 * 
 * https://www.geeksforgeeks.org/c/discrete-fourier-transform-and-its-inverse-using-c/
// C program for the above approach
#include <math.h>
#include <stdio.h>

// Function to calculate the DFT
void calculateDFT(int len)
{
    int xn[len];
    float Xr[len];
    float Xi[len];
    int i, k, n, N = 0;

    for (i = 0; i < len; i++) {

        printf("Enter the value "
               "of x[%d]: ",
               i);
        scanf("%d", &xn[i]);
    }

    printf("Enter the number of "
           "points in the DFT: ");
    scanf("%d", &N);
    for (k = 0; k < N; k++) {
        Xr[k] = 0;
        Xi[k] = 0;
        for (n = 0; n < len; n++) {
            Xr[k]
                = (Xr[k]
                   + xn[n] * cos(2 * 3.141592 * k * n / N));
            Xi[k]
                = (Xi[k]
                   - xn[n] * sin(2 * 3.141592 * k * n / N));
        }

        printf("(%f) + j(%f)\n", Xr[k], Xi[k]);
    }
}

// Driver Code
int main()
{
    int len = 0;
    printf("Enter the length of "
           "the sequence: ");
    scanf("%d4", &len);
    calculateDFT(len);

    return 0;
} 
 */
