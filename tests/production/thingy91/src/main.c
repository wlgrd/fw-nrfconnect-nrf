/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <string.h> // memset
#include <sensor.h>
#include <nrf_cloud.h>
#include <gpio.h>
#include <dk_buttons_and_leds.h>
#include <device.h>
#include <pwm.h>
#include <adc.h>
//#include <SEGGER_RTT.h>

#if defined(CONFIG_BOARD_NRF9160_PCA20035NS) ||                                \
	defined(CONFIG_BOARD_NRF9160_PCA20035)
#include <hal/nrf_saadc.h>
#define ADC_DEVICE_NAME "ADC_0"
#define ADC_RESOLUTION 8
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 5)
#define ADC_CHANNEL_ID0 0
#define ADC_CHANNEL_ID1 1
#define ADC_CHANNEL_ID2 2
#define ADC_CHANNEL_INPUT0 NRF_SAADC_INPUT_AIN5
#define ADC_CHANNEL_INPUT1 NRF_SAADC_INPUT_AIN6
#define ADC_CHANNEL_INPUT2 NRF_SAADC_INPUT_AIN7
#define ADC_OVERSAMPLING 2 /* 2^ADC_OVERSAMPLING samples are averaged */
#define ADC_BUFFER_SIZE 3

#else
#error "Unsupported board."
#endif

#define prod_assert_equal(a, b, err, msg)                              \
	if (a != b) {                                                      \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__,   \
		       msg);                                                   \
		return err;                                                    \
	}

#define prod_assert_not_equal(a, b, err, msg)                          \
	if (a == b) {                                                      \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__,   \
		       msg);                                                   \
		return err;                                                    \
	}
#define prod_assert_not_null(a, err, msg)                              \
	if (a == NULL) {                                                   \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__,   \
		       msg);                                                   \
		return err;                                                    \
	}
#define prod_assert_unreachable(msg)                                   \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__,   \
		       msg);                                                   \
		return -1;                                                     \

#define BUTTON_1 BIT(0)
#define BUTTON_2 BIT(1)
#define SWITCH_1 BIT(2)
#define SWITCH_2 BIT(3)

#define LEDS_PATTERN_WAIT (DK_LED2_MSK | DK_LED3_MSK)
#define LEDS_RED DK_LED1_MSK
#define LEDS_GREEN DK_LED2_MSK
#define LEDS_BLUE DK_LED3_MSK
#define BUZZER_PIN 28 // TP7

/* There is an issue with the pwm driver, causing a 300us delay */
#define PWM_OFFSET 300
#define PWM_CHANNEL 0
#define OSC_FREQ_HZ 2700
#define OSC_FREQ_US ((1000000UL / OSC_FREQ_HZ))
#define PERIOD (OSC_FREQ_US / 2) //> Need to set it half due to bug
#define DUTY_CYCLE (PERIOD / 4) //> Maximum 50% Duty cycle due to bug

#define SENSE_LED_R		0
#define SENSE_LED_G		1
#define SENSE_LED_B		2

#if defined(ENABLE_RTT_CMD_GET)
#include <SEGGER_RTT.h>
static u8_t m_rtt_keys[20];
static u8_t m_rtt_rx_keyindex = 0;
bool m_test_params_received = false;
static u8_t msg[3];
#endif // ENABLE_RTT_CMD_GET

static u8_t button_test_timeout = 80;
bool all_tests_succeeded = true;
struct sensor_value temp;

static u16_t m_sample_buffer[ADC_BUFFER_SIZE];

static const struct adc_channel_cfg m_channel_cfg0 = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID0,
	.input_positive = ADC_CHANNEL_INPUT0,
	.input_negative = ADC_CHANNEL_INPUT2,
};
static const struct adc_channel_cfg m_channel_cfg1 = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID1,
	.input_positive = ADC_CHANNEL_INPUT1,
};
static const struct adc_channel_cfg m_channel_cfg2 = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID2,
	.input_positive = ADC_CHANNEL_INPUT2,
};

static struct device *init_adc(void)
{
	int ret;
	struct device *adc_dev = device_get_binding("ADC_0");

	prod_assert_not_null(adc_dev, -ENODEV, "Cannot get ADC device");

	ret = adc_channel_setup(adc_dev, &m_channel_cfg0);
	prod_assert_equal(
		ret, 0, -EIO,
		"Setting up of the first channel failed with code %d");
	ret = adc_channel_setup(adc_dev, &m_channel_cfg1);
	prod_assert_equal(
		ret, 0, -EIO,
		"Setting up of the second channel failed with code %d");
	ret = adc_channel_setup(adc_dev, &m_channel_cfg2);
	prod_assert_equal(
		ret, 0, -EIO,
		"Setting up of the third channel failed with code %d");

	(void)memset(m_sample_buffer, 0, sizeof(m_sample_buffer));

	return adc_dev;
}

static enum adc_action adc_callback(struct device *dev,
				    const struct adc_sequence *sequence,
				    u16_t sampling_index)
{
	printk("%s: sampling %d\n", __func__, sampling_index);
	return ADC_ACTION_CONTINUE;
}

static int measure_voltage(void)
{
	int volatile ret;

	const struct adc_sequence sequence = {
		// .channels    = BIT(ADC_CHANNEL_ID0),
		.channels = BIT(ADC_CHANNEL_ID0) | BIT(ADC_CHANNEL_ID1) |
			    BIT(ADC_CHANNEL_ID2),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = ADC_RESOLUTION,
		// .oversampling  = ADC_OVERSAMPLING,
	};

	struct device *dev = init_adc();

	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
	k_sleep(100);
	ret = adc_read(dev, &sequence);
	prod_assert_equal(ret, 0, -EIO, "adc_read() failed with code %d\r\n");

	for (u8_t i = 0; i < ADC_BUFFER_SIZE; i++) {
		/* We only use 8bit adc resolution */
		u8_t sample_value = (m_sample_buffer[i] & 0xFF);
		float voltage = ((3600 * sample_value) / 255);

		printk("Sample  %d: 0x%02x\r\n", i, sample_value);
		printk("Voltage %d mV\r\n", (int)voltage);
	}
	printk("\r\n");
	return 0;
}
/* Blocking call that returns when button has changed state */
static void wait_for_new_btn_state(void)
{
	u32_t volatile state, newstate, timeout = 0;
	state = dk_get_buttons();
	newstate = state;
	printk("Waiting for button press...");
	dk_set_leds(LEDS_RED);
	while ((state == newstate) && (timeout < button_test_timeout)) {
		newstate = dk_get_buttons();
		k_sleep(100);
	}
	return;
}

static void run_test(int (*test)(void), char *testname)
{
	int ret;
	printk("============================\r\n");
	printk("Starting test: %s\r\n\r\n", testname);
	ret = test();
	if (ret != 0) {
		all_tests_succeeded = false;
		printk("\r\nFailed: %s\r\n", testname);
		return;
	}
	k_sleep(200);
	printk("\r\nSucceeded: %s\r\n", testname);
}

static int pca20035_ADXL372(void)
{
	int err = 0;
	dk_set_leds(LEDS_RED);
	struct device *dev;
	dev = device_get_binding("ADXL372");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
	err = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	prod_assert_equal(err, 0, -EIO, "Failed to fetch sensor data");
	k_sleep(10);
	err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to accel x");
	printk("ADXL372 X: %d.%d\r\n", temp.val1, temp.val2);
	// TODO: Fix, as float print hangs rtt for some reason.
	// printf("ADXL372 X: %.2f\r\n", sensor_value_to_double(&temp));
	// k_sleep(10);
	err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to accel y");
	printk("ADXL372 Y: %d.%d\r\n", temp.val1, temp.val2);
	err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to accel z");
	printk("ADXL372 Z: %d.%d\r\n", temp.val1, temp.val2);
	dk_set_leds(DK_NO_LEDS_MSK);
	return 0;
}

static int pca20035_ADXL362(void)
{
	int err = 0;
	dk_set_leds(LEDS_BLUE);
	volatile struct device *dev;
	dev = device_get_binding("ADXL362");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
	err = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	prod_assert_equal(err, 0, -EIO, "Failed to fetch sensor data");
	k_sleep(10);
	err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to accel x");
	printk("ADXL362 X: %d.%d\r\n", temp.val1, temp.val2);
	err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to accel y");
	printk("ADXL362 Y: %d.%d\r\n", temp.val1, temp.val2);
	err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to accel z");
	printk("ADXL362 Z: %d.%d\r\n", temp.val1, temp.val2);
	dk_set_leds(DK_NO_LEDS_MSK);
	return 0;
}

static int pca20035_BME680(void)
{
	int err = 0;

	dk_set_leds(LEDS_GREEN);
	struct device *dev;
	dev = device_get_binding("BME680");
	// prod_assert_not_null(dev, "Failed to get sensor");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
	err = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	prod_assert_equal(err, 0, -EIO, "Failed to fetch sensor data");
	k_sleep(10);
	err = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get temperature");
	printk("BME680 temperature: %d.%d\r\n", temp.val1, temp.val2);

	k_sleep(10);
	err = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get pressure");
	printk("BME680 pressure: %d.%d kPa\r\n", temp.val1, temp.val2);

	k_sleep(10);
	err = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get humidity");
	printk("BME680 humidity: %d.%d %%RH\r\n", temp.val1, temp.val2);

	k_sleep(10);
	err = sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get gas");
	printk("BME680 gas resistance: %d ohms\r\n", temp.val1);

	dk_set_leds(DK_NO_LEDS_MSK);
	return 0;
}
static int pca20035_BH1749(void)
{
	int err = 0;
	struct device *dev;
	struct device *gpio;
	static uint8_t rgb[3] = {0, 0, 0};

	gpio = device_get_binding(DT_GPIO_P0_DEV_NAME);
	prod_assert_not_null(gpio, -ENODEV, "Failed to get binding to gpio");
	gpio_pin_configure(gpio, SENSE_LED_R, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, SENSE_LED_G, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, SENSE_LED_B, GPIO_DIR_OUT);

	dev = device_get_binding("BH1749");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
	gpio_pin_write(gpio, SENSE_LED_R, 1);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);

	for (u8_t i = 0; i < 3; i++)
	{
		k_sleep(300);
		err = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
		prod_assert_equal(err, 0, -EIO, "Failed to fetch sensor data");
		k_sleep(200);
		err = sensor_channel_get(dev, SENSOR_CHAN_RED, &temp);
		prod_assert_equal(err, 0, -EIO, "Failed to get red");
		rgb[0] = temp.val1;
		err = sensor_channel_get(dev, SENSOR_CHAN_GREEN, &temp);
		prod_assert_equal(err, 0, -EIO, "Failed to get green");
		rgb[1] = temp.val1;
		err = sensor_channel_get(dev, SENSOR_CHAN_BLUE, &temp);
		prod_assert_equal(err, 0, -EIO, "Failed to get blue");
		rgb[2] = temp.val1;
		k_sleep(300);

		switch (i) {
			case 0:
				printk("BH1749 red: %d (Sum of others: %d)\r\n", rgb[0],
						(rgb[1] + rgb[2]));
				if(rgb[0] < (rgb[1] + rgb[2])) {
					prod_assert_unreachable("RED");
				}
					/* Set SENSE LEDs for next iteration */ 
					gpio_pin_write(gpio, SENSE_LED_R, 0);
					gpio_pin_write(gpio, SENSE_LED_G, 1);
					gpio_pin_write(gpio, SENSE_LED_B, 0);
				break;
			case 1:
				printk("BH1749 green: %d (Sum of others: %d)\r\n", rgb[1],
						(rgb[0] + rgb[2]));
				if(rgb[1] < (rgb[0] + rgb[2])) {
					prod_assert_unreachable("GREEN");
				}
					/* Set SENSE LEDs for next iteration */ 
					gpio_pin_write(gpio, SENSE_LED_R, 0);
					gpio_pin_write(gpio, SENSE_LED_G, 0);
					gpio_pin_write(gpio, SENSE_LED_B, 1);
				break;
			case 2:
				printk("BH1749 blue: %d (Sum of others: %d)\r\n", rgb[2],
						(rgb[1] + rgb[0]));
				if(rgb[2] < (rgb[1]/2 + rgb[0])-20) {
					prod_assert_unreachable("BLUE");
				}
				break;
			default:
				break;
		}

	}

	return 0;
}

static int pca20035_test_button(void)
{
	u32_t volatile state, newstate, timeout = 0,
					button_test_timeout = 200000;
	printk("[ACTION]: Please press the button ...\n");
	state = dk_get_buttons();
	newstate = state;
	dk_set_leds(LEDS_RED);
	while ((state == newstate) && (timeout < button_test_timeout)) {
		newstate = dk_get_buttons();
		timeout++;
		k_sleep(1);
	}
	printk("timeout: %d \n", timeout);
	prod_assert_not_equal(timeout, button_test_timeout, -ETIMEDOUT,
			      "Button test timed out");
	dk_set_leds(DK_NO_LEDS_MSK);
	return 0;
}

static int pca20035_test_buzzer(void)
{
	volatile int err_code = 0xFF;
	static struct device *dev;
	static u32_t period = PERIOD;
	static u32_t duty_cycle = DUTY_CYCLE;
	dev = device_get_binding(DT_NORDIC_NRF_PWM_PWM_0_LABEL);
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
	printk("Turning buzzer ON\n");
	err_code = pwm_pin_set_usec(dev, BUZZER_PIN, period, duty_cycle);
	prod_assert_equal(err_code, 0, -EIO, "Failed to set pwm pin");
	k_sleep(1500);
	printk("Turning buzzer OFF\n");
	err_code = pwm_pin_set_usec(dev, BUZZER_PIN, period, 0);
	prod_assert_equal(err_code, 0, -EIO, "Failed to clear pwm pin");
	return 0;
}

static int button_handler(u32_t buttons, u32_t has_changed)
{
	return;
}

#if defined(ENABLE_RTT_CMD_GET)
static void check_rtt_command(u8_t *data, u8_t len)
{
	// static u8_t * msg;
	memcpy(msg, data, 3);
	printk("Params received: %s \r\n", msg);
	m_test_params_received = true;
}
#endif

void main(void)
{
	int err;
	err = dk_leds_init();
	if (err) {
		printk("Could not initialize leds, err code: %d\n", err);
	}
	err = dk_buttons_init(&button_handler);
	if (err) {
		printk("Could not initialize buttons, err code: %d\n", err);
	}

	printk("Starting production test - thingy:91\r\n");

#if defined(ENABLE_RTT_CMD_GET)
	printk("Waiting for test parameters...\r\n");
	while (!m_test_params_received) {
		if (SEGGER_RTT_HasKey()) {
			// Fetch the first key in the buffer
			m_rtt_keys[m_rtt_rx_keyindex] = SEGGER_RTT_GetKey();

			// Q is set as "EOL", so parse when received
			if (m_rtt_keys[m_rtt_rx_keyindex] == 'Q') {
				check_rtt_command(m_rtt_keys,
						  ++m_rtt_rx_keyindex);
				m_rtt_rx_keyindex = 0;
				memset(m_rtt_keys, 0, sizeof(m_rtt_keys));
			} else {
				// Keep buffering data
				m_rtt_rx_keyindex++;
			}
		}
	}
#endif

	dk_set_leds(LEDS_RED);
	k_sleep(200);
	dk_set_leds(DK_NO_LEDS_MSK);
	dk_set_leds(LEDS_GREEN);
	k_sleep(200);
	dk_set_leds(DK_NO_LEDS_MSK);
	dk_set_leds(LEDS_BLUE);
	k_sleep(200);
	dk_set_leds(DK_NO_LEDS_MSK);
	printk(".\r\n");
	printk(".\r\n");
	printk(".\r\n");
	printk(".\r\n");
	/* This delay is added to improve rtt buffer loss on the host side.
         * Might be that we can improve this with other debuggers...
         */
	k_sleep(4000);
	while (1) {
		run_test(&pca20035_BH1749, "pca20035_BH1749");
		run_test(&pca20035_ADXL372, "pca20035_ADXL372");
		run_test(&pca20035_ADXL362, "pca20035_ADXL362");
		run_test(&pca20035_BME680, "pca20035_BME680");
		run_test(&pca20035_test_button, "pca20035_button");
		run_test(&pca20035_test_buzzer, "pca20035_buzzer");
		run_test(&measure_voltage, "measure_voltage");
		k_sleep(50);
		// Stop execution if test failed.
		all_tests_succeeded ? printk("\r\nTEST SUITE SUCCESS!\r\n") :
				      printk("\r\nTEST SUITE FAILED!\r\n");
		if (!all_tests_succeeded)
			break;
	}
	dk_set_leds(LEDS_PATTERN_WAIT);
}
