/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <string.h> // memset
#include <sensor.h>
#include <stdio.h>
#include <nrf_cloud.h>
#include <gpio.h>
#include <dk_buttons_and_leds.h>
#include <device.h>
#include <pwm.h>
#include <adc.h>
#include <nrf.h>
#include <zephyr.h>
// #include <SEGGER_RTT.h>

#define THRESHOLD_UPPER                 50
#define THRESHOLD_LOWER                 0
#define TRIGGER_ON_DATA_READY           0

#if defined(CONFIG_BOARD_NRF9160_PCA20035NS) ||	\
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
#define ADC_CHANNEL_INPUT1 NRF_SAADC_INPUT_AIN0
#define ADC_CHANNEL_INPUT2 NRF_SAADC_INPUT_AIN7
#define ADC_OVERSAMPLING 2 /* 2^ADC_OVERSAMPLING samples are averaged */
#define ADC_BUFFER_SIZE 3
#define LED_POLL_TIME 100
#define BUZZER_BEEP_INTERVAL 100

#else
#error "Unsupported board."
#endif

static K_SEM_DEFINE(sem, 0, 1);

#define prod_assert_equal(a, b, err, msg)				     \
	if (a != b) {							     \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__, \
		       msg);						     \
		return err;						     \
	}

#define prod_assert_not_equal(a, b, err, msg)				     \
	if (a == b) {							     \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__, \
		       msg);						     \
		return err;						     \
	}
#define prod_assert_not_null(a, err, msg)				     \
	if (a == NULL) {						     \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__, \
		       msg);						     \
		return err;						     \
	}
#define prod_assert_unreachable(msg)				     \
	printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__, \
	       msg);						     \
	return -1;						     \

#define BUTTON_1 BIT(0)
#define BUTTON_2 BIT(1)
#define SWITCH_1 BIT(2)
#define SWITCH_2 BIT(3)

#define LEDS_PATTERN_WAIT (DK_LED2_MSK | DK_LED3_MSK)
#define LEDS_RED DK_LED1_MSK
#define LEDS_GREEN DK_LED2_MSK
#define LEDS_BLUE DK_LED3_MSK
#define BUZZER_PIN 28 // TP7
#define TOGGLE_PIN 16 // AIN3

/* There is an issue with the pwm driver, causing a 300us delay */
#define PWM_OFFSET 300
#define PWM_CHANNEL 0
#define OSC_FREQ_HZ 2700
#define OSC_FREQ_US ((1000000UL / OSC_FREQ_HZ))
#define PERIOD (OSC_FREQ_US / 2)        // > Need to set it half due to bug
#define DUTY_CYCLE (PERIOD / 2)         // > Maximum 50% Duty cycle due to bug

#define SENSE_LED_R             0
#define SENSE_LED_G             1
#define SENSE_LED_B             2

bool all_tests_succeeded = true;
struct sensor_value temp;

static u16_t m_sample_buffer[ADC_BUFFER_SIZE];

static enum current_component {
	CUR_BASE,
	CUR_LEDS_RED,
	CUR_LEDS_GREEN,
	CUR_LEDS_BLUE,
	CUR_SENSE_LED_R,
	CUR_SENSE_LED_G,
	CUR_SENSE_LED_B,
	CUR_BUZZER
};

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

static void toggle_indicator_pin(void)
{
	struct device *gpio;
	static bool pin_state = false;

	gpio = device_get_binding(DT_GPIO_P0_DEV_NAME);

	if (pin_state) {
		gpio_pin_write(gpio, TOGGLE_PIN, 0);
		pin_state = false;
	} else {
		gpio_pin_write(gpio, TOGGLE_PIN, 1);
		pin_state = true;
	}
}

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

static int measure_current_and_voltage(enum current_component component)
{
	int volatile ret;
	struct device *dev;
	float ma_current;

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID1),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	k_sleep(200);
	toggle_indicator_pin();

	dev = device_get_binding("ADC_0");
	prod_assert_not_null(dev, -ENODEV, "Cannot get ADC device");

	ret = adc_read(dev, &sequence);
	prod_assert_equal(ret, 0, -EIO, "adc_read() failed with code %d\r\n");

	u8_t sample_value = (m_sample_buffer[0] & 0xFF);
	float voltage = ((3600.0 * sample_value) / 255.0);

	ma_current = (voltage * 1000.0 * 2.0) / 25.0;

	switch (component) {
		case CUR_BASE:
			printk("[CURRENT]: #Baseline# current: ");
			break;
		case CUR_LEDS_RED:
			printk("[CURRENT]: #LEDS_RED# current: ");
			break;
		case CUR_LEDS_GREEN:
			printk("[CURRENT]: #LEDS_GREEN# current: ");
			break;
		case CUR_LEDS_BLUE:
			printk("[CURRENT]: #LEDS_BLUE# current: ");
			break;
		case CUR_SENSE_LED_R:
			printk("[CURRENT]: #SENSE_LED_R# current: ");
			break;
		case CUR_SENSE_LED_G:
			printk("[CURRENT]: #SENSE_LED_G# current: ");
			break;
		case CUR_SENSE_LED_B:
			printk("[CURRENT]: #SENSE_LED_B# current: ");
			break;
		case CUR_BUZZER:
			printk("[CURRENT]: #Buzzer# current: ");
			break;
	}
	printf("%.3f mA - voltage: %.3f V\r\n", ma_current/1000, voltage/1000);

	toggle_indicator_pin();
	return 0;
}

static void run_test(int (*test)(void), char *testname)
{
	int ret;

	printk("============================\r\n");
	printk("Starting test: %s\r\n\r\n", testname);
	k_sleep(200);
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

	struct device *dev;
	dev = device_get_binding("ADXL372");
	if (dev == NULL) {
		printk("[ERROR] ADXL372, no binding established.\n");
		return 0;
	}
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
	return 0;
}

static int pca20035_ADXL362(void)
{
	int err = 0;

	struct device *dev;
	dev = device_get_binding("ADXL362");
	if (dev == NULL) {
		printk("[ERROR] ADXL362, no binding established.\n");
		return 0;
	}
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
	return 0;
}

static int pca20035_BME680(void)
{
	int err = 0;

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

	return 0;
}

static int pca20035_BH1749(void)
{
	struct device *dev;
	struct device *gpio;

	dev = device_get_binding("BH1749");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");

	gpio = device_get_binding(DT_GPIO_P0_DEV_NAME);
	prod_assert_not_null(gpio, -ENODEV, "Failed to get binding to gpio");

	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);
	k_sleep(LED_POLL_TIME);

	gpio_pin_write(gpio, SENSE_LED_R, 1);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);
	measure_current_and_voltage(CUR_SENSE_LED_R);
	k_sleep(LED_POLL_TIME);
	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);
	k_sleep(LED_POLL_TIME);


	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 1);
	gpio_pin_write(gpio, SENSE_LED_B, 0);
	measure_current_and_voltage(CUR_SENSE_LED_G);
	k_sleep(LED_POLL_TIME);
	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);
	k_sleep(LED_POLL_TIME);

	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 1);
	measure_current_and_voltage(CUR_SENSE_LED_B);
	k_sleep(LED_POLL_TIME);
	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);
	k_sleep(LED_POLL_TIME);

	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);

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
	measure_current_and_voltage(CUR_BUZZER);
	k_sleep(BUZZER_BEEP_INTERVAL);
	printk("Turning buzzer OFF\n");
	err_code = pwm_pin_set_usec(dev, BUZZER_PIN, period, 0);
	prod_assert_equal(err_code, 0, -EIO, "Failed to clear pwm pin");
	return 0;
}

void main(void)
{
	int err;
	struct device *gpio;
	struct device *adc_dev;

	err = dk_leds_init();
	if (err) {
		printk("Could not initialize leds, err code: %d\n", err);
	}

	gpio = device_get_binding(DT_GPIO_P0_DEV_NAME);
	prod_assert_not_null(gpio, -ENODEV, "Failed to get binding to gpio");
	gpio_pin_configure(gpio, TOGGLE_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, SENSE_LED_R, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, SENSE_LED_G, GPIO_DIR_OUT);
	gpio_pin_configure(gpio, SENSE_LED_B, GPIO_DIR_OUT);

	gpio_pin_write(gpio, SENSE_LED_R, 0);
	gpio_pin_write(gpio, SENSE_LED_G, 0);
	gpio_pin_write(gpio, SENSE_LED_B, 0);

	adc_dev = init_adc();
	prod_assert_not_null(adc_dev, -ENODEV, "Failed to get binding to adc");
	k_sleep(100);

	printk("Starting production test - thingy:91\r\n");

	dk_set_leds(DK_NO_LEDS_MSK);
	measure_current_and_voltage(CUR_BASE);
	k_sleep(LED_POLL_TIME);
	dk_set_leds(LEDS_RED);
	measure_current_and_voltage(CUR_LEDS_RED);
	k_sleep(LED_POLL_TIME);
	dk_set_leds(DK_NO_LEDS_MSK);
	k_sleep(LED_POLL_TIME);
	dk_set_leds(LEDS_GREEN);
	measure_current_and_voltage(CUR_LEDS_GREEN);
	k_sleep(LED_POLL_TIME);
	dk_set_leds(DK_NO_LEDS_MSK);
	k_sleep(LED_POLL_TIME);
	dk_set_leds(LEDS_BLUE);
	measure_current_and_voltage(CUR_LEDS_BLUE);
	k_sleep(LED_POLL_TIME);
	dk_set_leds(DK_NO_LEDS_MSK);
	k_sleep(LED_POLL_TIME);

	run_test(&pca20035_BH1749, "pca20035_BH1749");
	run_test(&pca20035_ADXL372, "pca20035_ADXL372");
	run_test(&pca20035_ADXL362, "pca20035_ADXL362");
	run_test(&pca20035_BME680, "pca20035_BME680");
	// run_test(&pca20035_test_button, "pca20035_button");
	run_test(&pca20035_test_buzzer, "pca20035_buzzer");
	//run_test(&measure_voltage, "measure_voltage");
	k_sleep(500);
	// Stop execution if test failed.
	all_tests_succeeded ? printk("\r\nTEST SUITE SUCCESS!\r\n") :
	printk("\r\nTEST SUITE FAILED!\r\n");
	printk("\n");
}
