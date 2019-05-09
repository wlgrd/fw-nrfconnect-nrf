/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <sensor.h>
#include <nrf_cloud.h>
#include <gpio.h>
#include <dk_buttons_and_leds.h>
#include <device.h>
#include <pwm.h>

#define prod_assert_equal(a, b, err, msg)                                      \
	if (a != b) {                                                          \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__,   \
		       msg);                                                   \
		return err;                                                    \
	}

#define prod_assert_not_equal(a, b, err, msg)                                  \
	if (a == b) {                                                          \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__,   \
		       msg);                                                   \
		return err;                                                    \
	}
#define prod_assert_not_null(a, err, msg)                                      \
	if (a == NULL) {                                                       \
		printk("Error at %s, line %d -  %s\r\n", __func__, __LINE__,   \
		       msg);                                                   \
		return err;                                                    \
	}

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

#if defined(ENABLE_RTT_CMD_GET)
#include <SEGGER_RTT.h>
static u8_t m_rtt_keys[20];
static u8_t m_rtt_rx_keyindex = 0;
bool m_test_params_received = false;
static u8_t msg[3];
#endif
static u8_t button_test_timeout = 80;
bool all_tests_succeeded = true;
struct sensor_value temp;
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
	printk("Starting test: %s\r\n", testname);
	ret = test();
	if (ret != 0) {
		all_tests_succeeded = false;
		printk("Failed: %s\r\n", testname);
		return;
	}
	k_sleep(200);
	printk("Succeeded: %s\r\n", testname);
}

static int pca20035_ADXL372(void)
{
	dk_set_leds(LEDS_RED);
	struct device *dev;
	dev = device_get_binding("ADXL372");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
	dk_set_leds(DK_NO_LEDS_MSK);
	return 0;
}

static int pca20035_ADXL362(void)
{
	dk_set_leds(LEDS_BLUE);
	volatile struct device *dev;
	dev = device_get_binding("ADXL362");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
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

	err = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get temperature");
	printk("BME680 temperature: %d.%d\r\n", temp.val1, temp.val2);

	err = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get pressure");
	printk("BME680 pressure: %d.%d kPa\r\n", temp.val1, temp.val2);

	err = sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get humidity");
	printk("BME680 humidity: %d.%d %%RH\r\n", temp.val1, temp.val2);

	err = sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &temp);
	prod_assert_equal(err, 0, -EIO, "Failed to get gas");
	printk("BME680 gas resistance: %d ohms\r\n", temp.val1);

	dk_set_leds(DK_NO_LEDS_MSK);
	return 0;
}
static int pca20035_BH1749(void)
{
	measure_voltage();
	struct device *dev;
	dev = device_get_binding("BH1749");
	prod_assert_not_null(dev, -ENODEV, "Failed to get binding");
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
	NRF_CLOCK_S->TASKS_HFCLKSTART = 1;
	k_sleep(5);
	err = dk_leds_init();
	if (err) {
		PRINT("Could not initialize leds, err code: %d\n", err);
	}
	err = dk_buttons_init(button_handler);
	if (err) {
		PRINT("Could not initialize buttons, err code: %d\n", err);
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
	// for(u8_t i = 0; i < 5; i++)
	// {
	//         dk_set_leds(LEDS_PATTERN_WAIT);
	//         k_sleep(1);
	//         dk_set_leds(DK_NO_LEDS_MSK);
	//         k_sleep(200);
	// };

	while (1) {
		run_test(&pca20035_ADXL372, "pca20035_ADXL372");
		run_test(&pca20035_ADXL362, "pca20035_ADXL362");
		run_test(&pca20035_BME680, "pca20035_BME680");
		run_test(&pca20035_BH1749, "pca20035_BH1749");
		run_test(&pca20035_test_button, "pca20035_button");
		run_test(&pca20035_test_buzzer, "pca20035_buzzer");
		k_sleep(100);
		// Stop execution if test failed.
		all_tests_succeeded ? printk("\r\nTEST SUITE SUCCESS!\r\n") :
				      printk("\r\nTEST SUITE FAILED!\r\n");
		if (!all_tests_succeeded)
			break;
	}
	dk_set_leds(LEDS_PATTERN_WAIT);
}
