/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <ztest.h>
#include <sensor.h>
#include <nrf_cloud.h>
#include <dk_buttons_and_leds.h>

#define BUTTON_1 BIT(0)
#define BUTTON_2 BIT(1)
#define SWITCH_1 BIT(2)
#define SWITCH_2 BIT(3)

#define LEDS_PATTERN_WAIT (DK_LED2_MSK | DK_LED3_MSK)
#define LEDS_PATTERN_ENTRY (DK_LED1_MSK | DK_LED2_MSK)
#define LEDS_PATTERN_DONE (DK_LED2_MSK | DK_LED3_MSK)
#define LEDS_ERROR_UNKNOWN (DK_ALL_LEDS_MSK)
#define LEDS_RED DK_LED1_MSK
#define LEDS_GREEN DK_LED2_MSK
#define LEDS_BLUE DK_LED3_MSK

static bool all_tests_succeeded = true;
static u8_t button_test_timeout = 2*10;                  // 8 seconds
static void blink_leds_blocking(u32_t led_color)
{
	dk_set_leds(led_color);
	k_sleep(50);
	dk_set_leds(DK_NO_LEDS_MSK);
	k_sleep(600);
}
static void ADXL372(void)
{
	struct device *dev;
	dev = device_get_binding(__func__);
	all_tests_succeeded = (dev ? all_tests_succeeded : false);
	zassert_not_null(dev, "Failed to get %s\n", __func__);
}

static void ADXL362(void)
{
	struct device *dev;
	dev = device_get_binding(__func__);
	all_tests_succeeded = (dev ? all_tests_succeeded : false);
	zassert_not_null(dev, "Failed to get %s \n", __func__);
}

static void BME680(void)
{
	struct device *dev;
	dev = device_get_binding(__func__);
	all_tests_succeeded = (dev ? all_tests_succeeded : false);
	zassert_not_null(dev, "Failed to get %s \n", __func__);
}
static void BH1749(void)
{
	struct device *dev;
	dev = device_get_binding(__func__);
	all_tests_succeeded = (dev ? all_tests_succeeded : false);
	zassert_not_null(dev, "Failed to get %s \n", __func__);
}

static void test_button(void)
{
	u32_t volatile state, newstate, timeout = 0;
	state = dk_get_buttons();
        newstate = state;
	printk("Waiting for button press...");
	dk_set_leds(LEDS_RED);
	while ((state == newstate) && (timeout < button_test_timeout)) {
		newstate = dk_get_buttons();
		timeout++;
		k_sleep(100);
	}
	PRINT("timeout: %d \n", timeout);
        all_tests_succeeded = ( (timeout != button_test_timeout) ? all_tests_succeeded : false);
	zassert_not_equal(timeout, button_test_timeout, "Button test timed out");
	dk_set_leds(DK_NO_LEDS_MSK);
}
static void button_handler(u32_t buttons, u32_t has_changed)
{

}

void test_main(void)
{
	int err;
        bool run_tests_again = true;
	err = dk_leds_init();
	if (err) {
		printk("Could not initialize leds, err code: %d\n", err);
	}
	err = dk_buttons_init(button_handler);
	if (err) {
		printk("Could not initialize buttons, err code: %d\n", err);
	}

	PRINT("Starting test\r\n");

	ztest_test_suite(thingy91_production, /* Name of test suite */
			 ztest_unit_test(ADXL362));
	while (run_tests_again == true) {
		ztest_run_test_suite(thingy91_production);
                if(all_tests_succeeded)
                {
                        blink_leds_blocking(LEDS_GREEN);
                        run_tests_again = false;
                }
                else
                {
                        blink_leds_blocking(LEDS_RED);
                        run_tests_again = false;
                }
		k_sleep(200);
		
	}
	while (1) {
		all_tests_succeeded ? blink_leds_blocking(LEDS_GREEN) :
				      blink_leds_blocking(LEDS_RED);
	}
}
