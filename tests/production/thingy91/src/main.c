/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <ztest.h>
#include <sensor.h>
#include <nrf_cloud.h>
#include <dk_buttons_and_leds.h>
#define BUTTON_1		BIT(0)
#define BUTTON_2		BIT(1)
#define SWITCH_1		BIT(2)
#define SWITCH_2		BIT(3)

#define LEDS_PATTERN_WAIT	(DK_LED2_MSK | DK_LED3_MSK)
#define LEDS_PATTERN_ENTRY	(DK_LED1_MSK | DK_LED2_MSK)
#define LEDS_PATTERN_DONE	(DK_LED2_MSK | DK_LED3_MSK)
#define LEDS_ERROR_UNKNOWN	(DK_ALL_LEDS_MSK)
#define LEDS_RED                DK_LED1_MSK
#define LEDS_GREEN              DK_LED2_MSK
#define LEDS_BLUE               DK_LED3_MSK

static void ADXL372(void)
{
        dk_set_leds(LEDS_RED);
	struct device *dev;
	dev = device_get_binding(__func__);
	zassert_not_null(dev, "Failed to get %s\n", __func__);
        dk_set_leds(DK_NO_LEDS_MSK);
}

static void ADXL362(void)
{
        dk_set_leds(LEDS_BLUE);
	struct device *dev;
	dev = device_get_binding(__func__);
	zassert_not_null(dev, "Failed to get %s \n", __func__);
        dk_set_leds(DK_NO_LEDS_MSK);
}

static void BME680(void)
{
        dk_set_leds(LEDS_GREEN);
	struct device *dev;
	dev = device_get_binding(__func__);
	zassert_not_null(dev, "Failed to get %s \n", __func__);
        dk_set_leds(DK_NO_LEDS_MSK);
}
static void BH1749(void)
{
	struct device *dev;
	dev = device_get_binding(__func__);
	zassert_not_null(dev, "Failed to get %s \n", __func__);
}

static void BUTTON(void)
{
        u32_t state, has_changed;

        
        while(1)
        {
                dk_set_leds(LEDS_RED);
                dk_read_buttons(&state, &has_changed);
                k_sleep(200);
                dk_set_leds(DK_NO_LEDS_MSK);
                k_sleep(200);
        };
                

        
}
static void button_handler(u32_t buttons, u32_t has_changed)
{
}

void test_main(void)
{
	int err;
	err = dk_leds_init();
	if (err) {
		printk("Could not initialize leds, err code: %d\n", err);
	}
	err = dk_buttons_init(button_handler);
	if (err) {
		printk("Could not initialize buttons, err code: %d\n", err);
	}

	PRINT("Starting production test - thingy:91\r\n");
	PRINT("Waiting for test parameters...\r\n");

        for(u8_t i = 0; i < 5; i++)
        {
	        dk_set_leds(LEDS_PATTERN_WAIT);
                k_sleep(1);
                dk_set_leds(DK_NO_LEDS_MSK);
                k_sleep(200);
        };
	PRINT("Got test parameters!\r\n");
	ztest_test_suite(thingy91_production,	/* Name of test suite */
		ztest_unit_test(ADXL372),      	/* Add tests... */
		ztest_unit_test(BME680),
		ztest_unit_test(ADXL362),
		ztest_unit_test(BH1749)
	);
	while(1)
	{
		ztest_run_test_suite(thingy91_production);
		k_sleep(100);
		break;
	}
	while(1);
}
