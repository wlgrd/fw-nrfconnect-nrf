/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <ztest.h>
#include <sensor.h>
#include <nrf_cloud.h>
#include <gpio.h>
#include <dk_buttons_and_leds.h>
//#include <pwm.h>
#include <SEGGER_RTT.h>

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
#define BUZZER_PIN              28

//#define ENABLE_RTT_CMD_GET

#if defined(ENABLE_RTT_CMD_GET)
static u8_t     m_rtt_keys[20];
static u8_t     m_rtt_rx_keyindex = 0;
bool            m_test_params_received = false;
static u8_t     msg[3];
#endif
static u8_t     button_test_timeout = 80;
bool            all_tests_succeeded = true;

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

static void test_buzzer(void)
{
        //static struct device *gpio_dev;
        static struct device *pwm_dev;
        pwm_dev = device_get_binding(DT_NORDIC_NRF_PWM_PWM_0_LABEL);
        //gpio_dev = device_get_binding(DT_GPIO_P0_DEV_NAME);

        if (!pwm_dev) {
                printk("Cannot bind pwm device");
                return -ENODEV;
	}
        pwm_pin_set_usec(pwm_dev, BUZZER_PIN, 5, 5);
        k_sleep(1200);
        pwm_pin_set_usec(pwm_dev, BUZZER_PIN, 10, 10);
        k_sleep(1200);
        // pwm_pin_set_usec(pwm_dev, BUZZER_PIN, 50, 50);
        // k_sleep(200);
        // pwm_pin_set_usec(pwm_dev, BUZZER_PIN, 0, 0);
}

static void button_handler(u32_t buttons, u32_t has_changed)
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
        #if defined(ENABLE_RTT_CMD_GET)
	PRINT("Waiting for test parameters...\r\n");
        while(!m_test_params_received)
        {
                if(SEGGER_RTT_HasKey())
                {
                        // Fetch the first key in the buffer
                        m_rtt_keys[m_rtt_rx_keyindex] = SEGGER_RTT_GetKey();

                        // Q is set as "EOL", so parse when received
                        if(m_rtt_keys[m_rtt_rx_keyindex] == 'Q')
                        {
                                check_rtt_command(m_rtt_keys, ++m_rtt_rx_keyindex);
                                m_rtt_rx_keyindex = 0;
                                memset(m_rtt_keys, 0, sizeof(m_rtt_keys));
                        } else{
                                // Keep buffering data
                                m_rtt_rx_keyindex++;
                        }
                }
        }
        #endif

        for(u8_t i = 0; i < 5; i++)
        {
	        dk_set_leds(LEDS_PATTERN_WAIT);
                k_sleep(1);
                dk_set_leds(DK_NO_LEDS_MSK);
                k_sleep(200);
        };
	PRINT("Got test parameters!\r\n");
	ztest_test_suite(thingy91_production,	/* Name of test suite */
		ztest_unit_test(test_buzzer)   /* Add tests... */
		// ztest_unit_test(ADXL372),
		// ztest_unit_test(BME680),
		// ztest_unit_test(ADXL362),
		// ztest_unit_test(BH1749)
	);
	while(1)
	{
		ztest_run_test_suite(thingy91_production);
		k_sleep(100);
		break;
	}
}
