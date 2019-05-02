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
#include <device.h>
#include <pwm.h>
#include <SEGGER_RTT.h>

#define BUTTON_1		BIT(0)
#define BUTTON_2		BIT(1)
#define SWITCH_1		BIT(2)
#define SWITCH_2		BIT(3)

#define LEDS_PATTERN_WAIT	(DK_LED2_MSK | DK_LED3_MSK)
#define LEDS_RED                DK_LED1_MSK
#define LEDS_GREEN              DK_LED2_MSK
#define LEDS_BLUE               DK_LED3_MSK
#define BUZZER_PIN              28      // TP7

/* There is an issue with the pwm driver, causing a 300us delay */
#define PWM_OFFSET      300
#define PWM_CHANNEL     0
#define OSC_FREQ_HZ     2700
#define OSC_FREQ_US     ((1000000UL / OSC_FREQ_HZ))
#define PERIOD          (OSC_FREQ_US / 2)               //> Need to set it half due to bug
#define DUTY_CYCLE      (PERIOD / 4)                    //> Maximum 50% Duty cycle due to bug

#if defined(ENABLE_RTT_CMD_GET)
static u8_t     m_rtt_keys[20];
static u8_t     m_rtt_rx_keyindex = 0;
bool            m_test_params_received = false;
static u8_t     msg[3];
#endif
static u8_t     button_test_timeout = 80;
bool            all_tests_succeeded = true;

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
		// timeout++;
		k_sleep(100);
	}
        return;
}
static void ADXL372(void)
{
        dk_set_leds(LEDS_RED);
	struct device *dev;
	dev = device_get_binding(__func__);
	if(dev == NULL) all_tests_succeeded = false;
	zassert_not_null(dev, "Failed to get %s\n", __func__);
        dk_set_leds(DK_NO_LEDS_MSK);
}

static void ADXL362(void)
{
        dk_set_leds(LEDS_BLUE);
	struct device *dev;
	dev = device_get_binding(__func__);
	if(dev == NULL) all_tests_succeeded = false;
	zassert_not_null(dev, "Failed to get %s \n", __func__);
        dk_set_leds(DK_NO_LEDS_MSK);
}

static void BME680(void)
{
        dk_set_leds(LEDS_GREEN);
	struct device *dev;
	dev = device_get_binding(__func__);
	if(dev == NULL) all_tests_succeeded = false;
	zassert_not_null(dev, "Failed to get %s \n", __func__);
        dk_set_leds(DK_NO_LEDS_MSK);
}
static void BH1749(void)
{
	struct device *dev;
	dev = device_get_binding(__func__);
	if(dev == NULL) all_tests_succeeded = false;
	zassert_not_null(dev, "Failed to get %s \n", __func__);
}

static void test_button(void)
{
	u32_t volatile state, newstate, timeout = 0, button_test_timeout = 200000;
	PRINT("[ACTION]: Please press the button ...\n");
	state = dk_get_buttons();
        newstate = state;	
	dk_set_leds(LEDS_RED);
	while ((state == newstate) && (timeout < button_test_timeout)) {
		newstate = dk_get_buttons();
		timeout++;
		k_sleep(1);
	}
	PRINT("timeout: %d \n", timeout);
        all_tests_succeeded = ( (timeout != button_test_timeout) ? all_tests_succeeded : false);
	zassert_not_equal(timeout, button_test_timeout, "Button test timed out\n");
	dk_set_leds(DK_NO_LEDS_MSK);
}

static void test_buzzer(void)
{
        volatile int err_code = 0xFF;
        static struct device *pwm_dev;
        static u32_t period = PERIOD;
        static u32_t duty_cycle = DUTY_CYCLE;
        pwm_dev = device_get_binding(DT_NORDIC_NRF_PWM_PWM_0_LABEL);
        printk("Turning buzzer ON\n");
        if (!pwm_dev) {
                zassert_unreachable("Cannot bind pwm device");
		all_tests_succeeded = false;
        }
        err_code = pwm_pin_set_usec(pwm_dev, BUZZER_PIN,
                                period, duty_cycle);
        if (err_code) {
                zassert_unreachable("Unable to turn buzzer ON\n");
		all_tests_succeeded = false;
                return;
        }
        k_sleep(1500);
        printk("Turning buzzer OFF\n");
        err_code = pwm_pin_set_usec(pwm_dev, BUZZER_PIN,
                                period, 0);
        if (err_code) {
                zassert_unreachable("Unable to turn buzzer OFF\n");
		all_tests_succeeded = false;
                return;
        }
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


	dk_set_leds(LEDS_RED);
	k_sleep(1000);
	dk_set_leds(DK_NO_LEDS_MSK);
	dk_set_leds(LEDS_GREEN);
	k_sleep(1000);
	dk_set_leds(DK_NO_LEDS_MSK);
	dk_set_leds(LEDS_BLUE);
	k_sleep(1000);
	dk_set_leds(DK_NO_LEDS_MSK);

	PRINT("Got test parameters!\r\n");
	ztest_test_suite(thingy91_production,	/* Name of test suite */
		ztest_unit_test(test_button),
		ztest_unit_test(ADXL372),      	/* Add tests... */
		ztest_unit_test(BME680),
		ztest_unit_test(ADXL362),
		ztest_unit_test(BH1749)
	);
	while(1)
	{
		ztest_run_test_suite(thingy91_production);
		k_sleep(100);
		// Stop execution if test failed.
		if(!all_tests_succeeded) break;
	}
	dk_set_leds(LEDS_PATTERN_WAIT);
}
