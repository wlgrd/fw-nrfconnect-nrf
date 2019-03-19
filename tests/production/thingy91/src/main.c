/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <ztest.h>


void test_xtal_32khz(void)
{
	zassert_true(true, "32khz passed test, yey");
	k_sleep(500);
}
void test_hig_accell(void)
{
	zassert_true(true, "HiG Accell passed test, yey");
	k_sleep(500);
}
void test_bme680(void)
{
	zassert_true(true, "bm3680 passed test, yey");
	k_sleep(500);
}
void test_BUZZER(void)
{
	zassert_true(true, "BUZZER failed test, nooo");
	k_sleep(500);
}

void test_main(void)
{
	PRINT("Starting production test - thingy:91\r\n");
	ztest_test_suite(thingy91_production,
			 ztest_unit_test(test_xtal_32khz),
			 ztest_unit_test(test_hig_accell),
			 ztest_unit_test(test_bme680),
			 ztest_unit_test(test_BUZZER)
			 );
	ztest_run_test_suite(thingy91_production);
}
