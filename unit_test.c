/*
 * unit_test.c
 *
 *  Created on: 02-Dec-2016
 *      Author: Satyanarayana
 */


#include "capsense.h"
#include <stdio.h>
#include "main.h"
uint8_t test_status;

uint8_t  test_touch()
{
	int output = touch_data(10);
	if (!(output > 0))
	{
		return test_status = 1 ;
	}
	else
	{
		return test_status = 0;
	}
}


uint8_t  test_not_touch()
{
	int output = touch_data(10);
	if (output > 0)
	{
		return test_status =1 ;
	}
	else
	{
		return test_status = 0;
	}
}

