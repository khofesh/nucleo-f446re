/*
 * 016rtc_lcd.c
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#include <stdio.h>
#include "stm32f446xx.h"
#include "ds1307.h"

#define SYSTICK_TIM_CLK 16000000UL

char *get_day_of_week(uint8_t day);
char *time_to_string(RTC_time_t *rtc_time);
char *date_to_string(RTC_date_t *rtc_date);
void number_to_string(uint8_t num, char *buf);
void init_systick_timer(uint32_t tick_hz);

int main()
{
    RTC_time_t current_time;
    RTC_date_t current_date;
    printf("RTC test\n");

    if (ds1307_init())
    {
        printf("RTC init has failed\n");
        while (1)
        {
        }
    }

    init_systick_timer(1);

    current_date.day = WEDNESDAY;
    current_date.date = 27;
    current_date.month = 12;
    current_date.year = 22;

    current_time.hours = 9;
    current_time.minutes = 30;
    current_time.seconds = 41;
    current_time.time_format = TIME_FORMAT_12HRS_PM;

    ds1307_set_current_date(&current_date);
    ds1307_set_current_time(&current_time);

    ds1307_get_current_time(&current_time);
    ds1307_get_current_date(&current_date);

    char *am_pm;
    char *timeInString;
    if (current_time.time_format != TIME_FORMAT_24HRS)
    {
        am_pm = (current_time.time_format) ? "PM" : "AM";
        timeInString = time_to_string(&current_time);
        printf("current time = %s %s\n", timeInString, am_pm);
    }
    else
    {
        timeInString = time_to_string(&current_time);
        printf("current time = %s\n", timeInString);
    }

    // 27/12/22 <wednesday>
    char *dateInString = date_to_string(&current_date);
    char *dayOfWeek = get_day_of_week(current_date.day);
    printf("current date = %s <%s>\n", dateInString, dayOfWeek);

    while (1)
    {
        /* code */
    }
}

/**
 * @brief Get the day of week object
 *
 * @param day
 * @return char*
 */
char *get_day_of_week(uint8_t day)
{
    char *days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

    return days[day - 1];
}

/**
 * @brief convert time to string
 * hh:mm:ss
 * @param rtc_time
 * @return char*
 */
char *time_to_string(RTC_time_t *rtc_time)
{
    static char buf[9];

    buf[2] = ':';
    buf[5] = ':';

    number_to_string(rtc_time->hours, buf);
    number_to_string(rtc_time->minutes, &buf[3]);
    number_to_string(rtc_time->seconds, &buf[6]);

    buf[8] = '\0';

    return buf;
}

/**
 * @brief convert date to strime
 * dd/mm/yy
 * @param rtc_date
 * @return char*
 */
char *date_to_string(RTC_date_t *rtc_date)
{
    static char buf[9];

    buf[2] = '/';
    buf[5] = '/';

    number_to_string(rtc_date->date, buf);
    number_to_string(rtc_date->month, &buf[3]);
    number_to_string(rtc_date->year, &buf[6]);

    buf[8] = '\0';

    return buf;
}

/**
 * @brief convert number to string
 *
 * @param num
 * @param buf
 */
void number_to_string(uint8_t num, char *buf)
{

    if (num < 10)
    {
        buf[0] = '0';
        buf[1] = num + 48;
    }
    else if (num >= 10 && num < 99)
    {
        buf[0] = (num / 10) + 48;
        buf[1] = (num % 10) + 48;
    }
}

void init_systick_timer(uint32_t tick_hz)
{
    uint32_t *pSRVR = (uint32_t *)0xE000E014;
    uint32_t *pSCSR = (uint32_t *)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK / tick_hz) - 1;

    // Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    // load the value in to SVR
    *pSRVR |= count_value;

    // do some settings
    *pSCSR |= (1 << 1); // Enables SysTick exception request:
    *pSCSR |= (1 << 2); // Indicates the clock source, processor clock source

    // enable the systick
    *pSCSR |= (1 << 0); // enables the counter
}

void SysTick_Handler()
{
    RTC_time_t current_time;
    RTC_date_t current_date;

    ds1307_get_current_time(&current_time);

    char *am_pm;
    char *timeInString;
    if (current_time.time_format != TIME_FORMAT_24HRS)
    {
        am_pm = (current_time.time_format) ? "PM" : "AM";
        timeInString = time_to_string(&current_time);
        printf("current time = %s %s\n", timeInString, am_pm);
    }
    else
    {
        timeInString = time_to_string(&current_time);
        printf("current time = %s\n", timeInString);
    }

    ds1307_get_current_date(&current_date);

    // 27/12/22 <wednesday>
    char *dateInString = date_to_string(&current_date);
    char *dayOfWeek = get_day_of_week(current_date.day);
    printf("current date = %s <%s>\n", dateInString, dayOfWeek);
}
