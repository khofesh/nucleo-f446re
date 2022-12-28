/*
 * 016rtc_lcd.c
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f446xx.h"
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK 16000000UL
#define PRINT_LCD

char *get_day_of_week(uint8_t day);
char *time_to_string(RTC_time_t *rtc_time);
char *date_to_string(RTC_date_t *rtc_date);
void number_to_string(uint8_t num, char *buf);
void init_systick_timer(uint32_t tick_hz);
static void mdelay(uint32_t cnt);

int main()
{
    RTC_time_t current_time;
    RTC_date_t current_date;

#ifndef PRINT_LCD
    printf("RTC test\n");
#else
    lcd_init();

    lcd_print_string("rtc test...");

    mdelay(2000);

    lcd_display_clear();
    lcd_display_return_home();
#endif

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
#ifndef PRINT_LCD

        printf("current time = %s %s\n", timeInString, am_pm);
#else
        lcd_print_string(timeInString);
        lcd_print_string(am_pm);
#endif
    }
    else
    {
        timeInString = time_to_string(&current_time);
#ifndef PRINT_LCD
        printf("current time = %s\n", timeInString);
#else
        lcd_print_string(timeInString);
#endif
    }

    // 27/12/22 <wednesday>
    char *dateInString = date_to_string(&current_date);
    char *dayOfWeek = get_day_of_week(current_date.day);

#ifndef PRINT_LCD
    printf("current date = %s <%s>\n", dateInString, dayOfWeek);

#else
    char *separation = "<";

    lcd_set_cursor(2, 1);
    lcd_print_string(dateInString);
    lcd_print_char((uint8_t)atoi(separation));
    lcd_print_string(dayOfWeek);
#endif

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

#ifndef PRINT_LCD
        printf("current time = %s %s\n", timeInString, am_pm);
#else
        lcd_set_cursor(1, 1);
        lcd_print_string(timeInString);
        lcd_print_string(am_pm);
#endif
    }
    else
    {
        timeInString = time_to_string(&current_time);
#ifndef PRINT_LCD
        printf("current time = %s\n", timeInString);
#else
        lcd_set_cursor(1, 1);
        lcd_print_string(timeInString);
#endif
    }

    ds1307_get_current_date(&current_date);

    // 27/12/22 <wednesday>
    char *dateInString = date_to_string(&current_date);
    char *dayOfWeek = get_day_of_week(current_date.day);

#ifndef PRINT_LCD
    printf("current date = %s <%s>\n", dateInString, dayOfWeek);
#else
    char *sepStart = "<";
    char *sepEnd = ">";

    lcd_set_cursor(2, 1);
    lcd_print_string(dateInString);
    lcd_print_char((uint8_t)atoi(sepStart));
    lcd_print_string(dayOfWeek);
    lcd_print_char((uint8_t)atoi(sepEnd));
#endif
}

static void mdelay(uint32_t cnt)
{
    for (uint32_t i = 0; i < (cnt * 1000); i++)
        ;
}
