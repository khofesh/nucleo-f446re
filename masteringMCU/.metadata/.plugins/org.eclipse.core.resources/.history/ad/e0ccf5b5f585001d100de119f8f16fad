/*
 * 016rtc_lcd.c
 *
 *  Created on: Dec 27, 2022
 *      Author: fahmad
 */

#include <stdio.h>
#include "stm32f446xx.h"
#include "ds1307.h"

char *get_day_of_week(uint8_t day);
char *time_to_string(RTC_time_t *rtc_time);
char *date_to_string(RTC_date_t *rtc_date);
void number_to_string(uint8_t num, char *buf);

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

    current_date.day = WEDNESDAY;
    current_date.date = 27;
    current_date.month = 12;
    current_date.year = 22;

    current_time.hours = 9;
    current_time.minutes = 30;
    current_time.seconds = 41;
    current_time.time_format = TIME_FORMAT_12HRS_PM;

    ds1307_get_current_date(&current_date);
    ds1307_get_current_time(&current_time);

    ds1307_get_current_time(&current_time);
    ds1307_get_current_date(&current_date);

    char *am_pm;
    if (current_time.time_format != TIME_FORMAT_24HRS)
    {
        am_pm = (current_time.time_format) ? "PM" : "AM";
        printf("current time = %s %s\n", time_to_string(&current_time), am_pm);
    }
    else
    {
        printf("current time = %s\n", time_to_string(&current_time));
    }

    // 27/12/22 <wednesday>
    printf("current date = %s <%s>\n", date_to_string(&current_date),
           get_day_of_week(current_date.day));

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
