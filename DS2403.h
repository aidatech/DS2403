// DS2403 RTC chip library for Arduino, v0.1, 2015-12-06
// LICENSE: GPL v3 (http://www.gnu.org/licenses/gpl.html)
// DS2403 Class is modified from DS3231 Class to suit for DS2403 RTC chip.
// DateTime Class is a modified version supporting automatically calculating the days 
// of the week via YY/MM/DD format.
// By Jun Fu(傅均) @ AidaTech, http://www.aidatech.cn  

// Original DS3231 Class is by Seeed Technology Inc( http://www.seeedstudio.com ).
// DateTime Class is a modified version supporting day-of-week.

// Original DateTime Class and its utility code is by Jean-Claude Wippler at JeeLabs
// http://jeelabs.net/projects/cafe/wiki/RTClib
// Released under MIT License http://opensource.org/licenses/mit-license.php

#ifndef DS2403_H
#define DS2403_H

#include "Arduino.h"
#define DS2403_ADDRESS	      0x32 //I2C Slave address see DS2403 datasheet

/* DS2403 Registers. Refer application manual */
#define DS2403_SEC_REG        0x00
#define DS2403_MIN_REG        0x01
#define DS2403_HOUR_REG       0x02
#define DS2403_WDAY_REG       0x03
#define DS2403_MDAY_REG       0x04
#define DS2403_MONTH_REG      0x05
#define DS2403_YEAR_REG       0x06

#define DS2403_ALARMSEC_REG     0x07
#define DS2403_ALARMMIN_REG     0x08
#define DS2403_ALARMHOUR_REG    0x09
#define DS2403_ALARMWDAY_REG    0x0A

#define DS2403_ALARMDAY_REG     0x0B
#define DS2403_ALARMMONTH_REG   0x0C
#define DS2403_ALARMYEAR_REG    0x0D

#define DS2403_ALARMENABLE_REG  0x0E
#define DS2403_CTR1_REG       0x0F
#define DS2403_CTR2_REG       0x10
#define DS2403_CTR3_REG           0x11
#define DS2403_MICROADJUST_REG    0x12
#define DS2403_BACKCNT_REG        0x13
#define DS2403_BEGINRAM_REG       0x14

#define EverySecond     0x01
#define EveryMinute     0x02
#define EveryHour       0x03

//--------------------------------------------------------------
//                        class DateTime
//--------------------------------------------------------------
// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
  public:
    DateTime (long t = 0);
    DateTime (uint16_t year, uint8_t month, uint8_t date,
              uint8_t hour, uint8_t mint, uint8_t sec);
    DateTime (const char* date, const char* time);

    uint8_t second() const      {
      return ss;
    }
    uint8_t minute() const      {
      return mm;
    }
    uint8_t hour() const        {
      return hh;
    }

    uint8_t date() const        {
      return d;
    }
    uint8_t month() const       {
      return m;
    }
    uint16_t year() const       {
      return 2000 + yOff;
    }

    uint8_t dayOfWeek() const   {
      return wday; /*Su=0 Mo=1 Tu=3 We=4 Th=5 Fr=6 Sa=7 */
    }

    // 32-bit time as seconds since 1/1/2000
    long get() const;

  protected:
    uint8_t yOff, m, d, hh, mm, ss, wday;
};

//--------------------------------------------------------------
//                        class DS2403
//--------------------------------------------------------------
// RTC DS2403 chip connected via I2C and uses the Wire library.
// Only 24 Hour time format is supported in this implementation
class DS2403 {
  public:
    uint8_t begin(void);
    uint8_t readRegister(uint8_t regaddress);
    void writeRegister(uint8_t regaddress, uint8_t value);
    void WriteTimeOn(void);
    void WriteTimeOff(void);
    void adjust(const DateTime& dt);  //Changes the date-time
    static DateTime now();            //Gets the current date-time

    //Decides the INT pin's output setting
    //periodicity can be any of following defines: EveryMinute, EveryHour
    void enableChrime(uint8_t periodicity);
    void enableInterrupts(boolean isRepeat, uint8_t hh24, uint8_t mm, uint8_t ss);
    void disableInterrupts();
    void clearINTStatus();

  protected:
    uint8_t intType, intPeriodicity, intHH24, intMM;
};

#endif


