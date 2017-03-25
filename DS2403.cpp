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

#include <Wire.h>
#include <avr/pgmspace.h>
#include "DS2403.h"
//#include "Arduino.h"

#define SECONDS_PER_DAY 86400L

////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed
static uint8_t daysInMonth []  = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d)
{
  if (y >= 2000)
    y -= 2000;
  uint16_t days = d;
  for (uint8_t i = 1; i < m; ++i)
    days += pgm_read_byte(daysInMonth + i - 1);
  if (m > 2 && y % 4 == 0)
    ++days;
  return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s)
{
  return ((days * 24L + h) * 60 + m) * 60 + s;
}

//string to digit number
static uint8_t conv2d(const char* p)
{
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------
//                        class DateTime implementation
//--------------------------------------------------------------
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (long t)
{
  ss = t % 60;
  t /= 60;
  mm = t % 60;
  t /= 60;
  hh = t % 24;
  uint16_t days = t / 24;
  uint8_t leap;
  for (yOff = 0; ; ++yOff)
  {
    leap = yOff % 4 == 0;
    if (days < 365 + leap)
      break;
    days -= 365 + leap;
  }
  for (m = 1; ; ++m)
  {
    uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
    if (leap && m == 2)
      ++daysPerMonth;
    if (days < daysPerMonth)
      break;
    days -= daysPerMonth;
  }
  d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t mint, uint8_t sec)
{
  if (year >= 2000)
    year -= 2000;
  yOff = (uint8_t) year;
  m = month;
  d = date;
  hh = hour;
  mm = mint;
  ss = sec;
  //Modified by JunFu to add weekday automatically.
  // Use Zeller: W=Y+[Y/4]+[C/4]-2C+[26(M+1)/10]+d-1
  int8_t wtmp = 0;
  if ( m < 3 )
    wtmp = yOff - 1 + (yOff - 1) / 4 + 20 / 4 - 2 * 20 + 26 * (m + 12 + 1) / 10 + d - 1;
  else
    wtmp = yOff + yOff / 4 + 20 / 4 - 2 * 20 + 26 * (m + 1) / 10 + d - 1;
  if (wtmp > 0)
    wday = wtmp % 7;
  else
    wday = (wtmp % 7 + 7) % 7;
}

DateTime::DateTime (const char* date, const char* time)
{
  // sample input: date = "Dec 26 2009", time = "12:34:56"
  // note: "Dec 06 2009" is correct, while "Dec 6 2009" is wrong
  yOff = conv2d(date + 9);//convert last two char in date string to digit
  // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
  switch (date[0])
  {
    case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
    case 'F': m = 2; break;
    case 'A': m = date[2] == 'r' ? 4 : 8; break;
    case 'M': m = date[2] == 'r' ? 3 : 5; break;
    case 'S': m = 9; break;
    case 'O': m = 10; break;
    case 'N': m = 11; break;
    case 'D': m = 12; break;
  }
  d = conv2d(date + 4);
  hh = conv2d(time);
  mm = conv2d(time + 3);
  ss = conv2d(time + 6);
  //Modified by JunFu to add weekday automatically.
  // Use Zeller: W=Y+[Y/4]+[C/4]-2C+[26(M+1)/10]+d-1
  int8_t wtmp = 0;
  if ( m < 3 )
    wtmp = yOff - 1 + (yOff - 1) / 4 + 20 / 4 - 2 * 20 + 26 * (m + 12 + 1) / 10 + d - 1;
  else
    wtmp = yOff + yOff / 4 + 20 / 4 - 2 * 20 + 26 * (m + 1) / 10 + d - 1;
  if (wtmp > 0)
    wday = wtmp % 7;
  else
    wday = (wtmp % 7 + 7) % 7;
}

long DateTime::get() const
{
  uint16_t days = date2days(yOff, m, d);
  return time2long(days, hh, mm, ss);
}

static uint8_t bcd2bin (uint8_t val)
{
  return val - 6 * (val >> 4);
}

static uint8_t bin2bcd (uint8_t val)
{
  return val + 6 * (val / 10);
}

////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------
//                        class DS2403 implementation
//--------------------------------------------------------------
//read a register and return its value
uint8_t DS2403::readRegister(uint8_t regaddress)
{
  Wire.beginTransmission(DS2403_ADDRESS);
  Wire.write((byte)regaddress);
  Wire.endTransmission();
  Wire.requestFrom(DS2403_ADDRESS, 1);
  return Wire.read();
}

//SD2403 write enable
void DS2403::WriteTimeOn(void)
{
  uint8_t CTR2Reg = readRegister(DS2403_CTR2_REG);
  uint8_t CTR1Reg = readRegister(DS2403_CTR1_REG);
  CTR2Reg |= 0b10000000;//set WRTC1=1
  CTR1Reg |= 0b10000100;//set WRTC3=1, WRTC2=1
  Wire.beginTransmission(DS2403_ADDRESS);
  Wire.write(DS2403_CTR2_REG);
  Wire.write(CTR2Reg);
  Wire.endTransmission();

  Wire.beginTransmission(DS2403_ADDRESS);
  Wire.write(DS2403_CTR1_REG);
  Wire.write(CTR1Reg);
  Wire.endTransmission();
}

//SD2403 write disable
void DS2403::WriteTimeOff(void)
{
  uint8_t CTR2Reg = readRegister(DS2403_CTR2_REG);
  uint8_t CTR1Reg = readRegister(DS2403_CTR1_REG);
  CTR2Reg &= 0b01111111;//set WRTC1=0
  CTR1Reg &= 0b01111011;//set WRTC3=0, WRTC2=0
  Wire.beginTransmission(DS2403_ADDRESS);
  Wire.write(DS2403_CTR1_REG);
  Wire.write(CTR1Reg);
  Wire.write(CTR2Reg);//Reg Addr auto plus
  Wire.endTransmission();
}

//write a value to register
void DS2403::writeRegister(uint8_t regaddress, uint8_t value)
{
  WriteTimeOn();
  Wire.beginTransmission(DS2403_ADDRESS);
  Wire.write((byte)regaddress);
  Wire.write((byte)value);
  Wire.endTransmission();
  WriteTimeOff();
}

uint8_t DS2403::begin(void) {
  //disable all interrupts(INTDE,INTAE,INTFE),INT pin high-impedance state
  uint8_t CTR2Reg = 0b00001000;//only FOBAT=1,that means allow INT output at VBAT mode if INT enable
  writeRegister(DS2403_CTR2_REG, CTR2Reg);
  delay(10);
  // set the clock to 24hr format
  uint8_t hrReg = readRegister(DS2403_HOUR_REG);
  hrReg |= 0b10000000;
  writeRegister(DS2403_HOUR_REG, hrReg);
  delay(10);
  return 1;//avoid be same to other begin functions
}

//Adjust the time-date specified in DateTime format
//writing any non-existent time-data may interfere with normal operation of the RTC
void DS2403::adjust(const DateTime& dt) {
  WriteTimeOn();
  Wire.beginTransmission(DS2403_ADDRESS);
  Wire.write((byte)DS2403_SEC_REG);  //begin from SEC Register address,write every register in succession
  Wire.write((byte)bin2bcd(dt.second()));
  Wire.write((byte)bin2bcd(dt.minute()));
  Wire.write((byte)bin2bcd(dt.hour()) | 0b10000000); //Make sure clock is still 24 Hour
  Wire.write((byte)dt.dayOfWeek());
  Wire.write((byte)bin2bcd(dt.date()));
  Wire.write((byte)bin2bcd(dt.month()));
  Wire.write((byte)bin2bcd(dt.year() - 2000));
  Wire.endTransmission();

  Wire.beginTransmission(DS2403_ADDRESS);
  Wire.write(DS2403_MICROADJUST_REG);//It is better to
  Wire.write(byte(0));//clear micro-time adjust register(12H)
  Wire.endTransmission();
  WriteTimeOff();
}

//Read the current time-date and return it in DateTime format
DateTime DS2403::now()
{
  Wire.requestFrom(DS2403_ADDRESS, 7);//read 7 registers in succession
  uint8_t ss = bcd2bin(Wire.read());
  uint8_t mm = bcd2bin(Wire.read());
  uint8_t hrreg = Wire.read();
  uint8_t wd =  Wire.read();
  uint8_t d = bcd2bin(Wire.read());
  uint8_t m = bcd2bin(Wire.read());
  uint16_t y = bcd2bin(Wire.read()) + 2000;
  Wire.endTransmission();
  uint8_t hh = bcd2bin((hrreg & 0b00111111)); //Ignore 24 Hour bit
  return DateTime (y, m, d, hh, mm, ss);
}

//Enable HH/MM/SS interrupt on INT pin once or periodically
void DS2403::enableInterrupts(boolean isRepeat, uint8_t hh24, uint8_t mm, uint8_t ss)
{
  uint8_t CTR2Reg = readRegister(DS2403_CTR2_REG);
  CTR2Reg &= 0b10001101; //bit6(IM),bit5(INTS1),bit4(INTS0),bit1(INTAE) need change
  CTR2Reg |= (isRepeat == true ? 0b01010010 : 0b00010010);//set IM=0(or 1),INTS1=0,INTS0=1,INTAE=1,others not change
  writeRegister(DS2403_CTR2_REG, CTR2Reg);
  writeRegister(DS2403_ALARMENABLE_REG, 0b00000111);//set time alarm enable reg
  writeRegister(DS2403_ALARMSEC_REG,  0b00000000 | bin2bcd(ss) );
  writeRegister(DS2403_ALARMMIN_REG,  0b00000000 | bin2bcd(mm));
  writeRegister(DS2403_ALARMHOUR_REG,  (0b00000000 | (bin2bcd(hh24) | 0b00000000)));
  //writeRegister(DS2403_ALARMWDAY_REG, 0b01111111 ); //everyday needs alarm
}

//Disable Interrupts. This is equivalent to begin() method.
void DS2403::disableInterrupts()
{
  begin(); //Restore to initial value
}

//Clears the interrrupt flag in status register (read & modify & write)
//This is equivalent to preparing the DS2403 INT pin to high for MCU to get ready for recognizing the next INT0 interrupt
void DS2403::clearINTStatus()
{
  uint8_t statusReg = readRegister(DS2403_CTR1_REG);
  statusReg &= 0b11011111;//set INTAF=0
  writeRegister(DS2403_CTR1_REG, statusReg);
}


//Enable periodic interrupt at INT pin.
void DS2403::enableChrime(uint8_t periodicity)
{
  uint8_t CTR2Reg = readRegister(DS2403_CTR2_REG);
  CTR2Reg &= 0b10001101;
  CTR2Reg |= 0b01010010;//set IM=1,INTS1=0,INTS0=1,INTAE=1,others not change
  writeRegister(DS2403_CTR2_REG, CTR2Reg);
  switch (periodicity)
  {
    case EveryMinute:
      writeRegister(DS2403_ALARMENABLE_REG, 0b00000001);
      writeRegister(DS2403_ALARMSEC_REG,  0b00000000 );
      break;

    case EveryHour:
      writeRegister(DS2403_ALARMENABLE_REG, 0b00000011);//set time alarm enable reg
      writeRegister(DS2403_ALARMSEC_REG,  0b00000000 );
      writeRegister(DS2403_ALARMMIN_REG,  0b00000000 );
      break;
  }
}




