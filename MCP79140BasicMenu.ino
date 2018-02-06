/*

  Test target MCP79410 device, such as on a click board.

  This code works on the Gekcreit UNO Compatible board from Banggood.
  This code works on the Duemilanova Compatible board from Seeeduino.
  This code works on the Nano compatible board from Banggood.

  Changes from earlier versions include the addition of a menu-driven user interface through the Serial Monitor

  Arduino uno I2C connections, this will vary on hardware, but should be standard with a click shield and module.
  SCL:  pin A5
  SDA:  pin A4

*/

#include <Wire.h>

// Relay control 
#define RELAY_ON 0
#define RELAY_OFF 1

// Digital IO at the arduino
#define Relay_1 3
#define Relay_2 5
#define Relay_3 11

 
#define RTCC_ADDRESS   0b01101111
#define EEPROM_ADDRESS 0b01010111

//time and configuration registers

#define SECONDS_REGISTER           0x00
#define MINUTES_REGISTER           0x01
#define HOURS_REGISTER             0x02
#define DAY_REGISTER               0x03
#define DATE_REGISTER              0x04
#define MONTH_REGISTER             0x05
#define YEAR_REGISTER              0x06
#define CONTROL_REGISTER           0x07
#define CALIBRATION                0x08
#define EEUNLOCK                   0x09

//alarm 0 Registers

#define ALARM0_SECONDS_REGISTER           0x0A
#define ALARM0_MINUTES_REGISTER           0x0B
#define ALARM0_HOURS_REGISTER             0x0D
#define ALARM0_DAY_REGISTER               0x0D
#define ALARM0_DATE_REGISTER              0x0E
#define ALARM0_MONTH_REGISTER             0x0F

//alarm 1 Registers

#define ALARM1_SECONDS_REGISTER           0x11
#define ALARM1_MINUTES_REGISTER           0x12
#define ALARM1_HOURS_REGISTER             0x13
#define ALARM1_DAY_REGISTER               0x14
#define ALARM1_DATE_REGISTER              0x15
#define ALARM1_MONTH_REGISTER             0x16


unsigned char txtSec;
unsigned char txtMin;
unsigned char txtHour;
unsigned char second;
unsigned char minute;
unsigned char hour;
unsigned char day;
unsigned char date;
unsigned char month;
unsigned char year;
unsigned char ctrl_byte;
unsigned char cal_byte;
unsigned char st_bit;
unsigned char oscon_bit;
unsigned char vbat_bit;
unsigned char vbat_en;
unsigned char hr_24;
unsigned char pm_bit;
unsigned char leap_bit;
unsigned char calsign;
unsigned char out_bit;
unsigned char extosc_bit;
unsigned char alm0_ena;
unsigned char alm1_ena;
unsigned char alm0_flag;
unsigned char alm1_flag;

unsigned char data, i;

const char *day_name[8] = { "Null Day Value", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
const char *month_name[13] = { "Null Month", "January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

unsigned char read_array[24];

// RTC6 click module connections
const int trig_outPin = 9;
const int mfpPin = 2; //RTCC MFP Pin is brought into Arduino on digital pin 2

uint8_t osc_running = 0;
uint8_t prev_hsec = 0;
uint8_t sec = 0;
uint8_t hsec = 0, temp;

void setup() {


  // initialize the serial communication:
  Serial.begin(115200);
  pinMode(trig_outPin, OUTPUT);
  pinMode(mfpPin, INPUT);
  


  // initialize I2C:
  Wire.begin();
  Wire.setClock(200000L);//optional, default is 100kHz

  printMenu();

  set_st();//Start_RTC();

  // Initialize the relays
  //-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(Relay_1, RELAY_OFF);
  digitalWrite(Relay_2, RELAY_OFF);
  digitalWrite(Relay_3, RELAY_OFF);
  
  //---( THEN set pins as outputs )----  
  pinMode(Relay_1, OUTPUT);   
  pinMode(Relay_2, OUTPUT);  
  pinMode(Relay_3, OUTPUT);  

  delay(500); //Check that all relays are inactive at Reset
}

void loop()
{

  if (Serial.available())
  {
    process_input();
  }

}

void clr_st()
{

  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(SECONDS_REGISTER);//register where the ST bit resides
  Wire.write(0x00);//clear entire register
  Wire.endTransmission();

}

void set_st()
{
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(SECONDS_REGISTER);//register where the ST bit resides
  Wire.write(0x80);//set ST in seconds register
  Wire.endTransmission();
}

uint8_t rd_sec()
{
  uint8_t r_val;

  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(SECONDS_REGISTER);//register where the ST bit resides
  Wire.endTransmission();//send STOP
  Wire.requestFrom(RTCC_ADDRESS, 1); //request one byte from RTCC/SRAM device
  while (!Wire.available()); //wait until first byte arrives
  r_val = Wire.read();

  return (r_val);
}

uint8_t rd_oscon()
{
  uint8_t r_val;

  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(DAY_REGISTER);//register where the OSCON bit resides
  Wire.endTransmission();//send STOP
  Wire.requestFrom(RTCC_ADDRESS, 1); //request one byte from RTCC/SRAM device
  while (!Wire.available()); //wait until first byte arrives
  r_val = Wire.read();

  return (r_val & 0x20);
}


void printMenu(void)
{
  Serial.print(F("\r\nMenu:\r\n"));
  Serial.print("------------------------------------\r\n");
  Serial.print(F("[r] Read and Display RTC Values\r\n"));
  Serial.print(F("[t] Set Time\r\n"));
  Serial.print(F("[d] Set Day\r\n"));
  Serial.print(F("[D] Set Date\r\n"));
  Serial.print(F("[p] Set AM/PM bit\r\n"));
  Serial.print(F("[P] Set 12/24 bit\r\n"));
  Serial.print(F("[M] Set Month\r\n"));
  Serial.print(F("[y] Set Year\r\n"));
  Serial.print(F("[s] Start Clock\r\n"));
  Serial.print(F("[S] Stop Clock\r\n"));
  Serial.print(F("[b] Enable Vbat\r\n"));
  Serial.print(F("[B] Disable Vbat\r\n"));
  Serial.print(F("[O] Turn OFF LEDs\r\n"));
  Serial.print(F("[o] Turn ON LEDs\r\n"));
  Serial.print(F("[x] Set EXTOSC to 1\r\n"));
  Serial.print(F("[X] Set EXTOSC to 0\r\n"));
  Serial.print(F("[a] Disable Alarms\r\n"));
  Serial.print(F("[A] Enable Alarms\r\n"));
  Serial.print(F("[u] Enable OUTput pin at 32kHz and clear CAL byte\r\n"));
  Serial.print(F("[C] Clear ALL Time Registers\r\n"));
  Serial.print(F("[m] Print this Menu\r\n"));
  Serial.print(F("[L] Check lumex from sensor\r\n"));
  Serial.print(F("[Z] Start experiment time:\r\n"));
  Serial.print(F("[z] End experiment time:\r\n"));
  Serial.print(F("[w] Lumex sampling frequency x sec:\r\n"));  
  Serial.print(F("Selection: "));
}



void process_input(void)
{
  uint8_t input;
  input = Serial.read();
  while (Serial.available()) {
    data = Serial.read();
    delay(15);  
  }
  switch (input)
  {
    case 'r':
      Serial.println(F("r"));
      ReadTime();
      FormatTime();
      Serial.print(F("\r\nTime is:\t\t\t"));
      print_time(hour, minute, second);


      if (hr_24)
      {
        if (pm_bit) Serial.println(F("PM"));
        else Serial.println(F("AM"));
      }
      else Serial.println(" ");
      Serial.print(F("Day is (1-7):\t\t\t"));
      Serial.println(day_name[day]);

      Serial.print(F("Calendar mm/dd/yy is:\t\t"));

      print_date(month, date, year);
      Serial.println(" ");
      Serial.print(F("Control register is:\t\t0x"));
      if (ctrl_byte < 0x10) leading_zero();
      Serial.println(ctrl_byte, HEX);
      Serial.print(F("Calibration register is:\t0x"));
      if (cal_byte < 0x10) leading_zero();
      Serial.println(cal_byte, HEX);
      Serial.print(F("Start bit is\t\t\t"));
      Serial.println(st_bit, HEX);
      Serial.print(F("Osc ON bit is\t\t\t"));
      Serial.println(oscon_bit, HEX);
      if (hr_24)
      {
        Serial.print(F("AM(0)/PM(1) bit is\t\t"));
        Serial.println(pm_bit, HEX);
      }
      else Serial.println(F("AM(0)/PM(1) bit is\t\tN/A"));
      Serial.print(F("12(1)/24(0) Hour bit is\t\t"));
      Serial.println( hr_24, HEX);
      Serial.print(F("Vbat EN bit is\t\t\t"));
      Serial.println(vbat_en, HEX);
      Serial.print(F("Vbat (Event) bit is\t\t"));
      Serial.println(vbat_bit, HEX);
      Serial.print(F("Leap Year bit is\t\t"));
      Serial.println(leap_bit, HEX);
      Serial.print(F("OUT bit is\t\t\t"));
      Serial.println(out_bit, HEX);
      Serial.print(F("EXTOSC bit is\t\t\t"));
      Serial.println(extosc_bit, HEX);
      Serial.print(F("Alarm0 Enable is\t\t"));
      Serial.println(alm0_ena, HEX);
      Serial.print(F("Alarm0 Flag is\t\t\t"));
      Serial.println(alm0_flag, HEX);
      Serial.print(F("Alarm1 Enable is\t\t"));
      Serial.println(alm1_ena, HEX);
      Serial.print(F("Alarm1 Flag is\t\t\t"));
      Serial.println(alm1_flag, HEX);
      break;
    case 't':

      Serial.println("t");
      ReadTime();
      FormatTime();
      Serial.print(F("Time is: "));
      print_time(hour, minute, second);
      Serial.print(F("\r\nEnter new Hour in nn format: "));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      //input_string = Serial.readString(); // Read data byte received from external terminal
      //data = input_string[0];
      data -= '0';
      //data = input_string[1];
      hour = data << 4;
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      hour |= data;
      if (hour < 0x0A) leading_zero();
      Serial.println( hour, HEX);
      while (Serial.available()) data = Serial.read();
      Serial.print(F("\r\nEnter new Minute in nn format: "));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal

      data -= '0';
      minute = data << 4;

      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      //data = input_string[1];
      data -= '0';
      minute |= data;
      if (minute < 0xA0) leading_zero();
      while (Serial.available()) data = Serial.read();
      Serial.println(minute, HEX);
      Serial.print(F("\r\nEnter new Second in nn format: "));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      second = data << 4;
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal

      data -= '0';
      second |= data;
      if (second < 0x0A) leading_zero();
      Serial.println(second, HEX);
      while (Serial.available()) data = Serial.read();

      SetTime();
      break;


    case 'd':
      Serial.println("d");
      ReadTime();
      FormatTime();
      Serial.print("Day is:                  ");
      Serial.print(day, HEX);
      Serial.print("  ");
      Serial.println(day_name[day]);
      Serial.print(F("\r\nEnter new Day in n format:"));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      while (Serial.available()) data = Serial.read();
      data -= '0';
      day = data;
      //set AM/PM bit
      WriteDay();
      Serial.print(F("\r\nDay is now: "));
      Serial.println(day, HEX);

      break;
    case 'D':
      Serial.println("D");
      ReadTime();
      FormatTime();
      Serial.print(F("Date is:  "));
      Serial.println(date, HEX);
      Serial.print(F("\r\nEnter new Date in nn format:"));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      date = data << 4;
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      date |= data;
      //set AM/PM bit
      WriteDate();
      Serial.print(F("\r\nDate is now:  "));
      Serial.println(date, HEX);

      break;
    case 'p':
      Serial.println("p");
      ReadTime();
      FormatTime();
      Serial.print(F("AM/PM bit is  "));
      Serial.println( pm_bit, HEX);

      Serial.print(F("Enter new AM/PM bit as (0 or 1):"));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      pm_bit = data;
      //set AM/PM bit
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

      Wire.write(HOURS_REGISTER);
      Wire.write((read_array[3] & (~0x20)) | (pm_bit << 5));
      Wire.endTransmission();
      Serial.print("AM/PM bit is now   ");
      Serial.println( pm_bit, HEX);

      break;

    case 'P':

      Serial.println("P");
      ReadTime();
      FormatTime();
      Serial.print(F("12/24 Hour bit is   "));
      Serial.println(hr_24, HEX);

      Serial.print(F("Enter new 12/24 bit as (12(1) or 24(0)):"));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      hr_24 = data;
      //set 12/24 Hour bit
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

      Wire.write(HOURS_REGISTER);
      Wire.write((read_array[3] & (~0x40)) | (hr_24 << 6));
      Wire.endTransmission();
      Serial.print(F("\r\n12/24 Hour bit is now  "));
      Serial.println(hr_24, HEX);
      break;

    case 'M':

      Serial.println("M");
      ReadTime();
      FormatTime();
      Serial.print(F("Month is: "));
      Serial.print(month);
      Serial.print(" ");
      Serial.println(month_name[month]);

      Serial.print(F("Enter new Month in nn format:"));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      month = data << 4;
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      month |= data;
      //set Leap Year bit
      WriteMonth();
      Serial.print(F("\r\nMonth is now: "));
      Serial.print(month);
      Serial.print(" ");
      Serial.println(month_name[month]);
      break;

    case 'm':
      printMenu();
      break;

    case 'y':
      Serial.println("y");
      ReadTime();
      FormatTime();
      Serial.print(F("Year is: "));
      Serial.println(year, HEX);

      Serial.print(F("Enter new Year in nn format:"));
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      year = data << 4;
      while (!Serial.available()); //pause here until receiving a command
      data = Serial.read(); // Read data byte received from external terminal
      data -= '0';
      year |= data;
      //set Year value
      WriteYear();
      Serial.print(F("\r\nYear is now: "));
      Serial.println(year, HEX);
      break;

    case 's':

      Serial.println("s");
      Serial.println(F("Start RTC"));
      Start_RTC();
      Serial.println(F("RTC Start Bit is Set"));
      break;

    case 'S':

      Serial.println("S");
      Serial.println(F("Stop RTC"));
      Stop_RTC();
      Serial.println(F("RTC Start Bit is Cleared"));
      break;
      

    case 'b':

      Serial.println("b");
      ReadTime();
      FormatTime();
      Serial.print(F("Vbat EN bit is "));
      Serial.println(vbat_en, HEX);
      Serial.println(F("Enabling Vbat EN bit "));

      //set VBATEN bit
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
      Wire.write(DAY_REGISTER);
      Wire.write(read_array[4] | 0x08);
      Wire.endTransmission();
      ReadTime();
      FormatTime();
      Serial.print(F("Vbat EN bit is now "));
      Serial.println(vbat_en, HEX);
      break;

    case 'B':
      Serial.println("B");
      ReadTime();
      FormatTime();
      Serial.print(F("Vbat EN bit is "));
      Serial.println(vbat_en, HEX);
      Serial.println(F("Disabling Vbat EN bit"));

      //clear VBATEN bit
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
      Wire.write(DAY_REGISTER);
      Wire.write(read_array[4] & (~0x08));
      Wire.endTransmission();
      ReadTime();
      FormatTime();
      Serial.print(F("Vbat EN bit is now "));
      Serial.println(vbat_en, HEX);
      break;
    
    // Turn OFF LEDs
    case 'O':

      Serial.println("O");
      Serial.println(F("Turning OFF LEDs!"));

      digitalWrite(Relay_1, RELAY_OFF);// set the Relay ON
      digitalWrite(Relay_2, RELAY_OFF);// set the Relay ON
      digitalWrite(Relay_3, RELAY_OFF);// set the Relay ON

      //Enable OUT pin to HIGH
      // Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
      // Wire.write(CONTROL_REGISTER);
      //Wire.write(0x80);
      // Wire.endTransmission();
  
      Serial.println(F("LEDs OFF!"));
      break;

    // Turn ON LEDs
    case 'o':

      Serial.println("o");
      Serial.println(F("Turning ON LEDs!"));

      digitalWrite(Relay_1, RELAY_ON);// set the Relay ON
      digitalWrite(Relay_2, RELAY_ON);// set the Relay ON
      digitalWrite(Relay_3, RELAY_ON);// set the Relay ON

      //Enable OUT pin to LOW
      // Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
      // Wire.write(CONTROL_REGISTER);
      // Wire.write(0x00);
      // Wire.endTransmission();
      
      Serial.println(F("LEDs ON!"));
      break;

    case 'a':

      Serial.println("a");
      Serial.println(F("Disable Alarms"));

      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
      Wire.write(CONTROL_REGISTER);
      Wire.write(0x00);
      Wire.endTransmission();

      delayMicroseconds(1);

      //Enable Alarm0 for 1 sec, Alarm1 for 1 min.
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
      Wire.write(ALARM0_SECONDS_REGISTER);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.endTransmission();
      Serial.println(F("Alarms Disabled"));
      break;

    case 'A':

      Serial.println("A");
      Serial.println(F("Enable Alarms"));

      //Enable both Alarms
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
      Wire.write(ALARM0_SECONDS_REGISTER);
      Wire.write(0x00);//Sec
      Wire.write(0x01);//min
      Wire.write(0x00);//hours
      Wire.write(0x00);//alm mask, wk day
      Wire.write(0x00);//date
      Wire.write(0x00);//month
      Wire.write(0x00);//hunsec
      Wire.write(0x50);//sec
      Wire.write(0x00);//min
      Wire.write(0x00);//hour
      Wire.write(0x00);//alm mask, wk day
      Wire.write(0x00);//date
      Wire.endTransmission();

      delayMicroseconds(1);

      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

      Wire.write(CONTROL_REGISTER);
      Wire.write(0x30);//10 = alm0, 20 = alm1, 30 = both alm's
      Wire.endTransmission();
      Serial.println(F("Alarms Enabled"));
      break;

    case 'u':

      Serial.println("u");
      Serial.println(F("Enabling OUT put pin at 32kHz"));

      //Enable OUT pin at 32kHz
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

      Wire.write(CONTROL_REGISTER);
      Wire.write(0x43);
      Wire.endTransmission();
      Serial.println(F("OUTput pin Enabled at 32kHz"));
      break;

    case 'x':
      Serial.println("x");
      Serial.println(F("Enabling EXTOSC input"));

      //Enable extosc input
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

      Wire.write(CONTROL_REGISTER);
      Wire.write(read_array[8] | 0x08);
      Wire.endTransmission();
      Serial.println(F("EXTOSC input Enabled"));
      break;

    case 'X':
      Serial.println("X");
      Serial.println(F("Disable EXTOSC input"));

      //Enable extosc input
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

      Wire.write(CONTROL_REGISTER);
      Wire.write(read_array[8] & ~0x08);
      Wire.endTransmission();
      Serial.println(F("EXTOSC input Disabled"));
      break;

    case 'C':
      Serial.println("C");
      Serial.println(F("Clear ALL Time Registers"));
      Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

      Wire.write(SECONDS_REGISTER);
      for (i = 0; i < 31; i++)
      {
        Wire.write(0x00);
      }
      Wire.endTransmission();
      break;
    default:
      Serial.println("?");
      break;
  }
  while (!Serial.available());
}


unsigned char bcd2dec(unsigned char bcd)
{
  unsigned char dec;
  dec =  bcd - 6 * (bcd >> 4);
  return dec;
}

unsigned char byte2str(unsigned char inbyte)
{
  unsigned char temp;
  temp = inbyte + '0';
  return temp;
}

unsigned char dec2bcd(unsigned char dec)
{
  unsigned char bcd;
  bcd = dec + 6 * (dec / 10);
  return bcd;
}

void Start_RTC(void)
{
  //buffer seconds register
  unsigned char temp;
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(SECONDS_REGISTER);//register where the OSCON bit resides
  Wire.endTransmission();//send STOP
  Wire.requestFrom(RTCC_ADDRESS, 1); //request 1 bytes from RTCC/SRAM device
  while (1 > Wire.available()); //wait until all bytes are buffered
  temp = Wire.read();// Read seconds

  //Wire.endTransmission();
  delayMicroseconds(1);

  //set ST bit
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(SECONDS_REGISTER);
  Wire.write(temp | 0x80);
  Wire.endTransmission();
  delayMicroseconds(1);

  //check if oscillator is running
  //while (!rd_oscon());
}


void Stop_RTC(void)
{
  //buffer seconds register
  unsigned char temp;
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(SECONDS_REGISTER);//register where the OSCON bit resides
  Wire.endTransmission();//send STOP
  Wire.requestFrom(RTCC_ADDRESS, 1); //request 1 bytes from RTCC/SRAM device
  while (1 > Wire.available()); //wait until all bytes are buffered
  temp = Wire.read();// Read seconds
  delayMicroseconds(1);

  //clear ST bit
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(SECONDS_REGISTER);
  Wire.write(temp & (~0x80));
  Wire.endTransmission();
  delayMicroseconds(1);

  //check if oscillator is running
  //while (rd_oscon());
}

void WriteSecond(void)
{
  read_array[1] = (second & 0x7f) | (st_bit << 7);
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(SECONDS_REGISTER);
  Wire.write(read_array[1]);
  Wire.endTransmission();
}

void WriteMinute(void)
{
  read_array[2] =  minute & 0x7f;
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(MINUTES_REGISTER);
  Wire.write(read_array[2]);
  Wire.endTransmission();
}

void WriteHour(void)
{
  if (hr_24) read_array[3] =  (hour & 0x1f) | (pm_bit << 5) | (hr_24 << 6) | (calsign << 7); //format in 12 hour mode
  else read_array[3] =  (hour & 0x3f) | (hr_24 << 6) | (calsign << 7);//format in 24 hour mode
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(HOURS_REGISTER);
  Wire.write(read_array[3]);
  Wire.endTransmission();

}

void WriteDay(void)
{
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(DAY_REGISTER);
  Wire.write((read_array[4] & (~0x07)) |  day);
  Wire.endTransmission();
}

void WriteDate(void)
{
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(DATE_REGISTER);
  Wire.write(date & 0x3F);
  Wire.endTransmission();
}

void WriteMonth(void)
{
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(MONTH_REGISTER);
  Wire.write((read_array[6] & (~0x1F)) | (month & 0x1F));
  Wire.endTransmission();
}

void WriteYear(void)
{
  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(YEAR_REGISTER);
  Wire.write(year);
  Wire.endTransmission();
}


void SetTime() {
  // Set desired time
  read_array[1] = (second & 0x7f) | (st_bit << 7);
  read_array[2] =  minute & 0x7f;
  if (hr_24) read_array[3] =  (hour & 0x1f) | (pm_bit << 5) | (hr_24 << 6) | (calsign << 7); //format in 12 hour mode
  else read_array[3] =  (hour & 0x3f) | (hr_24 << 6) | (calsign << 7);//format in 24 hour mode

  //read_array[4] = 0x00;
  read_array[5] = date;
  read_array[6] = month;
  read_array[7] = year;

  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(SECONDS_REGISTER);
  Wire.write(read_array[1]);
  Wire.write(read_array[2]);
  Wire.write(read_array[3]);
  Wire.endTransmission();

  delayMicroseconds(1);

  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address

  Wire.write(DATE_REGISTER);
  Wire.write(read_array[5]);
  Wire.write(read_array[6]);
  Wire.write(read_array[7]);
  Wire.endTransmission();
}

// Read time from the RTC
void ReadTime() {

  Wire.beginTransmission(RTCC_ADDRESS);//RTCC/SRAM address
  Wire.write(SECONDS_REGISTER);//register where the OSCON bit resides
  Wire.endTransmission();//send STOP
  Wire.requestFrom(RTCC_ADDRESS, 24); //request 24 bytes from RTCC/SRAM device, then send a NACK and a STOP at the end.
  while (24 > Wire.available()); //wait until all bytes are buffered
  read_array[0] = Wire.read();// Read seconds
  read_array[1] = Wire.read();
  read_array[2] = Wire.read();
  read_array[3] = Wire.read();
  read_array[4] = Wire.read();
  read_array[5] = Wire.read();
  read_array[6] = Wire.read();
  read_array[7] = Wire.read();
  read_array[8] = Wire.read();
  read_array[9] = Wire.read();
  read_array[10] = Wire.read();
  read_array[11] = Wire.read();
  read_array[12] = Wire.read();
  read_array[13] = Wire.read();
  read_array[14] = Wire.read();
  read_array[15] = Wire.read();
  read_array[16] = Wire.read();
  read_array[17] = Wire.read();
  read_array[18] = Wire.read();
  read_array[19] = Wire.read();
  read_array[20] = Wire.read();
  read_array[21] = Wire.read();
  read_array[22] = Wire.read();
  read_array[23] = Wire.read();

}

// Format time

void FormatTime() {
  // Format seconds, minutes and hours
  second = read_array[0] & 0x7F;
  minute = read_array[1] & 0x7F;
  hr_24 = (read_array[2] & 0x40) >> 6;
  if (hr_24) hour = read_array[2] & 0x1F;
  else hour = read_array[2] & 0x3F;
  day = read_array[3] & 0x07;
  date = read_array[4] & 0x3f;
  month = read_array[5] & 0x1f;
  year = read_array[6] & 0xff;
  ctrl_byte = read_array[7] & 0xff;
  cal_byte = read_array[8] & 0xff;
  st_bit = (read_array[0] & 0x80) >> 7;
  oscon_bit = (read_array[3] & 0x20) >> 5;
  vbat_bit = (read_array[3] & 0x10) >> 4;
  vbat_en = (read_array[3] & 0x08) >> 3;
  pm_bit = (read_array[2] & 0x20) >> 5;
  leap_bit = (read_array[5] & 0x20) >> 5;
  calsign = (read_array[2] & 0x80) >> 7;
  out_bit = (read_array[7] & 0x80) >> 7;
  extosc_bit = (read_array[7] & 0x08) >> 3;
  alm0_ena = (read_array[7] & 0x10) >> 4;
  alm0_flag = (read_array[14] & 0x08) >> 3;
  alm1_ena = (read_array[7] & 0x20) >> 5;
  alm1_flag = (read_array[21] & 0x08) >> 3;
}



void print_date(uint8_t monthv, uint8_t datev, uint8_t yearv)
{
  if (monthv < 0x10 )leading_zero();
  Serial.print(monthv, HEX);
  Serial.print("/");
  if (datev < 0x10 )leading_zero();
  Serial.print(datev, HEX);
  Serial.print("/");
  if (yearv < 0x10 )leading_zero();
  Serial.print(yearv, HEX);
}

void print_time(uint8_t hourv, uint8_t minutev, uint8_t secondv)
{
  if (hourv < 0x10 )leading_zero();
  Serial.print(hourv, HEX);
  Serial.print(":");
  if (minutev < 0x10 )leading_zero();
  Serial.print(minutev, HEX);
  Serial.print(":");
  if (secondv < 0x10 )leading_zero();
  Serial.print(secondv, HEX);
}

void print_date_time(uint8_t monthv, uint8_t datev, uint8_t yearv, uint8_t hourv, uint8_t minutev, uint8_t secondv)
{
  print_date(monthv, datev, yearv);
  Serial.print(" ");
  print_time(hourv, minutev, secondv);
}

void leading_zero()
{
  Serial.print(F("0"));
} 
