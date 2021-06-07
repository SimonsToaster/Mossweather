//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//Libraries

#include <avr/sleep.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ArduCAM.h>
#include "memorysaver.h"
extern "C"
{
#include "utility/twi.h"               // from Wire library, so we can do bus scanning
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//Parameters - Control how often data is logged, sampling time, when to take a picture etc

const int log_intervall = 60;          //Time in minutes between logging. Needs to be lower than 240 (4 hours)
const int log_time = 60;               //Time in seconds for in which the measured quantity should be averaged. Every 10 seconds a measurement is taken. Needs to be between 10 and 300 (5 min)
const int pic_time = 12;               //Hour of the day when the picture is taken. Needs to be between 1 and 24. 
char filename[] = "LOGFILE.CSV";       //Name of the logfile

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//Defined Pins

const int timerInterruptPin = 18;     //Pin at which the timer of the RTC will send its interrupt
const int SD_CS = 10;                 //I think this defines the pin for selecting the SD card for SPI
const int rst_rain = 2;               //Pins for resetting the rain and wind counters, need to be adjusted.
const int rst_wind = 4;
const int SPI_CS = 53;                 //I think this defines the pinfor seleecting the camera for SPI

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//Settings for the clock, sensors etc.

RTC_PCF8523 rtc;

File logfile;

Adafruit_BMP280 bmp = Adafruit_BMP280();

bool enableHeater = false;
Adafruit_SHT31 sht31_grnd = Adafruit_SHT31();
Adafruit_SHT31 sht31_surf = Adafruit_SHT31();
Adafruit_SHT31 sht31_air = Adafruit_SHT31();

Adafruit_TSL2591 tsl_up = Adafruit_TSL2591(0x40);
Adafruit_TSL2591 tsl_dwn = Adafruit_TSL2591(0x50);

ArduCAM myCAM( OV5642, SPI_CS );

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//Declared Functions  

//Interrupt service routine
volatile bool timer_set = false;

void wakeUp()
{
  sleep_disable(); 
  timer_set = false;
}

//Sleep routine
void Going_To_Sleep()
{
  sleep_enable();                                                             //Enabling sleep mode
  attachInterrupt(digitalPinToInterrupt(timerInterruptPin), wakeUp, FALLING); //attaching a interrupt to pin d2
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                                        //Setting the sleep mode, in our case full sleep
  delay(1000);                                                                //wait a second to allow the led to be turned off before going to sleep
  sleep_cpu();                                                                //activating sleep mode
  detachInterrupt(digitalPinToInterrupt(timerInterruptPin));
}

//Math functions
long a_average(long array[])
{
  long sum_av = 0;
  for (int i_av = 0; i_av < 10; i_av++)
  {
    sum_av = sum_av + array[i_av];
  }
  long a_av = sum_av / 10;
  return a_av;
}

double a_std(long array[])
{
  long sum = 0;
  for (int i_std1 = 0; i_std1 < 10; i_std1++)
  {
    sum = sum + array[i_std1];
  }
  long std_av = sum / 10;

  long sum_sq = 0;
  for (int i_std2 = 0; i_std2 < 10; i_std2++)
  {
    sum_sq = sum_sq + (sq(array[i_std2] - std_av));
  }
  double std = sqrt(1/9.0 * sum_sq);
  return std;
}

//ArduCAM takes a picture and saves it to the SD card
void myCAMSaveToSDFile()
{
  char str[8];
  byte buf[256];
  static int i = 0;
  static int k = 0;
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  bool is_header = false;
  File outFile;
  myCAM.clear_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);                     //Gets the camera out of sleep mode
  myCAM.flush_fifo();                                                 //Flush the FIFO
  myCAM.clear_fifo_flag();                                            //Clear the capture done flag
  
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start Capture"));
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  Serial.println(F("Capture Done."));
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //384K
  {
    Serial.println(F("Over size."));
    return;
  }
  if (length == 0) //0 kb
  {
    Serial.println(F("Size is 0."));
    return;
  }
  //Construct a file name
  long pictime = rtc.now().unixtime();

  // k = k + 1;
  itoa(pictime, str, 10);
  strcat(str, ".jpg");
  //Open the new file
  outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
  if (!outFile)
  {
    Serial.println(F("File open faild"));
    return;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while (length--)
  {
    temp_last = temp;
    temp = SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ((temp == 0xD9) && (temp_last == 0xFF)) //If find the end ,break while,
    {
      buf[i++] = temp; //save the last  0XD9
      

      //Close the file
      outFile.close();
      Serial.println(F("Image save OK."));
      is_header = false;
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
  outFile.close();
  myCAM.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);             //Sends Camera back to sleep
}

//Allows easy swiching between ports of the multiplexer
#define TCAADDR 0x70
void tcaselect(uint8_t i)
{
  if (i > 7)
    return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//Configures radiation sensors
void configureSensor(void)
{
  tsl_up.setGain(TSL2591_GAIN_MED);                // You can change the gain on the fly, to adapt to brighter/dimmer light situations 1x gain (bright light), 25x gain, 428x gain
  tsl_up.setTiming(TSL2591_INTEGRATIONTIME_300MS); // Changing the integration time gives you a longer time over which to sense light longer timelines are slower, but are good in very low light situtations! Possible: 100, 200, ..., 600.
  tsl_dwn.setGain(TSL2591_GAIN_MED);
  tsl_dwn.setTiming(TSL2591_INTEGRATIONTIME_300MS);
}


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//Setup function

void setup()
{

  //Initializing the RTC
  Serial.begin(115200);
  delay(100);

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (!rtc.initialized() || rtc.lostPower())
  {
    Serial.println("RTC not initialized, setting time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));       //When time needs to be set on a new device, or after a power loss, this line sets the RTC to the date & time this sketch was compiled
  }

  rtc.start();                                            //When the RTC was stopped and stays connected to the battery, it has to be restarted by clearing the STOP bit. Let's do this to ensure the RTC is running.

  pinMode(timerInterruptPin, INPUT_PULLUP);               //Set the pin attached to PCF8523 INT to be an input with pullup to HIGH. The PCF8523 interrupt pin will briefly pull it LOW at the end of a given countdown period, then it will be released to be pulled HIGH again.
  rtc.deconfigureAllTimers();
  rtc.enableCountdownTimer(PCF8523_FrequencyMinute, log_intervall);

  timer_set = true;

  //Initializing SD card
  SD.begin(SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD card not found");
    while (1);
  }
  
  Serial.println("card initialized");

  //creating logfile
  logfile = SD.open(filename, FILE_WRITE);

  if (!logfile)
  {
    Serial.println("couldnt create file");
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  logfile.println("Timestamp, Date/Time, Pressure[hPa], Std, TemperatureGround[°C], Std, TemperatureSurface[°C], Std, TemperatureAir[°C], Std, HumidityGround[%RH], Std, HumiditySurface[%RH], Std, HumidityAir[%RH], Std, RadiationUpwards[counts], Std, RadiationDownwards[counts], Std, PrecipitationLastHour[mm], WindVelocity,"); //Actually used header neds to be inserted
  Serial.println("Timestamp, Date/Time, Pressure[hPa], Std, TemperatureGround[°C], Std, TemperatureSurface[°C], Std, TemperatureAir[°C], Std, HumidityGround[%RH], Std, HumiditySurface[%RH], Std, HumidityAir[%RH], Std, RadiationUpwards[counts], Std, RadiationDownwards[counts], Std, PrecipitationLastHour[mm], WindVelocity,");

  logfile.close();

  //Scans the multiplexer to check if all Sensors are connected
  Wire.begin();
  Serial.println("\nTCAScanner ready!");

  for (uint8_t t = 0; t < 8; t++)
  {
    tcaselect(t);
    Serial.print("TCA Port #");
    Serial.println(t);

    for (uint8_t addr = 0; addr <= 127; addr++)
    {
      if (addr == TCAADDR || addr == 0x68 || addr == 0x3C)
        continue;

      uint8_t data;
      if (!twi_writeTo(addr, &data, 0, 1, 1))
      {
        Serial.print("Found I2C 0x");
        Serial.println(addr, HEX);
      }
    }
  }
  Serial.println("\ndone");

  //Initializing the pressure sensor
  tcaselect(0);
  Serial.println("tcaselect(0)");
  if (!bmp.begin())
  {
    Serial.println("Pressure sensor not detected on TCA line 0");
    while (1);
  }
  Serial.println("check passed");
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_NONE, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
    /* Operating Mode. _NORMAL, _FORCED, _SLEEP exist. Forced mode wakes sensor up, performs a measurement, then lets the sensor go back to sleep. Forced mode thus must be enabled before each measurement. Temp. oversampling set to _NONE since temperature isnt measured with this sensor. Pressure oversampling set to maximum to achieve lowest noise possible. Filtering. Set to max. Standby time. */
  Serial.println("BMP configured");

  //Initializing the Temperature and Humidity sensors
  tcaselect(1);
  Serial.println("tcaselect(1)");
  if (!sht31_grnd.begin())
  {
    Serial.println("Temperature/Humidity sensor not detected on TCA line 1");
    while (1);
  }
  Serial.println("check passed");

  tcaselect(2);
  Serial.println("tcaselect(2)");
  if (!sht31_surf.begin())
  {
    Serial.println("Temperature/Humidity sensor not detected on TCA line 2");
    while (1);
  }
  Serial.println("check passed");

  tcaselect(3);
    Serial.println("tcaselect(3)");
  if (!sht31_air.begin())
  {
    Serial.println("Temperature/Humidity sensor not detected on TCA line 3");
    while (1);
  }
  Serial.println("check passed");

  //Initializing the upward and downward radiation sensors
  tcaselect(4);
  Serial.println("tcaselect(4)");
  if (!tsl_up.begin())
  {
    Serial.println("Upwards radiation sensor not detected on TCA line 4");
    while (1);
  }
  Serial.println("check passed");
  configureSensor();
  Serial.println("tsl_up configured");

  tcaselect(5);
  Serial.println("tcaselect(5)");
  if (!tsl_dwn.begin())
  {
    Serial.println("Downwards radiation sensor not detected on TCA line 5");
    while (1);
  }
  Serial.println("check passed");
  configureSensor();
  Serial.println("tsl_down configured");

  //Checks if entered logging parameters are valid

  if (log_intervall > 240 || log_intervall < 1)
  {
    Serial.println("Time between logging events needs to be more than 1 Minute and less than 240 Minutes");
    while(1);
  }
  Serial.println("Logging intervall valid");
  if (log_time < 10 || log_time > 300)
  {
    Serial.println("Logging averaging time needs to be above 10 seconds and below 300 seconds");
    while(1);
  }
  Serial.println("Measuring time valid");
  if (pic_time < 1 || pic_time > 24)
  {
    Serial.println("Time at which the picture will be taken needs to be between 1 and 24");
    while(1);
  }
  Serial.println("Picture time valid");
  Serial.println("Logging parameters valid");

  //Activating counter for rain
  pinMode(rst_rain, INPUT);
  digitalWrite(rst_rain, HIGH);                           //How do pull ups behave when powerd down?

  //Activating counter for wind
  pinMode(rst_wind, INPUT);
  digitalWrite(rst_wind, HIGH);

  //Code for Arducam

  uint8_t vid, pid;
  uint8_t temp;
  Wire.begin();
  Serial.println(F("ArduCAM Start!"));

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);                             //set the CS as an output:
  SPI.begin();                                            // initialize SPI:
  myCAM.clear_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);         //Gets camera out of sleep mode

  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  while (1)
  {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);

    if (temp != 0x55)
    {
      Serial.println(F("SPI interface Error!"));
      delay(1000);
      continue;
    }
    else
    {
      Serial.println(F("SPI interface OK."));
      break;
    }
  }
  //Initialize SD Card
  SD.begin(SD_CS);
  while (!SD.begin(SD_CS))
  {
    Serial.println(F("SD Card Error!"));                  //Is this code here really needed?
    delay(1000);
  }
  Serial.println(F("SD Card detected."));

  while (1)
  {
    //Check if the camera module type is OV5642
    myCAM.wrSensorReg16_8(0xff, 0x01);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x42))
    {
      Serial.println(F("Can't find OV5642 module!"));
      delay(1000);
      continue;
    }
    else
    {
      Serial.println(F("OV5642 detected."));
      break;
    }
  }

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);      //VSYNC is active HIGH
  myCAM.OV5642_set_JPEG_size(OV5642_2592x1944);         //Resolution, possible OV5642_320x240 OV5642_640x480 OV5642_1024x768 OV5642_1280x960 OV5642_1600x1200 OV5642_2048x1536 OV5642_2592x1944 OV5642_1920x1080
  delay(1000);

  myCAM.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK);         //Sends camera back to sleep
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
//Loop function

void loop()
{
  delay(100);

  //Clearing and resetting logging timer
  if (timer_set == false)
  {
    rtc.deconfigureAllTimers();
    rtc.enableCountdownTimer(PCF8523_FrequencyMinute, log_intervall);

    timer_set = true;
  }
  Serial.print("Timer set to ");
  Serial.println(log_intervall);

  //Taking measurements using arrays
  
  long a_pres[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_temp_grnd[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_hum_grnd[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_temp_surf[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_hum_surf[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_temp_air[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_hum_air[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_rad_up[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  long a_rad_dwn[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  int sleep_time = log_time * 100;

  for (int i_mes = 0; i_mes < 10; i_mes++)
  {
    Serial.println(i_mes);

    tcaselect(0);
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                    Adafruit_BMP280::SAMPLING_NONE,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    a_pres[i_mes] = bmp.readPressure()*1000;
    Serial.print("recorded Pressure: ");
    Serial.println(a_pres[i_mes]);

    tcaselect(1);
    a_temp_grnd[i_mes] = sht31_grnd.readTemperature()*1000;
    Serial.print("recorded ground temperature: ");
    Serial.println(a_temp_grnd[i_mes]);
    a_hum_grnd[i_mes] = sht31_grnd.readHumidity()*1000;
    Serial.print("recorded ground humidity: ");
    Serial.println(a_hum_grnd[i_mes]);

    tcaselect(2);
    a_temp_surf[i_mes] = sht31_surf.readTemperature()*1000;
    Serial.print("recorded surface temperature: ");
    Serial.println(a_temp_surf[i_mes]);
    a_hum_surf[i_mes] = sht31_grnd.readHumidity()*1000;
    Serial.print("recorded surface humidity: ");
    Serial.println(a_hum_surf[i_mes]);

    tcaselect(3);
    a_temp_air[i_mes] = sht31_air.readTemperature()*1000;
    Serial.print("recorded air temperature: ");
    Serial.println(a_temp_surf[i_mes]);
    a_hum_air[i_mes] = sht31_air.readHumidity()*1000;
    Serial.print("recorded air humidity: ");
    Serial.println(a_hum_air[i_mes]);

    tcaselect(4);
    a_rad_up[i_mes] = tsl_up.getLuminosity(2);
    Serial.print("recorded upwards radiation: ");
    Serial.println(a_rad_up[i_mes]);

    tcaselect(5);
    a_rad_dwn[i_mes] = tsl_dwn.getLuminosity(2);
    Serial.print("recorded downwards radiation: ");
    Serial.println(a_rad_dwn[i_mes]);

    delay(sleep_time);
  }

  float av_pres = a_average(a_pres) * 0.0010;
  float std_pres = a_std(a_pres) * 0.0010;

  Serial.println(av_pres);
  Serial.println(std_pres);

  float av_temp_grnd = a_average(a_temp_grnd) * 0.0010;
  float std_temp_grnd = a_std(a_temp_grnd) * 0.0010;

  float av_temp_surf = a_average(a_temp_surf) * 0.0010;
  float std_temp_surf = a_std(a_temp_surf) * 0.0010;

  float av_temp_air = a_average(a_temp_air) * 0.0010;
  float std_temp_air = a_std(a_temp_air) * 0.0010;

  float av_hum_grnd = a_average(a_hum_grnd) * 0.0010;
  float std_hum_grnd = a_std(a_hum_grnd) * 0.001;

  float av_hum_surf = a_average(a_hum_surf) * 0.001;
  float std_hum_surf = a_std(a_hum_surf) * 0.001;

  float av_hum_air = a_average(a_hum_air) * 0.001;
  float std_hum_air = a_std(a_hum_air) * 0.001;

  float av_rad_up = a_average(a_rad_up);
  float std_rad_up = a_std(a_rad_up);

  float av_rad_dwn = a_average(a_rad_dwn);
  float std_rad_dwn = a_std(a_rad_dwn);

  //Precipitation
  tcaselect(6);
  Wire.requestFrom(0b0110010, 3); //check if binary adress is useable

  uint32_t cnt_rain = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
  Serial.println(cnt_rain);

  int pre = cnt_rain * 0.02 / 0.2;

  digitalWrite(rst_rain, LOW);
  delay(100);
  digitalWrite(rst_rain, HIGH);

  //Wind velocity
  tcaselect(7);
  Wire.requestFrom(0b0110010, 3); //check if binary adress is useable

  uint32_t cnt_wind = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
  Serial.println(cnt_wind);

  int wind_vel;                   //Conversion function needs to be determined empirically

  digitalWrite(rst_wind, LOW);
  delay(100);
  digitalWrite(rst_wind, HIGH);

  //Logging data to Logfile
  SD.begin(SD_CS);
  logfile = SD.open(filename, FILE_WRITE);

  DateTime now = rtc.now();

  logfile.print(now.unixtime());
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  logfile.print(", ");
  logfile.print(av_pres, 4);
  logfile.print(", ");
  logfile.print(std_pres, 4);
  logfile.print(", ");
  logfile.print(av_temp_grnd, 4);
  logfile.print(", ");
  logfile.print(std_temp_grnd, 4);
  logfile.print(", ");
  logfile.print(av_temp_surf, 4);
  logfile.print(", ");
  logfile.print(std_temp_surf, 4);
  logfile.print(", ");
  logfile.print(av_temp_air, 4);
  logfile.print(", ");
  logfile.print(std_temp_air, 4);
  logfile.print(", ");
  logfile.print(av_hum_grnd, 4);
  logfile.print(", ");
  logfile.print(std_hum_grnd, 4);
  logfile.print(", ");
  logfile.print(av_hum_surf, 4);
  logfile.print(", ");
  logfile.print(std_hum_surf, 4);
  logfile.print(", ");
  logfile.print(av_hum_air, 4);
  logfile.print(", ");
  logfile.print(std_hum_air, 4);
  logfile.print(", ");
  logfile.print(av_rad_up, 4);
  logfile.print(", ");
  logfile.print(std_rad_up, 4);
  logfile.print(", ");
  logfile.print(av_rad_dwn, 4);
  logfile.print(", ");
  logfile.print(std_rad_dwn, 4);
  logfile.print(", ");
  logfile.print(pre, 4);
  logfile.print(", ");
  logfile.println(wind_vel, 4);

  Serial.print(now.unixtime());
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
  Serial.print(", ");
  Serial.print(av_pres, 4);
  Serial.print(", ");
  Serial.print(std_pres, 4);
  Serial.print(", ");
  Serial.print(av_temp_grnd, 4);
  Serial.print(", ");
  Serial.print(std_temp_grnd, 4);
  Serial.print(", ");
  Serial.print(av_temp_surf, 4);
  Serial.print(", ");
  Serial.print(std_temp_surf, 4);
  Serial.print(", ");
  Serial.print(av_temp_air, 4);
  Serial.print(", ");
  Serial.print(std_temp_air, 4);
  Serial.print(", ");
  Serial.print(av_hum_grnd, 4);
  Serial.print(", ");
  Serial.print(std_hum_grnd, 4);
  Serial.print(", ");
  Serial.print(av_hum_surf, 4);
  Serial.print(", ");
  Serial.print(std_hum_surf, 4);
  Serial.print(", ");
  Serial.print(av_hum_air, 4);
  Serial.print(", ");
  Serial.print(std_hum_air, 4);
  Serial.print(", ");
  Serial.print(av_rad_up, 4);
  Serial.print(", ");
  Serial.print(std_rad_up, 4);
  Serial.print(", ");
  Serial.print(av_rad_dwn, 4);
  Serial.print(", ");
  Serial.print(std_rad_dwn, 4);
  Serial.print(", ");
  Serial.print(pre, 4);
  Serial.print(", ");
  Serial.println(wind_vel, 4);

  logfile.close();

  myCAMSaveToSDFile();

  if(now.hour()==pic_time)
  {
    myCAMSaveToSDFile();
  }

  Going_To_Sleep();
}