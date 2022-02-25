#include <pt100rtd.h>
#include <Adafruit_MAX31865.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//pushbutton
#define PUSH_BUTTON_PIN 2
#define PUSH_BUTTON_CLICKED HIGH
#define PUSH_BUTTON_RELEASED LOW

//enter
#define ENTER_BUTTON_PIN 6 
#define ENTER_BUTTON_CLICKED LOW
#define ENTER_BUTTON_RELEASED HIGH

//led
#define LED_PIN 4
#define LED_ON HIGH
#define LED_OFF LOW

//OverTempSensor
#define OVER_TEMP_SENSOR_PIN A0
#define OVER_TEMP_SENSOR_CORRECTION 0.8 //correction acquired at 23*C
#define OVER_TEMP_SENSOR_MIN_TEMP_LIMIT 19 // minimal temperature limit for OverTempSensor in *C
#define OVER_TEMP_SENSOR_MAX_TEMP_LIMIT 32 // maximal temperature limit for OverTempSensor in *C

// RTDSensor
#define RTD_PIN A1
#define RTD_CORRECTION -1.5 //correction acquired at 23*C
#define RTD_MIN_TEMP_LIMIT 19 // minimal temperature limit for OverTempSensor in *C
#define RTD_MAX_TEMP_LIMIT 32 // maximal temperature limit for OverTempSensor in *C
const float K = 0.0125327;
pt100rtd PT100 = pt100rtd();

// checkPeltiers
#define POWER_SUPPLY_RELAY_PIN 8 
#define RELAY_ON HIGH
#define RELAY_OFF LOW
#define PELTIERS_TEST_TEMP_LIMIT 3.0 // minimal temperature deviation in *C
#define PELTIERS_TEST_TIME 10000 // heating / cooling time in miliseconds

#define DISPLAY_ROWS 2
#define DISPLAY_COLUMNS 16

LiquidCrystal_I2C lcd(0x27,DISPLAY_COLUMNS, DISPLAY_ROWS); 

String displayBuffer[2];

bool checkPushButton();
bool checkLed();
void waitForEnter();
double readTempFromOverTempSensor();
bool readTempFromRTD();
bool checkPeltiers();
unsigned long readTimeInSeconds();
bool checkOvertTempSensorMeasurement (double measuredTemp);
bool temperatureSensorsTestResults[2] = {false, false};
bool testResult = false;
bool Print(int column, int row, String displayBuffer[2], String message);
void updateDisplay(int numberOfRows, String displayBuffer);

void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(PUSH_BUTTON_PIN, INPUT);
  pinMode(ENTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(POWER_SUPPLY_RELAY_PIN, OUTPUT);
  digitalWrite(POWER_SUPPLY_RELAY_PIN, RELAY_OFF);
}

void loop()       //do the next test only if previous has returned true
{
  testResult = false;
  Print(0, 0, displayBuffer, "Nacisnij <ENTER>");
  Print(0, 1, displayBuffer, "aby rozpoczac");
  updateDisplay(2, displayBuffer);
  waitForEnter();
  if(checkPushButton())
  {
    if(checkLed())
    {
      Print(0, 0, displayBuffer, "OverTemp TEST");
      Print(0, 1, displayBuffer, "");
      updateDisplay(2, displayBuffer);
      temperatureSensorsTestResults[0] = checkOvertTempSensorMeasurement(readTempFromOverTempSensor());
      Print(3, 0, displayBuffer, "RTD TEST");
      Print(0, 1, displayBuffer, "");
      updateDisplay(2, displayBuffer);
      temperatureSensorsTestResults[1] = readTempFromRTD();
      if(temperatureSensorsTestResults[0] && temperatureSensorsTestResults[1])
      {
        if(checkPeltiers())
        {
          testResult = true;
        }
      }
    }
  }
  delay(200);
  Print(2, 0, displayBuffer, "Wynik testu:");
  if (testResult)
  {
    Print(6, 1, displayBuffer, "PASS");
  }else
  {
    Print(6, 1, displayBuffer, "FAIL");
  }
  updateDisplay(2, displayBuffer);
  waitForEnter();
}

bool checkPushButton()
{
  Print(0, 0, displayBuffer, "Wcisnij przycisk");
  Print(0, 1, displayBuffer, "na probe'ie");
  updateDisplay(2, displayBuffer);
  while(digitalRead(PUSH_BUTTON_PIN) != PUSH_BUTTON_CLICKED)
  {
    
    if(digitalRead(ENTER_BUTTON_PIN) == ENTER_BUTTON_CLICKED)
    {
      while(digitalRead(ENTER_BUTTON_PIN) != ENTER_BUTTON_RELEASED)
      {
      }
      return false;
    }
  }
    while(digitalRead(PUSH_BUTTON_PIN) != PUSH_BUTTON_RELEASED)
  {
  }
  return true;
}

bool checkLed()
{
  digitalWrite(LED_PIN, LED_ON);
  Print(0, 0, displayBuffer, "Sprawdz LED!");
  Print(0, 1, displayBuffer, "");
  updateDisplay(2, displayBuffer);
  delay(2000);  
  Print(0, 0, displayBuffer, "LED OK?- wcisnij");
  Print(0, 1, displayBuffer, "przycisk probe'a");
  updateDisplay(2, displayBuffer);
  while(digitalRead(PUSH_BUTTON_PIN) != PUSH_BUTTON_CLICKED)
  {
    if(digitalRead(ENTER_BUTTON_PIN) == ENTER_BUTTON_CLICKED)
    {
      while(digitalRead(ENTER_BUTTON_PIN) != ENTER_BUTTON_RELEASED)
      {
      }
      delay(200);
      digitalWrite(LED_PIN, LED_OFF); 
      Print(0, 0, displayBuffer, "TEST FAILED");
      Print(0, 1, displayBuffer, "");
      updateDisplay(2, displayBuffer);  
      waitForEnter();
      return false;
    }
  }
  while(digitalRead(PUSH_BUTTON_PIN) != PUSH_BUTTON_RELEASED)
  {
  }
  digitalWrite(LED_PIN, LED_OFF);  
  return true;
}

double readTempFromOverTempSensor()
{
  int readAnalogVal = 0;
  double tempFactor = 22.5;
  double temp = 0;
  unsigned long refTime = 0;
  int counter = 0;
  refTime = readTimeInSeconds();
  
  // Time for sensor stabilization
  for (int i = 1; i >= (readTimeInSeconds() - refTime); )
  {  
  readAnalogVal = map( (analogRead(OVER_TEMP_SENSOR_PIN)), 0, 1023, 0, 5000);
  }
  
  refTime = readTimeInSeconds();
  // Measuring
   for (int i = 1; i >= (readTimeInSeconds() - refTime); )
  {  
  readAnalogVal = map( (analogRead(OVER_TEMP_SENSOR_PIN)), 0, 1023, 0, 5000);
  temp = temp + ( (readAnalogVal - 1375) / tempFactor);
  counter++;
  }
  temp = temp / counter;
  temp = temp + OVER_TEMP_SENSOR_CORRECTION;
  return temp;
}

unsigned long readTimeInSeconds()
{
  return (millis() / 1000);
}

bool readTempFromRTD()
{
  String message;
  char temperatureTmp[5];
  int counter = 0;
  float Rrtd;
  float readAnalogVal = 0;
  float temp = 0;
  unsigned long refTime = 0;
  bool result = false;
  refTime = readTimeInSeconds();

  // Time for sensor stabilization
  for (int i = 1; i >= (readTimeInSeconds() - refTime); )
  {  
    readAnalogVal = readAnalogVal + map( (analogRead(RTD_PIN)), 0, 1023, 0, 5000);
  }
  
  //reset values
  readAnalogVal = 0;
  refTime = readTimeInSeconds();

  // Measuring
  for(int i = 1; i >= (readTimeInSeconds() - refTime); )
  {
    readAnalogVal = readAnalogVal + map( (analogRead(RTD_PIN)), 0, 1023, 0, 5000);
    counter++;
  }
  readAnalogVal = readAnalogVal / counter;
  Rrtd = 100 + readAnalogVal * K;
  temp = PT100.celsius(Rrtd);
  temp = temp + RTD_CORRECTION;   
  if ( temp > RTD_MIN_TEMP_LIMIT && temp < RTD_MAX_TEMP_LIMIT)
  {
    dtostrf(temp, 5, 2, temperatureTmp);
    message = "temp: ";
    message += temperatureTmp;
    message += "*C";
    Print(0, 0, displayBuffer, "RTD - PASS");
    Print(0, 1, displayBuffer, message);
    updateDisplay(2, displayBuffer);
    result = true;
    delay(2000);
    
  }else
  {
    dtostrf(temp, 5, 2, temperatureTmp);
    message = "temp: ";
    message += temperatureTmp;
    message += "*C";
    Print(0, 0, displayBuffer, "RTD - FAIL");
    Print(0, 1, displayBuffer, message);
    updateDisplay(2, displayBuffer);
    result = false;
    waitForEnter();
    
  }
  return result;
}

bool checkPeltiers()
{
  String message;
  double temp = 0;
  double tmpTemp = 0;
  double refTemp = 0;
  char temperatureCharBuffer[4];
  bool result = false;
  Print(0, 0, displayBuffer, "PELTIERS TEST");
  Print(0, 1, displayBuffer, "");
  updateDisplay(2, displayBuffer);
  refTemp = readTempFromOverTempSensor();

  digitalWrite(POWER_SUPPLY_RELAY_PIN, RELAY_ON);
  delay(PELTIERS_TEST_TIME);;
  temp = readTempFromOverTempSensor();
  digitalWrite(POWER_SUPPLY_RELAY_PIN, RELAY_OFF);
  if ( (tmpTemp = abs(refTemp - temp)) >= PELTIERS_TEST_TEMP_LIMIT)
  {
    dtostrf(tmpTemp, 5, 2, temperatureCharBuffer);
    message = "dev: ";
    message += temperatureCharBuffer;
    message += "*C";
    Print(0, 0, displayBuffer, "Peltiers - PASS");
    Print(0, 1, displayBuffer, message);
    updateDisplay(2, displayBuffer);
    delay(2000);
    result = true;
  }else
  {
    dtostrf(tmpTemp, 5, 2, temperatureCharBuffer);
    message = "dev: +";
    message += temperatureCharBuffer;
    message += "*C";
    Print(0, 0, displayBuffer, "Peltiers - FAIL");
    Print(0, 1, displayBuffer, message);
    updateDisplay(2, displayBuffer);
    result = false;
    waitForEnter();
  }
  return result;
}

bool checkOvertTempSensorMeasurement (double measuredTemp)
{
  char temperatureTmp[5];
  String message;
  bool result = false;
  if ( measuredTemp > OVER_TEMP_SENSOR_MIN_TEMP_LIMIT && measuredTemp < OVER_TEMP_SENSOR_MAX_TEMP_LIMIT)
  {
    dtostrf(measuredTemp, 5, 2, temperatureTmp);
    message = "temp: ";
    message += temperatureTmp;
    message += "*C";
    Print(0, 0, displayBuffer, "OverTemp - PASS");
    Print(0, 1, displayBuffer, message);
    updateDisplay(2, displayBuffer);
    result = true;
    delay(2000);
  }else
  {
    dtostrf(measuredTemp, 4, 2, temperatureTmp);
    message = "temp: ";
    message += temperatureTmp;
    message += "*C";
    Print(0, 0, displayBuffer, "OverTemp - FAIL");
    Print(0, 1, displayBuffer, message);
    updateDisplay(2, displayBuffer);
    result = false;
    waitForEnter();
  }
  return result;
}

bool Print(int column, int row, String displayBuffer[2], String message)
{
  bool returnVal = false;
  String tmp;
  int max_position = (message.length() - 1)  + column;
  if(column >= 0 & column < DISPLAY_COLUMNS)
  {
    if(row >= 0 & row <= DISPLAY_ROWS)
    {
      if(max_position < DISPLAY_COLUMNS)
      {
        if(column != 0)
        {
          for(int i = 0; i < column; i++)
          {
            tmp += " ";
          }
          for(int i = column; i <= max_position; i++)
         {
          tmp += message[i - column];
         }
         if(max_position < DISPLAY_COLUMNS - 1)
         {
          for(int j = max_position; j < DISPLAY_COLUMNS; j++)
          {
            tmp += " ";
          }
         }
         displayBuffer[row] = tmp;
         returnVal = true;
        }else
        {
          displayBuffer[row] = message;
          if(max_position < DISPLAY_COLUMNS - 1)
         {
          for(int j = max_position; j < DISPLAY_COLUMNS; j++)
          {
            displayBuffer[row] += " ";
          }
          returnVal = true;
        }
       }
      }
    }
  }
  return returnVal; 
}

void updateDisplay(int numberOfRows, String displayBuffer[2])
{
  if(numberOfRows == 0)
  {
    lcd.setCursor(0,0);
    lcd.print(displayBuffer[0]);
  }else if (numberOfRows == 1)
  {
    lcd.setCursor(0,1);
    lcd.print(displayBuffer[1]);
  }else if(numberOfRows == 2)
  {
    lcd.setCursor(0,0);
    lcd.print(displayBuffer[0]);
    lcd.setCursor(0,1);
    lcd.print(displayBuffer[1]);
  }
  lcd.display();  
}

void waitForEnter()
{
  while(digitalRead(ENTER_BUTTON_PIN) == HIGH)
  { 
  }
  delay(80);
  while(digitalRead(ENTER_BUTTON_PIN) == LOW)
  {
  }
  delay(80);
}
