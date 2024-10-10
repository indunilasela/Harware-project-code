#include <SPI.h>
#include "max6675.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <RTClib.h>
#include <PZEM004Tv30.h>
#include <SoftwareSerial.h>
// Pin definitions for ultrasonic sensors
#define TRIG_PIN_1 4
#define ECHO_PIN_1 5
#define TRIG_PIN_2 2
#define ECHO_PIN_2 3
// Pin definitions for other components
#define untersonicLED_PIN 29
#define RELAY_PIN 38
#define CONTROL_PIN 8 // Control pin for spin switch
// Constants
const unsigned long constantTimeThreshold = 30000; // 30 seconds in milliseconds
const unsigned long ledDelayTime = 18000; // 18 seconds
const unsigned long noMotionDelayTime = 19000; // 19 seconds
const long distanceThreshold = 2; // Distance change threshold in cm
// Variables for ultrasonic sensors
long lastDistance1 = -1;
long lastDistance2 = -1;
unsigned long lastChangeTime1 = 0;
unsigned long lastChangeTime2 = 0;
bool outOfRange1 = false;
bool outOfRange2 = false;
// Variables for PIR sensor
const int pirSensorPin = 23;
const int pirBulbPin = 25;
unsigned long lastMotionTime = 0;
// Define the pins for the outputs
int hightempoutputPin = 33; // Red LED
int normaltempoutputPin3 = 35; // Relay for temperatures below 40°C (distinct pin)
int relaybulb = 31;
// Variables for thermocouples
int thermoSO1 = 43;
int thermoCS1 = 45;
int thermoSCK = 49;
int thermoSO2 = 41;
int thermoCS2 = 47;
int thermoSO3 = 39;
int thermoCS3 = 37;
double temp1, temp2, temp3;
MAX6675 thermocouple1(thermoSCK, thermoCS1, thermoSO1);
MAX6675 thermocouple2(thermoSCK, thermoCS2, thermoSO2);
MAX6675 thermocouple3(thermoSCK, thermoCS3, thermoSO3);
LiquidCrystal_I2C lcd(0x27, 20, 4); // Address 0x27, 20 characters wide and 4 lines
#define SDFILE_PIN_CS 53
#define SDcardfail_bulb 6
#define sdcardhave_bulb 7
#define powersuply_bulb 9
File sdFile;
const unsigned long interval2Sec = 2000;
unsigned long previousMillis2Sec = 0;
float totalEnergy = 0.0;
float todayEnergy = 0.0;
double sumthis = 0.0;
double sumthisactive = 0.0;
double sumthistoday = 0.0;
RTC_DS3231 rtc;
DateTime currentDate;
DateTime previousDate;
// Define SoftwareSerial pins
SoftwareSerial pzemSerial(10, 11); // RX, TX
// Create an instance of the PZEM004Tv30 class with SoftwareSerial
PZEM004Tv30 pzem(pzemSerial);
void setup() {
  Serial.begin(9600);
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  // Initialize other output pins
  pinMode(untersonicLED_PIN , OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(pirBulbPin, OUTPUT);
  pinMode(pirSensorPin, INPUT);
  pinMode(hightempoutputPin, OUTPUT);
  //pinMode(BuzzeroutputPin1, OUTPUT);
  pinMode(normaltempoutputPin3, OUTPUT);
  pinMode(relaybulb, OUTPUT);
  pinMode(CONTROL_PIN, INPUT_PULLUP); // Enable internal pull-up resistor on control pin
  // Ensure all output pins are initially off
  digitalWrite(untersonicLED_PIN , LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(pirBulbPin, LOW);
  digitalWrite(hightempoutputPin, LOW);
  //digitalWrite(BuzzeroutputPin1, LOW);
  digitalWrite(normaltempoutputPin3, LOW);
  digitalWrite(relaybulb, LOW);
  pzemSerial.begin(9600); // Initialize SoftwareSerial for PZEM
  Serial.println("PZEM-004T v3.0 Test");
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing LCD...");
  pinMode(SDFILE_PIN_CS, OUTPUT);
  pinMode(SDcardfail_bulb, OUTPUT);
  pinMode(sdcardhave_bulb, OUTPUT);
  pinMode(powersuply_bulb, OUTPUT);
  // Turn on the power supply bulb
  digitalWrite(powersuply_bulb, HIGH);
  Serial.println("Initializing SD card...");
  if (!SD.begin(SDFILE_PIN_CS)) {
    Serial.println("SD card initialization failed!");
    lcd.setCursor(0, 1);
    lcd.print("SD Card failed!");
    digitalWrite(SDcardfail_bulb, HIGH); 
    while (1);
  }
  Serial.println("SD card initialized successfully");
  lcd.setCursor(0, 1);
  lcd.print("SD Card initialized.");
  digitalWrite(sdcardhave_bulb, HIGH);
  delay(2000);
  lcd.clear();
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    lcd.setCursor(0, 2);
    lcd.print("RTC not found");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time!");
    rtc.adjust(DateTime(F(_DATE), F(TIME_)));
  }
  currentDate = rtc.now();
  previousDate = currentDate;
  // Read and sum up all the data from the current month's file
  totalCal();
  todayCal();
  Serial.println("Setup complete");
  delay(500); // Wait for MAX chip to stabilize
}
void loop() {
  int controlState = digitalRead(CONTROL_PIN); // Read the state of controlPin
  if (controlState == LOW) {
    // Only consider relay bulb and relay pin
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(relaybulb, HIGH);
    Serial.println("Control pin (8) LOW, relay pin (38) HIGH, bulb pin (31) HIGH");
  } else {
    // Consider other parts
    Serial.println("Control pin (8) HIGH, relay pin (38) LOW, bulb pin (31) LOW");
    long distance1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    long distance2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);
    handleDistance(distance1, lastDistance1, lastChangeTime1, outOfRange1);
    handleDistance(distance2, lastDistance2, lastChangeTime2, outOfRange2);
    handleMotionSensor();
    handleTemperatureSensor();
    handleRelay();
  }
  screensun();
  delay(100); // Wait for 100 milliseconds before the next loop
}
void totalCal() {
  String monthFileName = getMonthFileName(currentDate);
  sumthis = calculateTotalEnergy(monthFileName.c_str());
  lcd.setCursor(0, 3);
  lcd.print("Total(kWh):");
  lcd.setCursor(11, 3);
  lcd.print(sumthis,5); // Use print with 6 decimal places
}
void todayCal() {
  String todayFileName = getTodayFileName(currentDate);
  sumthistoday = calculateTotalEnergy(todayFileName.c_str());
  lcd.setCursor(0, 2);
  lcd.print("Today(kWh):");
  lcd.setCursor(11, 2);
  lcd.print(sumthistoday, 6); // Use print with 6 decimal places
}
String getTodayFileName(DateTime date) {
  String fileName = (date.month() < 10 ? "0" + String(date.month()) : String(date.month())) +
                    (date.day() < 10 ? "0" + String(date.day()) : String(date.day())) +
                    ".txt";
  return fileName;
}
// Function to get current month's file name based on the RTC date
String getMonthFileName(DateTime date) {
  const char* monthNames[] = {
    "January", "February", "March", "April", "May", "June",
    "July", "August", "September", "October", "November", "December"
  };
  String fileName = String(monthNames[date.month() - 1]) + ".txt";
  return fileName;
}
// Function to get current year's file name based on the RTC date
String getYearFileName(DateTime date) {
  String fileName = String(date.year()) + ".txt";
  return fileName;
}
void screensun(){
  currentDate = rtc.now();
  unsigned long currentMillis = millis();
  // Check if the day has changed
  if (currentDate.day() != previousDate.day()) {
    previousDate = currentDate;
    todayEnergy = 0.0; // Reset today's energy
    // Create a new file for the new day
    String todayFileName = getTodayFileName(currentDate);
    sdFile = SD.open(todayFileName.c_str(), FILE_WRITE);
    if (sdFile) {
      sdFile.close();
      Serial.println("New day file created: " + todayFileName);
    } else {
      Serial.println("Error creating new day file: " + todayFileName);
    }
  }
  // Check if the month has changed
  if (currentDate.month() != previousDate.month()) {
    previousDate = currentDate;
    totalEnergy = 0.0; // Reset total energy for the new month
    // Create a new file for the new month
    String monthFileName = getMonthFileName(currentDate);
    sdFile = SD.open(monthFileName.c_str(), FILE_WRITE);
    if (sdFile) {
      sdFile.close();
      Serial.println("New month file created: " + monthFileName);
    } else {
      Serial.println("Error creating new month file: " + monthFileName);
    }
    // Recalculate total energy for the new month
    totalCal();
  }
  // Check if the year has changed
  if (currentDate.year() != previousDate.year()) {
    previousDate = currentDate;
    // Create a new file for the new year
    String yearFileName = getYearFileName(currentDate);
    sdFile = SD.open(yearFileName.c_str(), FILE_WRITE);
    if (sdFile) {
      sdFile.close();
      Serial.println("New year file created: " + yearFileName);
    } else {
      Serial.println("Error creating new year file: " + yearFileName);
    }
  }
  // Every 2 seconds interval
  if (currentMillis - previousMillis2Sec >= interval2Sec) {
    previousMillis2Sec = currentMillis;
    // Readings from PZEM-004T v3.0 module
    float voltage = pzem.voltage();     // Voltage in volts
    float current = pzem.current();     // Current in amperes
    float powerFactor = pzem.pf();      // Power factor
    // Example time interval for each reading in seconds
    float timeSeconds = 2.0; // Corrected interval time to 2 seconds
    // Calculate energy for this reading
    float intervalEnergy = calculateEnergy(voltage, current, powerFactor, timeSeconds);
    // If energy is NaN, handle it
    if (isnan(intervalEnergy) || current == 0.000000) {
      totalCal();
      totalEnergy = 0;
      todayCal();
    }
    // Accumulate energy if it's non-zero
    if (intervalEnergy > 0.0000001) {
      totalEnergy += intervalEnergy;
      todayEnergy += intervalEnergy;
    }
    // Print interval energy to Serial monitor
    Serial.print("Energy over last 2 seconds (kWh): ");
    Serial.println(intervalEnergy, 8); // Print kWh with 8 decimal places
    // Print total and today's energy to Serial monitor
    Serial.print("Total Energy consumed (kWh): ");
    Serial.println(totalEnergy, 6); // Print kWh with 6 decimal places
    Serial.print("Today's Energy consumed (kWh): ");
    Serial.println(todayEnergy, 6); // Print kWh with 6 decimal places
    // Format the current date and time for logging
    String dateTimeString = String(currentDate.year()) + "/" +
                            (currentDate.month() < 10 ? "0" + String(currentDate.month()) : String(currentDate.month())) + "/" +
                            (currentDate.day() < 10 ? "0" + String(currentDate.day()) : String(currentDate.day())) + " " +
                            (currentDate.hour() < 10 ? "0" + String(currentDate.hour()) : String(currentDate.hour())) + ":" +
                            (currentDate.minute() < 10 ? "0" + String(currentDate.minute()) : String(currentDate.minute())) + ":" +
                            (currentDate.second() < 10 ? "0" + String(currentDate.second()) : String(currentDate.second()));
    // Update the current month's file with current month's energy consumption
    String monthFileName = getMonthFileName(currentDate);
    if ((intervalEnergy > 0.000000) || (current > 0.000000)) {
      if (intervalEnergy != 0.000000) {
        sdFile = SD.open(monthFileName.c_str(), FILE_WRITE);
        if (sdFile) {
          sdFile.println(intervalEnergy, 6);
          sdFile.close();
          Serial.println("Wrote to " + monthFileName);
        } else {
          Serial.println("Error opening " + monthFileName);
          lcd.setCursor(0, 0);
          lcd.print("Error opening");
        }
      } else {
        Serial.println("intervalEnergy is not a number");
        lcd.setCursor(0, 0);
        lcd.print("intervalEnergy not a number");
      }
    }
    // Update the current year's file with total energy consumption
    String yearFileName = getYearFileName(currentDate);
    if ((intervalEnergy > 0.000000) || (current > 0.000000)) {
      if (intervalEnergy != 0.000000) {
        sdFile = SD.open(yearFileName.c_str(), FILE_WRITE);
        if (sdFile) {
          sdFile.print(dateTimeString);
          sdFile.print(":");
          sdFile.println(totalEnergy, 6);
          sdFile.close();
          Serial.println("Wrote to " + yearFileName);
        } else {
          Serial.println("Error opening " + yearFileName);
          lcd.setCursor(0, 0);
          lcd.print("Error opening ");
        }
      } else {
        Serial.println("intervalEnergy is not a number");
        lcd.setCursor(0, 0);
        lcd.print("intervalEnergy not a number");
      }
    }
    // Update today's file with today's energy consumption
    if ((todayEnergy > 0.000000) || (current > 0.000000)) {
      if (totalEnergy != 0.00000) {
        String todayFileName = getTodayFileName(currentDate);
        sdFile = SD.open(todayFileName.c_str(), FILE_WRITE);
        if (sdFile) {
          sdFile.println(todayEnergy, 6);
          sdFile.close();
          Serial.println("Wrote to " + todayFileName);
        } else {
          Serial.println("Error opening " + todayFileName);
        }
        todayEnergy = 0.0;
      }
    }
    lcd.setCursor(0, 1);
    lcd.print("Now(kWh):");
    lcd.setCursor(9, 1);
    lcd.print(totalEnergy, 6); // Display total energy correctly
  }
  lcd.setCursor(0, 0);
  lcd.print("Date: ");
  print2Digits(currentDate.year(), lcd);
  lcd.print(".");
  print2Digits(currentDate.month(), lcd);
  lcd.print(".");
  print2Digits(currentDate.day(), lcd);
}
long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}
void print2Digits(int number, LiquidCrystal_I2C &lcd) {
  if (number < 10) {
    lcd.print("0");
  }
  lcd.print(number);
}
// Function to calculate energy consumption based on voltage, current, power factor, and time
float calculateEnergy(float voltage, float current, float powerFactor, float timeSeconds) {
  float realPower = voltage * current * powerFactor; // Calculate real power in watts
  float timeHours = timeSeconds / 3600.0; // Convert time to hours
  float energy_kWh = (realPower * timeHours) / 1000.0; // Calculate energy in kWh
  return energy_kWh;
}
float calculateTotalEnergy(const char* filename) {
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return 0.0; // Return 0.0 if the file couldn't be opened
  }
  double total = 0.0;
  String currentLine;
  while (file.available()) {
    currentLine = file.readStringUntil('\n');
    if (currentLine.length() > 0) {
      float value = currentLine.toFloat();
      if (!isnan(value)) {
        total += value;
      }
    }
  }
  file.close();
  return total;
}
void handleDistance(long distance, long &lastDistance, unsigned long &lastChangeTime, bool &outOfRange) {
  if (distance >= 2 && distance <= 70) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    outOfRange = false;
    if (abs(distance - lastDistance) >= distanceThreshold) {
      digitalWrite(untersonicLED_PIN , HIGH); // Turn on LED
      lastChangeTime = millis();
      lastDistance = distance;
    }
    if (millis() - lastChangeTime >= constantTimeThreshold) {
      digitalWrite(untersonicLED_PIN , LOW); // Turn off LED
    }
  } else {
    Serial.println("Distance out of range");
    outOfRange = true;
    // digitalWrite(untersonicLED_PIN , LOW); // Turn off LED if distance is out of range
  }
}
void handleMotionSensor() {
  int pirValue = digitalRead(pirSensorPin);
  if (pirValue == HIGH) {
    lastMotionTime = millis();
    digitalWrite(pirBulbPin, HIGH);
    Serial.println("Motion detected! LED bulb ON.");
  }
  if (millis() - lastMotionTime > ledDelayTime) {
    digitalWrite(pirBulbPin, LOW);
    Serial.println("No motion. LED bulb OFF.");
  }
}
void handleTemperatureSensor() {
  temp1 = thermocouple1.readCelsius();
  temp2 = thermocouple2.readCelsius();
  temp3 = thermocouple3.readCelsius();
  Serial.print("Thermocouple 1: ");
  Serial.print(temp1);
  Serial.print(" C | Thermocouple 2: ");
  Serial.print(temp2);
  Serial.print(" C | Thermocouple 3: ");
  Serial.print(temp3);
  Serial.println(" C");
  digitalWrite(hightempoutputPin, LOW);
  //digitalWrite(BuzzeroutputPin1, LOW);
  digitalWrite(normaltempoutputPin3, LOW);
  if ((temp1 > 160 ) || (temp2 > 160 ) || (temp3 > 160)) {
    digitalWrite(hightempoutputPin, HIGH); // Turn on the red LED
    Serial.println("Temperature above 160°C. Red LED ON.");
  } else if ((temp1 > 25 && temp1 <= 160) || (temp2 > 25 && temp2 <= 160) || (temp3 > 25 && temp3 <= 160)){
    digitalWrite(normaltempoutputPin3, HIGH); // Turn on the relay for low temperature
    Serial.println("Temperature below 160°C. Relay for low temperature ON.");
  }
}
void handleRelay() {
  bool highTemperature = (temp1 > 160 || temp2 >160 || temp3 >160);
  bool noMotion = (millis() - lastMotionTime > noMotionDelayTime);
  bool constantDistance1 = !outOfRange1 && (millis() - lastChangeTime1 >= constantTimeThreshold);
  bool constantDistance2 = !outOfRange2 && (millis() - lastChangeTime2 >= constantTimeThreshold);
  if (highTemperature || noMotion || constantDistance1 || constantDistance2) {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(relaybulb, HIGH);
    Serial.println("Relay ON due to high temperature, no motion, or constant distance.");
  } else {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(relaybulb,LOW);
    Serial.println("Relay OFF.");
  }
}