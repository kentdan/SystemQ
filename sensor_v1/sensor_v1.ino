
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ArduinoOTA.h>  // For enabling over-the-air updates
#include <WiFi.h>        // For connecting ESP32 to WiFi
const char* ssid = "WIFI-C54292C8";         // Change to your WiFi Network name
const char* password = "12345678";  // Change to your password

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define RelayPin 33
#define TdsSensorPin 27
#define PhSensorPin 13
#define VREF 3.3
#define SCOUNT 30
#define Offset 0.00
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

int pHArray[ArrayLenth];
int pHArrayIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;
float pHValue, voltage;
float ecValue;
float rounded_pH;

//wifi credential
//const char* ssid = "FarmQ";  // Change to your WiFi Network name
//const char* password = "ab12cd34";  // Change to your password

void setup() {
  Serial.begin(460800);
  pinMode(TdsSensorPin, INPUT);
  pinMode(PhSensorPin, INPUT);
  pinMode(RelayPin, OUTPUT);


  if (!display.begin(i2c_Address)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(1000);
  //WiFi.begin(ssid, password);  // Connect to WiFi - defaults to WiFi Station mode
  // Ensure WiFi is connected
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  // }
  //ArduinoOTA.begin();  // Starts OTA

}

void loop() {
  //ArduinoOTA.handle();
  static unsigned long analogSampleTimepoint = millis();
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
   //relay
  static unsigned long pumpTimepoint = 0;
  static unsigned long printTimepoint = 0;
  static boolean pumpOn = false; // Track the state of the pump

   // turn on pump after 25min
  if (!pumpOn && samplingTime - pumpTimepoint >= 1500000) {
    pumpTimepoint = samplingTime;
    digitalWrite(RelayPin, HIGH); // turn on pump
    pumpOn = true; // Update relay state
    Serial.println("Relay ON ");
  }

  // Turn off the pump after on for 5 minute
  if (pumpOn && samplingTime - pumpTimepoint >= 300000) {
    digitalWrite(RelayPin, LOW); // turn off pump
    pumpOn = false;// Update relay state
    Serial.println("Relay OFF ");
  }

  // TDS Sensor Readings
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  // pH Sensor Readings
   if (millis() - samplingTime > samplingInterval) {
    samplingTime = millis();
    pHArray[pHArrayIndex++] = analogRead(PhSensorPin);
    if (pHArrayIndex == ArrayLenth) pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) / 4096 * 5;
    pHValue = -2.3673 * voltage +  17.764;  //y= -2.3673 * voltage +  17.868;  -3.5641 * voltage +  22.504;  y = -2.4946x + 18.335

    // Check pH value
  }
  
  if (millis() - printTime > printInterval) {
    printTime = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      float compensationVoltage = averageVoltage / compensationCoefficient;
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

      //serial
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println(" ppm");
      // Serial.print("Voltage:");
      // Serial.print(voltage,2);
      // Serial.print("    pH value: ");
      // Serial.println(pHValue,2);
      // Clear display
      // display.clearDisplay();

      // // Set text size and color
      // display.setTextSize(1);
      // display.setTextColor(SH110X_WHITE);

      // // Print TDS Value
      // display.setCursor(0, 10);
      // display.print("TDS Value:");
      // display.setCursor(0, 20);
      // display.print(tdsValue, 0);
      // display.print(" ppm");

      // // Print pH Value
      // display.setCursor(70, 10);
      // display.print("pH Value:");
      // display.setCursor(70, 20);
      // if (pHValue > 14.0) {
      //   display.print("adjusting");
      // } else if (pHValue < 5.87) {
      //   display.print("Need KOH");
      // } else // Floor pHValue to one decimal place
      // rounded_pH = floor(pHValue * 10.0) / 10.0;
      // display.print(rounded_pH, 2);


      // // Display EC Value
      // display.setCursor(0, 40);
      // display.print("EC Value:");
      // display.setCursor(0, 50);
      // display.print(ecValue, 2);
      // display.print(" uS/cm"); // Adjust units based on your sensor's specifications

      // Display relay
      display.setCursor(70, 40);
      display.print("Pump:"); //relay
      display.setCursor(70, 50);
      display.print("Auto");
      // Display on OLED
      display.display();      
    }
  }
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to averaging!/n");
    return 0;
  }
  if (number < 5) {
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;
          max = arr[i];
        } else {
          amount += arr[i];
        }
      }
    }
    avg = (double)amount / (number - 2);
  }
  return avg;
}

int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}