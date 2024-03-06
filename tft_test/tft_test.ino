#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(); 
//#include <User_Setups/Setup0_WROOM32_ILI9341_parallel_TFT_016.h>
#define TdsSensorPin 27
#define relayPin 33

#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ArduinoOTA.h>  // For enabling over-the-air updates
#include <WiFi.h>        // For connecting ESP32 to WiFi
const char* ssid = "WIFI-C54292C8";         // Change to your WiFi Network name
const char* password = "12345678";  // Change to your password

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation

// median filtering algorithm
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

void setup(){
  Serial.begin(460800);
  pinMode(TdsSensorPin,INPUT);
  pinMode(relayPin, OUTPUT);
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  ArduinoOTA.begin();  // Starts OTA
  // Initialize the display
  tft.begin();
}
void loop() {
  ArduinoOTA.handle();
  static unsigned long pumpTimepoint = 0;
  static unsigned long analogSampleTimepoint = 0;
  static unsigned long printTimepoint = 0;
  static boolean pumpOn = false; // Track the state of the pump

  unsigned long currentTime = millis();

   // turn on pump after 25min
  if (!pumpOn && currentTime - pumpTimepoint >=  1800000 )  //30 min. 6000) //
  {
    pumpTimepoint = currentTime;
    digitalWrite(relayPin, HIGH); // turn on pump
    pumpOn = true; // Update relay state
    Serial.println("Relay ON ");
  }

  // Turn off the pump after on for 5 minute
  if (pumpOn && currentTime - pumpTimepoint >=  300000) // 5 min. test 3 second 3000)//
  {
    digitalWrite(relayPin, LOW); // turn off pump
    pumpOn = false;// Update relay state
    Serial.println("Relay OFF ");
  }

  // TDS sampling
  if (currentTime - analogSampleTimepoint >= 40) { //every 40 milliseconds, read the analog value from the ADC
    analogSampleTimepoint = currentTime;
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
    analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;
  }

  // Print TDS value
  if (currentTime - printTimepoint >= 800) {
    printTimepoint = currentTime;

    // Calculate the median filtered voltage value
    averageVoltage = getMedianNum(analogBuffer, SCOUNT) * (float)VREF / 4096.0;

    // Temperature compensation formula
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    // Temperature compensation
    float compensationVoltage = averageVoltage / compensationCoefficient;

    // Convert voltage value to TDS value
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm");
    // Clear the screen
    tft.fillScreen(TFT_BLACK);

    // Print TDS value on the TFT screen
    tft.setTextColor(TFT_WHITE); // Set text color
    tft.setTextSize(2); // Set text size
    tft.setCursor(10, 10); // Set cursor position
    tft.print("TDS Value: ");
    tft.print(tdsValue, 0);
    tft.println(" ppm");
  }
}
