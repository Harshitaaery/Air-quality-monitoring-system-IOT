#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define BMP280_I2C_ADDRESS 0x76 // I2C address of the BMP280 sensor
#define MQ135_ANALOG_PIN A0 // Analog pin the MQ135 sensor is connected to


Adafruit_BMP280 bmp; // Create a BMP280 object

void setup() {

  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  while (!Serial);

  Wire.begin(); // Initialize I2C communication

  if (!bmp.begin(BMP280_I2C_ADDRESS)) { // Initialize the BMP280 sensor
    Serial.println("Could not find a valid BMP280 sensor, check wiring or address!");
    while (1);
  }

  // Default settings for BMP280 sensor (optional, but recommended)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating mode
                  Adafruit_BMP280::SAMPLING_X2,     // Temperature oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time

}

void loop() {
  // Read temperature, pressure, and altitude from BMP280 sensor
  float temperature = bmp.readTemperature(); // Temperature in degrees Celsius (°C)
  float pressure = bmp.readPressure() / 100.0F; // Pressure in hectopascals (hPa)
  float altitude = bmp.readAltitude(1013.25); // Altitude in meters (adjust 1013.25 to your local pressure)

  float sensorValue = analogRead(MQ135_ANALOG_PIN);
  float R = (sensorValue*5)/1023.0;
  float ppm = 1200*R/2.73;

  String airQuality;
  if (ppm < 400) {
    airQuality = " Good";
  } else if (ppm >= 400 && ppm <= 1000) {
    airQuality = "Moderate";
  } else {
    airQuality = " Bad";
  }



  // Print sensor readings
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");

  Serial.print("Air Quality (PPM): ");
  Serial.println(ppm);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PPM: ");
  lcd.print(ppm);
  lcd.setCursor(0, 1);
  lcd.print("Quality:");
  lcd.print(airQuality);
  delay(1000); // Delay for visibility (adjust as needed)

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pr: ");
  lcd.print(pressure);
  lcd.print(" hPa");

  
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" *C");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Alt: ");
  lcd.print(altitude);
  lcd.print(" m");
  delay(1000);

}

