#include <ThingerESP8266.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1X15.h>

//Bit-Rate MCU
#define SERIAL_BAUD 115200

Adafruit_ADS1115 ads; /* ADS 16-bit version */

float multiplier = 0.0078125F;// ads1115 +/- 0.256 gain
double outpyr, irradiance;

//Konversi Nilai Ketinggian hPa ke mdpl
#define SEALEVELPRESSURE_HPA (1013.25)
//#define StdATM (101325)

Adafruit_BME280 bme; // I2C
unsigned long delayTime;

//Variabel BME280
float temperatureC; //Suhu (Celsius)
float PressurePa; //Tekanan Udara (Pascal)
float ApxAltitude; //Ketinggian (MDPL)
float Humidity; //Kelembapan Relatif (%RH)

//konfigurasi untuk thinger.io
#define USERNAME "Dzikry_Affatah"
#define DEVICE_ID "UNTIRTAStation"
#define DEVICE_CREDENTIAL "peP_JoddfU@?y&gA"

//variabel untuk thinger.io
ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

//konfigurasi WiFi
//const char* ssid = "HJ. ROMLI";
//const char* password = "123lololase";

const char* ssid = "DZIKRY";
const char* password = "12344321";

//const char* ssid = "UNTIRTAKU";
//const char* password = "untirtajawara";


//////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin (SERIAL_BAUD);
  while (!Serial) {} // Wait
  Serial.println(F("BME280 test"));

  unsigned status;

  // default settings
  status = bme.begin(0x76);  //Alamat I2C 0x76 atau 0x77
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  Serial.println("-- Default Test --");
  delayTime = 500;
  Serial.println();

  //////////////////////////////////////////////////////////////////

  ads.setGain(GAIN_SIXTEEN);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  // Setup 3V comparator on channel 0
  ads.startComparator_SingleEnded(0, 1000);


  //koneksi ke wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting…");
    delay(delayTime);
    ESP.restart();
  }
  //apabila terkoneksi
  thing.add_wifi(ssid, password);

  //data yang dikirim ke thinger.io
  thing["Dataku"] >> [](pson & out) {
    out["Temperatur"] = (temperatureC);
    out["Humidity"] = (Humidity);
    out["Tekanan"] = (PressurePa);
    out["Ketinggian"] = (ApxAltitude);
    out["IRadiasi"] = (irradiance);
  };
}
void loop()
{
  printValues();

  //////////////////////////////////////////////////////////////////
  int16_t adc0, results;
  results = ads.readADC_Differential_2_3();

  // Comparator will only de-assert after a read
  adc0 = ads.getLastConversionResults();
  outpyr = results * multiplier;
  irradiance = abs((outpyr / 7) * 1000);
  Serial.print("IRradiance: ");
  Serial.print (irradiance);
  Serial.println(adc0);

  delay(delayTime);
  thing.handle();

}

//////////////////////////////////////////////////////////////////
void printValues() {
  temperatureC = bme.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  PressurePa = (bme.readPressure() / 100.0F);
  Serial.print("Pressure = ");
  Serial.print(PressurePa);
  Serial.println(" hPa");

  /*StdATM = PressurePa * 1013.25;
    Serial.print("Pressure = ");
    Serial.print(StdATM);
    Serial.println(" atm");

    ApxAltitude = (bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.print("Approx. Altitude = ");
    Serial.print(ApxAltitude);
    Serial.println(" MDPL");*/

  Humidity = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(Humidity);
  Serial.println(" %RH");

  Serial.println();
}
