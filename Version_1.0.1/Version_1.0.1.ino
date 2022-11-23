//Version 1.0.1
//26/06/22

/////////////////////////////////////////////////////
//Bibliotecas Bmp180
#include <Wire.h>
#include <Adafruit_BMP085.h>
//Biblioteca Acelerometro
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//Bibliotecas GPS
#include <TinyGPS++.h>
//BibliotecasMQ135
#include "MQ135.h"
//Biblioteca DHT22
#include "DHTesp.h"
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//Bmp180
Adafruit_BMP085 bmp;
//Acelerometro
Adafruit_MPU6050 mpu;
//GPS
HardwareSerial neogps(1);
TinyGPSPlus gps;
#define RXD2 16
#define TXD2 17
//MQ135
#define SENSOR 12
//DHT 22
DHTesp dht;
/////////////////////////////////////////////////////

void setup(void) {
//BMP
  Serial.begin(115200);
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {}
  }
/////////////////////////////////////////////////////
//Acelerometro
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
/////////////////////////////////////////////////////
//DHT22
  dht.setup(27, DHTesp::DHT22);
  Serial.println();
  delay(2000);
}

  
void loop() {
/////////////////////////////////////////////////////
//Bmp180
 Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
    
  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
    
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(102000));
  Serial.println(" meters");
  Serial.println();
  delay(500);
/////////////////////////////////////////////////////
//Acelerometro
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
/////////////////////////////////////////////////////
// GPS
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }
  //If newData is true
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
    print_speed();
  }
  else
  {
    Serial.print("No Data");
  }   
/////////////////////////////////////////////////////
//MQ135
  MQ135 gasSensor = MQ135(SENSOR);
  float air_quality = gasSensor.getPPM();
  Serial.print("Air Quality: ");  
  Serial.print(air_quality);
  Serial.println("  PPM");   
  Serial.println();
  delay(2000); 
/////////////////////////////////////////////////////
//DHT22
  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();
  
  Serial.print("Temperatura: ");
  Serial.println(temperature);
  Serial.print("Humedad: ");
  Serial.println(humidity);    
  
  delay(2000);
}

void print_speed()
{    

   //String gps_speed = String(gps.speed.kmph());
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(),6);

    Serial.print("Lng: ");
    Serial.print(gps.location.lng(),6);

    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    
    Serial.print("SAT:");
    Serial.print(gps.satellites.value());


    Serial.print("ALT:");
    Serial.print(gps.altitude.meters(), 0);
  
}
