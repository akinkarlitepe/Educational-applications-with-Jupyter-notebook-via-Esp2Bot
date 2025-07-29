#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* BNO055 bağlantıları:
   VIN - 3.3V veya 5V
   GND - GND
   SDA - A4 (Arduino Uno)
   SCL - A5 (Arduino Uno)
   RST - Digital Pin 2 (opsiyonel)
*/

#define BNO055_SAMPLERATE_DELAY_MS (100) // 10Hz güncelleme hızı

// BNO055 sensörü oluştur
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(115200);
  
  // BNO055 başlatılıyor
  if(!bno.begin()) {
    Serial.print("BNO055 bulunamadı! Bağlantıları kontrol edin.");
    while(1);
  }
  
  delay(1000);
  
  // Harici kristal kullan
  bno.setExtCrystalUse(true);
  
  Serial.println("BNO055 hazır!");
  Serial.println("Format: qw,qx,qy,qz,euler_x,euler_y,euler_z");
}

void loop(void) {
  // Quaternion verilerini al
  imu::Quaternion quat = bno.getQuat();
  
  // Euler açılarını al (derece cinsinden)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  // Kalibrasyon durumunu kontrol et
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  // Veriyi seri porta gönder
  Serial.print(quat.w(), 4);
  Serial.print(",");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.print(quat.z(), 4);
  Serial.print(",");
  Serial.print(euler.x(), 2);
  Serial.print(",");
  Serial.print(euler.y(), 2);
  Serial.print(",");
  Serial.print(euler.z(), 2);
  Serial.print(",");
  Serial.print(system);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.println(mag);
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
