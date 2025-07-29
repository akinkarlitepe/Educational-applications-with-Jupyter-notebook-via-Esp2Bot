#include "ICM_20948.h"
#include <Wire.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;

float yaw = 0, pitch = 0, roll = 0;
unsigned long lastTime = 0;
float dt;

// Complementary filter değişkenleri
float alpha = 0.96;
float gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
float accelAngleX = 0, accelAngleY = 0;

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {};
  
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  
  SERIAL_PORT.println("ICM-20948 başarıyla başlatıldı!");
  SERIAL_PORT.println("Ham verileri kullanarak Euler açıları hesaplanacak...");
  lastTime = millis();
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); // Accelerometer, Gyroscope, Magnetometer ve Temperature verilerini oku
    
    // Ham verileri oku
    float accelX = myICM.accX();
    float accelY = myICM.accY(); 
    float accelZ = myICM.accZ();
    
    float gyroX = myICM.gyrX();
    float gyroY = myICM.gyrY();
    float gyroZ = myICM.gyrZ();
    
    // Zaman hesaplama
    unsigned long currentTime = millis();
    dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    
    // İlk çalıştırmada dt çok büyük olmasın
    if (dt > 1.0) dt = 0.01;
    
    // Accelerometer'dan açı hesaplama (derece)
    accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
    accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
    
    // Gyroscope'tan açı hesaplama (derece/saniye -> derece)
    gyroAngleX += gyroX * dt;
    gyroAngleY += gyroY * dt;
    gyroAngleZ += gyroZ * dt;
    
    // Complementary filter uygulama
    roll = alpha * (roll + gyroX * dt) + (1 - alpha) * accelAngleX;
    pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * accelAngleY;
    yaw = gyroAngleZ; // Yaw için sadece gyroscope kullan
    
    // NaN kontrolü
    if (isnan(roll)) roll = 0;
    if (isnan(pitch)) pitch = 0;
    if (isnan(yaw)) yaw = 0;
    
    // Değerleri sınırla (-180 ile +180 arası)
    if (roll > 180) roll -= 360;
    if (roll < -180) roll += 360;
    if (pitch > 180) pitch -= 360;
    if (pitch < -180) pitch += 360;
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    
    // Python'a veri gönderme (JSON formatında)
    SERIAL_PORT.print("{\"yaw\":");
    SERIAL_PORT.print(yaw, 2);
    SERIAL_PORT.print(",\"pitch\":");
    SERIAL_PORT.print(pitch, 2);
    SERIAL_PORT.print(",\"roll\":");
    SERIAL_PORT.print(roll, 2);
    SERIAL_PORT.println("}");
  }
  
  delay(50); // 20Hz güncelleme hızı
}