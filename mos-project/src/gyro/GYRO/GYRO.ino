// pins IMU A5 & A4 


#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;


void check_connect(){

  // Инициализация связи I2C
  Wire.begin();

  // Инициализация MPU6050
  Serial.println("Инициализация MPU6050...");
  mpu.initialize();

  // Проверка соединения с MPU6050
  if (mpu.testConnection()) {
    Serial.println("MPU6050 подключен.");
  } else {
    Serial.println("Ошибка подключения MPU6050.");
    while (1);
  }
}

// Функция для получения данных акселерометра
void getAccelerometerData(int16_t &ax, int16_t &ay, int16_t &az) {
    mpu.getAcceleration(&ax, &ay, &az);
}

// Функция для получения данных гироскопа
void getGyroscopeData(int16_t &gx, int16_t &gy, int16_t &gz) {
    mpu.getRotation(&gx, &gy, &gz);
}



void out_put_on_console(){

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Получение данных акселерометра
  getAccelerometerData(ax, ay, az);

  // Получение данных гироскопа
  getGyroscopeData(gx, gy, gz);

  // Вывод данных в сериал
  Serial.print("Акселерометр: ");
  Serial.print("X = "); Serial.print(ax);
  Serial.print(" | Y = "); Serial.print(ay);
  Serial.print(" | Z = "); Serial.println(az);

  Serial.print("Гироскоп: ");
  Serial.print("X = "); Serial.print(gx);
  Serial.print(" | Y = "); Serial.print(gy);
  Serial.print(" | Z = "); Serial.println(gz);

  delay(1000);

}

void getMonitorWithDegrees(float &angleX, float &angleY, float &angleZ, float &angleX1) {
  int16_t ax1, ay1, az1;
  int16_t gx1, gy1, gz1;
  
  // Получаем данные с акселерометра и гироскопа
  mpu.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);

  // Вычисляем углы для X, Y, Z в диапазоне 0-360
  angleX = atan2(ay1, sqrt(pow(ax1, 2) + pow(az1, 2))) * 180 / PI;
  if (angleX < 0) angleX += 360;

  angleY = atan2(ax1, sqrt(pow(ay1, 2) + pow(az1, 2))) * 180 / PI;
  if (angleY < 0) angleY += 360;

  angleZ = atan2(sqrt(pow(ax1, 2) + pow(ay1, 2)), az1) * 180 / PI;
  if (angleZ < 0) angleZ += 360;

  // Вычисляем дополнительный угол для X (зеркальный относительно 180 градусов)
  angleX1 = angleX - 180;
  if (angleX1 < 0) angleX1 += 360;
}




