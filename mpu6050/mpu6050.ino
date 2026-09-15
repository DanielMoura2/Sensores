#include <Wire.h>

// Endereço I2C do MPU6050
#define MPU6050_ADDR 0x68

// Registradores do MPU6050
#define ACCEL_XOUT_H 0x3B

void setup() {
  Serial.begin(9600);

  // Inicia comunicação I2C
  Wire.begin();

  // Acorda o MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);       // Registrador PWR_MGMT_1
  Wire.write(0x00);       // Ativa o sensor
  Wire.endTransmission();

  Serial.println("MPU6050 iniciado!");
}

void loop() {

  // Diz ao MPU6050 onde queremos começar a leitura
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);

  // Solicita 14 bytes
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  // Acelerômetro
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();

  // Pula temperatura
  Wire.read();
  Wire.read();

  // Giroscópio
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  // Mostra os valores
  Serial.print("Acelerometro: ");
  Serial.print("X = ");
  Serial.print(accelX);
  Serial.print(" | Y = ");
  Serial.print(accelY);
  Serial.print(" | Z = ");
  Serial.println(accelZ);

  Serial.print("Giroscopio: ");
  Serial.print("X = ");
  Serial.print(gyroX);
  Serial.print(" | Y = ");
  Serial.print(gyroY);
  Serial.print(" | Z = ");
  Serial.println(gyroZ);

  Serial.println("-------------------------");

  delay(500);
}