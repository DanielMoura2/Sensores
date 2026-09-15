#include <Wire.h>                 // Biblioteca para comunicação I2C
#include <Adafruit_BMP280.h>      // Biblioteca do BMP280


// Cria um objeto chamado "bmp" para representar o sensor
Adafruit_BMP280 bmp;


void setup() {

  // Inicia a comunicação Serial com o computador
  Serial.begin(9600);


  // Tenta inicializar o BMP280
  if (!bmp.begin()) {

    // Se não encontrar o sensor, mostra uma mensagem de erro
    Serial.println("ERRO: BMP280 nao encontrado!");
    Serial.println("Verifique as conexoes.");


    // Mantém o programa parado caso o sensor não seja encontrado
    while (1) {
      if (bmp.begin()) {
        Serial.println("BMP280 encontrado!");
        break;
      }

      Serial.println("BMP280 nao encontrado...");
      delay(1000);
      }
  }


  // Se chegou aqui, o sensor foi encontrado
  Serial.println("BMP280 encontrado!");

  // Pula uma linha
  Serial.println();
}


void loop() {

  // Lê a temperatura do BMP280
  float temperatura = bmp.readTemperature();


  // Lê a pressão em Pascal
  float pressao = bmp.readPressure();


  // Converte Pascal (Pa) para hectopascal (hPa)
  float pressao_hPa = pressao / 100.0;


  // Calcula uma altitude aproximada
  // usando 1013.25 hPa como pressão de referência
  float altitude = bmp.readAltitude(1013.25);


  // Mostra a temperatura
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");


  // Mostra a pressão
  Serial.print("Pressao: ");
  Serial.print(pressao_hPa);
  Serial.println(" hPa");


  // Mostra a altitude
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");


  // Separador visual
  Serial.println("-------------------------");


  // Espera 1 segundo antes da próxima leitura
  delay(1000);
}