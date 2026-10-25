#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// -----------------------------
// CONFIGURAÇÃO DO GPS
// -----------------------------

// RX do Arduino recebe do TX do GPS
// TX do Arduino envia para o RX do GPS
SoftwareSerial gpsSerial(4, 3);

TinyGPSPlus gps;


// -----------------------------
// SETUP
// -----------------------------

void setup()
{
    Serial.begin(9600);

    // NEO-6M normalmente inicia em 9600 baud
    gpsSerial.begin(9600);

    Serial.println("Iniciando GPS...");
}


// -----------------------------
// LOOP
// -----------------------------

void loop()
{
    // Enquanto houver dados vindos do GPS
    while (gpsSerial.available())
    {
        // Envia os dados recebidos para o TinyGPSPlus
        gps.encode(gpsSerial.read());
    }


    // Se o GPS conseguiu uma localização válida
    if (gps.location.isValid())
    {
        Serial.println("-----------------------------");

        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);

        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);

        Serial.print("Altitude: ");
        Serial.print(gps.altitude.meters());
        Serial.println(" m");

        Serial.print("Velocidade: ");
        Serial.print(gps.speed.mps());
        Serial.println(" m/s");

        Serial.print("Satélites: ");
        Serial.println(gps.satellites.value());

        Serial.println("-----------------------------");
    }
    else
    {
        Serial.println("Aguardando sinal GPS...");
    }

    delay(200);
}