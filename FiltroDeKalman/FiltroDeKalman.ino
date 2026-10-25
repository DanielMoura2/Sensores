
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// =====================================================
// SENSORES
// =====================================================

Adafruit_BMP280 bmp;
TinyGPSPlus gps;

// GPS: RX Arduino = pino 4; TX Arduino = pino 3
SoftwareSerial gpsSerial(4, 3);

#define MPU_ADDR 0x68

// =====================================================
// CONFIGURACAO
// =====================================================

const unsigned long INTERVALO_MS = 250;
const float G = 9.80665f;

// MPU6050 na escala padrao de +/- 2 g
const float LSB_POR_G = 16384.0f;

// Variancias iniciais do filtro (m^2 e (m/s)^2).
// Sao valores iniciais de ajuste, nao calibracoes universais.
const float R_BMP = 4.0f;
const float R_GPS = 25.0f;

// Pressao medida no local de lancamento
float pressaoInicialPa = 0.0f;

// Altitude GPS de referencia no lancamento
float altitudeGPSInicial = 0.0f;
bool referenciaGPSValida = false;

// =====================================================
// ESTADO DO FILTRO DE KALMAN
// x[0] = altitude relativa (m)
// x[1] = velocidade vertical (m/s)
// =====================================================

float altitudeKalman = 0.0f;
float velocidadeKalman = 0.0f;

// Matriz de covariancia P
float P00 = 25.0f;
float P01 = 0.0f;
float P10 = 0.0f;
float P11 = 100.0f;

// =====================================================
// LEITURA DO MPU6050
// =====================================================

bool lerAceleracaoVertical(float &aceleracao) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) {
    return false;
  }

  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());

  // Este exemplo pressupoe que Z aponta para cima.
  // Parado, o eixo Z deve medir aproximadamente +1 g.
  // Subtraimos a gravidade para estimar a aceleracao
  // vertical sem a contribuicao da gravidade.
  float az_ms2 = ((float)az / LSB_POR_G) * G;

  aceleracao = az_ms2 - G;

  return true;
}

// =====================================================
// PREDICAO DO FILTRO
// Modelo: movimento vertical com aceleracao medida
// =====================================================

void preverKalman(float a, float dt) {
  // Predicao do estado
  altitudeKalman += velocidadeKalman * dt
                  + 0.5f * a * dt * dt;

  velocidadeKalman += a * dt;

  // Predicao da covariancia: P = F * P * F' + Q
  float n00 = P00 + dt * (P01 + P10)
            + dt * dt * P11;
  float n01 = P01 + dt * P11;
  float n10 = P10 + dt * P11;
  float n11 = P11;

  // Ruido de processo: ajuste inicial.
  // A calibracao real depende do sensor e do foguete.
  const float q = 4.0f;

  n00 += 0.25f * dt * dt * dt * dt * q;
  n01 += 0.5f * dt * dt * dt * q;
  n10 += 0.5f * dt * dt * dt * q;
  n11 += dt * dt * q;

  P00 = n00;
  P01 = n01;
  P10 = n10;
  P11 = n11;
}

// =====================================================
// ATUALIZACAO DO KALMAN COM UMA ALTITUDE MEDIDA
// z = altitude relativa em metros
// R = variancia da medicao em m^2
// =====================================================

void atualizarKalmanAltitude(float z, float R) {
  float S = P00 + R;

  if (S <= 0.0f) {
    return;
  }

  // Ganho de Kalman
  float K0 = P00 / S;
  float K1 = P10 / S;

  // Residuo entre a medicao e a estimativa
  float erro = z - altitudeKalman;

  // Corrige altitude e velocidade
  altitudeKalman += K0 * erro;
  velocidadeKalman += K1 * erro;

  // Atualiza covariancia
  float antigoP00 = P00;
  float antigoP01 = P01;
  float antigoP10 = P10;
  float antigoP11 = P11;

  P00 = (1.0f - K0) * antigoP00;
  P01 = (1.0f - K0) * antigoP01;
  P10 = antigoP10 - K1 * antigoP00;
  P11 = antigoP11 - K1 * antigoP01;

  // Mantem a matriz aproximadamente simetrica
  float cruzado = 0.5f * (P01 + P10);
  P01 = cruzado;
  P10 = cruzado;
}

// =====================================================
// INICIALIZACAO
// =====================================================

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  Wire.begin();

  Serial.println(F("Inicializando sensores..."));

  // BMP280
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println(F("ERRO: BMP280 nao encontrado."));
    while (true) {
      delay(1000);
    }
  }

  // Configuracao do BMP280
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_125
  );

  // Acorda MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("ERRO: MPU6050 nao encontrado."));
    while (true) {
      delay(1000);
    }
  }

  // Mede a pressao inicial como referencia local.
  // Ligue o sistema parado no local de lancamento.
  delay(500);
  pressaoInicialPa = bmp.readPressure();

  if (!isfinite(pressaoInicialPa) || pressaoInicialPa <= 0) {
    Serial.println(F("ERRO: leitura do BMP280 invalida."));
    while (true) {
      delay(1000);
    }
  }

  Serial.println(F("Sensores iniciados."));
  Serial.println(F("Aguardando referencia de altitude do GPS."));
  Serial.println(F("Mantenha o sistema parado no lancamento."));
  Serial.println();

  Serial.println(
    F("tempo_s,altitude_km,velocidade_vertical_m_s,"
      "alt_bmp_km,alt_gps_km,aceleracao_m_s2")
  );
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================

unsigned long ultimoTempo = 0;
unsigned long inicioTempo = 0;

void loop() {
  // Processa continuamente os dados recebidos do GPS.
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  unsigned long agora = millis();

  if (agora - ultimoTempo < INTERVALO_MS) {
    return;
  }

  float dt = (agora - ultimoTempo) / 1000.0f;
  ultimoTempo = agora;

  if (inicioTempo == 0) {
    inicioTempo = agora;
  }

  // -----------------------------------------------
  // 1. Aceleracao vertical do MPU6050
  // -----------------------------------------------

  float aceleracao = 0.0f;
  bool aceleracaoValida = lerAceleracaoVertical(aceleracao);

  // Se a leitura falhar, nao inventa uma aceleracao.
  // A predicao continua usando aceleracao zero.
  if (!aceleracaoValida) {
    aceleracao = 0.0f;
  }

  // -----------------------------------------------
  // 2. Predicao de altitude e velocidade
  // -----------------------------------------------

  preverKalman(aceleracao, dt);

  // -----------------------------------------------
  // 3. Altitude barometrica relativa
  // -----------------------------------------------

  float pressaoAtualPa = bmp.readPressure();
  float altitudeBMP = NAN;

  if (isfinite(pressaoAtualPa) && pressaoAtualPa > 0.0f) {
    // A funcao recebe a pressao de referencia em hPa.
    altitudeBMP = bmp.readAltitude(pressaoInicialPa / 100.0f);

    if (isfinite(altitudeBMP)) {
      atualizarKalmanAltitude(altitudeBMP, R_BMP);
    }
  }

  // -----------------------------------------------
  // 4. Altitude GPS relativa
  // -----------------------------------------------

  float altitudeGPS = NAN;

  if (gps.altitude.isValid() &&
      gps.location.isValid() &&
      gps.satellites.isValid() &&
      gps.satellites.value() >= 4) {

    float altitudeAbsolutaGPS = gps.altitude.meters();

    // Define a referencia na primeira leitura valida.
    // Idealmente, isso ocorre antes do lancamento.
    if (!referenciaGPSValida) {
      altitudeGPSInicial = altitudeAbsolutaGPS;
      referenciaGPSValida = true;
    }

    altitudeGPS = altitudeAbsolutaGPS - altitudeGPSInicial;

    // GPS pode atualizar mais lentamente que 250 ms.
    // So usa a altitude quando chegou uma nova medicao.
    if (gps.altitude.isUpdated()) {
      atualizarKalmanAltitude(altitudeGPS, R_GPS);
    }
  }

  // -----------------------------------------------
  // 5. Saida
  // -----------------------------------------------

  float tempo = (agora - inicioTempo) / 1000.0f;

  Serial.print(tempo, 2);
  Serial.print(',');

  // Altitude em km
  Serial.print(altitudeKalman / 1000.0f, 5);
  Serial.print(',');

  // Velocidade vertical em m/s:
  // positiva = subindo; negativa = descendo
  Serial.print(velocidadeKalman, 2);
  Serial.print(',');

  if (isfinite(altitudeBMP)) {
    Serial.print(altitudeBMP / 1000.0f, 5);
  } else {
    Serial.print(F("nan"));
  }
  Serial.print(',');

  if (isfinite(altitudeGPS)) {
    Serial.print(altitudeGPS / 1000.0f, 5);
  } else {
    Serial.print(F("nan"));
  }
  Serial.print(',');

  Serial.println(aceleracao, 2);
}
