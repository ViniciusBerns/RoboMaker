#include <Arduino.h>

#if __has_include("esp_arduino_version.h")
  #include "esp_arduino_version.h"
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define NUCLEO_ESP32_3 1   // núcleo ESP32 versão 3.x (API de PWM nova)
#else
  #define NUCLEO_ESP32_3 0   // núcleo ESP32 versão 2.x (API de PWM antiga)
#endif

// ---- Pinos (conforme a sua montagem: GPIO do ESP32 -> pino do L298N) ----
const int PIN_ENA = 25;   // velocidade do motor ESQUERDO (PWM) (sem o jumper)
const int PIN_IN1 = 26;   // direção do motor esquerdo
const int PIN_IN2 = 27;   // direção do motor esquerdo
const int PIN_IN3 = 14;   // direção do motor direito (GPIO 14 dá um pulso curto ao ligar: um tranquinho nas rodas é normal)
const int PIN_IN4 = 13;   // direção do motor direito
const int PIN_ENB = 33;   // velocidade do motor DIREITO (PWM) (sem o jumper)

const int PWM_FREQ = 5000;    // 5 kHz
const int PWM_BITS = 8;       // valores de 0 a 255
const int VELOCIDADE = 200;   // 0 a 255 (comece com 200)

void pwmIniciar(int pino, int canal) {
#if NUCLEO_ESP32_3
  ledcAttach(pino, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(canal, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pino, canal);
#endif
}

void pwmEscrever(int pino, int canal, int duty) {
#if NUCLEO_ESP32_3
  ledcWrite(pino, duty);
#else
  ledcWrite(canal, duty);
#endif
}

// lado: 0 = esquerdo, 1 = direito.
// valor: -255 (ré a fundo) ... 0 (parado) ... +255 (frente a fundo)
void motor(int lado, int valor) {
  const int a  = (lado == 0) ? PIN_IN1 : PIN_IN3;
  const int b  = (lado == 0) ? PIN_IN2 : PIN_IN4;
  const int en = (lado == 0) ? PIN_ENA : PIN_ENB;
  if (valor >  255) valor =  255;
  if (valor < -255) valor = -255;
  if (valor > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW);  }
  else if (valor < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else                { digitalWrite(a, LOW);  digitalWrite(b, LOW);  }
  pwmEscrever(en, lado, abs(valor));
}

void passo(const char* texto, int esq, int dir) {
  Serial.println(texto);
  motor(0, esq);
  motor(1, dir);
  delay(2000);
  motor(0, 0);
  motor(1, 0);
  delay(1000);
}

void setup() {
  Serial.begin(115200);

  const int saidas[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  for (int p : saidas) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  pwmIniciar(PIN_ENA, 0);
  pwmIniciar(PIN_ENB, 1);
  motor(0, 0);
  motor(1, 0);

  Serial.println("Teste de motores: comeca em 3 segundos...");
  delay(3000);
}

void loop() {
  passo("1) roda ESQUERDA para FRENTE",  VELOCIDADE, 0);
  passo("2) roda ESQUERDA para TRAS",   -VELOCIDADE, 0);
  passo("3) roda DIREITA para FRENTE",   0, VELOCIDADE);
  passo("4) roda DIREITA para TRAS",     0, -VELOCIDADE);
  passo("5) AS DUAS para FRENTE",        VELOCIDADE,  VELOCIDADE);
  passo("6) AS DUAS para TRAS",         -VELOCIDADE, -VELOCIDADE);
  passo("7) GIRA para a ESQUERDA (no lugar)", -VELOCIDADE,  VELOCIDADE);
  passo("8) GIRA para a DIREITA (no lugar)",   VELOCIDADE, -VELOCIDADE);

  Serial.println("--- fim da sequencia; recomeca em 5 segundos ---");
  delay(5000);
}
