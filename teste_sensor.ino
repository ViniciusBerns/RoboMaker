/*
  =====================================================================
  TESTE DO SENSOR ULTRASSÔNICO HC-SR04 — ESP32 DevKit V1
  Arquivo: 03_teste_sensor.ino
  =====================================================================
  Ligações (detalhes na Parte 6 do guia):
    HC-SR04 VCC  -> 5 V (pino VIN do ESP32)
    HC-SR04 GND  -> GND
    HC-SR04 TRIG -> GPIO 18
    HC-SR04 ECHO -> resistor de 1 kΩ -> GPIO 19
                    e, no mesmo ponto do GPIO 19: resistor de 2 kΩ -> GND
                    (divisor de tensão: baixa o sinal de 5 V para 3,3 V)

    Sem resistores? Alimente o sensor com 3,3 V (VCC -> pino 3V3) e ligue o ECHO
    direto no GPIO 19: com 3,3 V de alimentação, o ECHO nunca passa de 3,3 V.

  Como usar: abra o Monitor Serial em 115200 e aproxime a mão do sensor.
  A distância mostrada deve acompanhar a mão: 10 cm, 20 cm, 50 cm...
  "sem eco" o tempo todo = fio solto, divisor errado ou sensor sem 5 V.
  =====================================================================
*/

#include <Arduino.h>

const int PIN_TRIG = 18;
const int PIN_ECHO = 19;

// Devolve a distância em centímetros, ou -1 se nenhum eco voltou.
float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);      // pulso de 10 microssegundos dispara a medição
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // mede quanto tempo o ECHO fica em HIGH (espera no máximo 25 ms, cerca de 4 m)
  unsigned long duracao = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (duracao == 0) return -1;

  // som: 0,0343 cm por microssegundo; o eco vai e volta, então divide por 2
  return duracao * 0.0343f / 2.0f;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.println("Teste do HC-SR04: aproxime a mao ou um objeto do sensor.");
}

void loop() {
  float d = medirDistanciaCm();
  if (d < 0) {
    Serial.println("sem eco (nada ate ~4 m, ou fio solto)");
  } else {
    Serial.print("distancia: ");
    Serial.print(d, 1);
    Serial.println(" cm");
  }
  delay(200);
}
