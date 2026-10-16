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
