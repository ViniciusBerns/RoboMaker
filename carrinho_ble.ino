#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#if __has_include("esp_arduino_version.h")
  #include "esp_arduino_version.h"
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define NUCLEO_ESP32_3 1   // núcleo ESP32 versão 3.x (API de PWM nova)
#else
  #define NUCLEO_ESP32_3 0   // núcleo ESP32 versão 2.x (API de PWM antiga)
#endif


const char* NOME_BLE = "Carrinho-01";  


const bool USAR_SENSOR_ULTRASSONICO = false;
const int  DIST_PARADA_CM = 20;   
const int  DIST_LIBERA_CM = 25;   

const int   VEL_INICIAL = 190;    
const int   VEL_MIN     = 110;    
const int   VEL_MAX     = 255;
const int   VEL_PASSO   = 20;     
const float FATOR_CURVA = 0.40;   
const int   VEL_AUTO    = 150;    
const int   RAMPA_PASSO = 40;     




const int PIN_ENA = 25;   // L298N ENA — velocidade do motor ESQUERDO (PWM) (sem o jumper)
const int PIN_IN1 = 26;   // L298N IN1 — direção do motor esquerdo
const int PIN_IN2 = 27;   // L298N IN2 — direção do motor esquerdo
const int PIN_IN3 = 14;   // L298N IN3 — direção do motor direito 
const int PIN_IN4 = 13;   // L298N IN4 — direção do motor direito
const int PIN_ENB = 33;   // L298N ENB — velocidade do motor DIREITO 

const int PIN_TRIG = 18;  // HC-SR04 TRIG
const int PIN_ECHO = 19;  // HC-SR04 ECHO 

const int PIN_LED = 2;   

const int PWM_FREQ = 5000;   // 5 kHz
const int PWM_BITS = 8;      // valores de 0 a 255

volatile bool btnFrente  = false;
volatile bool btnTras    = false;
volatile bool btnEsq     = false;
volatile bool btnDir     = false;
volatile bool modoAuto   = false;
volatile int  velocidade = VEL_INICIAL;
volatile bool statusMudou = true;  
volatile bool conectado   = false;  

bool  eraConectado = false;
bool  obstaculo = false;            
float distanciaCm = -1;             
int   leiturasPerto = 0;
int   leiturasLonge = 0;

int atualE = 0;                     
int atualD = 0;                     

unsigned long tSensor = 0;
unsigned long tMotor  = 0;
unsigned long reiniciarAnuncioEm = 0;

BLEServer*         servidor = nullptr;
BLECharacteristic* caracTx  = nullptr;


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


void motorEscrever(int lado, int valor) {
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

void pararMotoresAgora() {
  atualE = 0;
  atualD = 0;
  motorEscrever(0, 0);
  motorEscrever(1, 0);
}


int zonaMorta(int v) {
  return (abs(v) < VEL_MIN) ? 0 : v;
}


int suavizar(int atual, int alvo) {
  if (alvo == 0) return 0;                                    // parar é imediato
  if (atual != 0 && ((atual > 0) != (alvo > 0))) return 0;    // inverter o sentido passa por zero
  if (atual == 0) atual = (alvo > 0) ? VEL_MIN : -VEL_MIN;    // sai do zero já na velocidade mínima útil
  int dif = alvo - atual;
  if (dif >  RAMPA_PASSO) dif =  RAMPA_PASSO;
  if (dif < -RAMPA_PASSO) dif = -RAMPA_PASSO;
  return atual + dif;
}


float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);                    // pulso de 10 µs dispara a medição
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long duracao = pulseIn(PIN_ECHO, HIGH, 25000UL);   // espera no máximo 25 ms (~4 m)
  if (duracao == 0) return -1;
  return duracao * 0.0343f / 2.0f;                 // som: 0,0343 cm/µs; o eco vai e volta
}

void atualizarSensor() {
  distanciaCm = medirDistanciaCm();
  const bool perto = (distanciaCm > 0 && distanciaCm <= DIST_PARADA_CM);
  const bool longe = (distanciaCm < 0 || distanciaCm >= DIST_LIBERA_CM);   // sem eco = livre

  if (perto) {
    if (leiturasPerto < 100) leiturasPerto++;
    leiturasLonge = 0;
  } else if (longe) {
    if (leiturasLonge < 100) leiturasLonge++;
    leiturasPerto = 0;
  }

  if (!obstaculo && leiturasPerto >= 2) {
    obstaculo = true;
    statusMudou = true;
    Serial.println("OBSTACULO a frente");
  }
  if (obstaculo && leiturasLonge >= 2) {
    obstaculo = false;
    statusMudou = true;
    Serial.println("caminho LIVRE");
  }
}


enum Acao { ACAO_PARAR, ACAO_FRENTE, ACAO_RE, ACAO_ESQUERDA, ACAO_DIREITA };


Acao decidirAcaoAutonoma(float distancia, bool temObstaculo) {
  (void)distancia;   
  if (temObstaculo) return ACAO_DIREITA;
  return ACAO_FRENTE;
}

void alvosDoModoAutomatico(int &alvoE, int &alvoD) {
  alvoE = 0;
  alvoD = 0;
  if (!USAR_SENSOR_ULTRASSONICO) return;   
  switch (decidirAcaoAutonoma(distanciaCm, obstaculo)) {
    case ACAO_FRENTE:   alvoE =  VEL_AUTO; alvoD =  VEL_AUTO; break;
    case ACAO_RE:       alvoE = -VEL_AUTO; alvoD = -VEL_AUTO; break;
    case ACAO_ESQUERDA: alvoE = -VEL_AUTO; alvoD =  VEL_AUTO; break;
    case ACAO_DIREITA:  alvoE =  VEL_AUTO; alvoD = -VEL_AUTO; break;
    default: break;
  }
}


void calcularAlvos(int &alvoE, int &alvoD) {
  if (modoAuto) {
    alvosDoModoAutomatico(alvoE, alvoD);
    alvoE = zonaMorta(alvoE);
    alvoD = zonaMorta(alvoD);
    return;
  }

  int acel = (btnFrente ? 1 : 0) - (btnTras ? 1 : 0);   
  int giro = (btnDir ? 1 : 0) - (btnEsq ? 1 : 0);      


  if (obstaculo && acel > 0) acel = 0;

  const int v = velocidade;
  if (acel == 0 && giro == 0) {                 // nada apertado: parado
    alvoE = 0;
    alvoD = 0;
  } else if (acel == 0) {                       // só giro: gira no próprio eixo
    alvoE =  giro * v;
    alvoD = -giro * v;
  } else if (giro == 0) {                       // só frente ou só ré: em linha reta
    alvoE = acel * v;
    alvoD = acel * v;
  } else {                                      // frente/ré + lado: curva
    int dentroAbs = (int)(v * FATOR_CURVA);
    if (dentroAbs < VEL_MIN) dentroAbs = VEL_MIN;   // a roda de dentro nunca anda menos que o mínimo útil
    const int fora   = acel * v;
    const int dentro = acel * dentroAbs;
    if (giro > 0) { alvoE = fora;   alvoD = dentro; }   // curva à direita
    else          { alvoE = dentro; alvoD = fora;   }   // curva à esquerda
  }
  alvoE = zonaMorta(alvoE);
  alvoD = zonaMorta(alvoD);
}

void atualizarMotores() {
  int alvoE = 0, alvoD = 0;
  calcularAlvos(alvoE, alvoD);
  atualE = suavizar(atualE, alvoE);
  atualD = suavizar(atualD, alvoD);
  motorEscrever(0, atualE);
  motorEscrever(1, atualD);
}


void zerarBotoes() {
  btnFrente = false;
  btnTras   = false;
  btnEsq    = false;
  btnDir    = false;
}

void soltarTudo() {              // PARAR: solta todas as setas e sai do modo automático
  zerarBotoes();
  if (modoAuto) {
    modoAuto = false;
    statusMudou = true;
  }
}

void mudarVelocidade(int delta) {
  int v = velocidade + delta;
  if (v > VEL_MAX) v = VEL_MAX;
  if (v < VEL_MIN) v = VEL_MIN;
  velocidade = v;
  statusMudou = true;
  Serial.printf("Velocidade: %d\n", v);
}

void alternarModoAuto() {
  zerarBotoes();
  if (!USAR_SENSOR_ULTRASSONICO) {
    modoAuto = false;            // sem sensor, o modo automático fica desligado
    Serial.println("Modo automatico precisa do sensor (USAR_SENSOR_ULTRASSONICO = true)");
  } else {
    modoAuto = !modoAuto;
    Serial.println(modoAuto ? "Modo automatico LIGADO" : "Modo automatico desligado");
  }
  statusMudou = true;
}

// seta: 0 = frente, 1 = trás, 2 = esquerda, 3 = direita
void definirSeta(int seta, bool apertada) {
  if (apertada && modoAuto) {    // tocar numa seta cancela o modo automático
    modoAuto = false;
    statusMudou = true;
  }
  switch (seta) {
    case 0: btnFrente = apertada; break;
    case 1: btnTras   = apertada; break;
    case 2: btnEsq    = apertada; break;
    case 3: btnDir    = apertada; break;
    default: break;
  }
}

// Botões do Control Pad do app Bluefruit: 1 a 4 numerados; 5 = cima, 6 = baixo, 7 = esquerda, 8 = direita.
void tratarBotao(int botao, bool apertado) {
  Serial.printf("Botao %d %s\n", botao, apertado ? "apertado" : "solto");
  switch (botao) {
    case 5: definirSeta(0, apertado); break;
    case 6: definirSeta(1, apertado); break;
    case 7: definirSeta(2, apertado); break;
    case 8: definirSeta(3, apertado); break;
    case 1: if (apertado) soltarTudo(); break;                 // 1 = PARAR
    case 2: if (apertado) mudarVelocidade(-VEL_PASSO); break;  // 2 = mais lento
    case 3: if (apertado) mudarVelocidade(+VEL_PASSO); break;  // 3 = mais rápido
    case 4: if (apertado) alternarModoAuto(); break;           // 4 = modo automático
    default: break;
  }
}

// Texto simples (para testes): cada letra vale até a próxima letra.
void tratarTexto(char c) {
  switch (c) {
    case 'F': case 'f': zerarBotoes(); definirSeta(0, true); break;
    case 'B': case 'b': zerarBotoes(); definirSeta(1, true); break;
    case 'L': case 'l': zerarBotoes(); definirSeta(2, true); break;
    case 'R': case 'r': zerarBotoes(); definirSeta(3, true); break;
    case 'S': case 's': soltarTudo(); break;
    case '+': mudarVelocidade(+VEL_PASSO); break;
    case '-': mudarVelocidade(-VEL_PASSO); break;
    case 'A': case 'a': alternarModoAuto(); break;
    default: break;              // ignora qualquer outro caractere (espaço, fim de linha...)
  }
}


void processarDados(const uint8_t* d, size_t n) {
  size_t i = 0;
  while (i < n) {
    if (d[i] == '!') {
      if (i + 4 < n && d[i + 1] == 'B') {
        uint8_t soma = 0;
        for (int k = 0; k < 4; k++) soma += d[i + k];
        if ((uint8_t)(~soma) == d[i + 4]) {
          tratarBotao(d[i + 2] - '0', d[i + 3] == '1');
        }
        i += 5;
      } else {
        break;                   // outro tipo de pacote (cor, sensores...): ignora o resto
      }
    } else {
      tratarTexto((char)d[i]);
      i++;
    }
  }
}


#define UUID_SERVICO "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UUID_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // celular -> ESP32
#define UUID_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // ESP32 -> celular

class CallbackServidor : public BLEServerCallbacks {
  void onConnect(BLEServer* s) {
    conectado = true;
    Serial.println("BLE: celular conectado");
  }
  void onDisconnect(BLEServer* s) {
    conectado = false;
    soltarTudo();                // perdeu o controle: para o carrinho
    Serial.println("BLE: celular desconectado");
  }
};

class CallbackRx : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    auto valor = c->getValue();  // o tipo muda entre versões do núcleo ESP32; "auto" serve nas duas
    uint8_t dados[32];
    size_t n = valor.length();
    if (n > sizeof(dados)) n = sizeof(dados);
    for (size_t i = 0; i < n; i++) dados[i] = (uint8_t)valor[i];
    processarDados(dados, n);
  }
};

void iniciarBLE() {
  BLEDevice::init(NOME_BLE);
  servidor = BLEDevice::createServer();
  servidor->setCallbacks(new CallbackServidor());

  BLEService* servico = servidor->createService(UUID_SERVICO);

  caracTx = servico->createCharacteristic(UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  caracTx->addDescriptor(new BLE2902());

  BLECharacteristic* caracRx = servico->createCharacteristic(
      UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  caracRx->setCallbacks(new CallbackRx());

  servico->start();

  BLEAdvertising* anuncio = BLEDevice::getAdvertising();
  anuncio->addServiceUUID(UUID_SERVICO);
  anuncio->setScanResponse(true);    // o nome do carrinho vai na "resposta de busca"
  BLEDevice::startAdvertising();
}

// Avisa o celular (aparece no Control Pad e no UART do app). Mensagens curtas: cabem em 20 bytes.
void enviarStatus() {
  if (!conectado || caracTx == nullptr) return;
  char msg[32];
  snprintf(msg, sizeof(msg), "V=%d A=%d O=%d\n", (int)velocidade, (int)modoAuto, (int)obstaculo);
  caracTx->setValue((uint8_t*)msg, strlen(msg));
  caracTx->notify();
}

// LED azul da placa: pisca devagar = esperando o celular; aceso = conectado; pisca rápido = obstáculo.
void atualizarLed(unsigned long agora) {
  bool ligado;
  if (obstaculo)      ligado = ((agora / 100) % 2) == 0;
  else if (conectado) ligado = true;
  else                ligado = ((agora / 500) % 2) == 0;
  digitalWrite(PIN_LED, ligado ? HIGH : LOW);
}

// =====================================================================
// 9) setup() e loop()
// =====================================================================
void setup() {
  Serial.begin(115200);

  // Primeiro de tudo: motores parados.
  const int saidas[] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  for (int p : saidas) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  pwmIniciar(PIN_ENA, 0);
  pwmIniciar(PIN_ENB, 1);
  pararMotoresAgora();

  pinMode(PIN_LED, OUTPUT);
  if (USAR_SENSOR_ULTRASSONICO) {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);
  }

  iniciarBLE();
  Serial.printf("Pronto! Procure \"%s\" no app. Sensor: %s\n",
                NOME_BLE, USAR_SENSOR_ULTRASSONICO ? "LIGADO" : "desligado");
}

void loop() {
  const unsigned long agora = millis();

  // 1) sensor: uma leitura a cada 60 ms (só se estiver ligado)
  if (USAR_SENSOR_ULTRASSONICO && agora - tSensor >= 60) {
    tSensor = agora;
    atualizarSensor();
  }

  // 2) motores: atualizados a cada 20 ms
  if (agora - tMotor >= 20) {
    tMotor = agora;
    atualizarMotores();
  }

  // 3) LED e avisos ao celular
  atualizarLed(agora);
  if (statusMudou) {
    statusMudou = false;
    enviarStatus();
  }

  // 4) conexão: ao desconectar, garante motores parados e volta a anunciar o carrinho
  if (!conectado && eraConectado) {
    eraConectado = false;
    pararMotoresAgora();
    reiniciarAnuncioEm = agora + 300;
  }
  if (conectado && !eraConectado) {
    eraConectado = true;
    statusMudou = true;
  }
  if (reiniciarAnuncioEm != 0 && agora >= reiniciarAnuncioEm) {
    reiniciarAnuncioEm = 0;
    servidor->startAdvertising();
    Serial.println("BLE: anunciando de novo");
  }

  delay(1);
}
