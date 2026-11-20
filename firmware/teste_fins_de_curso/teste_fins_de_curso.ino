// Teste dos fins de curso da plataforma Y (Arduino Mega).
//
// Envia pela serial (115200) uma linha "FC <pino17> <pino14>" quando alguma chave muda e a cada
// 250 ms: 1 = chave apertada (LOW), 0 = solta. Leia com garra/scripts/ver_fins_de_curso.py.
//
// LED "L" (pino 13): aceso fixo = chave do pino 17; piscando = chave do pino 14.
//
// Seguranca: segura J1-J4 em 90 graus (1472 us, igual ao boot do garra_servos) para o braco nao
// cair; a plataforma (pino 12) fica sem sinal. Depois, regrave o garra_servos.ino.

#include <Servo.h>

const uint8_t PINO_A = 17;  // 2026-11-20: esquerda saiu do 0 (RX da USB) para o 17
const uint8_t PINO_B = 14;
const uint8_t LED = 13;
const uint8_t PINOS_SERVOS[4] = {9, 10, 11, 4};
const uint8_t PINO_Y = 12;
const int PULSO_INICIAL = 1472;

Servo servos[4];
int ultimo = -1;
unsigned long ultimo_envio = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PINO_A, INPUT_PULLUP);  // depois do Serial.begin: o pull-up continua valendo no RX
  pinMode(PINO_B, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(PINO_Y, OUTPUT);
  digitalWrite(PINO_Y, LOW);
  for (uint8_t i = 0; i < 4; i++) {
    servos[i].writeMicroseconds(PULSO_INICIAL);
    servos[i].attach(PINOS_SERVOS[i], 500, 2500);
  }
  Serial.println(F("TESTE FINS DE CURSO"));
}

void loop() {
  bool a = digitalRead(PINO_A) == LOW;
  bool b = digitalRead(PINO_B) == LOW;
  if (a) digitalWrite(LED, HIGH);
  else if (b) digitalWrite(LED, (millis() / 100) % 2);
  else digitalWrite(LED, LOW);

  int estado = (a ? 1 : 0) | (b ? 2 : 0);
  unsigned long agora = millis();
  if (estado != ultimo || agora - ultimo_envio >= 250) {
    ultimo = estado;
    ultimo_envio = agora;
    Serial.print(F("FC "));
    Serial.print(a ? 1 : 0);
    Serial.print(' ');
    Serial.println(b ? 1 : 0);
  }
}
