// Plataforma do eixo Y: DS3218 modificado para rotacao continua.
// Pulsos: 1400 us = esquerda, 1500 us = parado, 1600 us = direita.
//
// Comandos pela serial (9600 baud):
//   E - anda para a esquerda
//   D - anda para a direita
//   P - para
//   ? - mostra o estado atual
//
// Seguranca: se ficar TIMEOUT_MS sem receber nenhum comando enquanto
// anda, a plataforma para sozinha. Para andar continuamente, reenvie
// E ou D periodicamente (menos que TIMEOUT_MS).

#include <Servo.h>

Servo motor;

const int PINO_SERVO = 13;

const int PULSO_ESQUERDA = 1400;
const int PULSO_PARADO   = 1500;
const int PULSO_DIREITA  = 1600;

const unsigned long TIMEOUT_MS = 500;

int pulso = PULSO_PARADO;
unsigned long ultimoComando = 0;

void aplicar(int novo) {
  pulso = novo;
  motor.writeMicroseconds(pulso);
}

const char* estado() {
  if (pulso == PULSO_ESQUERDA) return "ESQUERDA";
  if (pulso == PULSO_DIREITA)  return "DIREITA";
  return "PARADO";
}

void setup() {
  // Ja anexa parado, para nao andar ao ligar.
  motor.attach(PINO_SERVO);
  motor.writeMicroseconds(PULSO_PARADO);

  Serial.begin(9600);
  Serial.println("PLATAFORMA Y pronta: E=esquerda D=direita P=parar ?=estado");
}

void loop() {
  while (Serial.available()) {
    char c = toupper(Serial.read());
    bool valido = true;

    if (c == 'E')      aplicar(PULSO_ESQUERDA);
    else if (c == 'D') aplicar(PULSO_DIREITA);
    else if (c == 'P') aplicar(PULSO_PARADO);
    else if (c == '?') { /* so responde */ }
    else valido = false;  // ignora \r, \n e lixo

    if (valido) {
      ultimoComando = millis();
      Serial.print("OK ");
      Serial.println(estado());
    }
  }

  if (pulso != PULSO_PARADO && millis() - ultimoComando > TIMEOUT_MS) {
    aplicar(PULSO_PARADO);
    Serial.println("TIMEOUT PARADO");
  }
}
