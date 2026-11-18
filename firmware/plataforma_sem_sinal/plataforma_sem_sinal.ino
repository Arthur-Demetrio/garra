// Plataforma do eixo Y: DS3218 modificado para rotacao continua.
//
// PARAR = CORTAR O SINAL. Sem pulso o servo desliga o motor e fica parado,
// sem depender do neutro (que nesse servo varia, ~1520 us). Andar = mandar
// o pulso de esquerda ou direita.
//
// Comandos pela serial (9600 baud, maiuscula ou minuscula):
//   E - anda para a esquerda
//   D - anda para a direita
//   P - para (corta o sinal)
//   ? - mostra o estado atual
//
// Seguranca:
//  - Ao ligar fica SEM SINAL (parada).
//  - Se ficar TIMEOUT_MS sem receber comando enquanto anda, corta o sinal.
//    Para andar continuamente, reenvie E ou D a cada menos de TIMEOUT_MS.
//  - Sem sinal o motor nao segura a plataforma no lugar (ok no eixo horizontal).

#include <Servo.h>

Servo motor;

const int PINO_SERVO = 8;

// Velocidades: distancia do neutro (~1520 us) define a velocidade.
const int PULSO_ESQUERDA = 1420;
const int PULSO_DIREITA  = 1620;

// Ao parar: manda o neutro por FREIO_MS e so depois corta o sinal. O DS3218 e
// digital e pode continuar executando o ULTIMO pulso recebido quando o sinal
// some; assim o ultimo pulso e o de parado, e nao o de andar.
const int PULSO_NEUTRO = 1520;
const unsigned long FREIO_MS = 300;

const unsigned long TIMEOUT_MS = 500;
unsigned long freioAte = 0;
bool freando = false;

int estado = 0;  // -1 esquerda, 0 parado (sem sinal), +1 direita
unsigned long ultimoComando = 0;

// Comeca a parada: neutro agora, corte do sinal depois de FREIO_MS (no loop).
void parar() {
  estado = 0;
  if (motor.attached()) {
    motor.writeMicroseconds(PULSO_NEUTRO);
    freando = true;
    freioAte = millis() + FREIO_MS;
  } else {
    cortarSinal();
  }
}

void cortarSinal() {
  freando = false;
  if (motor.attached()) {
    motor.detach();
  }
  pinMode(PINO_SERVO, OUTPUT);
  digitalWrite(PINO_SERVO, LOW);  // linha em nivel baixo = sem pulso
}

void andar(int sentido) {
  int pulso = (sentido < 0) ? PULSO_ESQUERDA : PULSO_DIREITA;
  // Define o pulso antes do attach para o primeiro pulso ja ser o certo.
  motor.writeMicroseconds(pulso);
  if (!motor.attached()) {
    motor.attach(PINO_SERVO);
  }
  motor.writeMicroseconds(pulso);
  estado = sentido;
  freando = false;
}

void mostrar() {
  Serial.print("OK ");
  if (estado < 0)      Serial.println("ESQUERDA");
  else if (estado > 0) Serial.println("DIREITA");
  else                 Serial.println("PARADO");
}

void setup() {
  cortarSinal();
  Serial.begin(9600);
  Serial.println("PLATAFORMA Y (para sem sinal): E=esquerda D=direita P=parar ?=estado");
}

void loop() {
  while (Serial.available()) {
    char c = toupper(Serial.read());
    bool valido = true;

    if (c == 'E')      andar(-1);
    else if (c == 'D') andar(1);
    else if (c == 'P') parar();
    else if (c == '?') { /* so responde */ }
    else valido = false;  // ignora \r, \n e lixo

    if (valido) {
      ultimoComando = millis();
      mostrar();
    }
  }

  if (estado != 0 && millis() - ultimoComando > TIMEOUT_MS) {
    parar();
    Serial.println("TIMEOUT PARADO");
  }

  if (freando && (long)(millis() - freioAte) >= 0) {
    cortarSinal();
  }
}
