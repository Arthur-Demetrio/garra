// Firmware da garra para Arduino Mega: recebe pulsos pela serial e aciona 3 servos DS3218MG.
//
// Protocolo (115200 baud, linhas terminadas em \n):
//   V                    -> responde "GARRA 1"
//   M <j1> <j2> <j3>*HH  -> pulsos em us; 0 = nao alterar a junta. HH = XOR, em hexadecimal,
//                           de todos os bytes antes do '*'. Responde "K", ou "E <motivo>".
//   <motor>,<angulo>     -> comando manual do time (ex.: "1,90"): motor 1 a 3, angulo 0 a 180
//                           no mesmo padrao do Servo.write. Para testes pelo Monitor Serial
//                           (115200 baud, final de linha "Nova linha"). Nao usa a calibracao!
//
// Seguranca:
//  - Ao ligar, todos os servos vao para 90 graus (PULSO_INICIAL), como no sketch do time. Esse
//    primeiro movimento e na velocidade maxima do servo, porque nao se sabe onde ele esta.
//    Dai em diante todo movimento, inclusive o primeiro comando, respeita a velocidade abaixo.
//  - Velocidade do time (firmware/ino atual): 1 grau (~10 us) a cada 40 ms, ~25 graus/s,
//    em qualquer tipo de comando. Nao bloqueia: os tres servos andam juntos.
//  - Se a comunicacao cair, os servos mantem a ultima posicao: soltar deixaria o braco cair.

#include <Servo.h>

// Mesmos pinos do sketch atual (firmware/ino atual/garra/garra.ino): motor 1 = J1, 2 = J2, 3 = J3.
const uint8_t PINOS[3] = {9, 10, 11};

const int PULSO_MIN = 500;
const int PULSO_MAX = 2500;
const unsigned long PERIODO_MS = 40;  // INTERVALO_MOVIMENTO_MS do time
const int PASSO_US = 10;              // ~1 grau: (2400 - 544) / 180 = 10.31 us por grau

// Conversao do Servo.write (angulo em graus -> pulso), para o comando manual.
const int SERVO_WRITE_MIN = 544;
const int SERVO_WRITE_MAX = 2400;
const int PULSO_INICIAL = 1472;  // 90 graus no Servo.write = q=0 do modelo (vertical)

Servo servos[3];
int alvo[3];
int atual[3];

char linha[48];
uint8_t tamanho = 0;
bool estourou = false;
unsigned long ultimo_passo = 0;

void aplicar(uint8_t i, int pulso) {
  alvo[i] = pulso;
}

int hexa(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Le um inteiro nao negativo e avanca o ponteiro; retorna -1 se nao houver numero valido.
long ler_numero(const char *&p) {
  while (*p == ' ') p++;
  if (*p < '0' || *p > '9') return -1;
  long n = 0;
  while (*p >= '0' && *p <= '9') {
    n = n * 10 + (*p - '0');
    if (n > 99999) return -1;
    p++;
  }
  return n;
}

void comando_movimento() {
  const char *asterisco = strchr(linha, '*');
  if (asterisco == NULL || strlen(asterisco) != 3) {
    Serial.println(F("E formato"));
    return;
  }
  uint8_t soma = 0;
  for (const char *c = linha; c < asterisco; c++) soma ^= (uint8_t)*c;
  int alto = hexa(asterisco[1]);
  int baixo = hexa(asterisco[2]);
  if (alto < 0 || baixo < 0 || soma != (uint8_t)(alto * 16 + baixo)) {
    Serial.println(F("E checksum"));
    return;
  }

  long pulsos[3];
  const char *p = linha + 1;
  for (uint8_t i = 0; i < 3; i++) {
    pulsos[i] = ler_numero(p);
    if (pulsos[i] < 0 || (pulsos[i] != 0 && (pulsos[i] < PULSO_MIN || pulsos[i] > PULSO_MAX))) {
      Serial.println(F("E faixa"));
      return;
    }
  }
  if (p != asterisco) {
    Serial.println(F("E formato"));
    return;
  }
  // So aplica depois de validar a linha inteira.
  for (uint8_t i = 0; i < 3; i++) {
    if (pulsos[i] != 0) aplicar(i, (int)pulsos[i]);
  }
  Serial.println(F("K"));
}

// "<motor>,<angulo>", como no sketch do time.
bool comando_manual() {
  const char *p = linha;
  long motor = ler_numero(p);
  if (motor < 0 || *p != ',') return false;
  p++;
  long angulo = ler_numero(p);
  if (angulo < 0 || *p != '\0') {
    Serial.println(F("Erro: formato invalido! Use [motor],[angulo], ex.: 1,90"));
    return true;
  }
  if (motor < 1 || motor > 3) {
    Serial.println(F("Erro: Motor invalido! Escolha 1, 2 ou 3."));
    return true;
  }
  if (angulo > 180) {
    Serial.println(F("Erro: Angulo invalido! Digite entre 0 e 180."));
    return true;
  }
  int pulso = SERVO_WRITE_MIN + (int)((long)angulo * (SERVO_WRITE_MAX - SERVO_WRITE_MIN) / 180);
  aplicar((uint8_t)(motor - 1), pulso);
  Serial.print(F("-> Motor "));
  Serial.print(motor);
  Serial.print(F(" indo para "));
  Serial.print(angulo);
  Serial.print(F(" graus ("));
  Serial.print(pulso);
  Serial.println(F(" us)"));
  return true;
}

void processar_linha() {
  if (strcmp(linha, "V") == 0) {
    Serial.println(F("GARRA 1"));
  } else if (linha[0] == 'M') {
    comando_movimento();
  } else if (comando_manual()) {
  } else {
    Serial.println(F("E comando"));
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) {
    // Define o pulso antes do attach para o servo nao ir ao padrao da biblioteca (1500 us).
    atual[i] = alvo[i] = PULSO_INICIAL;
    servos[i].writeMicroseconds(PULSO_INICIAL);
    servos[i].attach(PINOS[i], PULSO_MIN, PULSO_MAX);
  }
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (estourou) {
        Serial.println(F("E tamanho"));
      } else if (tamanho > 0) {
        linha[tamanho] = '\0';
        processar_linha();
      }
      tamanho = 0;
      estourou = false;
    } else if (tamanho < sizeof(linha) - 1) {
      linha[tamanho++] = c;
    } else {
      estourou = true;
    }
  }

  unsigned long agora = millis();
  if (agora - ultimo_passo >= PERIODO_MS) {
    ultimo_passo = agora;
    for (uint8_t i = 0; i < 3; i++) {
      if (atual[i] == alvo[i]) continue;
      int erro = alvo[i] - atual[i];
      if (erro > PASSO_US) erro = PASSO_US;
      if (erro < -PASSO_US) erro = -PASSO_US;
      atual[i] += erro;
      servos[i].writeMicroseconds(atual[i]);
    }
  }
}
