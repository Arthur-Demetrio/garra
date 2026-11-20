// Firmware da garra para Arduino Mega: recebe pulsos pela serial e aciona 3 servos DS3218MG
// (J1-J3) e o servo DS-S006MP dos dedos (J4).
//
// Protocolo (115200 baud, linhas terminadas em \n):
//   V                    -> responde "GARRA 1"
//   M <j1> <j2> <j3> [<j4>]*HH -> pulsos em us; o J4 (dedos) e opcional. 0 = nao alterar a junta. HH = XOR, em hexadecimal,
//                           de todos os bytes antes do '*'. Responde "K", ou "E <motivo>".
//   S                    -> sensor de cubo nos dedos: responde "S 1" (chave fechada = cubo preso)
//                           ou "S 0". So responde quando perguntado: nada e enviado sozinho.
//   Y <E|D|P|?>          -> plataforma do eixo Y (servo de rotacao continua no pino 12):
//                           E = esquerda (1428 us), D = direita (1628 us), P = para (1528 us),
//                           ? = so consulta. Responde "Y ESQUERDA", "Y DIREITA" ou "Y PARADO".
//                           Se andar TIMEOUT_Y_MS sem receber outro comando Y, para sozinha e
//                           envia "Y TIMEOUT" (unica mensagem nao pedida). Para andar continuo,
//                           reenviar o comando a cada < 500 ms.
//                           Parar = neutro por FREIO_Y_MS e depois SEM SINAL (detach): o
//                           neutro desse servo varia e, so com ele, a plataforma anda devagar.
//   Y L E / Y L D        -> anda DEVAGAR (+-Y_LENTO_US do neutro, ~7 mm/s); mesmo timeout do E/D.
//   Y H E / Y H D        -> HOMING (como impressora 3D): vai devagar ate a chave do lado, recua,
//                           volta mais devagar ate apertar de novo e solta a chave. Sozinho, sem
//                           precisar reenviar. Responde no fim "Y HOME E OK" / "Y HOME D OK", ou
//                           "Y HOME E ERRO" se alguma fase passar de HOME_FASE_MAX_MS. Qualquer
//                           outro comando Y cancela ("Y HOME E CANCELADO").
//   Y X <pulso>          -> TESTE de parada: aplica um pulso FORA da faixa normal (100 a 2900 us), para
//                           ver se um pulso invalido desliga o motor do servo digital. Mesmo timeout
//                           e mesmos fins de curso. Responde "Y PULSO <n>".
//   Y <pulso>            -> TESTE: aplica esse pulso (us) na plataforma, ate +-Y_PULSO_FAIXA do neutro,
//                           para medir o neutro real. Responde "Y PULSO <n>" (ou "E faixa"). Mesmo
//                           timeout e mesmos fins de curso do E/D (menor que o neutro = esquerda).
//   F                    -> fins de curso da plataforma: responde "F <esq> <dir>" (1 = apertada).
//                           Com a chave de um lado apertada, a plataforma NAO anda para esse lado:
//                           "Y E"/"Y D" para ele responde "Y FIM ESQUERDA"/"Y FIM DIREITA" e para;
//                           andando, ao apertar a chave, para sozinha e envia a mesma mensagem.
//   <motor>,<angulo>     -> comando manual do time (ex.: "1,90"): motor 1 a 4, angulo 0 a 180
//                           no mesmo padrao do Servo.write. Para testes pelo Monitor Serial
//                           (115200 baud, final de linha "Nova linha"). Nao usa a calibracao!
//
// Sensor de cubo: chave fim de curso NORMALMENTE ABERTA nos dedos, entre o pino 3 e o GND, com o
// pull-up interno. Solta = HIGH (sem cubo); apertada pelo cubo = LOW. Leitura com debounce.
//
// Seguranca:
//  - Ao ligar, todos os servos vao para 90 graus (PULSO_INICIAL), como no sketch do time. Esse
//    primeiro movimento e na velocidade maxima do servo, porque nao se sabe onde ele esta.
//    Dai em diante todo movimento, inclusive o primeiro comando, respeita a velocidade abaixo.
//  - Velocidade: ~1,5 grau (15 us) a cada 20 ms, ~73 graus/s (3x o sketch do time),
//    em qualquer tipo de comando. Nao bloqueia: os tres servos andam juntos.
//  - Se a comunicacao cair, os servos mantem a ultima posicao: soltar deixaria o braco cair.

#include <Servo.h>

// Mesmos pinos do sketch atual (firmware/ino atual/garra/garra.ino): motor 1 = J1, 2 = J2, 3 = J3.
// Motor 4 = J4 (dedos), DS-S006MP no pino 4: ~90 graus de curso util em 500-2500 us, entao o
// angulo do comando manual nao e o angulo real do servo. Alimentar com 4,8 a 6 V.
const uint8_t N_SERVOS = 4;
const uint8_t PINOS[N_SERVOS] = {9, 10, 11, 4};  // 2026-11-19: dedos no 4 (antes 12)

const int PULSO_MIN = 500;
const int PULSO_MAX = 2500;
const unsigned long PERIODO_MS = 20;  // mesmo periodo do sinal do servo (50 Hz)
const int PASSO_US = 15;              // ~1,5 grau: (2400 - 544) / 180 = 10.31 us por grau

// Conversao do Servo.write (angulo em graus -> pulso), para o comando manual.
const int SERVO_WRITE_MIN = 544;
const int SERVO_WRITE_MAX = 2400;
const int PULSO_INICIAL = 1472;  // 90 graus no Servo.write = q=0 do modelo (vertical)

// Plataforma Y: DS3218 modificado para rotacao continua (pulso = velocidade, nao posicao).
// Nao entra na rampa de velocidade dos outros servos: o pulso e aplicado direto.
// Pino 12 desde 2026-11-19 (antes 8). Nao use o 13: o bootloader pisca o LED do 13 no reset e a plataforma dava um tranco.
const uint8_t PINO_Y = 12;
// Neutro medido em 2026-11-20 com a camera ("Y <pulso>" de 5 em 5 us): parada entre 1525 e 1530;
// 1520 (valor antigo) deslizava ~2 mm/s para a esquerda. Esquerda/direita a +-100 us dele.
const int PULSO_Y_PARADO = 1528;
const int PULSO_Y_ESQUERDA = PULSO_Y_PARADO - 100;   // = Y_RAPIDO_US
const int PULSO_Y_DIREITA = PULSO_Y_PARADO + 100;
const unsigned long TIMEOUT_Y_MS = 500;
// Velocidades lentas (medidas em 2026-11-20: +-30 us ~ 7 mm/s, +-15 us ~ 3 mm/s).
const int Y_RAPIDO_US = 100;      // velocidade normal ("Y E/D") e recuo do homing (mais forca para descolar)
const int Y_LENTO_US = 30;        // "Y L E/D" e aproximacao do homing
const int Y_MUITO_LENTO_US = 18;  // segundo toque do homing
// Homing: recuo depois do primeiro toque, tempo andando depois de soltar a chave, limite por fase.
const unsigned long HOME_RECUO_MS = 500;
const unsigned long HOME_SOLTA_MS = 150;
const unsigned long HOME_FASE_MAX_MS = 20000;
// "Y X <pulso>": faixa larga so para o teste de parada (a biblioteca Servo limita ao min/max do attach).
const int Y_X_MIN = 100;
const int Y_X_MAX = 2900;   // a biblioteca Servo guarda o max em int8: no maximo ~2912 us
// "Y <pulso>" (teste/medida do neutro): so aceita pulsos ate esta distancia do neutro.
const int Y_PULSO_FAIXA = 150;
// O DS3218 e digital e repete o ultimo pulso quando o sinal some: ao parar, manda o neutro
// por esse tempo e so entao corta o sinal (testado com firmware/plataforma_sem_sinal).
const unsigned long FREIO_Y_MS = 300;

// Fins de curso da plataforma Y: chave NORMALMENTE ABERTA entre o pino e o GND, pull-up interno
// (apertada = LOW). Nunca use o pino 0 (RX da USB): com a chave apertada o Arduino deixava de receber
// comandos do computador. A esquerda saiu do 0 para o 17 em 2026-11-20 (RX2, Serial2 nao usada).
const uint8_t PINO_FC_ESQ = 17;  // lado esquerdo do robo (y aumenta, comando "Y E")
const uint8_t PINO_FC_DIR = 14;  // lado direito do robo (y diminui, comando "Y D")
const unsigned long DEBOUNCE_FC_MS = 5;

const uint8_t PINO_SENSOR = 3;
const unsigned long DEBOUNCE_MS = 20;  // a chave precisa ficar estavel esse tempo

Servo servos[N_SERVOS];
Servo servo_y;
int pulso_y = PULSO_Y_PARADO;
unsigned long ultimo_comando_y = 0;
bool freando_y = false;
unsigned long freio_y_ate = 0;
enum FaseHome { H_NADA, H_APROX, H_RECUO, H_TOQUE, H_SOLTA };
FaseHome home_fase = H_NADA;
int home_lado = 0;                  // -1 = esquerda, +1 = direita
unsigned long home_fase_ini = 0;
bool fc_esq = false, fc_dir = false;     // estado filtrado das chaves (true = apertada)
bool fc_esq_lido = false, fc_dir_lido = false;
unsigned long fc_esq_mudou = 0, fc_dir_mudou = 0;
bool cubo_preso = false;           // estado ja filtrado da chave
bool leitura_anterior = false;
unsigned long mudou_em = 0;
int alvo[N_SERVOS];
int atual[N_SERVOS];

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

  // J1-J3 obrigatorios; J4 opcional (o calibrar_servos.py manda so 3).
  long pulsos[N_SERVOS] = {0, 0, 0, 0};
  const char *p = linha + 1;
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    if (i == 3 && *p == '*') break;
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
  for (uint8_t i = 0; i < N_SERVOS; i++) {
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
  if (motor < 1 || motor > N_SERVOS) {
    Serial.println(F("Erro: Motor invalido! Escolha 1, 2, 3 ou 4."));
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

void atualizar_sensor() {
  bool leitura = digitalRead(PINO_SENSOR) == LOW;  // LOW = chave fechada = cubo
  unsigned long agora = millis();
  if (leitura != leitura_anterior) {
    leitura_anterior = leitura;
    mudou_em = agora;
  } else if (agora - mudou_em >= DEBOUNCE_MS) {
    cubo_preso = leitura;
  }
}

void filtrar_fc(uint8_t pino, bool &estado, bool &lido, unsigned long &mudou, unsigned long agora) {
  bool leitura = digitalRead(pino) == LOW;
  if (leitura != lido) {
    lido = leitura;
    mudou = agora;
  } else if (agora - mudou >= DEBOUNCE_FC_MS) {
    estado = leitura;
  }
}

void atualizar_fins_de_curso() {
  unsigned long agora = millis();
  filtrar_fc(PINO_FC_ESQ, fc_esq, fc_esq_lido, fc_esq_mudou, agora);
  filtrar_fc(PINO_FC_DIR, fc_dir, fc_dir_lido, fc_dir_mudou, agora);
}

void responder_estado_y() {
  if (pulso_y == PULSO_Y_ESQUERDA) Serial.println(F("Y ESQUERDA"));
  else if (pulso_y == PULSO_Y_DIREITA) Serial.println(F("Y DIREITA"));
  else if (pulso_y == PULSO_Y_PARADO) Serial.println(F("Y PARADO"));
  else {
    Serial.print(F("Y PULSO "));
    Serial.println(pulso_y);
  }
}

// Pulso menor que o neutro anda para a esquerda; maior, para a direita.
bool bloqueado_fc(int pulso) {
  return (pulso < PULSO_Y_PARADO && fc_esq) || (pulso > PULSO_Y_PARADO && fc_dir);
}

bool fc_do_lado(int lado) { return lado < 0 ? fc_esq : fc_dir; }

// Pulso andando para o lado (-1 esquerda, +1 direita) com essa distancia do neutro.
int pulso_lado(int lado, int delta) { return PULSO_Y_PARADO + lado * delta; }

void fim_home(const __FlashStringHelper *resultado) {
  Serial.print(F("Y HOME "));
  Serial.print(home_lado < 0 ? 'E' : 'D');
  Serial.print(' ');
  Serial.println(resultado);
  home_fase = H_NADA;
}

void cortar_sinal_y() {
  freando_y = false;
  if (servo_y.attached()) servo_y.detach();
  pinMode(PINO_Y, OUTPUT);
  digitalWrite(PINO_Y, LOW);  // sem pulso: o servo desliga o motor
}

// bruto = true ("Y <pulso>"): aplica o pulso como esta, inclusive o proprio neutro, sem frear nem
// cortar o sinal (para medir em qual pulso a plataforma fica parada).
void aplicar_y(int pulso, bool bruto) {
  // Nunca anda para o lado de uma chave apertada (sair dela, para o outro lado, pode).
  if (bloqueado_fc(pulso)) {
    Serial.println(pulso < PULSO_Y_PARADO ? F("Y FIM ESQUERDA") : F("Y FIM DIREITA"));
    pulso = PULSO_Y_PARADO;
    bruto = false;
  }
  pulso_y = pulso;
  if (bruto) {
    freando_y = false;
    if (!servo_y.attached()) servo_y.attach(PINO_Y, PULSO_MIN, PULSO_MAX);
    servo_y.writeMicroseconds(pulso);
    return;
  }
  if (pulso == PULSO_Y_PARADO) {
    if (servo_y.attached() && !freando_y) {
      servo_y.writeMicroseconds(PULSO_Y_PARADO);   // freia; o loop corta o sinal depois
      freando_y = true;
      freio_y_ate = millis() + FREIO_Y_MS;
    }
    return;
  }
  freando_y = false;
  servo_y.writeMicroseconds(pulso);
  if (!servo_y.attached()) servo_y.attach(PINO_Y, PULSO_MIN, PULSO_MAX);
  servo_y.writeMicroseconds(pulso);
}

void entrar_fase_home(FaseHome fase, int pulso) {
  home_fase = fase;
  home_fase_ini = millis();
  ultimo_comando_y = millis();
  aplicar_y(pulso, false);
}

// Um passo do homing, chamado a cada volta do loop (antes da protecao dos fins de curso).
void passo_home(unsigned long agora) {
  if (home_fase == H_NADA) return;
  ultimo_comando_y = agora;   // o homing anda sozinho: nao deixa o timeout de 500 ms parar
  bool chave = fc_do_lado(home_lado);
  unsigned long na_fase = agora - home_fase_ini;
  switch (home_fase) {
    case H_APROX:   // devagar ate a chave
      if (chave) entrar_fase_home(H_RECUO, pulso_lado(-home_lado, Y_RAPIDO_US));
      break;
    case H_RECUO:   // afasta um pouco, em velocidade normal (a chave precisa soltar)
      if (!chave && na_fase >= HOME_RECUO_MS) entrar_fase_home(H_TOQUE, pulso_lado(home_lado, Y_MUITO_LENTO_US));
      break;
    case H_TOQUE:   // volta bem devagar: este toque e a posicao confiavel
      if (chave) entrar_fase_home(H_SOLTA, pulso_lado(-home_lado, Y_MUITO_LENTO_US));
      break;
    case H_SOLTA:   // solta a chave e para logo depois
      if (!chave && na_fase >= HOME_SOLTA_MS) {
        aplicar_y(PULSO_Y_PARADO, false);
        fim_home(F("OK"));
        return;
      }
      break;
    default:
      break;
  }
  if (home_fase != H_NADA && agora - home_fase_ini > HOME_FASE_MAX_MS) {
    aplicar_y(PULSO_Y_PARADO, false);
    fim_home(F("ERRO"));
  }
}

void iniciar_home(int lado) {
  home_lado = lado;
  // Ja na chave: comeca recuando; senao, aproxima devagar.
  if (fc_do_lado(lado)) entrar_fase_home(H_RECUO, pulso_lado(-lado, Y_RAPIDO_US));
  else entrar_fase_home(H_APROX, pulso_lado(lado, Y_LENTO_US));
  Serial.print(F("Y HOME "));
  Serial.print(lado < 0 ? 'E' : 'D');
  Serial.println(F(" INICIO"));
}

// "Y E", "Y D", "Y P", "Y ?", "Y L E", "Y L D", "Y H E", "Y H D" ou "Y <pulso>" (espacos opcionais).
void comando_y() {
  const char *p = linha + 1;
  while (*p == ' ') p++;
  // Qualquer comando Y que nao seja "?" cancela um homing em andamento.
  if (home_fase != H_NADA && *p != '?') {
    aplicar_y(PULSO_Y_PARADO, false);
    fim_home(F("CANCELADO"));
  }
  char modo = (char)toupper(*p);
  if (modo == 'X') {
    p++;
    while (*p == ' ') p++;
    char *fim;
    long pulso = strtol(p, &fim, 10);
    if (fim == p || *fim != '\0') {
      Serial.println(F("E formato"));
      return;
    }
    if (pulso < Y_X_MIN || pulso > Y_X_MAX) {
      Serial.println(F("E faixa"));
      return;
    }
    if (bloqueado_fc((int)pulso)) {
      aplicar_y((int)pulso, false);   // recusa e avisa "Y FIM ..."
      return;
    }
    freando_y = false;
    pulso_y = (int)pulso;
    if (servo_y.attached()) servo_y.detach();
    servo_y.attach(PINO_Y, Y_X_MIN, Y_X_MAX);
    servo_y.writeMicroseconds((int)pulso);
    ultimo_comando_y = millis();
    responder_estado_y();
    return;
  }
  if (modo == 'L' || modo == 'H') {
    p++;
    while (*p == ' ') p++;
    char lado = (char)toupper(*p);
    if ((lado != 'E' && lado != 'D') || p[1] != '\0') {
      Serial.println(F("E formato"));
      return;
    }
    int l = lado == 'E' ? -1 : 1;
    if (modo == 'H') {
      iniciar_home(l);
      return;
    }
    aplicar_y(pulso_lado(l, Y_LENTO_US), false);
    ultimo_comando_y = millis();
    responder_estado_y();
    return;
  }
  if (isdigit(*p)) {
    char *fim;
    long pulso = strtol(p, &fim, 10);
    if (*fim != '\0') {
      Serial.println(F("E formato"));
      return;
    }
    if (pulso < PULSO_Y_PARADO - Y_PULSO_FAIXA || pulso > PULSO_Y_PARADO + Y_PULSO_FAIXA) {
      Serial.println(F("E faixa"));
      return;
    }
    aplicar_y((int)pulso, true);
    ultimo_comando_y = millis();
    responder_estado_y();
    return;
  }
  char c = (char)toupper(*p);
  if (c == '\0' || p[1] != '\0') {
    Serial.println(F("E formato"));
    return;
  }
  if (c == 'E') aplicar_y(PULSO_Y_ESQUERDA, false);
  else if (c == 'D') aplicar_y(PULSO_Y_DIREITA, false);
  else if (c == 'P') aplicar_y(PULSO_Y_PARADO, false);
  else if (c != '?') {
    Serial.println(F("E comando"));
    return;
  }
  ultimo_comando_y = millis();
  responder_estado_y();
}

void processar_linha() {
  if (strcmp(linha, "V") == 0) {
    Serial.println(F("GARRA 1"));  // nao mudar: o driver ROS e o calibrar_servos.py esperam isso
  } else if (strcmp(linha, "S") == 0) {
    Serial.println(cubo_preso ? F("S 1") : F("S 0"));
  } else if (strcmp(linha, "F") == 0) {
    Serial.print(F("F "));
    Serial.print(fc_esq ? 1 : 0);
    Serial.print(' ');
    Serial.println(fc_dir ? 1 : 0);
  } else if (linha[0] == 'Y' || linha[0] == 'y') {
    comando_y();
  } else if (linha[0] == 'M') {
    comando_movimento();
  } else if (comando_manual()) {
  } else {
    Serial.println(F("E comando"));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PINO_SENSOR, INPUT_PULLUP);
  cubo_preso = leitura_anterior = digitalRead(PINO_SENSOR) == LOW;
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    // Define o pulso antes do attach para o servo nao ir ao padrao da biblioteca (1500 us).
    atual[i] = alvo[i] = PULSO_INICIAL;
    servos[i].writeMicroseconds(PULSO_INICIAL);
    servos[i].attach(PINOS[i], PULSO_MIN, PULSO_MAX);
  }
  // Fins de curso (chave para o GND, pull-up interno).
  pinMode(PINO_FC_ESQ, INPUT_PULLUP);
  pinMode(PINO_FC_DIR, INPUT_PULLUP);
  fc_esq = fc_esq_lido = digitalRead(PINO_FC_ESQ) == LOW;
  fc_dir = fc_dir_lido = digitalRead(PINO_FC_DIR) == LOW;
  // Plataforma Y comeca parada, sem sinal.
  cortar_sinal_y();
}

void loop() {
  atualizar_sensor();
  atualizar_fins_de_curso();
  passo_home(millis());
  // Andando para um lado e a chave desse lado apertou: para na hora (vale mesmo sem a serial).
  if (bloqueado_fc(pulso_y)) {
    aplicar_y(pulso_y, false);   // aplicar_y troca pelo neutro e avisa "Y FIM ..."
  }
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
  if ((pulso_y != PULSO_Y_PARADO || (servo_y.attached() && !freando_y)) &&
      agora - ultimo_comando_y > TIMEOUT_Y_MS) {
    aplicar_y(PULSO_Y_PARADO, false);
    Serial.println(F("Y TIMEOUT"));
  }
  if (freando_y && (long)(agora - freio_y_ate) >= 0) {
    cortar_sinal_y();
  }

  if (agora - ultimo_passo >= PERIODO_MS) {
    ultimo_passo = agora;
    for (uint8_t i = 0; i < N_SERVOS; i++) {
      if (atual[i] == alvo[i]) continue;
      int erro = alvo[i] - atual[i];
      if (erro > PASSO_US) erro = PASSO_US;
      if (erro < -PASSO_US) erro = -PASSO_US;
      atual[i] += erro;
      servos[i].writeMicroseconds(atual[i]);
    }
  }
}
