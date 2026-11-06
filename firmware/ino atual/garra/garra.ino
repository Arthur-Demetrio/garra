#include <Servo.h>

// Cria os objetos para os três motores
Servo motor1;
Servo motor2;
Servo motor3;

// Define os pinos digitais do Arduino Mega conectados ao sinal dos servos
const int pinoMotor1 = 9;
const int pinoMotor2 = 10;
const int pinoMotor3 = 11;
const int INTERVALO_MOVIMENTO_MS = 40;

void moverSuavemente(Servo &motor, int alvo) {
  int atual = motor.read();
  int passo = alvo >= atual ? 1 : -1;

  for (int posicao = atual; posicao != alvo; posicao += passo) {
    motor.write(posicao);
    delay(INTERVALO_MOVIMENTO_MS);
  }
  motor.write(alvo);
}

void setup() {
  Serial.begin(9600);

  // Anexa cada motor ao seu respectivo pino
  motor1.attach(pinoMotor1);
  motor2.attach(pinoMotor2);
  motor3.attach(pinoMotor3);

  Serial.println("CONTROLE DE 3 SERVOS (0 a 180 GRAUS)");
  Serial.println("Arduino Mega 2560");
  Serial.println("-----------------------------------------");
  Serial.println("Comando: Digite [NumeroDoMotor],[Angulo]");
  Serial.println("Exemplo: 1,90 (Move o motor 1 para 90 graus)");
  Serial.println("Exemplo: 2,180 (Move o motor 2 para 180 graus)");
  Serial.println("-----------------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    // Lê o número do motor escolhido (1, 2 ou 3)
    int numMotor = Serial.parseInt();
    
    // Lê a vírgula separadora
    if (Serial.read() == ',') {
      int angulo = Serial.parseInt();

      // Valida se o ângulo está entre 0 e 180
      if (angulo >= 0 && angulo <= 180) {
        
        switch (numMotor) {
          case 1:
            moverSuavemente(motor1, angulo);
            Serial.print("-> Motor 1 posicionado em: ");
            Serial.print(angulo);
            Serial.println("°");
            break;
            
          case 2:
            moverSuavemente(motor2, angulo);
            Serial.print("-> Motor 2 posicionado em: ");
            Serial.print(angulo);
            Serial.println("°");
            break;
            
          case 3:
            moverSuavemente(motor3, angulo);
            Serial.print("-> Motor 3 posicionado em: ");
            Serial.print(angulo);
            Serial.println("°");
            break;
            
          default:
            Serial.println("Erro: Motor invalido! Escolha 1, 2 ou 3.");
            break;
        }

      } else {
        Serial.println("Erro: Angulo invalido! Digite entre 0 e 180.");
      }
    }

    // Limpa o buffer restante da serial
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}