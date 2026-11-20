#include <Servo.h>

Servo motor;

const int pinoServo = 13;

int pulso = 1520;

void setup() {
  Serial.begin(9600);

  motor.attach(pinoServo);
  motor.writeMicroseconds(pulso);

  Serial.println("CALIBRACAO DS3218");
  Serial.println("Digite:");
  Serial.println("1 - Aumentar pulso");
  Serial.println("2 - Diminuir pulso");
  Serial.println("3 - Parar (1500 us)");
  Serial.println("4 - Mostrar valor atual");
}

void loop() {

  if (Serial.available()) {

    char comando = Serial.read();

    if (comando == '1') {
      pulso += 5;
    }

    else if (comando == '2') {
      pulso -= 5;
    }

    else if (comando == '3') {
      pulso = 1500;
    }

    else if (comando == '4') {
      Serial.print("Pulso atual: ");
      Serial.println(pulso);
    }

    pulso = constrain(pulso, 1000, 2000);

    motor.writeMicroseconds(pulso);

    Serial.print("Pulso enviado: ");
    Serial.println(pulso);
  }
}