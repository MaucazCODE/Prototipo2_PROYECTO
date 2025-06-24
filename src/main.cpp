#include <Wire.h>
#include <VL53L0X.h>
#include <AccelStepper.h>

#define dirPin 3
#define stepPin 4
#define Enable 5

int M1_0 = 15;
int M1_1 = 2;
int M1_2 = 4;
int M1_3 = 5;

int M2_0 = 13;
int M2_1 = 12;
int M2_2 = 14;
int M2_3 = 27;


int M3_0 = 26;
int M3_1 = 25;
int M3_2 = 33;
int M3_3 = 35;

int del = 5;


AccelStepper motorIZQ(AccelStepper::FULL4WIRE, M1_0, M1_1, M1_2, M1_3);
AccelStepper motorDER(AccelStepper::FULL4WIRE, M2_0, M2_1, M2_2, M2_3);
AccelStepper motorLIDAR(AccelStepper::FULL4WIRE, M3_0, M3_1, M3_2, M3_3);

enum estado_rumba{
  GIRANDO,AVANZANDO,DETENIDO
};
estado_rumba estado = GIRANDO;


float Pasos_por_grado = 200;
float Pasos_por_mm = 500;



VL53L0X sensor;

int valorDelay = 1200;
float paso_angulo = 2.4;
float angulo_max = 360.0;
int pasos_por_vuelta = angulo_max / paso_angulo;

float distancia_maxima = 0;
float angulo_objetivo = 0;

//parametros de prueba
float distancia_prueba = 100;
float angulo_prueba= 0;


int lidarDelta = 200;  // la distancia a moverse desde el centro
int lidarTarget = 200; // posición objetivo inicial relativa
bool movingPositive = true; // bandera para dirección de movimiento


void avanzando(float mm) {
 //uno 
  long steps = mm * Pasos_por_mm;

  motorDER.moveTo(motorDER.currentPosition()+steps);
  motorIZQ.moveTo(motorIZQ.currentPosition()+steps);

}


void volteandoIZQ(float grados) {
 //uno 
  long steps = grados * Pasos_por_grado;

  motorDER.moveTo(motorDER.currentPosition()+steps);
}


void volteandoDER(float grados) {
 //uno 
  long steps = grados * Pasos_por_grado;

  motorIZQ.moveTo(motorIZQ.currentPosition()+steps);
}


void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");  
  motorDER.setMaxSpeed(1000);
  motorDER.setAcceleration(500);
  motorIZQ.setMaxSpeed(1000);
  motorIZQ.setAcceleration(500);


  motorLIDAR.setMaxSpeed(1000);
  motorLIDAR.setAcceleration(500);
  motorLIDAR.moveTo(100);
  printf("radar posicion inicial");

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, LOW);
  digitalWrite(dirPin, HIGH);  // Sentido horario

  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!sensor.init()) {
    Serial.println("Sensor no detectado.");
    while (1);
  }

  sensor.setTimeout(500);
  sensor.setSignalRateLimit(0.1);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  sensor.setMeasurementTimingBudget(20000);

  Serial.println("Iniciando escaneo de 360°...");

  // Escanear 360°
  for (int i = 0; i < pasos_por_vuelta; i++) {
    int distancia = sensor.readRangeSingleMillimeters();

    if (distancia == 0) continue; // Ignorar lecturas erróneas
    if (distancia > distancia_maxima) {
      distancia_maxima = distancia;
      angulo_objetivo = i * paso_angulo;
    }

    // Enviar datos por serial en formato CSV
    Serial.print(i * paso_angulo, 1);
    Serial.print(",");
    Serial.print(distancia);
    Serial.println(",mm");

    // Avanzar motor
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(valorDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(valorDelay);
  }

  // Imprimir resultado final
  Serial.println("ESCANEO COMPLETO");
  Serial.print("Punto más lejano detectado: ");
  Serial.print(distancia_maxima);
  Serial.print(" mm en ángulo ");
  Serial.print(angulo_objetivo, 1);
  Serial.println("°");

  // Girar motor hacia el ángulo del punto más lejano
  int pasos_a_regresar = round((angulo_max - angulo_objetivo) / paso_angulo);
  digitalWrite(dirPin, LOW);  // Sentido antihorario

  for (int i = 0; i < pasos_a_regresar; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(valorDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(valorDelay);
  }

  Serial.print("Motor reposicionado al ángulo objetivo: ");
  Serial.print(angulo_objetivo, 1);
  Serial.println("°");

  //angulo_objetivo cuando trabaje
  //angulo_prueba para probar
  printf("empezando volteo");
  volteandoIZQ(angulo_objetivo);

}




void loop() {
  motorDER.run();
  motorIZQ.run();
  motorLIDAR.run();
  
  
  if (motorDER.distanceToGo() == 0 && motorIZQ.distanceToGo() == 0 && motorLIDAR.distanceToGo() == 0) {
    if (estado == GIRANDO) {
      Serial.println("Giro completo. Avanzando...");
      //distancia_maxima funcionamiento
      //distancia_prueba para prueba 
      avanzando(distancia_prueba);
      estado = AVANZANDO;
    } else if (estado == AVANZANDO) {
      Serial.println("Avance completo. Detenido.");
      estado = DETENIDO;
    }
  }
    
  
    

  /* //distancia_maxima funcionamiento
      //distancia_prueba para prueba Q
      //volteandoIZQ();
      // Oscilación del LIDAR
      if (movingPositive) {
        lidarTarget = -lidarDelta;
      } else {
        lidarTarget = lidarDelta;
      }

      motorLIDAR.moveTo(lidarTarget);
      if (motorLIDAR.currentPosition() == lidarTarget){
        movingPositive = !movingPositive;
      }

      Serial.printf("Moviendo LIDAR a: %d\n", lidarTarget);
  */

}