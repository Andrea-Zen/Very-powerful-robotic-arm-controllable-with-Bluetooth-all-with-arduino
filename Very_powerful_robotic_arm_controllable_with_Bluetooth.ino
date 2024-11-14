#include <SoftwareSerial.h>  // Libreria per la comunicazione seriale con modulo Bluetooth
#include <Wire.h>            // Libreria per comunicazione I2C
#include <MPU6050.h>         // Libreria per il sensore MPU6050
#include <Servo.h>           // Libreria per i servomotori

// Inizializzazione del modulo Bluetooth
SoftwareSerial bluetooth(10, 11);  // Pin RX, TX

// Inizializzazione del sensore MPU6050
MPU6050 mpu;

// Dichiarazione dei servomotori
Servo servoBase, servoShoulder, servoElbow, servoGripper;

// Angoli per i movimenti dei servomotori
int angleBase = 90;
int angleShoulder = 90;
int angleElbow = 90;
int angleGripper = 90;

// LED RGB
const int redPin = 6;
const int greenPin = 5;
const int bluePin = 3;

// Variabile per il comando Bluetooth
char bluetoothCommand;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);  // Inizia la comunicazione Bluetooth
  Wire.begin();            // Inizia la comunicazione I2C
  mpu.initialize();        // Inizializza il sensore MPU6050

  // Collegamento dei servomotori ai pin
  servoBase.attach(3);
  servoShoulder.attach(4);
  servoElbow.attach(5);
  servoGripper.attach(6);

  // Inizializzazione dei pin LED RGB
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Posizione di default dei servomotori
  resetPosition();
}

void loop() {
  handleBluetoothCommands();  // Gestisci i comandi Bluetooth
  handleGestures();           // Gestisci i gesti tramite MPU6050
  delay(10);                  // Ritardo per stabilità
}

// Funzione per gestione dei comandi Bluetooth
void handleBluetoothCommands() {
  if (bluetooth.available()) {
    bluetoothCommand = bluetooth.read();

    switch (bluetoothCommand) {
      case 'O':  // Apri pinza
        angleGripper = 180;
        servoGripper.write(angleGripper);
        break;
      case 'C':  // Chiudi pinza
        angleGripper = 0;
        servoGripper.write(angleGripper);
        break;
      case 'R':  // Reset posizione
        resetPosition();
        break;
      default:
        break;
    }
  }
}

// Funzione per interpretare i movimenti del sensore MPU6050
void handleGestures() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  angleBase = map(ax, -17000, 17000, 0, 180);
  angleShoulder = map(ay, -17000, 17000, 0, 180);
  angleElbow = map(az, -17000, 17000, 0, 180);

  servoBase.write(constrain(angleBase, 10, 170));
  servoShoulder.write(constrain(angleShoulder, 10, 170));
  servoElbow.write(constrain(angleElbow, 10, 170));

  updateLED();  // Aggiorna LED RGB in base alla posizione
}

// Funzione per aggiornare i colori del LED RGB
void updateLED() {
  if (angleBase < 45) {
    setLED(255, 0, 0); // Rosso: inclinato a sinistra
  } else if (angleBase > 135) {
    setLED(0, 0, 255); // Blu: inclinato a destra
  } else {
    setLED(0, 255, 0); // Verde: posizione centrale
  }
}

// Funzione per impostare i colori del LED
void setLED(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

// Funzione per reimpostare i servomotori alla posizione iniziale
void resetPosition() {
  angleBase = 90;
  angleShoulder = 90;
  angleElbow = 90;
  angleGripper = 90;

  servoBase.write(angleBase);
  servoShoulder.write(angleShoulder);
  servoElbow.write(angleElbow);
  servoGripper.write(angleGripper);
  setLED(255, 255, 255);  // Bianco: posizione di reset
}
