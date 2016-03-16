

/*
  This program controls the Followers on the bicycle field experiments.
  The code was develope with five main branches and its been assemble on a single sketch, those branches are

  --Speedometer code, responsible for the speed camculations and sensor, it uses a single interrupt and some timers
  --Handlebars HCI motor controller, this code controls the speed and direction os the motor to comunicate the rider acceleration instructions
  --Xbee comunication module: responscible for sending ans recieving data, it will use AT mode and strings to send data and recieve instructions
  --GPS reading: the sole proporse its to update GPS readings from the GPS module once a second
  --Control Formula
  Hardware and software developed at HCI Lab - ICESI university -  Cali, Colombia, by Juan Manuel Salamanca and Daniel Vinasco
*/
//------------------------LIBRARIES------------------------\\

//GPS
#include <SoftwareSerial.h> //viertual serial for the GPS
#include <TinyGPS++.h> //Helps with the handling of the GPS
#include <math.h>

//------------------------VARIABLES------------------------\\

//Debbugging
unsigned long time3; //keep track for USB serial COM
int reportCycle;// how often it sends data to USB serial com
int blueLedPin = 3; //has send or recieve setup string, Blue LED
int whiteLedPin = A0; //gps is on and working, White LED
int greenLedPin = A1;// got sat signal and coords, Green LED
int redLedPin = A2; //has speed for control Formula RED LED


//Speedometer
unsigned long time1; // Control for debouncing the sensor and tracking speed
double   mySpeed;// The actual speed calculated each interruption
int sensorPIN = 2; // this must be one of the interruptables 0,1,2,3,7
double circuit = 1.52;// wheel diameter in cm to calculate covered distance vs time on each lap
double minSpeed = 2.75;

//Handlebar Motor Variables
int E2 = 6;    // Speed Control - aslo the pin number
int M2 = 7;    // Direction Control - also the pin number

//Xbee Com
String beaconMsg;
unsigned long time4;
unsigned long timerBeaconLeader;

//GPS
SoftwareSerial serialGPS(8, 9); // Creates virtual serial for the GPS (RX, TX) pins, must jump gps pins (0,1) to Arduino pins (8,9)
TinyGPSPlus gps; //GPS object
double myLat; //My latitud
double myLng; //My longitud
int gpsOnOffPIN = 12; //this pin switch the gps on and off (one HIGH with switch)
unsigned long time2;
bool loopLights = true;

//control Formula

// 1. General variables
double totalDistance = 0; // Total distance
double lastAccelerationPlatoon = 0; // Last acceleration calculated with CACC
double localLeaderAcceleration = 0; // Acceleration of leader
double localLeaderSpeed = 0; // // Speed of leader

//2 . Platoon's parameters (CACC)
double  alpha1 = 0.5;
double alpha2 = 0.5;
double alpha3 = 0.3;
double alpha4 = 0.1;
double alpha5 = 0.04;
double alphaLag = 0.8; // para aterrisar la aceleracion
double length_vehicle_front = 2; //ancho de la bici
double desiredSpacing = 4;//variable programable via botones
int beaconInterval = 1000; // How often a beacon is sent
int platoonInterval = 5000;// how often the desire acceleration its calculated
// Platoon Size its 4

int bikeID = 1; // bikes 1 throw 3 are fallowers, and 0 the leader
double bikeAcc[3];
double bikeLat[3];
double bikeLng[3];
double bikeSpeed[3];

double bikeAccL = 0;
double bikeLatL = 0;
double bikeLngL = 0;
double bikeDistL = 0;
double bikeSpeedL = 0;

int rel_speed_front = 0; //velocidad relativa con respecto a la bici del frente
int closestBike = bikeID - 1;
unsigned long time5;
const double Pi = 3.141593;

//setup
bool setupLoop = true;
//------------------------SETUP------------------------\\

void setup() {
  //debugging
  reportCycle = 1000;
  Serial.begin(9600); //USB serial
  time3 = millis();

  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(whiteLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  //speedometer Setup
  pinMode(sensorPIN, INPUT);
  attachInterrupt(1, speedSensor, RISING);//interrupt for pin 2
  time1 = millis();

  //Handlebar Motor
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);

  //Xbee Com
  Serial1.begin(9600); //XBee serial1
  time4 = millis();

  //GPS
  serialGPS.begin(4800); //GPS SoftSerial


  pinMode(gpsOnOffPIN, OUTPUT); //starts GPS
  digitalWrite(gpsOnOffPIN, HIGH);
  delay(500);
  digitalWrite(gpsOnOffPIN, LOW);
  time2 = millis();

  //control Formula
  time5 = millis();
}
//------------------------Program Loop------------------------\\

void loop() {
  //setup
  while (setupLoop)
    setMeUp();


  //speedometer
  speedometer();

  //GPS
  readFromGPS();
  //xbee
  readBeacons();
  sendBeacon();



  if (millis() - time5 > platoonInterval&&!loopLights) {//this loop will start after sat signal its confirm at the intervals
    time5 = millis();
    if (mySpeed > minSpeed) {
      getAccelerationPlatoon();
    } 
    
  }
  if (lastAccelerationPlatoon < -1 &&!loopLights) {
    motorControl(1)   ;
    digitalWrite(greenLedPin, LOW);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(redLedPin, HIGH);
  } 
  if (lastAccelerationPlatoon > 1 &&!loopLights) {
    motorControl(2) ;
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(redLedPin, LOW);
  }
  if (lastAccelerationPlatoon >= -1 && lastAccelerationPlatoon <= 1&&!loopLights&&mySpeed>0) {
    motorControl(3) ;
    digitalWrite(greenLedPin, LOW);
    digitalWrite(blueLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
  }
  if (mySpeed==0) {
    motorControl(3) ;
    digitalWrite(greenLedPin, LOW);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    lastAccelerationPlatoon=0;
  }

}
//------------------------Functions------------------------\\

//debugging


//speedometer
void speedSensor() {// this fuction its executeded each time the sensor interrupts arduino

  if (millis() - time1 > 100) {
    mySpeed = (circuit * 1000) / ((millis() - time1));
    if (millis() - time1 > 3000)
      mySpeed = 0;
    time1 = millis();
  }
}
void speedometer() {
  if (millis() - time1 > 2000) //checks if the wheel hasn't move for 2 seconds and assumes it has stopt, resert speed to 0
    mySpeed = 0;
  if (mySpeed > minSpeed) {
    digitalWrite(whiteLedPin, HIGH);
  } else {
    digitalWrite(whiteLedPin, LOW);
  }
}

//Handlebar Motor

void motorControl(int dirMotor) // controla direccion del motor donde 1 es adelante, 2 atras y 3 parado
{
  if (dirMotor == 1) { //motor arranca adelante
    digitalWrite(M2, HIGH);
    analogWrite (E2, 150);

  }
  if (dirMotor == 2) { //motor atras
    digitalWrite(M2, LOW);
    analogWrite (E2, 150);

  }
  if (dirMotor == 3) {
    digitalWrite(M2, LOW);
    analogWrite (E2, 0);
  }
}

//Xbee Com
void sendBeacon()// pruebas de envio de datos por serial un char a la vez
{
  if (millis() - time4 > (beaconInterval - 1)) {
    time4 = millis();
    Serial1.print(bikeID + String(myLat, 6) + "|" + String(myLng, 6) + "|" + String(mySpeed, 4) + "*" + String(lastAccelerationPlatoon, 2)+ "#") ; //leader also sends its timer millis() after the
    Serial.println(bikeID + String(myLat, 6) + "|" + String(myLng, 6) + "|" + String(mySpeed, 4) + "*" + String(lastAccelerationPlatoon, 2)+ "#") ; //leader also sends its timer millis() after the
    
    Serial1.flush();
  }
}
void readBeacons() {

  int beaconFrom;
  int index1;
  long timerTemp;
  String linea;

  if (Serial1.available()) {
    linea = Serial1.readStringUntil('#');
    //linea.remove(0,1);
    beaconFrom = linea.charAt(0);
    if (beaconFrom == '0') {
      index1 = linea.lastIndexOf('*');
      timerTemp = (linea.substring(index1 + 1, linea.length())).toFloat();

      if  (timerTemp > timerBeaconLeader + beaconInterval) {
        timerBeaconLeader = timerTemp;
        //Serial1.print(linea+"#");
        //Serial.println("beacon resend");
      } else {
        beaconFrom = 666;
      }

    }
    linea.remove(0, 1);

    if (beaconFrom == '0' || beaconFrom == '1' || beaconFrom == '2' || beaconFrom == '3') {


      int bc = beaconFrom - '0';

      index1 = linea.indexOf('|');
      bikeLat[bc] = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      bikeLng[bc] = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('*');
      bikeSpeed[bc] = (linea.substring(0, index1)).toFloat();

    }
    if (beaconFrom == 'S') {
      //setupreading
      linea.remove(0, 1);

      index1 = linea.indexOf('|');
      desiredSpacing = (linea.substring(0, index1)).toFloat();
      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      beaconInterval = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      platoonInterval = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);

      index1 = linea.indexOf('|');
      minSpeed = (linea.substring(0, index1)).toFloat();

      linea.remove(0, index1 + 1);
      for (int i = 0; i < bikeID; i++) {
        index1 = linea.indexOf('|');
        circuit = (linea.substring(0, index1)).toFloat();
        linea.remove(0, index1 + 1);
      }
    }

  }

}
//GPS
void readFromGPS() {//reads gps serial and updates position information

  if (millis() - time2 > 999) {
    while (serialGPS.available())
    {
      gps.encode(serialGPS.read());
      if (loopLights) {
        digitalWrite(whiteLedPin, HIGH);
      }
    }


    myLat = fabs(gps.location.lat());
    myLng = fabs(gps.location.lng());
    time2 = millis();
    
    
    
    //myLat = 3.340335;
    // myLng =fabs(-76.528955);
    
    
    
    if (myLat != 0 && loopLights) {
      for (int i = 0; i < 12; i++) {
        digitalWrite(greenLedPin, HIGH);
        digitalWrite(whiteLedPin, HIGH);
        digitalWrite(blueLedPin, HIGH);
        digitalWrite(redLedPin, HIGH);
        delay(250);
        digitalWrite(greenLedPin, LOW);
        digitalWrite(whiteLedPin, LOW);
        digitalWrite(blueLedPin, LOW);
        digitalWrite(redLedPin, LOW);
        delay(250);
      }
      loopLights = false;
    }

  }
}

//control Formula


void getAccelerationPlatoon() {
  // 1. PREPARE

  double dY;
  double dX;

  double distanceBetweenActualAndFront;
  double minDist;

  double nodeFrontAcceleration;
  // 2. START PLATOON

  //Calculate distance between nodes and current to determine the nearest to the current

  dX = fabs(bikeLat[closestBike] - myLat) * (Pi / 180) * 6356750;
  dY = (bikeLng[closestBike] - myLng) * (Pi / 180) * (6378200 * cos((bikeLat[closestBike] + myLat) / 2));



  minDist = sqrt(exp(dX) + exp(dY));


  double  spacing_error = -minDist + length_vehicle_front + desiredSpacing;
  //d. Calculate (Acceleration desired) A_des /// esta es la formula que calcula la aceleracion deserada
  double A_des = alpha1 * bikeAcc[closestBike] + alpha2 * bikeAccL - alpha3 * rel_speed_front - alpha4 * (mySpeed - bikeSpeedL) - alpha5 * spacing_error;
  Serial.println("a_des " + String(A_des, 6));
  //e. Calculate desired acceleration adding a delay //acelerecacion realista
  lastAccelerationPlatoon = (alphaLag * A_des) + ((1 - alphaLag) * lastAccelerationPlatoon);


  Serial.flush();
  Serial.println("acc platoon " + String(lastAccelerationPlatoon, 6));

}
//setup
void setMeUp() {

  int beaconFrom;
  int index1;
  String linea;

  if (Serial1.available()) {
    linea = Serial1.readStringUntil('#');
    beaconFrom = linea.charAt(0);
    //  Serial.println(linea);
    if (beaconFrom == 'S') {
      //setuprerading
      linea.remove(0, 1);
      Serial.println(linea);
      index1 = linea.indexOf('|');
      desiredSpacing = (linea.substring(0, index1)).toFloat();
      Serial.println(desiredSpacing);
      linea.remove(0, index1 + 1);
      Serial.println(linea);
      index1 = linea.indexOf('|');
      beaconInterval = (linea.substring(0, index1)).toFloat();
      Serial.println(beaconInterval);
      linea.remove(0, index1 + 1);
      Serial.println(linea);
      index1 = linea.indexOf('|');
      platoonInterval = (linea.substring(0, index1)).toFloat();
      Serial.println(platoonInterval);
      linea.remove(0, index1 + 1);
      Serial.println(linea);
      index1 = linea.indexOf('|');
      minSpeed = (linea.substring(0, index1)).toFloat();
      Serial.println(minSpeed);
      linea.remove(0, index1 + 1);
      Serial.println(linea);
      for (int i = 0; i < bikeID; i++) {
        index1 = linea.indexOf('|');
        circuit = (linea.substring(0, index1)).toFloat();
        linea.remove(0, index1 + 1);
      }
      Serial.println(circuit);
      setupLoop = false;
      digitalWrite(blueLedPin, HIGH);

    }
  }
}
