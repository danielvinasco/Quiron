/*

   <==QUIRON=====]}============
          --.   /|
         _\"/_.'/
       .'._._,.'
       :/ \{}/
      (L  /--',----._
          |          \\
         : /-\ .'-'\ / |
          \\, ||    \|
           \/ ||    ||

  This program controls the Leader on the Bicycle field experiments.
  the code was develope with four main branches and its been assemble on a single sketch, those branches are

  --Speedometer code, responsible for the speed camculations and sensor, it uses a single interrupt and some timers
  --Handlebars HCI motor controller, this code controls the speed and direction os the motor to comunicate the rider acceleration instructions
  --Xbee comunication module: responscible for sending ans recieving data, it will use AT mode and strings to send data and recieve instructions
  --GPS reading: the sole proporse its to update GPS readings from the GPS module once a second

  Hardware and software developed at HCI Lab - ICESI university -  Cali, Colombia, by Juan Manuel Salamanca and Daniel Vinasco
*/
//------------------------LIBRARIES------------------------\\

//GPS
#include <SoftwareSerial.h> //viertual serial for the GPS
#include <TinyGPS++.h> //Helps with the handling of the GPS

//------------------------VARIABLES------------------------\\

//Debbugging
unsigned long  time5; //keep track for USB serial COM
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
double minSpeed = 2.75; // algorithm starts to calculate acceleration at this speed
double wheels[4] = {1.52, 1.52, 1.52, 1.52};

//Xbee Com
String beaconMsg;
long time4;
unsigned long timerBeaconLeader;
 String linea;

//GPS
SoftwareSerial serialGPS(8, 9); // Creates virtual serial for the GPS (RX, TX) pins, must jump gps pins (0,1) to Arduino pins (8,9)
TinyGPSPlus gps; //GPS object
String myLat; //My latitud
String myLng; //My longitud
int gpsOnOffPIN = 12; //this pin switch the gps on and off (one HIGH with switch)
long time2;
bool loopLights = true;



// Platoon Size its 4

int bikeID = 0; // bikes 1 throw 3 are fallowers, and 0 the leader
int beaconInterval = 1000;
int platoonInterval = 5000;
int desiredSpacing = 4;
//setup
bool setupLoop = true;    


//------------------------SETUP------------------------\\

void setup() {
  //debugging
  reportCycle = 1000;
  Serial1.begin(9600); //USB serial
  time5 = millis();

  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(whiteLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  digitalWrite(greenLedPin, LOW);
    digitalWrite(blueLedPin, LOW);
    digitalWrite(redLedPin, LOW);
    digitalWrite(whiteLedPin, LOW);

  //speedometer Setup

  pinMode(sensorPIN, INPUT);
  attachInterrupt(1, speedSensor, RISING);//interrupt for pin 2
  time1 = millis();


  //Xbee Com
  Serial1.begin(9600); //XBee serial1


  //GPS
  serialGPS.begin(4800); //GPS SoftSerial

  pinMode(gpsOnOffPIN, OUTPUT); //starts GPS
  digitalWrite(gpsOnOffPIN, HIGH);
  delay(500);
  digitalWrite(gpsOnOffPIN, LOW);
  time2 = millis();
  time4 = millis();

  //setup
  setupLoop = true;


  //digitalWrite(blueLedPin,HIGH);
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

  sendBeacon();
//  readBeacons();
}

//------------------------Functions------------------------\\

//debugging

void sendToserial(String texto) {//send text to serial one each second, for debugging
  if (millis() - time5 > reportCycle) {
    time5 = millis();
    Serial.println(texto);

  }
}

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

  if (mySpeed >= minSpeed) {
        digitalWrite(redLedPin, HIGH);
        digitalWrite(blueLedPin, HIGH);
        digitalWrite(greenLedPin, HIGH);
        digitalWrite(whiteLedPin, HIGH);
  }else{
        digitalWrite(redLedPin, LOW);
        digitalWrite(blueLedPin, LOW);
        digitalWrite(greenLedPin, LOW);
        digitalWrite(whiteLedPin, LOW);
  }
}


//Xbee Com
void sendBeacon()// pruebas de envio de datos por serial un char a la vez
{
  if (millis() - time4 > beaconInterval - 1) {
    time4 = millis();
    Serial1.print(bikeID + myLat + "|" + myLng + "|" + mySpeed + '*' + millis() + '#'); //leader also sends its timer millis() after the
    //Serial.println(bikeID + myLat+ "|" + myLng + "|" + mySpeed + '*' + millis() + '#'); //leader also sends its timer millis() after the
    Serial1.flush();
  
  }
}
void readBeacons() {
   if(Serial1.available()){
    linea = Serial1.readString();
    Serial.println(linea);
  }
}


//GPS
void readFromGPS() {//reads gps serial and updates position information

  if (millis() - time2 > 999) {
    time2 = millis(); 
    while (serialGPS.available())
    {
      gps.encode(serialGPS.read());
       if(loopLights)
       digitalWrite(whiteLedPin, HIGH);
    }

    myLat = String(fabs(gps.location.lat()),6);
    myLng = String(fabs(gps.location.lng()),6);
    Serial.println(myLat+ "|" + myLng +" * "+ millis()); //debug
   //     myLat = "3.340138";
   // myLng = "76.528947";
    if (myLat !="0.000000"&& loopLights) {

         for (int i = 0; i < 12; i++) {
        digitalWrite(redLedPin, HIGH);
        digitalWrite(blueLedPin, HIGH);
        digitalWrite(greenLedPin, HIGH);
        digitalWrite(whiteLedPin, HIGH);
        delay(250);
        digitalWrite(redLedPin, LOW);
        digitalWrite(blueLedPin, LOW);
        digitalWrite(greenLedPin, LOW);
        digitalWrite(whiteLedPin, LOW);
        delay(250);
      }
      loopLights = false;
    }
  }

    

}
//setup
void setMeUp() {
  bool loop1 = true;
  String lineas;
  char peeked;
  double tempDiameter;

  //desiredSpacing;
  //beaconInterval;
  //platoonInterval;
  while (Serial.available() == 0) {
    Serial.println("Press any key to start");
    delay(2000);

  }

  peeked = Serial.read();
  while (loop1) {
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("   +++++++++++++++++++++");
    Serial.println("   + Welcome to Quiron +");
    Serial.println("   +++++++++++++++++++++");
    Serial.println("Bike platoon control software");
    Serial.println("");
    Serial.println("  ----Setup---- ");
    Serial.println("Desired Spacing:          " + String(desiredSpacing) + " mts");
    Serial.println("Bikes beacon interval:    " + String(beaconInterval) + " msec");
    Serial.println("Control formula interval: " + String(platoonInterval) + " msec");
    Serial.println("Would you like to make changes? y/n");
    while ((peeked != 'y') && (peeked != 'n')) {
      peeked = Serial.read();
      //    Serial.println("no no no");
    }

    if (peeked == 'y') {
      lineas = Serial.readString();
      Serial.println("");
      Serial.println("");
      Serial.println("");
      Serial.println("");
      Serial.println("Desired Spacing:          ?");
      while (Serial.available() == 0) {
      }
      lineas = Serial.readString();
      desiredSpacing = lineas.toInt();
      Serial.println("");

      Serial.println(String(desiredSpacing) + " mts");
      Serial.println("");
      Serial.println("");
      Serial.println("Bikes beacon interval:    ?");
      while (Serial.available() == 0) {
      }
      lineas = Serial.readString();
      beaconInterval = lineas.toInt();
      Serial.println("");
      Serial.println(String(beaconInterval) + " msec");
      Serial.println("");
      Serial.println("");
      Serial.println("Control formula interval: ?");
      Serial.println("");
      while (Serial.available() == 0) {
      }
      lineas = Serial.readString();
      platoonInterval = lineas.toInt();
      Serial.println("");

      Serial.println(String(platoonInterval) + " msec");
      Serial.println("");
      Serial.println("");
      Serial.println("Test minimum speed: ?");
      Serial.println("");
      while (Serial.available() == 0) {
      }
      lineas = Serial.readString();
      minSpeed = lineas.toFloat();
      Serial.println(String(minSpeed) + " mts/sec");
/*
      for (int i = 0; i < 4; i++) {
        Serial.println("");
        Serial.println("");
        Serial.println("Wheel diameter for bike " + String(i) + ": ? in mm∂");
        while (Serial.available() == 0) {
        }
        lineas = Serial.readString();
        tempDiameter = lineas.toFloat();
        wheels[i] = 6.2831 * (tempDiameter / 2000);
        Serial.println("");

        Serial.println("Diameter: " + String(tempDiameter) + "mm circumference: " + String(wheels[i] ) + " mts");
        Serial.println("");
      }*/
      peeked = 'a';
    } else {

      loop1 = false;
      setupLoop = false;

      Serial1.println("S" + String(desiredSpacing) + "|" + String(beaconInterval) + "|" + String(platoonInterval) + "|" + String(minSpeed,2) + "|" + String(wheels[1],2) + "|" + String(wheels[2],2) + "|" + String(wheels[3],2) +  "#");
      circuit = wheels[0];
      digitalWrite(blueLedPin, HIGH);
    }

  }


  Serial.println("  <==QUIRON=====]}============");
  Serial.println("          --.   /|");
  Serial.println("         _\\\"/_.'/\\");
  Serial.println("       .'._._,.'");
  Serial.println("       :/ \\{}/");
  Serial.println("      (L  /--',----._");
  Serial.println("          |          \\");
  Serial.println("         : /-\\ .'-'\\ / |");
  Serial.println("          \\\\, ||    \\|");
  Serial.println("           \\/ ||    ||");

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.print("S" + String(desiredSpacing) + "|" + String(beaconInterval) + "|" + String(platoonInterval) + "|" + String(minSpeed) + "|" + String(wheels[1]) + "|" + String(wheels[2]) + "|" + String(wheels[3]) +  "#");
  Serial.println("");
  Serial.println("");

}
