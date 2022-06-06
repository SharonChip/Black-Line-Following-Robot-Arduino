#include <Wire.h>
#include <SparkFun_APDS9960.h>

#define ENA 9             
#define ENB 5
#define IN1 6
#define IN2 4
#define IN3 10
#define IN4 11
#define sen3 A3 // H
#define sen1 7 // M
#define sen2 3 // M
#define sen4 A1 // SR
#define sen5 A2 // SL 


int disPin1 = A0; 
int distance; 
float volt_val;

int last;

//---------- Proximity Sensor ----------
  SparkFun_APDS9960 apds = SparkFun_APDS9960();
  uint8_t proximity_data = 0; //dis sensor


  uint16_t ambient_light = 0;
  uint16_t red_light = 0;
  uint16_t green_light = 0;
  uint16_t blue_light = 0;

  int red = 13;
  int blue = 12;
  int green = 8;




void setup() {
  
  Serial.begin(9600);
  pinMode(ENA , OUTPUT);
  pinMode(ENB , OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2 , OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4 , OUTPUT);
  pinMode(sen1 , INPUT);
  pinMode(sen2 , INPUT);
  pinMode(sen3, INPUT);
  pinMode(sen4, INPUT);
  pinMode(sen5, INPUT);
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);

  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);


//---------- Proximity Sensor Setup ----------

  // Initialize APDS-9960 (configure I2C and initial values)
  if (apds.init()) {
    //Serial.println(F("APDS-9960 initialization complete"));
  } else {
    //Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

   // Start running the APDS-9960 proximity sensor (no interrupts)
  if (apds.enableProximitySensor(false)) {
    //Serial.println(F("Proximity sensor is now running"));
  } else {
    //Serial.println(F("Something went wrong during sensor init!"));
  }
  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    //Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
//---------- Color Sensor Setup ----------
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    //Serial.println(F("Light sensor is now running"));
  } else {
    //Serial.println(F("Something went wrong during light sensor init!"));
  }



}

void loop() {
  double A=1 , B=2 , C=2 , D=3 , E=3;
  
  Serial.print("Proximity: ");
  Serial.println(proximity_data);

// ---------- Distance & Color ----------
   apds.readProximity(proximity_data);

   while(proximity_data > 70){
    apds.readProximity(proximity_data);
    analogWrite(ENA,0);
    analogWrite(ENB,0);
    Serial.println("-----------------------------");
    Serial.print("Proximity: ");
    Serial.println(proximity_data);
    Serial.println("-----------------------------");
    
    apds.readBlueLight(blue_light);
    apds.readRedLight(red_light);
    apds.readGreenLight(green_light);
    if(red_light > green_light*0.8 && red_light > blue_light){
      digitalWrite(red, HIGH);
      digitalWrite(blue, LOW);
      digitalWrite(green, LOW);
    }else if(green_light*0.8 > red_light && green_light*0.8 > blue_light){
      digitalWrite(red, LOW);
      digitalWrite(blue, LOW);
      digitalWrite(green, HIGH);      
    }else if(blue_light > red_light && blue_light > green_light*0.8){
      digitalWrite(red, LOW);
      digitalWrite(blue, HIGH);
      digitalWrite(green, LOW);
    }
    Serial.print("Ambient: ");
    Serial.print(ambient_light);
    Serial.print(" Red: ");
    Serial.print(red_light);
    Serial.print(" Green: ");
    Serial.print(green_light);
    Serial.print(" Blue: ");
    Serial.println(blue_light);
    
   }
    digitalWrite(red, LOW);
    digitalWrite(blue, LOW);
    digitalWrite(green, LOW);
   /*
  if(proximity_data > 20 ){ //apds.readProximity(proximity_data) >= 110 && apds.enableProximitySensor(false)
  analogWrite(ENA,0); // left DC
  analogWrite(ENB,0); // right DC
  Serial.println("-----------------------------");
  Serial.print("Proximity: ");
  Serial.println(proximity_data);
  Serial.println("-----------------------------");
  }else{
  */


// ---------- Driving ----------
  
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  int senval1 = digitalRead(sen1);
  Serial.print("1: ");
  Serial.println(senval1);
  int senval2 = digitalRead(sen2);
  Serial.print("2: ");
  Serial.println(senval2);
  int senval3=digitalRead(sen3);
  Serial.print("3: ");
  Serial.println(senval3);
  int senval4=digitalRead(sen4);
  Serial.print("4: ");
  Serial.println(senval4);
  int senval5=digitalRead(sen5);
  Serial.print("5: ");
  Serial.println(senval5);

/*
  switch (last){
    case 0:
      analogWrite(ENB,150);
      analogWrite(ENA,150);
      Serial.println("last - forword");
      break;
    case 1:
      analogWrite(ENA,150);
      analogWrite(ENB,80);
      Serial.println("last - turn right");
      break;
    case 2:
      analogWrite(ENB,150);
      analogWrite(ENA,70); 
      Serial.println("last - turn left");
      break;
  }
  */


  if(senval1 == 0 && senval2 == 0 && senval3 == 1 && senval4 == 0 && senval5 == 0 || (senval4 == 1 && senval5 == 1)|| (senval1 == 1 && senval2 == 1)){
    analogWrite(ENB,170);
    analogWrite(ENA,170);
    Serial.println("forword");
    last = 0;
  }
  if(senval1 == 0 && senval2 == 0 && senval3 == 1 && senval4 == 0 && senval5 == 1){
    analogWrite(ENB,150);
    analogWrite(ENA,140);
    Serial.println("turn left");
    last = 2;
  }
    if(senval1 == 0 && senval2 == 0 && senval3 == 1 && senval4 == 1 && senval5 == 0){
    analogWrite(ENA,160);
    analogWrite(ENB,150);
    Serial.println("turn right");
    last = 1;
  }
  if(senval1 == 0 && senval2 == 0 && senval3 == 0 && senval4 == 1 && senval5 == 0){
    analogWrite(ENB,140);
    analogWrite(ENA,70); // 100
    Serial.println("turn sharp left");
    last = 2;
  }
  if(senval1 == 0 && senval2 == 0 && senval3 == 0 && senval4 == 0 && senval5 == 1){
    analogWrite(ENA,140);
    analogWrite(ENB,80); //90
    Serial.println("turn sharp right");
    last = 1;
  }
  if(senval1 == 0 && senval2 == 1 && senval3 == 0 && senval4 == 0 && senval5 == 0){
    analogWrite(ENA,0);
    analogWrite(ENB,120);
    Serial.println("turn 90 left");
    last = 2;
  }
  if(senval1 == 1 && senval2 == 0 && senval3 == 0 && senval4 == 0 && senval5 == 0){
    analogWrite(ENA,120);
    analogWrite(ENB,0);
    Serial.println("turn 90 right");
    last = 1;
  }
  if(senval1 == 1 && senval2 == 0 && senval3 == 1 && senval4 == 1 && senval5 == 0){
    analogWrite(ENA,130);
    analogWrite(ENB,100);
    Serial.println("turn zigzag right");
    last = 1;
  }
    if(senval1 == 0 && senval2 == 1 && senval3 == 1 && senval4 == 0 && senval5 == 1){
    analogWrite(ENA,150);
    analogWrite(ENB,110);
    Serial.println("turn zigzag left");
    last = 2;
  }

}
