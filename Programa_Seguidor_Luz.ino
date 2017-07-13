/*
 * Code d'exemple pour une photorésistance.
 */

// Fonction setup(), appelée au démarrage de la carte Arduino
 int pinIN6 = 6;
  int pinIN7 = 9;
  const int buzzer = 12;
  
  
void setup() {

  // Initialise la communication avec le PC
  Serial.begin(9600);
  PWM_Mode_Setup();
  pinMode(pinIN6, OUTPUT);
  pinMode(pinIN7, OUTPUT);
   pinMode(buzzer, OUTPUT);
  
}
// Fonction loop(), appelée continuellement en boucle tant que la carte Arduino est alimentée


// # Editor    :Jiang from DFRobot
// # Data      :18.09.2012

// # Product name:ultrasonic scanner 
// # Product SKU:SEN0001
// # Version :  0.2

// # Description:
// # The Sketch for scanning 180 degree area 4-500cm detecting range

// # Connection:
// #       Pin 1 VCC (URM V3.2) -> VCC (Arduino)
// #       Pin 2 GND (URM V3.2) -> GND (Arduino)
// #       Pin 4 PWM (URM V3.2) -> Pin 3 (Arduino)
// #       Pin 6 COMP/TRIG (URM V3.2) -> Pin 5 (Arduino)
// #
int URPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
int URTRIG=5; // PWM trigger pin
 
unsigned int Distance=0;
uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};    // distance measure command
 

void loop()
{
 PWM_Mode();
 delay(20);
  // Mesure la tension sur la broche A0
  int valeur = analogRead(A1);
  int valeur2 = analogRead(A2);
  int diferencia = valeur2 - valeur;
  int speed1 = valeur/6;
  int beee = 7;
  if (Distance <= 4){
    analogWrite(pinIN6,0);
    analogWrite(pinIN7,0);
  }
  else {
   
 if (Distance <= 20){
     tone(buzzer, 1000); // Send 1KHz sound signal..

      if (valeur > valeur2){
     analogWrite(pinIN6,25);
      analogWrite(pinIN7,0);}
      else{
      analogWrite(pinIN6,0);
      analogWrite(pinIN7,25);  
      }
 }
   else{
    noTone(buzzer);
  if (diferencia >=-200 && diferencia <=200){
  Serial.println("frente");
  analogWrite(pinIN6,speed1);
  analogWrite(pinIN7,speed1);
  }
 
  else
  {if (valeur >= valeur2){
  Serial.println("derecha");
  analogWrite(pinIN6,50);
  analogWrite(pinIN7,0);
  }
  else {
  Serial.println("izquierda");
  analogWrite(pinIN6,0);
  analogWrite(pinIN7,50);
  }
  }
  
  // Envoi la mesure au PC pour affichage et attends 250ms
  Serial.println(valeur);
  Serial.println(valeur2);
  delay(200);
}                      //PWM mode setup function
 }
}
 
void PWM_Mode_Setup()
{ 
  pinMode(URTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                  // Set to HIGH
  
  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command
  
  for(int i=0;i<4;i++)
  {
      Serial.write(EnPwmCmd[i]);
  } 
}
 
void PWM_Mode()
{                              // a low pull on pin COMP/TRIG  triggering a sensor reading
    digitalWrite(URTRIG, LOW);
    digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses
     
    unsigned long DistanceMeasured=pulseIn(URPWM,LOW);
     
    if(DistanceMeasured>=10200)
    {              // the reading is invalid.
      Serial.println("Invalid");    
    }
    else
    {
      Distance=DistanceMeasured/50;           // every 50us low level stands for 1cm
      Serial.print("Distance=");
      Serial.print(Distance );
      Serial.println("cm");
     }


     

     

}
