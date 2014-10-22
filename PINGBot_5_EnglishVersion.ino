/*
#####################################################################################
#  	Arquivo:            PINGBot_5.pde                                             
#       Micro-processador:  Arduino UNO ou Teensy 2.0++      
#  	Linguagem:	    Wiring / C /Processing /Fritzing / Arduino IDE
#       Versão:             V.5
#						
#	Objetivo:           Robot Explorador
#										  
#	Funcionamento:	    Once activated, the robot reads the distance 
#                           (right, left and front) in order to decide what is 
#                           the best way to go.
#                           Decided the best way (which is always the longest way, 
#                           if there is no best way to go, the robot decides to give 
#                           half back) it goes forward until it find an obstacle 
#                           closer than 30 cm  from it and then the robot will stop 
#                           and it will perform the decision procedure again.
#                           And so on...
#                           
#
#			
#   Autor:              Marcelo Moraes 
#   Data:               29/11/12	
#   Local:              Sorocaba - SP	
#					
#####################################################################################
 
  Este projeto contém código de domínio público.
  A modificação é permitida sem aviso prévio.
*/


/*
Motors:
Motor 1 - the right motor
Motor 2 - the left motor

Input pins:
IN 1 e IN 2 - pins to drive the motor 1
IN 3 e IN 4 - pins to drive the motor 2

Enable pins:
EN 1 - motor 1 enable pin
EN 2 - motor 2 enable pin

The anable pin acts as a PWM for the (speed and spining) motor's control
To drive the motors we need to put LOW/HIGH in the input pins to determine
the direction (clockwise or anti-clockwise)

Take care when wiring the motors to the correct direction
*/

// Libraries
#include <Servo.h>      // servo-motors lead up.
#include <Ultrasonic.h> // PING sensor lead up.

// objects definition
Servo myservo; // my own servo object

// variables and constants definition
int echopin = 3;               // HC-SR04 echo pin
int trigerpin = 4;             // HC-SR04 trigger pin
int in1 = 6;                   // motor 1, 1st pin
int in2 = 7;                   // motor 1, 2nd pin
int in3 = 8;                   // motor 2, 1st pin
int in4 = 9;                   // motor 2, 2nd pin
int enable1 = 5;               // motor 1, PWM pin to speed control
int enable2 = 10;              // motor 2, PWM pin to speed control
int BUZZER = 12;               // buzzer pin
int Velocidade1 = 90;          // motor 1 speed definition (0 to 255) 
int Velocidade2 = 80;          // motor 2 speed definition (0 to 255)
int distLimite = 20;           // distance limite in centimeter (20cm)

unsigned long pulsetime = 0;   // beat time of ping sensor
unsigned long distancia = 0;   // stores the distance values



void setup(){
  Serial.begin(9600);          // serial communication initialization, just for debug
                               // to show in the LCD display
  myservo.attach(10);          // attach servo motor to pin 10
  pinMode(enable1, OUTPUT);    // motor pins definition
  pinMode(enable2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  pinMode(trigerpin, OUTPUT);  // trigger pin definition as putput
  pinMode(echopin, INPUT);     // echo pin definition as input
  
  pinMode(BUZZER, OUTPUT);     // buzzer pin definition as output
  
  myservo.write(90);           // servo start in 90 degree position 
  pararMotor();                // robot starts stopped
  Serial.println("stopped");   // print message in the console
  delay(5000);                 // stay 5 seconds stopped before starts
  
}


void loop(){
  // variables initialization
  int distEsquerda = 0;
  int distDireita = 0;
  int distFrente = 0;
  
  // distance initial reads
  Serial.println("thinking...");   
  delay(1000);                     // pause  
  myservo.write(180);              // servo position to 180 degrees
  delay(1000);                     // waiting the sensor stabilize the reading
  distEsquerda = lerDistancia();   // reads left distance
  Serial.print("Left Dist: ");         
  Serial.print(distEsquerda);
  Serial.println(" cm.");
  myservo.write(90);               // servo position to 90 degrees
  delay(1000);                     // waiting the sensor stabilize the reading
  distFrente = lerDistancia();     // reads front distance
  Serial.print("Front Dist: ");         
  Serial.print(distFrente);
  Serial.println(" cm.");
  myservo.write(0);                // servo position to 0 degrees
  delay(1000);                     // waiting the sensor stabilize the reading
  distDireita = lerDistancia();    // reads right distance 
  Serial.print("Right Dist: ");         
  Serial.print(distDireita);
  Serial.println(" cm.");
  
  // at the end put servo to 90 degrees position (robot looking ahead)
  myservo.write(90);
  delay(1000);                     // waiting the sensor stabilize the reading  
  
  // best way to go determination
  if((distFrente < distEsquerda) && (distFrente < distDireita)){ // if front distance is less then left or right distance
  // check who is greater (right or left)
    if(distEsquerda > distDireita){
      Serial.println("turning to left");     
      motorDireito(600);                        // to turn left, just right motor works
      delay(300);
    }
    else{
      if(distEsquerda == distDireita){ // when left and right distance are the same (especila case)
        Serial.println("turning 180 degrees");  
        girar180graus(600);                     // turn 180 degree, both motors works (one in each direction)
        delay(300);
      }
      else{
        Serial.println("turning right");    
        motorEsquerdo(600);                     // to turn right, just left motor works
        delay(300);
      }
    }
  }
  // clear all variables
  distEsquerda = 0;
  distDireita = 0;
  distFrente = 0;
  // go ahead till find another obstacle (obstacle distance less than 20 cm)
  int x = 1;                               // support variable
  while(x){                                // while true
    Serial.println("going forward");           
    paraFrente();                          // go ahead
    distFrente = lerDistancia();           // take the front distance
    Serial.print("Front Dist: ");               
    Serial.print(distFrente);
    Serial.println(" cm.");
    if(distFrente < distLimite){           // if front distance is less than the limite
      pararMotor();                        // motor stopped
      delay(200);                          
      x = 0;                               // invalidates the while loop and goes to the main loop
                                           // running from the beginning again
                                           // reading distances and the best path decision
      Alerta(BUZZER, 2000, 100);           // sound alarm
      Serial.println("stopped");            
    }
  } 
}



// ########################### functions set ###########################

// half back
void girar180graus(long tempo){
  analogWrite(enable1, Velocidade1);  // motor 1 speed (PWM).
  analogWrite(enable2, Velocidade2);  // motor 2 speed(PWM).
  digitalWrite(in1, LOW);             // motor 1 forward direction
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);            // motor 2 backward direction
  digitalWrite(in4, LOW);
  delay(tempo);                       // motor will be drived for this time (this is an empiric value)
  pararMotor();                       // motors off at the end
}

// turn left
void motorDireito(long tempo){       // the time is in miliseconds
                                     // adjustable in the code it self, if necessary
  analogWrite(enable1, Velocidade1); // motor 1 on
  analogWrite(enable2, 0);           // motor 2 off
  digitalWrite(in1, LOW);            // turn to left
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW);            
  digitalWrite(in4, LOW);
  delay(tempo);                      // motor will be drived for this time (this is an empiric value)
  pararMotor();                      // motors off at the end
}

// turn right
void motorEsquerdo(long tempo){      // the time is in miliseconds 
                                     // adjustable in the code it self, if necessary
  analogWrite(enable1, 0);           // motor 1 off
  analogWrite(enable2, Velocidade2); // motor 2 on
  digitalWrite(in1, LOW);            // turn right
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  
  digitalWrite(in4, HIGH);           // motor 2 ligado, rodando para frente
  delay(tempo);                      // motor will be drived for this time (this is an empiric value)
  pararMotor();                      // motors off at the end
}

// forward
void paraFrente(){                   
                                     
  analogWrite(enable1, Velocidade1); // motor 1 on
  analogWrite(enable2, Velocidade2); // motor 2 on
  digitalWrite(in1, LOW);            // go forward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// stopped
void pararMotor() {
  digitalWrite(in1, LOW);            // motor 1 and motor 2 off
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enable1, 0);           // motor 1 disabled
  analogWrite(enable2, 0);           // motor 2 disabled
}

// backward
void motorRe(long tempo){            
                                     
  analogWrite(enable1, Velocidade1); // motor 1 on (reverse direction)
  analogWrite(enable2, Velocidade2); // motor 2 on (reverse direction)
  digitalWrite(in1, HIGH);           
  digitalWrite(in2, LOW);            // go backward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(tempo);
  pararMotor();
}

// sound alarm
void Alerta (unsigned char BUZZER, int frequencia, long tempoMilisegundos){
  int Contador, x=0;                                           // support variable
  long pausa = (long)(1000000/frequencia);                     // figure out the sound pause
  long loopTempo = (long)((tempoMilisegundos*1000)/(pausa*2)); // figure out the sound loop time
  while(x<3){                                                  // do this 3 times
    for (Contador=0;Contador<loopTempo;Contador++){            
      digitalWrite(BUZZER,HIGH);                               // buzzer on
      delayMicroseconds(pausa);                                // pause
      digitalWrite(BUZZER,LOW);                                // buzzer off
      delayMicroseconds(pausa);                                // pause
    }
    delay(100);                                                
    x++;                                                       // next
  }
}

// distance reading
int lerDistancia(){
  // at the beggining generates a 10 ms pulse on trigger pin
  digitalWrite(trigerpin, LOW);        // to ensure startup wrist and not generate false pulses
  digitalWrite(trigerpin, HIGH);       // put trigger pin at high level
  delayMicroseconds(10);               // for 10 ms.
  digitalWrite(trigerpin,LOW);         // put trigger pin at low level
  pulsetime = pulseIn(echopin, HIGH);  // reads echo pin pulse
  return pulsetime/58;                 // convert to centimeter and returns
                                       
}

// END OF COMPILATION
