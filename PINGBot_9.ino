/*
#####################################################################################
#  	Arquivo:            PINGBot_4.pde                                             
#       Micro-processador:  Arduino UNO ou Teensy 2.0++      
#  	Linguagem:	    Wiring / C /Processing /Fritzing / Arduino IDE
#       Versão:             V.4
#						
#	Objetivo:           Robot Explorador
#										  
#	Funcionamento:	    Uma vez acionado, o Robot segue em frente até encontrar um
#                           obstáculo.
#                           Se a distancia for menor que X cm ele gira para a esquerda e segue em frente.
#                           Caso a distância da esquerda for menor que X cm, ele olha para
#                           a direita e executa o mesmo processo.
#                           Caso nem a direita nem a esquerda forneçam espaço para a manobra,
#                           ele gira 180 graus para e faz o ciclo novamente.
#                           X é variável "distLimite";  pode ser ajustado no programa. É a distância mínimapara 
#                           detectar um objeto
#
#
#
#   http://arduinobymyself.blogspot.com.br/			
#   Autor:              Marcelo Moraes 
#   Data:               25/11/12	
#   Local:              Sorocaba - SP	
#					
#####################################################################################
 
  Este projeto contém código de domínio público.
  A modificação é premitida sem aviso prévio.
 */


/*
Motores:
Motor 1 - motor da direita
Motor 2 - motor da esquerda
Entradas:
IN 1 e IN 2 - pinos de acionamento do motor 1
IN 3 e IN 4 - pinos de acionamento do motor 2
*/

// inclusão de bibliotecas.
#include <Servo.h>       // inclui biblioteca de manipulação de servo-motores.
#include <Ultrasonic.h>  // inclui biblioteca de naminpulação do sensor PING.

// definição de objetos.
Servo myservo;

// definição de variáveis e pinos.
int echopin = 3;               // a saída echo do HC-SR04 ligado no pin 9.
int trigerpin = 4;             // a saída triger do HC-SR04 ligado no pin 8.
int inA1 = 10;                 // pino 1 do motor 1; direito
int inA2 = 11;                 // pino 2 do motor 1; direito
int inB1 = 5;                  // pino 1 do motor 2; esquerdo
int inB2 = 6;                  // pinto 2 do motor 2; esquerdo
int BUZZER = 12;               // define pino do buzzer
unsigned long pulsetime = 0;   // variável que faz a leitura do pulso.
unsigned long distancia = 0;   // variável que que armazena a distância.        
int distLimite = 20;           // distancia limite para o sensor em cm



// executado na inicialização do Arduino
void setup(){
  Serial.begin(9600);          // inicializa a comunicação serial
  myservo.attach(10);          // anexa o servo motor lógico ao físico no pino 10.
  pinMode(inA1, OUTPUT);       // pinos dos motores como saídas 
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(trigerpin, OUTPUT);  // define o pino triger como saída.
  pinMode(echopin, INPUT);     // define o pino echo como entrada.
  pinMode(BUZZER, OUTPUT);     // pino buzzer como saída
  set_motors(0,0);             // inicialmente motor parado
  Serial.println("PARADO");    // imprime na serial
  Alerta(BUZZER, 2000, 100);   // sinal de alerta
  delay(5000);                 // espera 5s antes de iniciar
}

// loop rpincipal do Arduino
void loop(){
  distancia = lerDistancia();              // chama a funcao que retorna a distancia em cm.
  Serial.print("DistF.: ");                // imprime na serial
  Serial.print(distancia);                 
  Serial.println(" cm");                   
  if (distancia > distLimite){             // se a distância for maior que 20 cm.
    Serial.println("SEGUE EM FRENTE");     // imprime na serial
    //delay(1000);                           // atraso na execução
    set_motors(100,100);                   // segue em frente.
  }
  else {                                   // senão, a distância frontal é MENOR ou IGUAL a 20 centímetros.
    Serial.println("PARADO");              // imprime na serial
    set_motors(0,0);                       // pare o motor.
    Alerta(BUZZER, 2000, 100);             // sinal de alerta
    //delay(1000);                         // aguarde 1 segundo.
    int distEsquerda = 0;                  // inicializa variável de apoio
    int distDireita = 0;                   // inicializa variável de apoio
    myservo.write(0);                      // posiciona o servo a 0 graus.
    delay(1000);                           // espera o servo chegar no posição e o sensor PING estabilizar a leitura.
    distEsquerda = lerDistancia();         // chama a funcao que retorna a distancia em cm.
    Serial.print("DistE.: ");              // imprime na serial
    Serial.print(distEsquerda);                 
    Serial.println(" cm");                   
    myservo.write(90);                     // gira o servo para 90 graus - esta deve ser a posicao inicial ou central do servo.
    //delay(1000);                           // atraso na execução
        
    // analisando o que foi lido na esquerda.
    if (distEsquerda > distLimite){        // se a distancia ESQUERDA for maior que distancia limite.
      Serial.println("GIRO A ESQUERDA");   // imprime na serial
      set_motors(-50,50);                  // chama funcao que gira para ESQUERDA (aciona motor-1 "direita" por 500 ms).
      delay(200);                          // configurar empiricamente
      set_motors(0,0);                     // para motor depois do tempo
    }
    else {                                 // senão, a distancia esquerda é MENOR ou IGUAL a distancia limite. Então vamos analisar a distancia DIREITA.
      myservo.write(180);                  // posiciona o servo a 180 graus.
      delay(1000);                         // espera o servo chegar na posição e o sensor PING estabilizar a leitura.
      distDireita = lerDistancia();        // chama função que retorna a distância em cm.
      Serial.print("DistD.: ");            // imprime na serial
      Serial.print(distDireita);                 
      Serial.println(" cm");
      myservo.write(90);                   // gira o servo para 90 graus - esta deve ser a posição inicial ou central do servo.
      
      // analisando o que foi lido na direita
      if(distDireita > distLimite){        // se a distância DIREITA for maior que distancia limite.
        Serial.println("GIRO A DIREITA");  // imprime na serial
        set_motors(50,-50);                 // chama funcao que gira para DIREITA (aciona o motor-2 "esquerda" por 500 ms).
        delay(200);                        // configurar empiricamente
        set_motors(0,0);                   // para motor depois do tempo
      }
    }
    if((distDireita <= distLimite) && (distEsquerda <= distLimite)) {                // além da distância FRONTAL, tanto a distância DIREITA 
                                           // quanto a distância ESQUERDA, são menores ou iguais a distancia limite, então.
      Serial.println("GIRO A 180 GRAUS");  // imprime na serial
      set_motors(50,-50);                  // chama função que gira meia volta no robot.
      delay(400);                          // configurar empiricamente
      set_motors(0,0);                     // para motor depois do tempo  
      myservo.write(90);                   // posiciona o servo a 90 graus.
      //delay(1000);                         // espera o servo chegar na posição e o sensor PING estabilizar a leitura.
    }       
  }
}


// ########################### set de funções ###########################

// Acionamento dos motores
void set_motors(int left_speed, int right_speed){
  if(right_speed >= 0 && left_speed >= 0){
    analogWrite(inA1, 0);
    analogWrite(inA2, right_speed);
    analogWrite(inB1, 0);
    analogWrite(inB2, left_speed);
  }
  if(right_speed >= 0 && left_speed < 0){
    left_speed = -left_speed;
    analogWrite(inA1, 0);
    analogWrite(inA2, right_speed);
    analogWrite(inB1, left_speed);
    analogWrite(inB2, 0);
  }
  if(right_speed < 0 && left_speed >= 0){
    right_speed = -right_speed;
    analogWrite(inA1, right_speed);
    analogWrite(inA2, 0);
    analogWrite(inB1, 0);
    analogWrite(inB2, left_speed);
  } 
}

// função que retorna a distância em cm 
int lerDistancia(){
  // primeiro geramos um pulso de 10 ms no pino trigger do sensor PING.
  digitalWrite(trigerpin, LOW);        // para garantir a inicialização do pino e não gerar pulso em falso
  digitalWrite(trigerpin, HIGH);       // coloca o pino trigger en nível alto.
  delayMicroseconds(10);               // por 10 ms.
  digitalWrite(trigerpin,LOW);         // coloca o pino trigger en nível baixo.
  pulsetime = pulseIn(echopin, HIGH);  // faz a leitura do pulso no pino echo do sensor.
  return pulsetime/58;                 // converte para cm e retorna da função
                                       // veja o cálculo completo em "http://arduinobymyself.blogspot.com.br/"
}

// função de alerta sonoro
void Alerta (unsigned char BUZZER, int frequencia, long tempoMilisegundos){
  int Contador, x=0;                                           // variável de apoio
  long pausa = (long)(1000000/frequencia);                     // calcula a pausa
  long loopTempo = (long)((tempoMilisegundos*1000)/(pausa*2)); // calcula o loop
  while(x<3){                                                  // faz 3 vezes
    for (Contador=0;Contador<loopTempo;Contador++){            // executa enquanto menor que contador=loop tempo
      digitalWrite(BUZZER,HIGH);                               // liga buzzer
      delayMicroseconds(pausa);                                // pausa
      digitalWrite(BUZZER,LOW);                                // desliga buzzer
      delayMicroseconds(pausa);                                // pausa
    }
    delay(100);                                                // atraso na execução
    x++;                                                       // próximo x
  }
}

// FIM DA COMPILAÇÃO
