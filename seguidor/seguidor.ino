
#define INT1 13
#define INT2 12
#define INT3 11
#define INT4 10

#define ENA 5

#define ENB 6




void setup() {
  
  pinMode(INT1, OUTPUT);
  pinMode(INT2, OUTPUT);
  pinMode(INT3, OUTPUT);
  pinMode(INT3, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  parar();
}

void loop() {
  analogWrite(ENA,255);
  analogWrite(ENB,255);
  frente();
  delay(500);
  girar_eixo_direita();
  delay(500);
  girar_eixo_esquerda();
  delay(1000);
  parar();
}

void ligar(int pino){
  digitalWrite(pino,HIGH);
}

void desligar(int pino){
  digitalWrite(pino,LOW);
}

void frente_motorA(){
  ligar(INT2);
  desligar(INT1);
}

void frente_motorB(){
    ligar(INT3);
    desligar(INT4);
}

void frente(){
  frente_motorA();
  frente_motorB();
}

void tras_motorA(){
  ligar(INT1);
  desligar(INT2);
  
}
void tras_motorB(){
  ligar(INT4);
  desligar(INT3);
}

void tras(){
  tras_motorA();
  tras_motorB();
}

void parar_motorA(){
  desligar(INT1);
  desligar(INT2);
}
void parar_motorB(){
  desligar(INT3);
  desligar(INT4);
}
void parar(){
  parar_motorA();
  parar_motorB();
}
void girar_eixo_direita(){
  frente_motorA();
  tras_motorB();
}
void girar_eixo_esquerda(){
  frente_motorB();
  tras_motorA();
}
