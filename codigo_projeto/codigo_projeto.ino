/* Código implementado pelos discentes Jessica Lo e Lucas Vitkoski, como parte do projeto da disciplina TE331-Instrumentação Eletrônica.
   Controle da altura de um objeto dentro de um tubo, por meio da alteração
   do valor do sinal PWM pelo monitor serial;
   Leitura da distância do objeto pelo sensor VL53l0X
*/

#include "Adafruit_VL53L0X.h"

// ------------------ Variáveis ---------------------- //

unsigned int Ts = 37;
int PWM = 0;
unsigned long tempo = 0;

float altura = 0;
float Kp = 0.1834;
float Ki = (0.057 * Ts) / 1000;
float Kd = (0.1052 / Ts) * 1000;
float setPoint = 200;
float erro, intErro, difAltura = 0;
float alturaAnt = 13;

//float startTime = 0;
//bool ativo = 0;
//bool primeira = 0;
 
// --------------------------------------------------- //

Adafruit_VL53L0X lox = Adafruit_VL53L0X(); // Parâmetro da biblioteca

// -------------------- Setup ------------------------ //

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); //limita o tempo de espera de caracteres de entrada da serial

  // espera até o monitor serial abrir
  while (! Serial) {
    delay(1);
  }

  if (!lox.begin(VL53L0X_I2C_ADDR, false, &Wire, lox.VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }

  pinMode(3, OUTPUT);
  analogWrite(3, 0);
}

// --------------------------------------------------- //

// -------------------- Loop ------------------------- //

void loop() {
  tempo = millis();
  //Serial.println(millis());

  VL53L0X_RangingMeasurementData_t medida;
  lox.rangingTest(&medida, false); // mudar para 'true' caso deseje acessar os dados de depuração

  /*if (primeira == 1) {
    startTime = millis();
    primeira = 0;
    }

    if (ativo == 1) {
    //Serial.print(millis() - startTime);
    }*/

  if (medida.RangeStatus != 4) {
    altura = altura * 0.66 + 0.34 * medida.RangeMilliMeter; 
    //Serial.println(altura);
  } else {
    Serial.println(" out of range ");
  }

// ----------------- CONTROLADOR PID --------------------- //

  erro = setPoint - altura;

  //Serial.print("erro: "); Serial.println(erro);

  if (erro > (setPoint * 0.1) || erro < (-setPoint * 0.1)) {
    intErro = 0;
  }
  else {
    intErro += erro;
  }

  difAltura = altura - alturaAnt;

  PWM = int(erro * Kp + intErro * Ki - difAltura * Kd) + 53; // Função de controle do PWM

  alturaAnt = altura;

  PWM = constrain(PWM, 25, 70); // Deixa o valor do PWM entre 25 e 70
  
  //Serial.print("PWM: "); Serial.println(PWM);

  analogWrite(3, PWM);


  /*Serial.print("difAltura: "); Serial.println(difAltura);
    Serial.print("altura: "); Serial.println(altura);
    Serial.print("alturaAnt: "); Serial.println(alturaAnt);
    Serial.print("intErro: "); Serial.println(intErro);*/

  Serial.print(setPoint, 1); Serial.print(" ");
  Serial.print(PWM); Serial.print(" ");
  //Serial.print(intErro); Serial.print(" ");
  Serial.println(altura);

// ---------- Entrada de dados pela serial ------------ //

  if (Serial.available() > 0) {  //verifica se há dados na serial
    setPoint = Serial.parseInt(); //lê caracteres ASCII da serial e converte em um número inteiro
    setPoint = constrain(setPoint, 100, 300);  //limita os valores da variável PWM entre 0 a 255
    while(Serial.available())Serial.read();

    /* while (Serial.available()) Serial.read();
      if (ativo == 0) {
       primeira = 1;
       ativo = 1;
      }*/
  }

  while ((tempo + Ts) > millis()) {}
}
