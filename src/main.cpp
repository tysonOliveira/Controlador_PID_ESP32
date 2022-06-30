/* =================================================================================
COntrolador PID com ESP32

ESP-WROOM-32
Board: DevKitV1
Compilador: Arduino IDE 1.8.19

Autor: 
Data: 

================================================================================= */
#include <Arduino.h>

#define adc_in       4
#define light_ctr   23

// Atualização do PID
#define dt 100  

// meas = valor medido / set_point = valor que queremos atingir 
float pid_control(float meas, float set_point);

int time_upt=0, pid_upt=0;

// Alteramos os valores para fazer a sintonia PID
float kp = 0.0,
      ki = 0.0,
      kd = 0.0;

void setup() 
{
  Serial.begin(115200);
  pinMode(adc_in, INPUT);
  pinMode(light_ctr, OUTPUT);

  // Configuração PWM do esp32
  ledcAttachPin(light_ctr, 0);   // pin, canal
  ledcSetup(0, 2000, 12);        // canal, frequencia em hertz, resolução(bits)
  ledcWrite(0, 2048);            // canal, duty cycle  
}

void loop() 
{
  float pwm_val;

  if(millis()-pid_upt >= dt)
  {
    pwm_val = pid_control(analogRead(adc_in), 2800.0);
    ledcWrite(0, pwm_val + 2048);
    pid_upt = millis();
  }

  if(millis()-time_upt >= 741)
  {
    Serial.println(analogRead(adc_in));
    
    time_upt = millis();
  }
  
}

float pid_control(float meas, float set_point)
{
  static float  last_meas;
  
  static float integral;

  float         error_meas, 
                proportional,
                derivative;
  
  // O erro é a diferença entre o valor que queremos atingir e o valor medido
  error_meas = set_point - meas;   

  // Proportional é a constante kp multiplicada pelo erro
  proportional = kp * error_meas;

  // Integral é a soma dos erros infinitesimais ao longo do tempo
  // somando o resultado de (error_meas * ki) * dt ao longo do tempo obtemos a integral
  // dt é a base de tempo que está em milisegundos e precisamos converter para segundos.
  integral += (error_meas * ki) * (dt/1000.0);

  // ((medida_anterior - medida_atual) * constante_derivativa) / (base_de_tempo_em_milisegundos / 1000)
  derivative = ((last_meas - meas) * kd) / (dt/1000.0);

  // Atualiza a medida anterior
  last_meas = meas;

  return (proportional+integral+derivative);
}