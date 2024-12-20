#include <Arduino.h>
#include <DFRobot_DHT11.h>

DFRobot_DHT11 DHT;
#define DHT11_PIN 10 //pin for digital input of humidity and temperature sensor
const int Pin1 = A0; // pin for analog input of moisture sensor

/**********Output Pin Definitions***********/
byte PWM_pump = 3;
byte PWM1 = 5;
byte PWM2 = 6;
byte EN = 7;
/************************ */
byte PWM_signal;

unsigned long int timer_duration; // watering duration in ms
static int watering_timer_flag = 0; // if watering_timer_flag = 1, activate timer
int setRPM; // duty cycle value for fan PWM signal

void set_watering_timer(unsigned long int millisec); // function that turns off pump after 'timer duration' of ms has past
void change_direction(); // function that changes the direction of fan rotation

void setup() { /* Initial condition setup (making sure everything is off)*/
  Serial.begin(9600); 
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
  analogWrite(PWM_pump,0);
  digitalWrite(EN, LOW);
  PWM_signal=PWM1;
}

void loop() {
  /************************FAN_CONTROL*******************************/
  digitalWrite(EN, HIGH); // turn on the enable pin for MOSFET drivers
  DHT.read(DHT11_PIN); // reading the humidity and temperature data

  if(DHT.humidity <= 50) { //checking if humidity procentage is below or equal 50%
    setRPM = 0; //Fan is Off (when humidity is below or at 50%)
  }
  else if(DHT.humidity > 50 && DHT.humidity <= 60) {
    setRPM = 51; //20% duty cycle
  }
  else if(DHT.humidity > 60 && DHT.humidity <= 80) {
    setRPM = 90; //35% duty cycle
  }
  else if(DHT.humidity > 80 && DHT.humidity <= 90) {
    setRPM = 128; //50% duty cycle
  }

  else if(DHT.humidity > 98 && DHT.humidity <= 100){
    change_direction();
  }

  analogWrite(PWM_signal,setRPM); // sending a PWM signal to H-brige (motor)


 /* Serial.print("temp: ");
  Serial.print(DHT.temperature);
  Serial.print("  humi: ");
  Serial.println(DHT.humidity);
  Serial.print("Fan PWM: ");Serial.println(setRPM);*/
/************************PUMP_CONTROL*******************************/
  float TotMoisture = 0; // total bit analog sum after oversampling
  float MoistureBits = 0; // the average value of bits
  float MoistureVolt = 0; // value of bits converted to voltage

  if(watering_timer_flag == 0){ // cheching if the watering timer if off

    for (int x=0; x<64; x++){  // oversampling for more acurate reading
      TotMoisture += analogRead(Pin1);
    }

    MoistureBits = TotMoisture/64.0;
    MoistureVolt = (MoistureBits/1024.0)*5000.0;

    if (MoistureVolt <= 2800.0){// comparing soil moisture to threshold
      timer_duration = 4000; // 4 seconds of watering
      watering_timer_flag = 1; // turn on watering timer
    }

    if (MoistureVolt > 2800.0 && MoistureVolt <= 3200.0){
      timer_duration = 1000; // 1 second of watering
      watering_timer_flag = 1;
    }

    if (MoistureVolt > 3200.0 && MoistureVolt <= 3400.0){
      timer_duration = 500; // 0.5 second of watering
      watering_timer_flag = 1;
    }

    //Serial.print("MoistureBits: ");Serial.println(MoistureBits);
    //Serial.print("MoistureVolt: ");Serial.println(MoistureVolt);
  }

  set_watering_timer(timer_duration); // setting the duration of the timer
}


void set_watering_timer(unsigned long int millisec){
  static long unsigned int t_start;
  if(watering_timer_flag == 1){
    analogWrite(PWM_pump,128); // begin watering (pump ON)
    t_start = millis(); // saving the start time of timer
    watering_timer_flag = 2;
  }
  if(watering_timer_flag == 2){
    if(t_start + millisec < millis()){ // checking if timer duration has past
     watering_timer_flag = 0; 
     analogWrite(PWM_pump,0); // turn off pump
     delay(2000); // wait for the water to go into the soil
    }
  }
}

void change_direction(){
    for(int i = setRPM; i > 0; i--){ // ramps down the fan speed until it stops
      setRPM--;
      analogWrite(PWM_signal,setRPM);
      delay(10); // if duty cycle was 50%, it takes 1.28 seconds to stop the fan
    }
    if(setRPM == 0){ // chehcking if the fan is off
      digitalWrite(EN, LOW); // turn off MOSFET drivers

      if(PWM_signal == PWM2) PWM_signal = PWM1;// channge the PWM supply pin for the H-bridge
      else PWM_signal = PWM2; 

      delay(1000); // 1 second of deadtime
      digitalWrite(EN, HIGH); // turn on MOSFET drivers
      setRPM = 191; // 75% duty cycle 
    }
}