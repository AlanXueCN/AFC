// This program generates pwm signal at pin 4, 5, 6, 7 of arduino Nano (Atmega328)
// i.e. it can be used to drive upto four esc or four servo

float frequency = 50, T = 1/frequency, loop_timer=T*1000000, t_loop_previous; // frequecy is update rate of esc pulse (50Hz works fine)
unsigned long start_time_pw, time_now;
unsigned long falling_time_pw_1, falling_time_pw_2, falling_time_pw_3, falling_time_pw_4;

void setup(){
  DDRD |= B11110000; //setting pin 4,5,6,7 for output
  Serial.begin(57600);
  t_loop_previous=micros();
}

void loop(){
  if(micros()<8000000){ // esc require 1000 us pulse for some time when it is powered for working
    command_esc(1000,1000,1000,1000);
  }
  else{
    command_esc(800+analogRead(0),1000,1600,2200); // this command generates pwm signal at arduino digital pin number 3,5,6,7
  }
  
  while(micros()-t_loop_previous <= loop_timer){}
  t_loop_previous=micros();
}

void command_esc(unsigned long pw_1,unsigned long pw_2,unsigned long pw_3,unsigned long pw_4){
  PORTD |= B11110000; // set pin 4,5,6,7 High
  start_time_pw = micros();                                             // record starting time of pulse width
  falling_time_pw_1 = start_time_pw + pw_1;                             // calculate time of falling edge on pin 4
  falling_time_pw_2 = start_time_pw + pw_2;                             // calculate time of falling edge on pin 5
  falling_time_pw_3 = start_time_pw + pw_3;                             // calculate time of falling edge on pin 6
  falling_time_pw_4 = start_time_pw + pw_4;                             // calculate time of falling edge on pin 7

  while(PORTD >= 16){                                                   // wait till all 4 pulses are set low (NOTE: This will take minimum 1000us and maximum 2000us)
    time_now = micros();                                                //Record the current time
    if(time_now >= falling_time_pw_1)PORTD &= B11101111;                //Set pin 4 to low if pulse width time is reached
    if(time_now >= falling_time_pw_2)PORTD &= B11011111;                //Set pin 5 to low if pulse width time is reached
    if(time_now >= falling_time_pw_3)PORTD &= B10111111;                //Set pin 6 to low if pulse width time is reached
    if(time_now >= falling_time_pw_4)PORTD &= B01111111;                //Set pin 7 to low if pulse width time is reached
  }
}
