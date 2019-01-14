// This program reads pwm signal on pin 8, 9, 10, 11, 12 on arduino Nano (Atmega328)
// i.e. it can detect upto 5 channel pwm signal
//---------------------------------------------------------------------------------
// time related variables starts
/* SampleFrequency is frequency at which void loop repeats itself
 * dt is time bewteen execution of two consicutive void loop
 * loop_timer is dt in microseconds
 * t stores time stamp at the point it is asigned to micros() function*/
float SampleFrequency = 100, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;
// time related variables ends
//---------------------------------------------------------------------------------


//---------------------------------------------------------------------------------
// receiver starts
// RC receive for four channel on digital pin 8,9,10,11 of arduino uno
unsigned long counter_1, counter_2, counter_3, counter_4, counter_5, current_count; //time width value of each PWM input signal
bool last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state, last_CH5_state; //stores previous values of signal 1/0
float receiver_input[5]; // save receiver inputs as float for further calculateion in float
//---------------------------------------------------------------------------------


//---------------------------------------------------------------------------------
// throttle_setpoint, yaw_setpoint, pitch_setpoint, roll_setpoint & mode are calculated from 5 channels of receiver pin
// arm_status is calculated from combination of different channels of receiver pin (throttle(Ch_3) and yaw(Ch_4) combination is used here) */
float throttle_setpoint, yaw_setpoint, pitch_setpoint, roll_setpoint;
int mode=0;
int arm_status;

float roll_input_dead_band_delta = 12; // in PWM
float roll_input_dead_band_min = 1500 - roll_input_dead_band_delta; // in PWM
float roll_input_dead_band_max = 1500 + roll_input_dead_band_delta; // in PWM
float roll_input_max = 1880; // in PWM
float roll_input_min = 1044; // in PWM
float roll_setpoint_max = 25; // in degree
float roll_setpoint_min = -roll_setpoint_max; // in degree
float roll_scale_factor_positive = roll_setpoint_max/(roll_input_max - roll_input_dead_band_max); // scale factor positive
float roll_scale_factor_negative = roll_setpoint_min/(roll_input_min - roll_input_dead_band_min); // scale factor negative

float pitch_input_dead_band_delta = 12; // in PWM
float pitch_input_dead_band_min = 1500 - pitch_input_dead_band_delta; // in PWM
float pitch_input_dead_band_max = 1500 + pitch_input_dead_band_delta; // in PWM
float pitch_input_max = 1968; // in PWM
float pitch_input_min = 1152; // in PWM
float pitch_setpoint_max = 25; // in degree
float pitch_setpoint_min = -pitch_setpoint_max; // in degree
float pitch_scale_factor_positive = pitch_setpoint_max/(pitch_input_max - pitch_input_dead_band_max); // scale factor positive
float pitch_scale_factor_negative = pitch_setpoint_min/(pitch_input_min - pitch_input_dead_band_min); // scale factor negative

float yaw_input_dead_band_delta = 12; // in PWM
float yaw_input_dead_band_min = 1500 - yaw_input_dead_band_delta; // in PWM
float yaw_input_dead_band_max = 1500 + yaw_input_dead_band_delta; // in PWM
float yaw_input_max = 1904; // in PWM
float yaw_input_min = 1032; // in PWM
float yaw_setpoint_max = 100; // in degree/s
float yaw_setpoint_min = -yaw_setpoint_max; // in degree
float yaw_scale_factor_positive = yaw_setpoint_max/(yaw_input_max - yaw_input_dead_band_max); // scale factor positive
float yaw_scale_factor_negative = yaw_setpoint_min/(yaw_input_min - yaw_input_dead_band_min); // scale factor negative

float throttle_input_min = 1052;
float throttle_input_max = 1924;
float throttle_setpoint_min = 1000;
float throttle_setpoint_max = 2000;
float throttle_scale_factor = (throttle_setpoint_max - throttle_setpoint_min)/(throttle_input_max - throttle_input_min);

float arm_throttle_min_1 = throttle_input_min + 32;
float arm_throttle_min_2 = throttle_input_min + 52; //1104
float arm_yaw_min = yaw_input_min + 100;
float arm_yaw_max = yaw_input_max - 100;

// reference angles and angular velocity are those values at which vehicles hover at one position without drifting in any direction in horizontal plane
// roll_reference and pitch_reference are in degree(i.e. angle) but yaw_reference is in degree/sec(i.e. angular velocity)
float roll_reference=0, pitch_reference=0, yaw_reference=0; 
// receiver ends
//---------------------------------------------------------------------------------


void setup(){
  Serial.begin(57600);
  // below 6 lines of code enable hardware interrupt subrouting when atate of pin 8, 9, 10, 11, 12 changes for detecting 5 channels of receiver
  PCICR |= (1 << PCIE0);   //enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0); //set pin 8 for detecting state change
  PCMSK0 |= (1 << PCINT1); //set pin 9 for detecting state change
  PCMSK0 |= (1 << PCINT2); //set pin 10 for detecting state change
  PCMSK0 |= (1 << PCINT3); //set pin 11 for detecting state change
  PCMSK0 |= (1 << PCINT4); //set pin 12 for detecting state change
  
  t = micros();
}

void loop(){
  print_receiver_raw_input();
  interpret_receiver_command();
  //print_setpoints();

  while(micros()-t<loop_timer){}
  t = micros();
}

ISR(PCINT0_vect){
  current_count = micros();
  //channel 1 of receiver is connected to  pin8
  if(PINB & B00000001){                                                     // if input at pin 8 is HIGH during inteerrupt
    if(last_CH1_state == 0){                                                // if previous state of pin 8 was LOW (i.e. pulse is rising)
      last_CH1_state = 1;                                                   // saves the current state (HIGH) of pin 8
      counter_1 = current_count;                                            // stores this time(time at which pulse is rising) as in variable counter_1
    }
  }
  else if(last_CH1_state == 1){                                             // if previous state of pin 8 was high and interrupt has occured => pulse is falling => state at pin 8 is going to be LOW
    last_CH1_state = 0;                                                     // saves the current state (LOW) of pin 8
    receiver_input[0] = current_count - counter_1;                          // Channel 1 is current_time - timer_1.
  }
  
  //channel 2 of receiver is connected to  pin9
  if(PINB & B00000010){
    if(last_CH2_state == 0){
      last_CH2_state = 1;
      counter_2 = current_count;
    }
  }
  else if(last_CH2_state == 1){
    last_CH2_state = 0;
    receiver_input[1] = current_count-counter_2;
  }

  //channel 3 of receiver is connected to  pin10
  if(PINB & B00000100){
    if(last_CH3_state == 0){
      last_CH3_state = 1;
      counter_3 = current_count;
    }
  }
  else if(last_CH3_state == 1){
    last_CH3_state = 0;
    receiver_input[2] = current_count-counter_3;
  }

  //channel 4 of receiver is connected to  pin11
  if(PINB & B00001000){
    if(last_CH4_state == 0){
      last_CH4_state = 1;
      counter_4 = current_count;
    }
  }
  else if(last_CH4_state == 1){
    last_CH4_state = 0;
    receiver_input[3] = current_count-counter_4;
  }

  //channel 5 of receiver is connected to  pin12
  if(PINB & B00010000){
    if(last_CH5_state == 0){
      last_CH5_state = 1;
      counter_5 = current_count;
    }
  }
  else if(last_CH5_state == 1){
    last_CH5_state = 0;
    receiver_input[4] = current_count-counter_5;
  }
}

void print_receiver_raw_input(){
  Serial.print("CH(1-2-3-4-5): ");Serial.print("\t");
  Serial.print(receiver_input[0]);Serial.print("\t"); // roll
  Serial.print(receiver_input[1]);Serial.print("\t"); // pitch
  Serial.print(receiver_input[2]);Serial.print("\t"); // throttle
  Serial.print(receiver_input[3]);Serial.print("\t"); // yaw
  Serial.println(receiver_input[4]);                  // mode/auxiliary(aux)
}

void interpret_receiver_command(){
  // calculating roll setpoint in degree
  roll_setpoint = 0; // angle in degree
  if(receiver_input[0]>roll_input_dead_band_max) roll_setpoint = (receiver_input[0]-roll_input_dead_band_max)*roll_scale_factor_positive;
  else if(receiver_input[0]<roll_input_dead_band_min) roll_setpoint = (receiver_input[0]-roll_input_dead_band_min)*roll_scale_factor_negative;
  roll_setpoint = roll_setpoint + roll_reference;

  // calculating pitch setpoint in degree
  pitch_setpoint = 0; // angle in degree
  if(receiver_input[1]>pitch_input_dead_band_max) pitch_setpoint = (receiver_input[1]-pitch_input_dead_band_max)*pitch_scale_factor_positive;
  else if(receiver_input[1]<pitch_input_dead_band_min) pitch_setpoint = (receiver_input[1]-pitch_input_dead_band_min)*pitch_scale_factor_negative;
  pitch_setpoint = pitch_setpoint + pitch_reference;

  // calculating throttle setpoint in pwm
  throttle_setpoint = throttle_setpoint_min + (receiver_input[2] - throttle_input_min)*throttle_scale_factor ; // in PWM

  // calculating yaw setpoint in degree/sec
  yaw_setpoint = 0; // angle in degree
  if(receiver_input[3]>yaw_input_dead_band_max) yaw_setpoint = (receiver_input[3]-yaw_input_dead_band_max)*yaw_scale_factor_positive;
  else if(receiver_input[3]<yaw_input_dead_band_min) yaw_setpoint = (receiver_input[3]-yaw_input_dead_band_min)*yaw_scale_factor_negative;
  yaw_setpoint = yaw_setpoint + yaw_reference;
  
  mode = receiver_input[4]/1500;

  // detecting arming and disarming of motors using throttle and yaw stick
  // throttle low + yaw right = armed, throttle low + yaw left = disarmed
  if(receiver_input[2]<=arm_throttle_min_1 && receiver_input[3]<=arm_yaw_min){
      arm_status = 0;      
  }
  if(receiver_input[2]<=arm_throttle_min_1 && receiver_input[3]>=arm_yaw_max){
    arm_status = 1;
  }
  if(receiver_input[2]>arm_throttle_min_2 && arm_status==1){
    arm_status = 2;
  }
}

void print_setpoints(){
  Serial.print("t_y_p_r_mode_arm: "); Serial.print("\t");
  Serial.print(throttle_setpoint); Serial.print("\t");   // throttle
  Serial.print(yaw_setpoint); Serial.print("\t");        // yaw
  Serial.print(pitch_setpoint); Serial.print("\t");      // pitch
  Serial.print(roll_setpoint); Serial.print("\t");       // roll
  Serial.print(mode); Serial.print("\t");                // mode
  Serial.println(arm_status);                            // arm_status
}
