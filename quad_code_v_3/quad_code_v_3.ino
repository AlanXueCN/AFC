//---------------------------------------------------------------------------------
// time related variables starts
/* SampleFrequency is frequency at which void loop repeats itself
 * dt is time bewteen execution of two consicutive void loop
 * loop_timer is dt in microseconds
 * t stores time stamp at the point it is asigned to micros() function*/
float SampleFrequency = 100, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;
// time related variables ends
//---------------------------------------------------------------------------------

// angle starts
#include<Wire.h>
const int MPU_addr=0x68; 
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float GyXOff, GyYOff, GyZOff;
float acc_sensitivity=4096, gyro_sensitivity=65.5;
float p, q, r;
float phi_d_gyro, th_d_gyro, psi_d_gyro;
float phi_gyro, th_gyro, psi_gyro;
float AcX_P,AcY_P,AcZ_P;
float amx, amy, amz;
float phi_acc, th_acc;
float k1=0.95, k2=0.004;
int start;
// angle ends

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
float arm_throttle_min_2 = throttle_input_min + 92; //1144
float arm_yaw_min = yaw_input_min + 100;
float arm_yaw_max = yaw_input_max - 100;

// reference angles and angular velocity are those values at which vehicles hover at one position without drifting in any direction in horizontal plane
// roll_reference and pitch_reference are in degree(i.e. angle) but yaw_reference is in degree/sec(i.e. angular velocity)
float roll_reference=1.9, pitch_reference=-3.45, yaw_reference=0; 
// receiver ends
//---------------------------------------------------------------------------------


// controller starts
float esc_1=1000, esc_2=1000, esc_3=1000, esc_4=1000;
float pid_p_gain_roll = 2; //1.3               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.03*0; //0.04               //Gain setting for the roll I-controller
float pid_d_gain_roll = 25; // 15(stableoscillation),18(stab+some osc),20 (stab+very less osc), 22.5, 25            //Gain setting for the roll D-controller
int pid_max_roll = 400;

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 1.5; //4.0                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02*0; // 0.02               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0; //0.0                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;

float pid_error_temp, pid_i_mem_roll, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_output_yaw, pid_last_yaw_d_error;
// controller ends

// esc starts
unsigned long start_time_pw, time_now;
unsigned long falling_time_pw_1, falling_time_pw_2, falling_time_pw_3, falling_time_pw_4;
// esc ends

void setup(){
  DDRD |= B11110000; //setting pin 4,5,6,7 for output
  // below 6 lines of code enable hardware interrupt subrouting when state of pin 8, 9, 10, 11, 12 changes for detecting 5 channels of receiver
  PCICR |= (1 << PCIE0);   //enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0); //set pin 8 for detecting state change
  PCMSK0 |= (1 << PCINT1); //set pin 9 for detecting state change
  PCMSK0 |= (1 << PCINT2); //set pin 10 for detecting state change
  PCMSK0 |= (1 << PCINT3); //set pin 11 for detecting state change
  PCMSK0 |= (1 << PCINT4); //set pin 12 for detecting state change
  //Serial.begin(57600);
  Wire.begin();
  TWBR = 12;     
  setup_mpu_6050_registers();
  //calc_gyro_offset();
  GyXOff = -138;
  GyYOff = -33.14;
  GyZOff = -31.12;
  
  t = micros();
}

void loop(){
  calculate_angles();
  //print_angles();
  //print_receiver_raw_input();
  interpret_receiver_command();
  //print_setpoints();

  if(micros()<8000000){
    arm_status = 0;
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }
  else{
    if (arm_status==0){
      esc_1 = 1000;
      esc_2 = 1000;
      esc_3 = 1000;
      esc_4 = 1000;
    }
    else if (arm_status==1){
      esc_1 = arm_throttle_min_2;
      esc_2 = arm_throttle_min_2;
      esc_3 = arm_throttle_min_2;
      esc_4 = arm_throttle_min_2; //1144
    }
    else if (arm_status==2){
      calculate_esc_command();
    }
  }
  command_esc(esc_1,esc_2,esc_3,esc_4);

  while(micros()-t<loop_timer){}
  t = micros();
}

void calculate_angles(){
  read_mpu_6050_data();
  LPF_acc();
  acc_angle_calc();
  gyro_angle_calc();
  gyro_drift_correction();
  //print_angles();
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission(true);                                          //End the transmission
  //low pass filter
  Wire.beginTransmission(MPU_addr);                                    //Start communicating with the MPU-6050
  Wire.write(0x1A);                                                    //Send the requested starting register
  Wire.write(0x03);                                                    //Set the requested starting register
  Wire.endTransmission(true); 
}

void read_mpu_6050_data(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  while(Wire.available() < 14); 
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void LPF_acc(){
  AcX = AcX*(1-k1) + k1*AcX_P;
  AcY = AcY*(1-k1) + k1*AcY_P;
  AcZ = AcZ*(1-k1) + k1*AcZ_P;
  AcX_P = AcX;
  AcY_P = AcY;
  AcZ_P = AcZ;
}

void acc_angle_calc(){
  amx = AcX/acc_sensitivity;
  amy = AcY/acc_sensitivity;
  amz = AcZ/acc_sensitivity;
  
  float g = sqrt(amx*amx + amy*amy + amz*amz);
  phi_acc = atan2(amy,amz);
  th_acc = -asin(amx/g);
}

void calc_gyro_offset(){
  float n=500;
  Serial.println("");
  Serial.println("Calliberating Gyroscope !");
  for(int i=0;i<n;i++){
    if(i % 100 == 0){
      Serial.print("- ");
    }
    read_mpu_6050_data();
    GyXOff=GyXOff+GyX;
    GyYOff=GyYOff+GyY;
    GyZOff=GyZOff+GyZ;
    delay(5); 
  }
  GyXOff=GyXOff/n;
  GyYOff=GyYOff/n;
  GyZOff=GyZOff/n;
  Serial.println("");
  Serial.println("Gyro Caliberation Complete!!");
  Serial.println("Offsets are:");
  Serial.print("GyXOff = "); Serial.println(GyXOff);
  Serial.print("GyYOff = "); Serial.println(GyYOff);
  Serial.print("GyZOff = "); Serial.println(GyZOff);
  Serial.print("\n");
}

void gyro_angle_calc(){
  GyX -= GyXOff;
  GyY -= GyYOff;
  GyZ -= GyZOff;
  p = GyX/gyro_sensitivity;
  q = GyY/gyro_sensitivity;
  r = GyZ/gyro_sensitivity;

  p = p*PI/180;
  q = q*PI/180;
  r = r*PI/180;
  
  phi_d_gyro =  p + (q*sin(phi_gyro) + r*cos(phi_gyro))*tan(th_gyro);
  th_d_gyro  =  q*cos(phi_gyro) - r*sin(phi_gyro);
  psi_d_gyro = (q*sin(phi_gyro) + r*cos(phi_gyro))/cos(th_gyro);
  
  phi_gyro = phi_gyro + phi_d_gyro*dt;
  th_gyro  = th_gyro  + th_d_gyro*dt;
  psi_gyro = psi_gyro + psi_d_gyro*dt;

  if(start==0){
    start = 1;
    phi_gyro = phi_acc;
    th_gyro  = th_acc;
  }
}

void gyro_drift_correction(){
  phi_gyro = phi_gyro + k2*(phi_acc - phi_gyro);
  th_gyro = th_gyro + k2*(th_acc - th_gyro);
}

void print_angles(){
  Serial.print(phi_gyro*180/PI,3); Serial.print("\t");
  Serial.print(th_gyro*180/PI,3); Serial.print("\t");
  Serial.println(psi_gyro*180/PI,3);
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
  yaw_setpoint = -1*yaw_setpoint;
  
  mode = receiver_input[4]/1500;

  // detecting arming and disarming of motors using throttle and yaw stick
  // throttle low + yaw right = armed, throttle low + yaw left = disarmed
  if(receiver_input[2]<arm_throttle_min_1 && receiver_input[3]<arm_yaw_min){
      arm_status = 0;      
  }
  if(receiver_input[2]<arm_throttle_min_1 && receiver_input[3]>arm_yaw_max){
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

void calculate_esc_command(){
  //Roll calculations
  pid_error_temp = roll_setpoint - phi_gyro*57.29578;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pitch_setpoint - th_gyro*57.29578;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = yaw_setpoint - psi_gyro*57.29578;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
  pid_last_yaw_d_error = pid_error_temp;
  
  esc_1 = throttle_setpoint - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_2 = throttle_setpoint + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
  esc_3 = throttle_setpoint + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_4 = throttle_setpoint - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

  // keep motor running to prevent jerk during motor start
  if (esc_1 < throttle_setpoint_min) esc_1 = throttle_setpoint_min;
  if (esc_2 < throttle_setpoint_min) esc_2 = throttle_setpoint_min;
  if (esc_3 < throttle_setpoint_min) esc_3 = throttle_setpoint_min;
  if (esc_4 < throttle_setpoint_min) esc_4 = throttle_setpoint_min;

  // limiting the size of pulse to 2000us
  if(esc_1 > throttle_setpoint_max-20) esc_1 = throttle_setpoint_max-20;
  if(esc_2 > throttle_setpoint_max-20) esc_2 = throttle_setpoint_max-20;
  if(esc_3 > throttle_setpoint_max-20) esc_3 = throttle_setpoint_max-20;
  if(esc_4 > throttle_setpoint_max-20) esc_4 = throttle_setpoint_max-20;
}

void command_esc(unsigned long pw_1,unsigned long pw_2,unsigned long pw_3,unsigned long pw_4){
  PORTD |= B11110000; // set pin 4,5,6,7 High
  start_time_pw = micros(); // record starting time of pulse width
  falling_time_pw_1 = start_time_pw + pw_1; // calculate time of falling edge on pin 4
  falling_time_pw_2 = start_time_pw + pw_2; // calculate time of falling edge on pin 5
  falling_time_pw_3 = start_time_pw + pw_3; // calculate time of falling edge on pin 6
  falling_time_pw_4 = start_time_pw + pw_4; // calculate time of falling edge on pin 7

  while(PORTD >= 16){ // wait till all 4 pulses are set low (NOTE: This will take minimum 1000us and maximum 2000us)
    time_now = micros();                                                //Record the current time
    if(time_now >= falling_time_pw_1)PORTD &= B11101111;                //Set pin 4 to low if pulse width time is reached
    if(time_now >= falling_time_pw_2)PORTD &= B11011111;                //Set pin 5 to low if pulse width time is reached
    if(time_now >= falling_time_pw_3)PORTD &= B10111111;                //Set pin 6 to low if pulse width time is reached
    if(time_now >= falling_time_pw_4)PORTD &= B01111111;                //Set pin 7 to low if pulse width time is reached
  }
}
