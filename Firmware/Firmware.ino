#include <Wire.h>  // Included by Arduino IDE
#include <LiquidCrystal_I2C.h>  // Download it here: http://electronoobs.com/eng_arduino_liq_crystal.php
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Constants for the new temperature reading method
#define HISTORY_SIZE 5
#define RT0 100000   // Ω
#define B 3950       // K
#define VCC 5        // Supply voltage
#define R 100000     // R=100KΩ

// LCD Initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Define LCD address as 0x27. Also try 0x3f if it doesn't work.

// PID Settings
float Kp = 2;        // Proportional gain. Adjust based on your system's response.
float Ki = 0.0025;   // Integral gain. Adjust based on your system's response.
float Kd = 9;        // Derivative gain. Adjust based on your system's response.
float MAX_PID_VALUE = 180;  // Max PID output value. Adjust based on your system's capabilities.

// PID Initialization
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune aTune(&Input, &Output);
bool tuning = false;

// PID Variables
float PID_Output = 0;
float PID_P, PID_I, PID_D;
float PID_ERROR, PREV_ERROR;
int setTemperature = 150;
int desiredCycles = 5;
int cycleCounter = 0; 

// Inputs and Outputs
int but_1 = 12;
int but_2 = 11;
int but_3 = 10; 
int but_4 = 9;
int SSR = 3;
int buzzer = 6;
int Thermistor1_PIN = A0;
int Thermistor2_PIN = A1;  // Changed from A0 to A1

// Buttons debouncing
bool lastBut_3State = HIGH;
bool lastBut_4State = HIGH;
unsigned long lastDebounceTime_3 = 0;
unsigned long lastDebounceTime_4 = 0;
const long debounceDelay = 50;

// Variables
unsigned int millis_before, millis_before_2;  // We use these to create the loop refresh rate
unsigned int millis_now = 0;
float seconds = 0;  // Variable used to store the elapsed time
int running_mode = 0;  // We store the running selected mode here
int selected_mode = 0;  // Selected mode for the menu
int max_modes = 2;  // For now, we only work with 1 mode...
bool but_3_state = true;  // Store the state of the button (HIGH OR LOW)
bool but_4_state = true;  // Store the state of the button (HIGH OR LOW)
float temperature = 0;  // Store the temperature value here
float pwm_value = 255;  // The SSR is OFF with HIGH, so 255 PWM would turn OFF the SSR
float MIN_PID_VALUE = 0;
float cooldown_temp = 40;  // When is ok to touch the plate
unsigned long lastPrintTime = 0; 

// Temperature Reading Method Variables
float T0 = 25 + 273.15;
float historyA0[HISTORY_SIZE];
float historyA1[HISTORY_SIZE];
int historyIndex = 0;
bool firstRun = true;
bool idleState = true;

// ================== USER CONFIGURABLE SETTINGS ==================
// Screen Refresh Settings
float refresh_rate = 500;  // LCD refresh rate in milliseconds. Adjust as necessary.

// Temperature Reading Interval
float pid_refresh_rate = 50;  // PID Refresh rate in milliseconds. Adjust as necessary.

// Reflow Settings
float preheat_setpoint = 140;  // Mode 1 preheat ramp value is 140-150ºC
float soak_setpoint = 150;  // Mode 1 soak is 150ºC for a few seconds
float reflow_setpoint = 130;  // Mode 1 reflow peak is 130ºC
float temp_setpoint = 0;  // Used for PID control

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud
  //Define the pins as outputs or inputs
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, HIGH);        //Make sure we start with the SSR OFF (is off with HIGH)
  pinMode(buzzer, OUTPUT); 
  digitalWrite(buzzer, LOW);  
  pinMode(but_1, INPUT_PULLUP);
  pinMode(but_2, INPUT_PULLUP);
  pinMode(but_3, INPUT_PULLUP);
  pinMode(but_4, INPUT_PULLUP);
  pinMode(Thermistor1_PIN, INPUT);
  pinMode(Thermistor2_PIN, INPUT);

  lcd.init();
  lcd.backlight();
  tone(buzzer, 1800, 200);     
  millis_before = millis();
  millis_now = millis();
  T0 = 25 + 273.15;  // Initialize T0 for the new temperature reading method

  ///////////////////////PID///////////////////////////
  Setpoint = 150;  // Your desired temperature
  aTune.SetOutputStep(50);
  aTune.SetControlType(1);
  aTune.SetNoiseBand(1);
  aTune.SetLookbackSec(30);
  digitalWrite(SSR, HIGH);  // Make sure SSR is OFF
  myPID.SetOutputLimits(MIN_PID_VALUE, MAX_PID_VALUE); // Set output limits for PID
  /////////////////////////////////////////////////////////
}

// Function to read temperature using the new method
float readTemperature() {
  // Read from both sensors
  float newTempA0 = readRawTemperature(Thermistor1_PIN);
  float newTempA1 = readRawTemperature(Thermistor2_PIN);

  // Initialize history with first reading
  if (firstRun) {
    for (int i = 0; i < HISTORY_SIZE; i++) {
      historyA0[i] = newTempA0;
      historyA1[i] = newTempA1;
    }
    firstRun = false;
  }

  // Calculate average and standard deviation
  float avgA0 = average(historyA0, HISTORY_SIZE);
  float avgA1 = average(historyA1, HISTORY_SIZE);
  float stdDevA0 = standardDeviation(historyA0, HISTORY_SIZE, avgA0);
  float stdDevA1 = standardDeviation(historyA1, HISTORY_SIZE, avgA1);

  // Update history arrays
  historyA0[historyIndex] = newTempA0;
  historyA1[historyIndex] = newTempA1;

  // Check for rapid changes and take appropriate action
  if (abs(newTempA0 - avgA0) > (2.0 * stdDevA0) || abs(newTempA1 - avgA1) > (2.0 * stdDevA1)) {
    // Rapid change detected, take appropriate action (e.g., log a warning, trigger an alert, etc.)
  }

  // Update history index
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  // Compute the final temperature
  float finalTemp = (average(historyA0, HISTORY_SIZE) + average(historyA1, HISTORY_SIZE)) / 2.0;

  return finalTemp;
}

float readRawTemperature(int pin) {
  float VRT = analogRead(pin);              
  VRT  = (VCC / 1023.00) * VRT;      
  float VR = VCC - VRT;
  float RT = VRT / (VR / R);               
  float ln = log(RT / RT0);
  float TX = (1 / ((ln / B) + (1 / T0))); 
  TX =  TX - 273.15;                 
  return TX;
}

float average(float arr[], int size) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

float standardDeviation(float arr[], int size, float mean) {
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += pow(arr[i] - mean, 2);
  }
  return sqrt(sum / size);
}

void loop() {
  millis_now = millis();
  if(millis_now - millis_before_2 > pid_refresh_rate){    //Refresh rate of the PID
    millis_before_2 = millis(); 
    
    temperature = readTemperature();
    Input = temperature;  // Update Input with the current temperature

    if (tuning) {
      int val = aTune.Runtime();
      Serial.println(Input);  // Send the current temperature to the Serial Plotter

      if (millis() - lastPrintTime >= 5000) {  // Check if 5 seconds have passed
        // Print the intermediate values of Kp, Ki, and Kd
        Serial.print("Current Kp: ");
        Serial.print(aTune.GetKp());
        Serial.print(" Ki: ");
        Serial.print(aTune.GetKi());
        Serial.print(" Kd: ");
        Serial.println(aTune.GetKd());

        lastPrintTime = millis();  // Update the last print time
      }

      if (val != 0) {
        tuning = false;
        
        // Display the new PID values in the Serial Monitor
        Serial.print("Tuning complete! Kp: ");
        Serial.print(aTune.GetKp());
        Serial.print(" Ki: ");
        Serial.print(aTune.GetKi());
        Serial.print(" Kd: ");
        Serial.println(aTune.GetKd());
        
        // Update PID parameters and switch back to AUTOMATIC mode
        myPID.SetTunings(aTune.GetKp(), aTune.GetKi(), aTune.GetKd());
        myPID.SetMode(AUTOMATIC);
      }
    } else if (!idleState) {
      myPID.Compute();
      analogWrite(SSR, Output);

      if (temperature >= setTemperature) {
        cycleCounter++;
        
        if (cycleCounter >= desiredCycles) {
          running_mode = 0;
        }
      }
    }
    
    if(running_mode == 1){   
      if(seconds == 0) {
        temp_setpoint = temperature;  // Set the initial setpoint to the current temperature
      } else {
        if(temperature < preheat_setpoint){
          temp_setpoint = seconds*1.666;  // Reach 150ºC till 90s (150/90=1.666)
        }  
          
        if(temperature > preheat_setpoint && seconds < 90){
          temp_setpoint = soak_setpoint;               
        }   
          
        else if(seconds > 90 && seconds < 110){
          temp_setpoint = reflow_setpoint;                 
        }
      }
       
      //Calculate PID
      PID_ERROR = temp_setpoint - temperature;
      PID_P = Kp*PID_ERROR;
      PID_I = PID_I+(Ki*PID_ERROR);      
      PID_D = Kd * (PID_ERROR-PREV_ERROR);
      PID_Output = PID_P + PID_I + PID_D;
      //Define maximun PID values
      if(PID_Output > MAX_PID_VALUE){
        PID_Output = MAX_PID_VALUE;
      }
      else if (PID_Output < MIN_PID_VALUE){
        PID_Output = MIN_PID_VALUE;
      }
      //Since the SSR is ON with LOW, we invert the pwm singal
      pwm_value = 255 - PID_Output;
      
      analogWrite(SSR,pwm_value);           //We change the Duty Cycle applied to the SSR
      
      PREV_ERROR = PID_ERROR;
      
      if(seconds > 130){
        digitalWrite(SSR, HIGH);            //With HIGH the SSR is OFF
        temp_setpoint = 0;
        running_mode = 10;                  //Cooldown mode        
      }     
    }//End of running_mode = 1


    //Mode 10 is between reflow and cooldown
    if(running_mode == 10){
      lcd.clear();
      lcd.setCursor(0,1);     
      lcd.print("    COMPLETE    ");
      tone(buzzer, 1800, 1000);    
      seconds = 0;              //Reset timer
      running_mode = 11;
      delay(3000);
    }    
  }//End of > millis_before_2 (Refresh rate of the PID code)
  

  
  millis_now = millis();
  if(millis_now - millis_before > refresh_rate){          //Refresh rate of prntiong on the LCD
    millis_before = millis();   
    seconds = seconds + (refresh_rate/1000);              //We count time in seconds
    

    //Mode 0 is with SSR OFF (we can selcet mode with buttons)
    if(running_mode == 0){ 
        digitalWrite(SSR, HIGH);  // With HIGH the SSR is OFF
        myPID.SetMode(MANUAL);  // Deactivate PID here
        lcd.clear();
        lcd.setCursor(0,0);     
        lcd.print("T: ");
        lcd.print(temperature,1);   
        lcd.setCursor(9,0);      
        lcd.print("SSR OFF"); 
        
        lcd.setCursor(0,1);
        if(selected_mode == 0){
          lcd.print("Select Mode");     
        }
        else if(selected_mode == 1){
          lcd.print("REFLOW MODE");     
        }
        else if(selected_mode == 2){
          lcd.print("PID TUNING");     
        }
    }

     //Mode 11 is cooldown. SSR is OFF
     else if(running_mode == 11){ 
      if(temperature < cooldown_temp){
        running_mode = 0; 
        tone(buzzer, 1000, 100); 
      }
      digitalWrite(SSR, HIGH);
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("T: ");
      lcd.print(temperature,1);   
      lcd.setCursor(9,0);      
      lcd.print("SSR OFF"); 
       
      lcd.setCursor(0,1);       
      lcd.print("    COOLDOWN    ");  
    }

    else if(running_mode == 1){            
      lcd.clear();
      lcd.setCursor(0,0);     
      lcd.print("T: ");
      lcd.print(temperature,1);  
      lcd.setCursor(9,0);       
      lcd.print("SSR ON"); 
       
      lcd.setCursor(0,1); 
      lcd.print("S");  lcd.print(temp_setpoint,0); 
      lcd.setCursor(5,1);     
      lcd.print("PWM");  lcd.print(pwm_value,0); 
      lcd.setCursor(12,1); 
      lcd.print(seconds,0);  
      lcd.print("s");         
    }
    
    
  }

  // Button Debouncing for but_3
  bool but_3Reading = digitalRead(but_3);
  if (but_3Reading != lastBut_3State) {
    lastDebounceTime_3 = millis();
  }
  if ((millis() - lastDebounceTime_3) > debounceDelay) {
    if (but_3Reading != but_3_state) {
      but_3_state = but_3Reading;
      if (but_3_state == LOW) {
        selected_mode++;
        tone(buzzer, 2300, 40);
        if(selected_mode > max_modes){
          selected_mode = 0;
        }
      }
    }
  }
  lastBut_3State = but_3Reading;

  // Button Debouncing for but_4
  bool but_4Reading = digitalRead(but_4);
  if (but_4Reading != lastBut_4State) {
    lastDebounceTime_4 = millis();
  }
  if ((millis() - lastDebounceTime_4) > debounceDelay) {
    if (but_4Reading != but_4_state) {
      but_4_state = but_4Reading;
      if (but_4_state == LOW) {
        if(running_mode == 1){
          digitalWrite(SSR, HIGH);        //With HIGH the SSR is OFF
          running_mode = 0;
          selected_mode = 0; 
          tone(buzzer, 2500, 150);
          delay(130);
          tone(buzzer, 2200, 150);
          delay(130);
          tone(buzzer, 2000, 150);
          delay(130);
        }
        
        but_4_state = false;
        if(selected_mode == 0){
          running_mode = 0;
        }
        else if(selected_mode == 1){
          running_mode = 1;
          myPID.SetMode(AUTOMATIC);
          tone(buzzer, 2000, 150);
          delay(130);
          tone(buzzer, 2200, 150);
          delay(130);
          tone(buzzer, 2400, 150);
          delay(130);
          seconds = 0;                    //Reset timer
        }
        else if(selected_mode == 2){
          running_mode = 1;
            if (!tuning) {
                myPID.SetMode(AUTOMATIC);  // Set PID to AUTOMATIC
                tuning = true;
                // No need to explicitly start the autotuning mode
            }
        }
      }
    }
  }
  lastBut_4State = but_4Reading;
}