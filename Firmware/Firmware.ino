#include <Wire.h>  // Included by Arduino IDE
#include <LiquidCrystal_I2C.h>  // Download it here: http://electronoobs.com/eng_arduino_liq_crystal.php
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Constants for the new temperature reading method
#define HISTORY_SIZE 5
#define RT0 100000   // Ω
#define B 3950       // K
#define VCC 3.3        // Supply voltage
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
float temp_setpoint = 0;

// Inputs and Outputs
int but_3 = 10; 
int but_4 = 9;
int SSR = 4;
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
float preheat_setpoint = 100;  // Mode 1 preheat ramp value is 100ºC
float soak_setpoint = 150;  // Mode 1 soak is 150ºC for a few seconds
float reflow_setpoint = 180;  // Mode 1 reflow peak is 180ºC
float cooldown_setpoint = 0;
// Time zones
float preheat_time = 60;  // Preheat time in seconds
float soak_time = 90;     // Soak time in seconds
float reflow_time = 40;   // Reflow time in seconds
float cooling_rate = 3.0; // Cooling rate in degrees per second
float min_temp = 0; // Temperature to reach for the cooldown
float cooldown_time = 10;

float total_time_before_cooling = preheat_time + soak_time + reflow_time;

enum ReflowState {
  IDLE,
  PREHEAT_WARMUP,
  PREHEAT,
  SOAK_WARMUP,
  SOAK,
  REFLOW_WARMUP,
  REFLOW,
  COOLDOWN
};

ReflowState current_state = IDLE;
unsigned long state_start_time = 0;

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud
  //Define the pins as outputs or inputs
  pinMode(SSR, OUTPUT);
  digitalWrite(SSR, HIGH);        //Make sure we start with the SSR OFF (is off with HIGH)
  pinMode(buzzer, OUTPUT); 
  digitalWrite(buzzer, LOW);  
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
  Setpoint = 150;  // PID Calibration temperature
  aTune.SetOutputStep(50);
  aTune.SetControlType(1);
  aTune.SetNoiseBand(1);
  aTune.SetLookbackSec(30);
  digitalWrite(SSR, HIGH);  // Make sure SSR is OFF
  myPID.SetOutputLimits(MIN_PID_VALUE, MAX_PID_VALUE); // Set output limits for PID
  /////////////////////////////////////////////////////////

  analogReference(EXTERNAL);
}

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

  // Check for outliers and update history arrays accordingly
  if (abs(newTempA0 - avgA0) > (2.0 * stdDevA0)) {
    // Outlier detected for sensor A0, ignore this reading and keep the last valid value in the history array
    historyA0[historyIndex] = historyA0[(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  } else {
    // Valid reading, update the history array
    historyA0[historyIndex] = newTempA0;
  }

  if (abs(newTempA1 - avgA1) > (2.0 * stdDevA1)) {
    // Outlier detected for sensor A1, ignore this reading and keep the last valid value in the history array
    historyA1[historyIndex] = historyA1[(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE];
  } else {
    // Valid reading, update the history array
    historyA1[historyIndex] = newTempA1;
  }

  // Update history index
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  // Compute the final temperature using exponential moving average for smoothing
  float finalTemp = (exponentialMovingAverage(historyA0, HISTORY_SIZE) + exponentialMovingAverage(historyA1, HISTORY_SIZE)) / 2.0;

  return finalTemp;
}

float exponentialMovingAverage(float arr[], int size) {
  float alpha = 0.1; // Smoothing factor, adjust based on your needs
  float ema = arr[0]; // Initialize with the first value

  for (int i = 1; i < size; i++) {
    ema = (1 - alpha) * ema + alpha * arr[i];
  }

  return ema;
}

float readRawTemperature(int pin) {
  float voltage = (analogRead(pin) / 1023.0) * VCC;
  float RT = (voltage * R) / (VCC - voltage); 

  float ln = log(RT / RT0);
  float TX = (1 / ((ln / B) + (1 / T0))); 
  TX = TX - 273.15; // Convert Kelvin to Celsius

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

char line1[16], line2[16];
float prev_temperature = -1;
float prev_temp_setpoint = -1;
float prev_pwm_value = -1;
int prev_running_mode = -1;
int prev_selected_mode = -1;
float prev_seconds = -1;

void updateDisplay(float temperature, float temp_setpoint, float pwm_value, int running_mode, int selected_mode, float seconds, const char* transition_phase = nullptr) {
  
  char temp_str[6];
  dtostrf(temperature, 5, 1, temp_str);
  
  if (temperature != prev_temperature) {
    sprintf(line1, "T: %s", temp_str);
    lcd.setCursor(0,0);     
    lcd.print(line1);
    prev_temperature = temperature;
  }

  if (transition_phase) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(transition_phase);
    delay(1000); // Delay for a second to allow the user to read the message
    lcd.clear();
  }

  if (running_mode != prev_running_mode) {
    lcd.setCursor(9,0);
    if (running_mode == 0) {
      lcd.print("SSR OFF   ");
    } else if (running_mode == 1) {
      lcd.print("SSR ON    ");
    } else if (running_mode == 10) {
      lcd.setCursor(0,1);     
      lcd.print("    COOLING    ");
    } else if (running_mode == 11) {
      lcd.setCursor(0,1);     
      lcd.print("    COMPLETE    ");
    }
    prev_running_mode = running_mode;
  }

  if (running_mode == 0 && selected_mode != prev_selected_mode) {
    lcd.setCursor(0,1);
    printSelectedMode(selected_mode);
    prev_selected_mode = selected_mode;
  }

  if (running_mode == 1 && (temp_setpoint != prev_temp_setpoint || pwm_value != prev_pwm_value || seconds != prev_seconds)) {
    sprintf(line2, "S%d PWM%d %ds    ", int(temp_setpoint), int(pwm_value), int(seconds));
    lcd.setCursor(0,1); 
    lcd.print(line2);
    prev_temp_setpoint = temp_setpoint;
    prev_pwm_value = pwm_value;
    prev_seconds = seconds;
  }
}

void printSelectedMode(int selected_mode) {
  lcd.print("                "); // Clear the line
  lcd.setCursor(0,1);
  switch(selected_mode) {
    case 0:
      lcd.print("Select Mode");     
      break;

    case 1:
      lcd.print("REFLOW MODE");     
      break;

    case 2:
      lcd.print("PID TUNING");     
      break;

    default:
      // Handle invalid selected_mode values here
      break;
  }
}

void calculatePID(float temperature) {
  //Calculate PID
  PID_ERROR = temp_setpoint - temperature;
  PID_P = Kp*PID_ERROR;
  PID_I = PID_I+(Ki*PID_ERROR);      
  PID_D = Kd * (PID_ERROR-PREV_ERROR);
  PID_Output = PID_P + PID_I + PID_D;
  
  //Define maximum PID values
  if(PID_Output > MAX_PID_VALUE){
    PID_Output = MAX_PID_VALUE;
  }
  else if (PID_Output < MIN_PID_VALUE){
    PID_Output = MIN_PID_VALUE;
  }
  
  //Since the SSR is ON with LOW, we invert the pwm signal
  pwm_value = 255 - PID_Output;
  
  analogWrite(SSR, pwm_value);           //We change the Duty Cycle applied to the SSR
  
  PREV_ERROR = PID_ERROR;
}

void loop() {

  static float last_temp_setpoint = -1;

  millis_now = millis();
  if(millis_now - millis_before_2 > pid_refresh_rate){    //Refresh rate of the PID
    millis_before_2 = millis(); 

    const char* transition_message = nullptr; // Add this line to hold the transition message
    
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

    if (running_mode == 1) {
      myPID.SetMode(AUTOMATIC);
      if (current_state == IDLE) {
        current_state = PREHEAT_WARMUP;
        temp_setpoint = preheat_setpoint;
        state_start_time = millis();
      }

        unsigned long elapsed_time = millis() - state_start_time;

        // State machine logic
        switch (current_state) {

          case PREHEAT_WARMUP:
            if (temperature >= temp_setpoint) {
              current_state = PREHEAT;
              state_start_time = millis(); // Start the timer when the target temperature is reached
            }
            Serial.print("Preheat Warmup\n");
            calculatePID(temperature);
            break;

          case PREHEAT:
            if (elapsed_time >= preheat_time * 1000) {
              current_state = SOAK_WARMUP;
              temp_setpoint = soak_setpoint;
              state_start_time = 0;
            }
            Serial.print("Preheat Phase - Waiting ");Serial.print(preheat_time);Serial.print(" sec\n");
            calculatePID(temperature);
            break;

          case SOAK_WARMUP:
            if (temperature >= temp_setpoint) {
              current_state = SOAK;
              state_start_time = millis();
            }
            Serial.print("Soak Warmup\n");
            calculatePID(temperature);
            break;

          case SOAK:
            if (elapsed_time >= soak_time * 1000) {
              current_state = REFLOW_WARMUP;
              temp_setpoint = reflow_setpoint;
              state_start_time = 0;
            }
            Serial.print("Soak Phase - Waiting ");Serial.print(soak_time);Serial.print(" sec\n");
            calculatePID(temperature);
            break;

          case REFLOW_WARMUP:
            if (temperature >= temp_setpoint) {
              current_state = REFLOW;
              state_start_time = millis();
            }
            Serial.print("Reflow Warmup\n");
            calculatePID(temperature);
            break;

          case REFLOW:
            if (elapsed_time >= reflow_time * 1000) {
              current_state = COOLDOWN;
              temp_setpoint = cooldown_setpoint;
              state_start_time = 0;
            }
            Serial.print("Reflow Phase - Waiting ");Serial.print(reflow_time);Serial.print(" sec\n");
            calculatePID(temperature);
            break;

          case COOLDOWN:
            if (elapsed_time >= cooldown_time * 1000) {
              temp_setpoint = cooldown_setpoint;
              state_start_time = 0;
              running_mode = 10;
              selected_mode = 0;
              state_start_time = 0;
            }
            Serial.print("Cooldown Phase\n");
            analogWrite(SSR, 255);
            break;
        }

    } else {
      current_state = IDLE;
      temp_setpoint = 0;
      state_start_time = 0;
    }

    //Mode 10 is between reflow and cooldown
    if(running_mode == 10) {
      if(temp_setpoint > min_temp) {
        temp_setpoint -= cooling_rate;
      } else {
        // The temperature has reached the minimum value, so you can turn off the SSR
        digitalWrite(SSR, HIGH);
        lcd.clear();
        lcd.setCursor(0,1);     
        lcd.print("    COMPLETE    ");
        tone(buzzer, 1800, 1000);    
        seconds = 0;              //Reset timer
        running_mode = 11;
        delay(3000);
      }
    }
  }//End of > millis_before_2 (Refresh rate of the PID code)
  

  
  millis_now = millis();
  if(millis_now - millis_before > refresh_rate){          //Refresh rate of prntiong on the LCD
    millis_before = millis();   
    seconds = seconds + (refresh_rate/1000);              //We count time in seconds

    const char* transition_message = nullptr; // Add this line to hold the transition message

    // Detect phase transitions and set the transition message
    if (temp_setpoint != last_temp_setpoint) {
      if (temp_setpoint == preheat_setpoint) {
        transition_message = "PREHEAT PHASE";
      } else if (temp_setpoint == soak_setpoint) {
        transition_message = "SOAK PHASE";
      } else if (temp_setpoint == reflow_setpoint) {
        transition_message = "REFLOW PHASE";
      } else if (temp_setpoint == cooldown_temp) {
        transition_message = "COOL DOWN PHASE";
      }
      last_temp_setpoint = temp_setpoint; // Update the last setpoint
    }

    updateDisplay(temperature, temp_setpoint, pwm_value, running_mode, selected_mode, seconds, transition_message);

     //Mode 11 is cooldown. SSR is OFF
     if(running_mode == 11){ 
      if(temperature < cooldown_temp){
        running_mode = 0; 
        tone(buzzer, 1000, 100); 
      }
      digitalWrite(SSR, HIGH);
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
          seconds = 0;  
        }
      }
    }
  }
  lastBut_4State = but_4Reading;
}