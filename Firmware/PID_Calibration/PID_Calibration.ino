#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define RT0 100000   // Ω
#define B 3950       // K
#define VCC 3.3      // Supply voltage
#define R 100000     // R=100KΩ

int SSR = 4;
int Thermistor1_PIN = A0;
int Thermistor2_PIN = A1;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 2, 0.0025, 9, DIRECT); // Initial PID values
PID_ATune aTune(&Input, &Output);
float Kp = 2; 
float Ki = 0.0025;
float Kd = 9; 
float PID_Output = 0;
float PID_P, PID_I = 0, PID_D;
float PID_ERROR, PREV_ERROR = 0;
float temperature = 0;
float temp_setpoint = 150;
float pwm_value = 255;
float MAX_PID_VALUE = 180;
float MIN_PID_VALUE = 0;
unsigned long millis_now = 0;
unsigned long millis_before = 0;
float refresh_rate = 500;
float seconds = 0;

bool tuning = true;

unsigned long lastPrintTime = 0;
#define HISTORY_SIZE 5

float T0 = 25 + 273.15;
float historyA0[HISTORY_SIZE];
float historyA1[HISTORY_SIZE];
int historyIndexA0 = 0;
int historyIndexA1 = 0;
bool firstRun = true;

void setup() {
  Serial.begin(9600);
  pinMode(SSR, OUTPUT);
  pinMode(Thermistor1_PIN, INPUT);
  pinMode(Thermistor2_PIN, INPUT);
  digitalWrite(SSR, HIGH);  // Make sure we start with the SSR OFF (is off with HIGH)
  
  // Initiate auto-tuning process here
  aTune.SetOutputStep(50); // Setting the step size for the output during tuning
  aTune.SetControlType(1); // Using PI control type for tuning
  aTune.SetNoiseBand(1); // Setting the noise band
  aTune.SetLookbackSec(30); // Setting the lookback period to observe the max and min values of the input signal

  // Setting the initial setpoint to the target temperature
  temp_setpoint = 70; // replace with your target temperature
}

void loop() {
  millis_now = millis();

  if (millis_now - millis_before > refresh_rate) { // Refresh rate of the PID
    millis_before = millis(); 
    seconds = seconds + (refresh_rate / 1000); // We count time in seconds
    temperature = readTemperature();

    calculatePID(temperature); // Call your PID calculation function

    if(tuning) {
      Serial.print("Tuning mode initiated...");
      // Auto tuning process
      int val = aTune.Runtime();
      if (val != 0) {
        tuning = false;
        // Autotune completed, print the results
        Kp = aTune.GetKp();
        Ki = aTune.GetKi();
        Kd = aTune.GetKd();
        
        // Here, update your PID constants in your calculatePID function or globally
        // ... (update Kp, Ki, Kd wherever they are defined)

        Serial.print("Tuning completed! Kp: "); Serial.print(Kp); 
        Serial.print(" Ki: "); Serial.print(Ki); 
        Serial.print(" Kd: "); Serial.println(Kd);
      }
    }

    // Print Debug Info
    Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" °C, ");
    Serial.print("Setpoint: "); Serial.print(temp_setpoint); Serial.print(" °C, ");
    Serial.print("PWM: "); Serial.println(pwm_value);
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


float readTemperature() {
  float newTempA0 = readRawTemperature(Thermistor1_PIN);
  float newTempA1 = readRawTemperature(Thermistor2_PIN);

  if (firstRun) {
    for (int i = 0; i < HISTORY_SIZE; i++) {
      historyA0[i] = newTempA0;
      historyA1[i] = newTempA1;
    }
    firstRun = false;
  }

  float avgA0 = average(historyA0, HISTORY_SIZE);
  float avgA1 = average(historyA1, HISTORY_SIZE);

  if (abs(newTempA0 - avgA0) <= (0.2 * avgA0)) {
    historyA0[historyIndexA0] = newTempA0;
    historyIndexA0 = (historyIndexA0 + 1) % HISTORY_SIZE;
  }
  if (abs(newTempA1 - avgA1) <= (0.2 * avgA1)) {
    historyA1[historyIndexA1] = newTempA1;
    historyIndexA1 = (historyIndexA1 + 1) % HISTORY_SIZE;
  }

  float finalTemp = (average(historyA0, HISTORY_SIZE) + average(historyA1, HISTORY_SIZE)) / 2.0;
  static float previousTemp = finalTemp;
  finalTemp = 0.8 * previousTemp + 0.2 * finalTemp;
  previousTemp = finalTemp;

  return finalTemp;
}

float readRawTemperature(int pin) {
  float voltage = (analogRead(pin) / 1023.0) * VCC;
  float RT = (voltage * R) / (VCC - voltage); 

  float ln = log(RT / RT0);
  float TX = (1 / ((ln / B) + (1 / (25 + 273.15)))); 
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