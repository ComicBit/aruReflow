/****************************************************************************
  sTune PID_v1 Example (MAX31856, PTC Heater / SSR / Software PWM)
  This sketch does on-the-fly tuning and PID control. Tuning parameters are
  quickly determined and applied during temperature ramp-up to setpoint.
  View results using serial plotter.
  Reference: https://github.com/Dlloydev/sTune/wiki/Examples_MAX31856_PTC_SSR
  ****************************************************************************/
#include <sTune.h>
#include <PID_v2.h>

int relayPin = 4;
int Thermistor1_PIN = A0;
int Thermistor2_PIN = A1;

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 100;  // runPid interval = testTimeSec / samples
const uint16_t samples = 500;
const float inputSpan = 200;
const float outputSpan = 1000;
float outputStart = 0;
float outputStep = 50;
float tempLimit = 150;
uint8_t debounce = 1;

// variables
double input, output, setpoint = 50, kp, ki, kd; // PID_v1 
float Input, Output, Setpoint = 50, Kp, Ki, Kd; // sTune

// temperature reading
#define RT0 100000   // Ω
#define B 3950       // K
#define VCC 3.3      // Supply voltage
#define R 100000     // R=100KΩ
bool firstRun = true;
const int HISTORY_SIZE = 2; // or any other size you prefer
float historyA0[HISTORY_SIZE];
float historyA1[HISTORY_SIZE];
int historyIndexA0 = 0;
int historyIndexA1 = 0;

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printOFF);
PID myPID(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);

void setup() {
  pinMode(relayPin, OUTPUT);
  pinMode(Thermistor1_PIN, INPUT);
  pinMode(Thermistor2_PIN, INPUT);
  Serial.begin(9600);
  delay(3000);
  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);
}

void loop() {
  float optimumOutput = tuner.softPwm(relayPin, Input, Output, Setpoint, outputSpan, debounce);

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input = readTemperature();
      tuner.plotter(Input, Output, Setpoint, 0.5f, 1); // output scale 0.5, plot every 3rd sample
      break;

    case tuner.tunings: // active just once when sTune is done
      tuner.GetAutoTunings(&Kp, &Ki, &Kd); // sketch variables updated by sTune
      myPID.SetOutputLimits(0, outputSpan * 0.1);
      myPID.SetSampleTime(outputSpan - 1);
      debounce = 0; // ssr mode
      setpoint = Setpoint, output = outputStep, kp = Kp, ki = Ki, kd = Kd;
      Output = outputStep;
      myPID.SetMode(AUTOMATIC); // the PID is turned on
      myPID.SetTunings(kp, ki, kd); // update PID with the new tunings
      Serial.println("Tuning completed. Applying new tunings...");
      Serial.println("Kp: "); Serial.println(Kp);
      Serial.println("\nKi: "); Serial.println(Ki);
      Serial.println("\nKd: "); Serial.println(Kd);
      break;

    case tuner.runPid: // active once per sample after tunings
      Input = readTemperature();
      Serial.print("Current Temperature: "); Serial.println(Input);
      input = Input;
      myPID.Compute();
      Output = output;
      tuner.plotter(Input, optimumOutput, Setpoint, 0.5f, 3);
      break;
  }
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
  finalTemp = 0.2 * previousTemp + 0.8 * finalTemp; // changed the weights here
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