#include <Arduino.h>
#include <driver/adc.h>
#include "arduinoFFT.h"

// Constants
#define SAMPLES 256
#define SAMPLING_FREQUENCY 100
#define BUZZER_PIN 25
#define ADC_PIN 34

// Variables
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT;

// Thresholds
const double MIN_BPM = 40.0;
const double MAX_BPM = 100.0;

void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  analogReadResolution(12);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("Heart Rate Monitoring with FFT and AI Filter");
}

double calculateHeartRate() {
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(ADC_PIN) * (3.3 / 4095.0);
    vImag[i] = 0;
    delayMicroseconds(1000000 / SAMPLING_FREQUENCY);
  }

  FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double peakFreq = FFT.majorPeak();
  double bpm = peakFreq * 60;

  return bpm;
}

// Simulated AI check (replace with real ML logic)
bool isValidReading(double bpm) {
  return bpm > MIN_BPM && bpm < MAX_BPM;
}

void loop() {
  double bpm = calculateHeartRate();
  Serial.print("Heart Rate (BPM): ");
  Serial.println(bpm);

  if (isnan(bpm) || bpm <= 0) {
    Serial.println("Invalid BPM detected.");
    return;
  }

  if (!isValidReading(bpm)) {
    Serial.println("Abnormal BPM detected. Activating alarm!");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    Serial.println("Valid BPM.");
    digitalWrite(BUZZER_PIN, LOW);
  }

  delay(1000);
}