/*
  metal_detector_nano_example.ino
  --------------------------------------------------------
  Example for Arduino Nano/UNO.
  Send a clean serial line that Jetson can parse:
      METAL,1,adc_value
      METAL,0,adc_value
*/

const int SENSOR_PIN = A0;
const int LED_PIN = 13;
const int THRESHOLD = 600;     // tune for your coil/amplifier
const int N_SAMPLES = 8;

int readFilteredADC() {
  long sum = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    sum += analogRead(SENSOR_PIN);
    delay(2);
  }
  return sum / N_SAMPLES;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  int adc = readFilteredADC();
  int hit = (adc >= THRESHOLD) ? 1 : 0;

  digitalWrite(LED_PIN, hit ? HIGH : LOW);

  Serial.print("METAL,");
  Serial.print(hit);
  Serial.print(",");
  Serial.println(adc);

  delay(50);   // 20 Hz
}
