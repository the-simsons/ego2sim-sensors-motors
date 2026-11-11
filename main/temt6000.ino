const int IN1 = A0;

const int NUM_SAMPLES = 10;
const float ALPHA = 0.2;

int samples[NUM_SAMPLES];
int sampleIndex = 0;
long sampleSum = 0;
float filtered = 0;

void setup() {
  Serial.begin(115200);
  pinMode(IN1, INPUT);

  int first = analogRead(IN1);
  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = first;
  }
  sampleSum = (long)first * NUM_SAMPLES;
  filtered = first;
  return;
}

void loop() {
  int light = analogRead(IN1);

  // Moving average
  sampleSum -= samples[sampleIndex];
  samples[sampleIndex] = light;
  sampleSum += light;
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;
  float avg = (float)sampleSum / NUM_SAMPLES;

  // Exponential moving average
  filtered = ALPHA * avg + (1.0 - ALPHA) * filtered;

  Serial.print(light);      // cap only (hardware)
  Serial.print(" ");
  Serial.println(filtered); // cap + software
  delay(5);
  return;
}
