void setup() {
  Serial.begin(250000);
}

void loop() {
  test_micros();
  test_micros();
}

void test_micros(){
  long tests = 1000000;
  long int start_time = micros();
  long int finish_time;
  for(int i = 0; i < tests; i++){
    Serial.println(micros());
  }
  finish_time = micros() - start_time;
  Serial.print("total time: "); Serial.println(finish_time);
  Serial.print(" unit microsecond time per call to millis: "); Serial.println((float) finish_time / tests);
}

