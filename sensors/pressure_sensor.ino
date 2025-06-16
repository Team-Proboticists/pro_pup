uint8_t pins[4] = { A0, A1, A2, A3 };
uint8_t data[4];

void setup() {
  // put your setup code here, to run once:
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(pins[i], INPUT);
  }
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(analogRead(pins[i]));
    Serial.print(" ");
  }
  Serial.println();
}
