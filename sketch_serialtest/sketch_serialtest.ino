void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000);
}

void loop() {
  ;
}

void serialEvent() {
  while (Serial.available() > 0) {
    char char_bytes[5];
    int k = Serial.readBytes(char_bytes, 5);
    if (k < 5) {
      Serial.println("Timeout has been exceeded");
      return;
    }
    uint8_t bin_bytes[5];
    for (int i = 0; i < 5; i++)
      bin_bytes[i] = byte(char_bytes[i]);
    Serial.write(bin_bytes, 5);
  }
}
