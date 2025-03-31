// These should already be declared in your main .ino file, but ensure they're accessible
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;
//volatile long left_encoder_count = 0;
//volatile long right_encoder_count = 0;

void leftEncoderEvent() {
  if (digitalRead(3) == LOW) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

void rightEncoderEvent() {
  if (digitalRead(9) == LOW) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
}

void initEncoders() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);

  attachInterrupt(digitalPinToInterrupt(2), leftEncoderEvent, RISING);
  attachInterrupt(digitalPinToInterrupt(8), rightEncoderEvent, RISING);
}
void leftEncoderISR() {
    // handle left encoder logic here
    // Basic quadrature decoding using SIGNAL_A and SIGNAL_B
  if (digitalRead(3) == LOW) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

void rightEncoderISR() {
    // handle right encoder logic here
    if (digitalRead(9) == LOW) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }

}
long readEncoder(int i) {
  if (i == LEFT) return left_encoder_ticks;
  else return right_encoder_ticks;
}

void resetEncoders() {
  left_encoder_ticks = 0;
  right_encoder_ticks = 0;
}

