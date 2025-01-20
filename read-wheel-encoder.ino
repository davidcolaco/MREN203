/**
 * @file read-wheel-encoder.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to read from a digital quadrature encoder.
 * @version 2.0
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2022
 *
 */

// Encoder digital pins
const byte SIGNAL_A = 8;
const byte SIGNAL_B = 9;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;

// Counter to keep track of the last number of ticks [integer]
long encoder_ticks_last = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks()
{
    if (digitalRead(SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks++;
    }
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);

    // Every time SIGNAL_A goes HIGH, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicks, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Do this if the encoder has moved
    if (encoder_ticks != encoder_ticks_last)
    {
        // Print some stuff to the serial monitor
        Serial.print("Encoder ticks: ");
        Serial.print(encoder_ticks);
        Serial.print("\n");

        // Record the current number of encoder ticks
        encoder_ticks_last = encoder_ticks;
    }

    // Short delay [ms]
    delay(100);
}