#include <AltSoftSerial.h>
AltSoftSerial kayakinator;

const int ledPin = 4; // Pin number where the LED is connected

char c = ' ';
// boolean NL = true;

void setup()
{
    Serial.begin(9600);
    Serial.print("Sketch:   ");   Serial.println(__FILE__);
    Serial.print("Uploaded: ");   Serial.println(__DATE__);
    Serial.println(" ");

    kayakinator.begin(9600);
    Serial.println("kayakinator started at 9600");

    pinMode(ledPin, OUTPUT); // Set the LED pin as an OUTPUT
}

void loop()
{
    // Read from the Bluetooth module and send to the Arduino Serial Monitor
    if (kayakinator.available()){
        c = kayakinator.read();
        Serial.write(c);
        Serial.print("!");

        // Check if received command is "ledon" or "ledoff" and control the LED accordingly
        if (c == '1'){
          Serial.println("Setting to HIGH");

          digitalWrite(ledPin, HIGH); // Turn on the LED
        }
        else if (c == '0'){
          Serial.println("Setting to LOW");
          digitalWrite(ledPin, LOW); // Turn off the LED
        }
      }

    // Read from the Serial Monitor and send to the Bluetooth module
    if (Serial.available())
    {
        c = Serial.read();

        // do not send line end characters to the HM-10
        if (c != 10 && c != 13)
        {
            kayakinator.write(c);
        }

        // Echo the user input to the main window.
        // If there is a new line print the ">" character.
        //if (NL)
        //{
        //    Serial.print("\r\n>");
        //    NL = false;
        //}
        //Serial.write(c);
        //if (c == 10)
        //{
        //    NL = true;
        //}
    }
}
