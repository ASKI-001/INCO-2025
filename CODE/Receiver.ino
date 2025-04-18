#include <AltSoftSerial.h>
// The serial connection to the HC-12 module
AltSoftSerial hc12;
String receivedData = ""; // Buffer to store incoming data

const int red = 4;    // Red LED -- pin 4
const int yellow = 6; // Yellow LED -- pin 6
const int blue = 5;   // Blue LED -- pin 5

void setup()
{
    pinMode(red, OUTPUT);
    pinMode(yellow, OUTPUT);
    pinMode(blue, OUTPUT);
    Serial.begin(9600);
    hc12.begin(9600);
}

void loop()
{
    // Check HC-12
    while (hc12.available())
    {
        // Read the incoming data
        char incomingChar = hc12.read();
        receivedData += incomingChar;

        // Check if the incoming character is a newline character
        if (incomingChar == '\n')
        {
            //received data --> float
            float receivedFloat = receivedData.toFloat();

           
            Serial.println(receivedFloat);

            //Danger Zone
            if (receivedFloat < 100)
            {
                digitalWrite(red, HIGH);
                digitalWrite(yellow, LOW);
                digitalWrite(blue, LOW);
            }//Warning Zone
            else if (receivedFloat < 500 && receivedFloat > 100)
            {
                digitalWrite(red, LOW);
                digitalWrite(yellow, HIGH);
                digitalWrite(blue, LOW);
            }//Safe Zone
            else if (receivedFloat >= 500)
            {
                digitalWrite(red, LOW);
                digitalWrite(yellow, LOW);
                digitalWrite(blue, HIGH);
            }
            // Clear the buffer
            receivedData = "";
        }
        // delay(1000);
    }
}
