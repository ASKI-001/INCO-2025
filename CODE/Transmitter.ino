#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <math.h>

// GPS Module
// RX --> 3  TX --> 4
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// HC-12 Module
// RX --> 8  TX --> 9
AltSoftSerial hc12; 

// Hotspot Location 6.809216606324435, 79.9913294765288
const double refLatitude = 6.900513;   // latitude
const double refLongitude = 79.871794; // longitude

unsigned long previousMillis = 0;
const long interval = 2; // Signal Transmit lag 2ms
double lastDistance = 0.0; 

void setup()
{
    Serial.begin(9600);
    ss.begin(GPSBaud);
    hc12.begin(9600);

    Serial.println(F("GPS Turned On"));
    //Serial.print(F("Testing TinyGPS++ library v. "));
    //Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println();
}

void loop()
{
    // Process GPS data
    while (ss.available() > 0)
    {
        char c = ss.read();
        if (gps.encode(c)) // Process the GPS data
        {
            // ChCheck GPS location is valid
            if (gps.location.isValid())
            {
                // Get current latitude and longitude
                double currentLatitude = gps.location.lat();
                double currentLongitude = gps.location.lng();

                // Calculate distance in meters
                double distance = calculateDistance(refLatitude, refLongitude, currentLatitude, currentLongitude);

                // Display information
                displayInfo(currentLatitude, currentLongitude, distance);

                // Store the calculated distance
                lastDistance = distance;
            }
            else
            {
                Serial.println(F("Waiting for GPS Signal......."));
            }
        }
    }

    // If GPS Module is not detected
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS Detectod Check Wiring"));
        while (true);
    }
    if (lastDistance < 500 && lastDistance != 0)
    {
        transmitDistance(lastDistance);
    }
}

void displayInfo(double lat, double lng, double distance)
{
    /*Serial.print(F("Location: "));
    Serial.print(lat, 15);
    Serial.print(F(","));
    Serial.print(lng, 15);

    Serial.print(F("  Distance from reference: "));
    Serial.println(distance, 4); // Display distance with 2 decimal places
    Serial.println(F(" meters"));
    */
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    // Earth's radius in meters
    const double earthRadius = 6371000.0;

    // Convert latitude and longitude from degrees to radians
    lat1 = degToRad(lat1);
    lon1 = degToRad(lon1);
    lat2 = degToRad(lat2);
    lon2 = degToRad(lon2);

    // Difference in coordinates
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;

    // Haversine formula
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1) * cos(lat2) *
                   sin(dLon / 2.0) * sin(dLon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    // Calculate distance in meters
    double distance = earthRadius * c;

    return distance;
}

// Renamed function to avoid conflict with the `radians` macro
double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

void transmitDistance(double distance)
{
    
    
    // Transmit the distance
    hc12.print(distance);
    hc12.print("\n"); 
    // Display the transmitted distance
    Serial.print(distance);
    Serial.print("\n"); // Add newline character
}
