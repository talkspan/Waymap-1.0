#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <QMC5883LCompass.h> // or HMC5883L library depending on your sensor

#define RXPin 4
#define TXPin 3
#define GPSBaud 9600

SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;
QMC5883LCompass compass;

struct Waypoint {
  double lat;
  double lon;
};
const int maxWaypoints = 20;
Waypoint waypoints[maxWaypoints];
int waypointCount = 0;
int currentWaypoint = 0;

float smoothedLat = 0.0, smoothedLon = 0.0;
const float alpha = 0.1; // Smoothing factor

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  compass.init();
  compass.setCalibration(-1579, 2433, -1402, 3297, -3771, -153);
  Serial.println("Waiting for waypoint JSON...");
  while (!loadWaypointsFromSerial());
  Serial.println("Waypoints loaded!");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    smoothedLat = alpha * gps.location.lat() + (1 - alpha) * smoothedLat;
    smoothedLon = alpha * gps.location.lng() + (1 - alpha) * smoothedLon;

    float destLat = waypoints[currentWaypoint].lat;
    float destLon = waypoints[currentWaypoint].lon;

    double dist = TinyGPSPlus::distanceBetween(smoothedLat, smoothedLon, destLat, destLon);

    if (dist < 2.5) {
      Serial.print("Reached waypoint #");
      Serial.println(currentWaypoint + 1);
      currentWaypoint++;
      if (currentWaypoint >= waypointCount) {
        Serial.println("All waypoints completed.");
        stopMotors();
        while (true); // End
      }
      return;
    }

    float desiredHeading = TinyGPSPlus::courseTo(smoothedLat, smoothedLon, destLat, destLon);
    compass.read();
    float currentHeading = compass.getAzimuth();
    float headingError = desiredHeading - currentHeading;

    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    adjustMotors(headingError);
  }
}

void adjustMotors(float error) {
  // Example: range -180 to 180 → adjust left/right motor speeds
  Serial.print("Heading error: ");
  Serial.println(error);
  // Replace this with real motor control code
}

void stopMotors() {
  Serial.println("Stopping motors.");
  // Replace with real motor stop logic
}

// Parses simple JSON like [{ "lat": ..., "lon": ... }, ...]
bool loadWaypointsFromSerial() {
  String json = "";
  unsigned long start = millis();
  while (millis() - start < 10000) {
    if (Serial.available()) {
      char c = Serial.read();
      json += c;
    }
  }

  json.trim();
  if (!json.startsWith("[")) return false;

  int idx = 0;
  int startPos = 0;
  while ((startPos = json.indexOf("{", startPos)) != -1 && idx < maxWaypoints) {
    int endPos = json.indexOf("}", startPos);
    String block = json.substring(startPos, endPos + 1);
    double lat = extractNumber(block, "\"lat\":");
    double lon = extractNumber(block, "\"lon\":");
    waypoints[idx++] = {lat, lon};
    startPos = endPos + 1;
  }
  waypointCount = idx;
  return idx > 0;
}

double extractNumber(String src, String key) {
  int idx = src.indexOf(key);
  if (idx == -1) return 0;
  idx += key.length();
  int end = idx;
  while (end < src.length() && (isdigit(src[end]) || src[end] == '.' || src[end] == '-')) end++;
  return src.substring(idx, end).toDouble();
}
