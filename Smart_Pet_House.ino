#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

// Define DHT11 parameters
#define DHTPIN 17
#define DHTTYPE DHT11

// Define ultrasonic sensor pins
const int trigPin = 5;
const int echoPin = 16;

// Define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// Define the servo pin
const int servoPin = 18;

// Define the DC motor (vent) pin
const int ventPin = 2;

// Define the water level sensor pin
const int waterLevelPin = 34;

// Variables to store distance data
long duration;
float distanceCm;
float distanceInch;

// Initialize the DHT11 sensor
DHT dht(DHTPIN, DHTTYPE);

// Set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// Set LCD address, number of columns, and rows
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// Create a Servo object
Servo myservo;

// Adafruit IO credentials
#define IO_USERNAME  "Smart_Pet_House"
#define IO_KEY       "aio_SXiC07z0LkzYsFy4kIwsADl6u9ud"

// Wi-Fi credentials
const char* ssid = "MHRedmi";
const char* password = "Masato1230";

// Initialize the Wi-Fi client
WiFiClient client;

// Initialize the Adafruit IO MQTT client
Adafruit_MQTT_Client mqtt(&client, "io.adafruit.com", 1883, IO_USERNAME, IO_KEY);

// Define MQTT feeds
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish ventFeed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/vent");
Adafruit_MQTT_Publish waterLevelFeed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/waterlevel");
Adafruit_MQTT_Publish presenceFeed = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/presence");

// Global variables to store sensor data
float globalHumidity;
float globalTemperature;
int globalWaterLevelPercent;
bool globalVentStatus = false;
String globalPresenceStatus = "Not Present"; // Default to "Not Present"
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 15000; // 15 seconds interval

void connectWiFi() {
  WiFi.begin(ssid, password);
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to Wi-Fi");
  } else {
    Serial.println("Failed to connect to Wi-Fi");
  }
}

void connectMQTT() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      Serial.println("Failed to connect to MQTT");
      return;
    }
  }
  Serial.println("MQTT Connected!");
}

void setup() {
  // Initialize the LCD
  lcd.init();
  // Turn on LCD backlight
  lcd.backlight();
  
  // Initialize the DHT11 sensor
  dht.begin();
  
  // Initialize the serial communication
  Serial.begin(115200);
  
  // Initialize the ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach the servo to the specified pin
  myservo.attach(servoPin);
  myservo.write(0); // Ensure the servo is in the closed position initially
  
  // Initialize the DC motor (vent) pin
  pinMode(ventPin, OUTPUT);
  digitalWrite(ventPin, LOW); // Ensure the vent is off initially

  // Initialize the water level sensor pin
  pinMode(waterLevelPin, INPUT);

  // Connect to Wi-Fi
  connectWiFi();

  // Connect to Adafruit IO
  connectMQTT();
}

void loop() {
  // Check if MQTT is connected, if not, reconnect
  connectMQTT();
  
  // Read humidity and temperature from the DHT11 sensor
  globalHumidity = dht.readHumidity();
  globalTemperature = dht.readTemperature();

  // Check if any reads failed and exit early
  if (isnan(globalHumidity) || isnan(globalTemperature)) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    delay(2000);  // Wait a few seconds before retrying
    lcd.clear();
    return;
  }

  // Display temperature on LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(globalTemperature);
  lcd.print(" C");

  // Display humidity on LCD
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(globalHumidity);
  lcd.print(" %");

  // Ultrasonic sensor distance measurement
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
  distanceInch = distanceCm * CM_TO_INCH;

  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);

  // Check if the distance is less than 5 cm and open the door if true
  if (distanceCm < 5) {
    myservo.write(90); // Open the door (adjust the angle as needed)
    globalPresenceStatus = "Pet Present"; // Update presence status
  } else {
    myservo.write(0); // Close the door
    globalPresenceStatus = "Not Present"; // Update presence status
  }

  // Read water level sensor value (analog)
  int waterLevel = analogRead(waterLevelPin);
  
  // Map the analog value to a percentage
  globalWaterLevelPercent = map(waterLevel, 0, 4095, 0, 100);

  // Display water level sensor value in the Serial Monitor
  Serial.print("Water Level: ");
  Serial.print(globalWaterLevelPercent);
  Serial.println(" %");

  // Turn on the vent if the temperature exceeds a certain threshold
  if (globalTemperature > 20) { // Example threshold
    digitalWrite(ventPin, HIGH); // Turn on the vent
    globalVentStatus = true;
  } else {
    digitalWrite(ventPin, LOW); // Turn off the vent
    globalVentStatus = false;
  }

  // Check if it's time to publish data
  unsigned long currentTime = millis();
  if (currentTime - lastPublishTime > publishInterval) {
    // Publish data to Adafruit IO
    if (!humidityFeed.publish((float)globalHumidity)) {
      Serial.println("Failed to publish humidity");
    }
    if (!tempFeed.publish((float)globalTemperature)) {
      Serial.println("Failed to publish temperature");
    }
    if (!waterLevelFeed.publish((int32_t)globalWaterLevelPercent)) {
      Serial.println("Failed to publish water level");
    }
    if (!ventFeed.publish((int32_t)globalVentStatus)) {
      Serial.println("Failed to publish vent status");
    }
    if (!presenceFeed.publish(globalPresenceStatus.c_str())) { // Publish presence status
      Serial.println("Failed to publish presence status");
    }
    // Update the last publish time
    lastPublishTime = currentTime;
  }

  // Wait 1 second between each sensor read loop
  delay(1000);
  lcd.clear();
}
