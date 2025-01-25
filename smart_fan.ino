#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//Define Pins and variables for Smart Fan
#define DHTPIN PB9        // DHT11 data output pin 
#define DHTTYPE DHT11     // Define DHT11 sensor type in DHT.h header file
#define LED_LOW PA1       // Low temperature range LED
#define LED_MEDIUM PA4    // Medium temperature range LED on A4
#define LED_HIGH PA7      // High temperature range LED on A7
#define FAN_INA PB12      // L9110 fan module INA pin
#define FAN_INB PB13      // L9110 fan module INB pin
#define IR_SENSOR PB10   // IR sensor input pin
#define STATUS_LED PA8    // Status LED to indicate system ON/OFF

const float TEMP_LOW_MAX = 30.0;    // Max temperature for low range
const float TEMP_MEDIUM_MIN = 30.0; // Min temperature for medium range
const float TEMP_MEDIUM_MAX = 35.0; // Max temperature for medium range
const float TEMP_HIGH_MIN = 35.0;   // Min temperature for high range

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

bool systemOn = false;      // System toggle state
unsigned long lastDebounceTime = 0; // For IR sonsor debounce
const unsigned long debounceDelay = 1000; // 1s debounce delay
int lastActiveRange = 0;    // Store the last active temperature range (1 = low, 2 = medium, 3 = high)

//Setup system and initialize functions
void setup() {
  dht.begin();
  pinMode(LED_LOW, OUTPUT);
  pinMode(LED_MEDIUM, OUTPUT);
  pinMode(LED_HIGH, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(FAN_INA, OUTPUT);
  pinMode(FAN_INB, OUTPUT);
  pinMode(IR_SENSOR, INPUT_PULLUP);
  digitalWrite(LED_LOW, LOW);
  digitalWrite(LED_MEDIUM, LOW);
  digitalWrite(LED_HIGH, LOW);
  digitalWrite(STATUS_LED, LOW);
  analogWrite(FAN_INA, 0); 
  analogWrite(FAN_INB, 0); 
}

// Check IR sensor button to toggle input
void loop() {
  int buttonState = digitalRead(IR_SENSOR);
  static int lastButtonState = HIGH;

  // Debounce the button to avoid noise affecting input
  if (buttonState == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    systemOn = !systemOn;       
    lastDebounceTime = millis(); 
  }
  lastButtonState = buttonState;

  // Status LED: Indicate system ON/OFF state
  digitalWrite(STATUS_LED, systemOn ? HIGH : LOW);

  if (!systemOn) {
    // If system is OFF, turn off the fan but leave temperature LEDs on 
    analogWrite(FAN_INA, 0); 
    analogWrite(FAN_INB, 0); 
    return; 
  }

  //If system is ON: Read temperature and control LEDs and fan
  float temperature = dht.readTemperature();

  // Check if reading failed
  if (isnan(temperature)) {
    // If DHT sensor fails, turn off the fan and turn on all 3 LEDs
    analogWrite(FAN_INA, 0);
    analogWrite(FAN_INB, 0);
    digitalWrite(LED_LOW, HIGH);
    digitalWrite(LED_MEDIUM, HIGH);
    digitalWrite(LED_HIGH, HIGH);
    return;
  }

  // Determine temperature range and control LEDs and fan
  if (temperature < TEMP_LOW_MAX) {
    
    // Low Range: Turn on LOW LED, set fan to 100 speed
    digitalWrite(LED_LOW, HIGH);
    digitalWrite(LED_MEDIUM, LOW);
    digitalWrite(LED_HIGH, LOW);
    analogWrite(FAN_INA, 0);
    analogWrite(FAN_INB, 100);
    lastActiveRange = 1;
  } else if (temperature >= TEMP_MEDIUM_MIN && temperature <= TEMP_MEDIUM_MAX) {
   
    // Medium Range: Turn on MEDIUM LED, set fan to 166 speed
    digitalWrite(LED_LOW, LOW);
    digitalWrite(LED_MEDIUM, HIGH);
    digitalWrite(LED_HIGH, LOW);
    analogWrite(FAN_INA, 0);  
    analogWrite(FAN_INB, 166); 
    lastActiveRange = 2;
  } else if (temperature > TEMP_HIGH_MIN) {
    
    // High Range: Turn on HIGH LED, set fan to 255 speed
    digitalWrite(LED_LOW, LOW);
    digitalWrite(LED_MEDIUM, LOW);
    digitalWrite(LED_HIGH, HIGH);
    analogWrite(FAN_INA, 0);   
    analogWrite(FAN_INB, 255); 
    lastActiveRange = 3;
  }
}
