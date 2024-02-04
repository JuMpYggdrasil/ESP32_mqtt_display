#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>


#define WIFI_STA_NAME "JUMP"
#define WIFI_STA_PASS "025260652"

#define MQTT_SERVER   "siriprapawat.trueddns.com"
#define MQTT_PORT     14283
// #define MQTT_USERNAME "esp32_1"
// #define MQTT_PASSWORD "123456"
#define MQTT_NAME     "ESP32_DISPLAY"

#define LED_PIN 23
#define LED_BUILTIN 2

WiFiClient client;
PubSubClient mqtt(client);

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x3F, lcdColumns, lcdRows);  

// unsigned long previousMillis = 0;  // will store last time LED was updated
// const long interval = 1000;  // interval at which to blink (milliseconds)


void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String topic_str = topic, payload_str = (char*)payload;
  Serial.println("[" + topic_str + "]: " + payload_str);

  // digitalWrite(LED_PIN, (payload_str == "ON") ? HIGH : LOW);
  lcd.clear();
  // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print("[" + topic_str + "]");
  // set cursor to first column, second row
  lcd.setCursor(0,1);
  lcd.print(payload_str);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_STA_NAME);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP address:");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(5000);
  
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);
}

void loop() {
  if (mqtt.connected() == false) {
    Serial.print("MQTT connection... ");
    if (mqtt.connect(MQTT_NAME)) {
    // if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("connected");
      mqtt.subscribe("#");
    } else {
      Serial.println("failed");
      delay(5000);
    }
  } else {
    mqtt.loop();
  }

  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   // save the last time you blinked the LED
  //   previousMillis = currentMillis;

    
  // }

}