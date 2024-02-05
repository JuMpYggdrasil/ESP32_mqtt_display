#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

#define WIFI_STA_NAME "JUMP"
#define WIFI_STA_PASS "025260652"

#define MQTT_SERVER "siriprapawat.trueddns.com"
#define MQTT_PORT 14283
#define MQTT_NAME "ESP32_DISPLAY"

#define LED_PIN 23
#define LED_BUILTIN 2

#define I2C_ADDRESS 0x3F

// Define a message structure
typedef struct {
  int data;
  char topic[100];// better to use array of char instead of String (*trust me)
  char payload[100];
  // Add other fields as needed
} MyMessage;

// Create message queue
#define QUEUE_LENGTH 10
#define ITEM_SIZE sizeof(MyMessage)
QueueHandle_t xQueueCore0ToCore1;
// SemaphoreHandle_t xSerialMutex;
TaskHandle_t Task1 = NULL;
TaskHandle_t Task2 = NULL;

WiFiClient client;
PubSubClient mqtt(client);

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(I2C_ADDRESS, lcdColumns, lcdRows);

// Function
void callback(char *topic, byte *payload, unsigned int length) {
  MyMessage messageToSend;

  payload[length] = '\0';

  String topic_str = topic, payload_str = (char *)payload;
  // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
  Serial.println("[" + topic_str + "]: " + payload_str);
  // xSemaphoreGive(xSerialMutex);

  messageToSend.data = random(100);
  strcpy(messageToSend.topic, topic_str.c_str());
  strcpy(messageToSend.payload, payload_str.c_str());
  xQueueSend(xQueueCore0ToCore1, &messageToSend, portMAX_DELAY);
}

// Tasks 2
void DisplayTask(void *pvParameters) {
  MyMessage receivedMessage;

  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP address:");
  vTaskDelay(pdMS_TO_TICKS(10));  // Delay for 5 seconds
  lcd.setCursor(0, 1);
  vTaskDelay(pdMS_TO_TICKS(10000));  // Delay for 5 seconds
  lcd.print(WiFi.localIP());
  vTaskDelay(pdMS_TO_TICKS(5000));  // Delay for 5 seconds

  while (1) {
    // Receive message from the queue
    if (xQueueReceive(xQueueCore0ToCore1, &receivedMessage, portMAX_DELAY)) {

      // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.print("DisplayTask received message: ");
      Serial.println(receivedMessage.topic);
      // xSemaphoreGive(xSerialMutex);

      lcd.clear();
      lcd.setCursor(0, 0);
      // String topic_str_display = receivedMessage.topic.substring(0, 15);
      lcd.print(receivedMessage.topic);
      vTaskDelay(pdMS_TO_TICKS(10));
      // String payload_str_display = receivedMessage.payload.substring(0, 15);
      lcd.setCursor(0, 1);

      lcd.print(receivedMessage.payload);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // must use for clear WDT
  }
}

// Tasks 1
void MqttTask(void *pvParameters) {
  MyMessage messageToSend;

  // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_STA_NAME);
  // xSemaphoreGive(xSerialMutex);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(5000));  // Delay for 5 seconds
    // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    Serial.print(".");
    // xSemaphoreGive(xSerialMutex);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  digitalWrite(LED_BUILTIN, HIGH);

  // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // xSemaphoreGive(xSerialMutex);

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);

  while (1) {
    if (mqtt.connected() == false) {
      // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
      Serial.print("MQTT connection... ");
      // xSemaphoreGive(xSerialMutex);
      if (mqtt.connect(MQTT_NAME)) {
        // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
        Serial.println("connected");
        // xSemaphoreGive(xSerialMutex);
        mqtt.subscribe("#");
      } else {
        // xSemaphoreTake(xSerialMutex, portMAX_DELAY);
        Serial.println("failed");
        // xSemaphoreGive(xSerialMutex);
        vTaskDelay(pdMS_TO_TICKS(5000));  // Delay for 5 seconds
      }
    } else {
      mqtt.loop();
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // must use for clear WDT
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Create message queue
  xQueueCore0ToCore1 = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);

  // Create the initial task
  // xTaskCreate(MqttTask, "MqttTask", 20000, NULL, 1, NULL);
  // xTaskCreate(DisplayTask, "DisplayTask", 20000, NULL, 2, NULL);
  xTaskCreatePinnedToCore(MqttTask, "MqttTask", 5000, NULL, 1, &Task1, 0);
  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 5000, NULL, 1, &Task2, 1);

  // Create a mutex for Serial
  // xSerialMutex = xSemaphoreCreateMutex();

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Code inside loop is not executed as tasks are scheduled by FreeRTOS
}
