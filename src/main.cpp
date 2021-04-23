/*
 * ESP12F_Relay_X4 Board MQTT software.
 * 
 * This software implements MQTT functionalities using onboard ESP12F (ESP8266) to 
 * connection with WiFi. MQTT payload is in JSON format, was described below:
 * 
 * {
 *   "ry1":1,
 *   "ry2":0,
 *   "ry3":0,
 *   "ry4":0
 * } 
 * 
 * In case of update, a publish is launched on a status topic with the same JSON format.
 * 
 * Anderson Pereira <anderson.m.azevedo@outlook.com>
 * 
 */

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Number of relays
#define MAX_RELAY_NUM 4

// Relay pins
#define RELAY_1 16
#define RELAY_2 14
#define RELAY_3 12
#define RELAY_4 13

// Pulse switch pins
#define PULSE_SW_1 4
#define PULSE_SW_2 0
#define PULSE_SW_3 2
#define PULSE_SW_4 15

// Led pin
#define LED 5

const String BOARD_NAME = "board 01";
const String RELAY_NAMES[] = {"ry1", "ry2", "ry3", "ry4"};

char* ssid = (char*)"your ssid";
char* password = (char*)"your password";
char* mqttServer = (char*)"your mktt server ";
char* topic = (char*)"your action topic";
char* topic_status = (char*)"your status topic";
int mqttPort = 1883; // Usually 1883 port is used

WiFiClient espClient;
PubSubClient client(espClient);

void estabilish_wifi_connection(const char* ssid, const char* passwd);
void estabilish_mqtt_connection ( PubSubClient *mqtt_client );

void mqtt_subscribe_to_brocker( PubSubClient *client, char* mqttServer, int mqttPort, 
            char* topic, void (*mqtt_callback)(char* topic, byte* payload, unsigned int length) );

void configure_gpio_pins ();
void mqtt_callback(char* topic, byte* payload, unsigned int length);


void setup(void) {
    configure_gpio_pins();
    Serial.begin(115200);

    estabilish_wifi_connection(ssid,password);
    mqtt_subscribe_to_brocker(&client, mqttServer, mqttPort, topic, mqtt_callback );
}

void loop() {
    estabilish_wifi_connection(ssid,password);
    estabilish_mqtt_connection(&client);
    client.loop();
}


void configure_gpio_pins() {
  pinMode (RELAY_1, OUTPUT);
  pinMode (RELAY_2, OUTPUT);
  pinMode (RELAY_3, OUTPUT);
  pinMode (RELAY_4, OUTPUT);

  pinMode (PULSE_SW_1, INPUT_PULLUP);
  pinMode (PULSE_SW_2, INPUT_PULLUP);
  pinMode (PULSE_SW_3, INPUT_PULLUP);
  pinMode (PULSE_SW_4, INPUT_PULLUP);

  pinMode (LED, OUTPUT);
}


void estabilish_wifi_connection(const char* ssid, const char* passwd) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("WiFi not connected, trying with SSID: ");
        Serial.println(ssid);

        WiFi.begin(ssid, passwd);
        while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
        }
        
        Serial.println();

        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());

    }
}


void estabilish_mqtt_connection( PubSubClient *mqtt_client ) {
    String clientId = "ESP12FClient-Board01";

    while (!mqtt_client->connected()) {
        Serial.print("Connecting to MQTT with ID: ");
        Serial.println(clientId);

        if (mqtt_client->connect(clientId.c_str())) {
            Serial.println("connected");
        } 
        else {
            Serial.print("failed with state: ");
            Serial.println(mqtt_client->state());
            delay(2000);
        }
    }
}


void mqtt_subscribe_to_brocker( PubSubClient *client, char* mqttServer, int mqttPort, 
            char* topic, void (*mqtt_callback)(char* topic, byte* payload, unsigned int length) ) {

    client->setServer(mqttServer, mqttPort);
    client->setCallback(mqtt_callback);

    estabilish_mqtt_connection(client);

    client->subscribe(topic);
}


void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    DynamicJsonDocument doc(100);
    char buff[40];

    Serial.print("Message arrived in topic: ");
    Serial.println(topic);

    deserializeJson(doc, payload);
    JsonObject obj = doc.as<JsonObject>();

    for(size_t i=0; i < MAX_RELAY_NUM; i++) {
        JsonVariant ry = obj[RELAY_NAMES[i]];

        if ( ry.isNull() ) 
            continue;

        switch (i) {
            case 0:
                digitalWrite(RELAY_1, ry.as<bool>());
                break;
            case 1:
                digitalWrite(RELAY_2, ry.as<bool>());
                break;
            case 2:
                digitalWrite(RELAY_3, ry.as<bool>());
                break;  
            case 3:
                digitalWrite(RELAY_4, ry.as<bool>());
                break;     
            default:
                break;
        }    
    }

    sprintf(buff,"{'ry1':%u,'ry2':%u,'ry3':%u,'ry4':%u}", 
                digitalRead(RELAY_1), digitalRead(RELAY_2), 
                digitalRead(RELAY_3), digitalRead(RELAY_4)
            );

    client.publish(topic_status, buff, true);
}