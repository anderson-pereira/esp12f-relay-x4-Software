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
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>

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

ESP8266WebServer server(80);

void estabilish_wifi_connection(const char* ssid, const char* passwd);
void estabilish_mqtt_connection ( PubSubClient *mqtt_client );
void configure_gpio_pins ();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void mqtt_update_status_topic (char* topic);
void json_interpreter(byte* payload);
void rest_get_server_status ();
void rest_put_server_status ();
void rest_start_server();
void read_pulse_sw_state(bool update_internal_states);
unsigned long previous_millis = 0;

void setup(void) {
    configure_gpio_pins();
    read_pulse_sw_state(true);
    Serial.begin(115200);
    Serial.println("\n");
    previous_millis = millis();

    WiFi.enableAP(0);
    client.setServer(mqttServer, mqttPort);
    client.setCallback(mqtt_callback);
    rest_start_server();
}

void loop() {

    estabilish_wifi_connection(ssid,password);

    if (WiFi.status() == WL_CONNECTED) {
        estabilish_mqtt_connection(&client);
        client.loop();
        server.handleClient();
    }

    if (millis() - previous_millis >= 500) {
        previous_millis = millis();
        read_pulse_sw_state(false);
    }
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

// Non blocking method
void estabilish_wifi_connection(const char* ssid, const char* passwd) {

    static unsigned long previous_millis = millis();
    static bool try_connect = false;

    if ( (WiFi.status() != WL_CONNECTED) && !try_connect ) {
        Serial.print("Connecting to WiFi with SSID: ");
        Serial.println(ssid);
        WiFi.begin(ssid, passwd);
        try_connect = true;
    }

    if (try_connect) {
        if (millis() - previous_millis >= 500) {
            previous_millis = millis();
            if (WiFi.status() == WL_CONNECTED) {
                Serial.print("Connected with IP: ");
                Serial.println(WiFi.localIP());
                try_connect = false;
            }
        }
    }
}

// Non blocking method
void estabilish_mqtt_connection( PubSubClient *mqtt_client ) {

    static unsigned long previous_millis = millis();
    static bool try_connect = false;

    String clientId = "ESP12FClient-Board01";

    if (WiFi.status() != WL_CONNECTED)
        return;

    if ( (!mqtt_client->connected()) && !try_connect ) {
        Serial.print("Connecting to MQTT Broker: ");
        Serial.println(mqttServer);
        try_connect = true;
    }

    if (try_connect) {
        if (millis() - previous_millis >= 500) {
            previous_millis = millis();
            if (mqtt_client->connect(clientId.c_str())) {
                Serial.print("Connected to MQTT Broker with ID: ");
                Serial.println(clientId);
                mqtt_client->subscribe(topic);
                try_connect = false;
            } 
            else {
                Serial.print("MQTT connection failed with state: ");
                Serial.println(mqtt_client->state());
            }
        }
    }
}


void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Message arrived in topic: ");
    for(unsigned i=0; i<length; i++) Serial.print((char)payload[i]);
    Serial.println();

    json_interpreter(payload);
}


void mqtt_update_status_topic (char* topic) {
    char buff[40];

    if ( (WiFi.status() != WL_CONNECTED) || (!client.connected()) )
        return;

    sprintf(buff,"{\"ry1\":%u,\"ry2\":%u,\"ry3\":%u,\"ry4\":%u}", 
                digitalRead(RELAY_1), digitalRead(RELAY_2), 
                digitalRead(RELAY_3), digitalRead(RELAY_4)
            );
    client.publish(topic, buff, true);
}
    

void json_interpreter(byte* payload) {
    DynamicJsonDocument doc(100);
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
    mqtt_update_status_topic(topic_status);
}


void rest_start_server() {
    server.on("/", HTTP_GET, rest_get_server_status);
    server.on("/", HTTP_PUT, rest_put_server_status);
    server.onNotFound([] () {
        server.send(404, "text/plain", "Route Not Found!");
    });

    server.begin();
    Serial.println("REST Server started!");
}


void rest_get_server_status() {
    char buff[40];
    sprintf(buff,"{\"ry1\":%u,\"ry2\":%u,\"ry3\":%u,\"ry4\":%u}", 
        digitalRead(RELAY_1), digitalRead(RELAY_2), 
        digitalRead(RELAY_3), digitalRead(RELAY_4)
        );
    server.send(200, "application/json", buff);
}


void rest_put_server_status() {
    char buff[50];

    Serial.println("Message arrived in REST: ");
    Serial.println(server.arg("plain"));
    
    server.arg("plain").toCharArray(buff, server.arg("plain").length() + 1);
    json_interpreter((byte *)buff ); 
    server.send ( 200, "application/json", server.arg("plain") );
}


void read_pulse_sw_state(bool update_internal_states) {
    static bool last_states[MAX_RELAY_NUM];

    if (update_internal_states) {
        last_states[0] = digitalRead(PULSE_SW_1);
        last_states[1] = digitalRead(PULSE_SW_2);
        last_states[2] = digitalRead(PULSE_SW_3);
        last_states[3] = digitalRead(PULSE_SW_4);
        return;
    }

    if ((last_states[0] != digitalRead(PULSE_SW_1)) || (last_states[1] != digitalRead(PULSE_SW_2)) || 
            (last_states[2] != digitalRead(PULSE_SW_3)) || ( last_states[3] != digitalRead(PULSE_SW_4)) ) {

        if (last_states[0] != digitalRead(PULSE_SW_1)){
            last_states[0] = digitalRead(PULSE_SW_1);
            digitalWrite(RELAY_1, !digitalRead(RELAY_1));
        }

        if (last_states[1] != digitalRead(PULSE_SW_2)){
            last_states[1] = digitalRead(PULSE_SW_2);
            digitalWrite(RELAY_2, !digitalRead(RELAY_2));
        }

        if (last_states[2] != digitalRead(PULSE_SW_3)){
            last_states[2] = digitalRead(PULSE_SW_3);
            digitalWrite(RELAY_3, !digitalRead(RELAY_3));
        }

        if ( last_states[3] != digitalRead(PULSE_SW_4)){
            last_states[3] = digitalRead(PULSE_SW_4);
            digitalWrite(RELAY_4, !digitalRead(RELAY_4));
        }

        mqtt_update_status_topic(topic_status);
    }
}