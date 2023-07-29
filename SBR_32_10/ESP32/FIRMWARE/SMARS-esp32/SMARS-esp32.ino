
#include "Arduino.h"
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
//#include <TB6612_ESP32.h>

#include "config.h"
#include "web.h"

/** ESP32 robot tank with wifi and one joystick web control sketch. 
    Based on SMARS modular robot project using esp32 and tb6612.
    https://www.thingiverse.com/thing:2662828

    for complete complete program: https://github.com/nkmakes/SMARS-esp32

    Made by nkmakes.github.io, August 2020.

    -----------------------------------------
    Camera stream based upon:
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    Adapted by Manos Zeakis for ESP32 and TB6612FNG
*/
#define MAX_ACCEL (200)
#define ANGLE_Kp  450.0
#define ANGLE_Kd  30.0
#define ANGLE_Ki  0.0

#define VELOCITY_Kp  0.007
#define VELOCITY_Kd  0.0
#define VELOCITY_Ki  0.0005

#define DIVISOR 10000.0
static float VELOCITY_MAX = 10.0f;
static float STEERING_MAX = 2.0f;
#define MAX_PACKET_SIZE 96
#define DIVISOR 10000.0

char packet[MAX_PACKET_SIZE];
uint8_t packet_size = 0;

float pid_settings[6] = {
  ANGLE_Kp, ANGLE_Kd, ANGLE_Ki,
  VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki
};



//#define ACCESSPOINT

void setup()
{
  Serial.begin(38400);
  #ifdef ACC_POINT
  // Create AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
#else
  WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }
    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
#endif
  // HTTP handler assignment
  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });

  // start server
  webserver.addHandler(&events);
  webserver.begin();
  server.listen(82);
  Serial.print("Is server live? ");
  Serial.println(server.available());
 
}

double convertToPercentage(int input) {
    double percentage = (static_cast<double>(input) / 255);// * 100;
    //double newPercentage = percentage - 100;
    //return newPercentage;
    return percentage;
}
 
// handle http messages
void handle_message(WebsocketsMessage msg) {
  double speed, steer;
  int _speed, _steer;
  commaIndex = msg.data().indexOf(',');
  _speed = msg.data().substring(0, commaIndex).toInt();
  _steer = msg.data().substring(commaIndex + 1).toInt();
  
  speed = convertToPercentage(_speed);
  steer = convertToPercentage(_steer);

  float velocity = speed * VELOCITY_MAX;
  float steering = steer * STEERING_MAX;

String response = "c";
response += (int) (velocity * DIVISOR);
response += ";";
response += (int) (steering * DIVISOR);
response += "\n\r";



  Serial.print(response);
  //Serial.println(speed);
  //Serial.print("Steer: ");
  //Serial.println(steer);
  //motor1.drive(LValue);
 // motor2.drive(RValue);
}

void send_float_array(float *a, uint8_t size) {
    for (int i = 0; i < size; i++) {
        Serial.print((long)(a[i] * DIVISOR));
        if (i < size - 1) {
            Serial.print(';');
        }
    }
    Serial.print("\r\n");
}

void parse_float_array(char *p, uint8_t p_size, float *dest) {
    char buf[16];
    long value;
    uint8_t buf_size = 0;
    uint8_t index = 0;
    for (uint8_t i = 0; i < p_size; i++) {
        if ((p[i] >= '0' && p[i] <= '9') || p[i] == '+' || p[i] == '-') {
            buf[buf_size++] = p[i];
        } else if (p[i] == ';') {
            buf[buf_size] = '\0';
            buf_size = 0;
            value = atol(buf);
            dest[index++] = ((float)value) / DIVISOR;
        }
    }
    buf[buf_size] = '\0';
    value = atol(buf);
    dest[index] = ((float)value) / DIVISOR;
}

void parse_settings(char *p, uint8_t p_size) {
    parse_float_array(p, p_size, pid_settings);
    //anglePID.setSettings(pid_settings[0], pid_settings[1], pid_settings[2]);
    //velocityPID.setSettings(pid_settings[3], pid_settings[4], pid_settings[5]);
}


bool handle_packet(char *p, uint8_t p_size) {
    switch (p[0]) {
        case 'r':
            //send_float_array(pid_settings, 6);
            break;
        case 's':
            parse_settings(&p[1], p_size - 1);
            
            return true;
            break;
        case 'c':
            //parse_control(&p[1], p_size - 1);
            // send_float_array(joystick, 2);
            break;
    }
    return false;
}
 
 bool deb = false;
void loop()
{
  auto client = server.accept();
  client.onMessage(handle_message);
  while (client.available()) {
    if(deb) Serial.println("Sono qui");
    deb = false;
    client.poll();

    while (Serial.available()) {
      int c = Serial.read();
      if (c == '\n') {
          continue;
      } else if (c == '\r') {
          packet_size = 0;
          deb = false;
          if(handle_packet(packet, packet_size)){
            //Serial.println("Sono qui");
            //long i;
            char buffer [sizeof(long)*6+10];
            char tmp [sizeof(long)+1];
            for (int i = 0; i < 6; i++) {
              ltoa ((long)(pid_settings[i] * DIVISOR), tmp, 10);
              strcat(buffer, tmp);
             
              if (i < 5) {
                strcat(buffer, ";");
                
              }
            }   
            strcat(buffer, "\r\n");
            
            events.send("ping",NULL,millis());
            events.send(String(buffer).c_str(),"dati",millis());
            deb = true;
            Serial.print("buffer = ");
            Serial.println(sizeof(buffer));
          }
      } else {
          packet[packet_size++] = (uint8_t) c;
      }
    }
    
  }
}