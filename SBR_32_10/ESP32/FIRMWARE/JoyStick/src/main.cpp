
#include "Arduino.h"
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
// #include <TB6612_ESP32.h>

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
#define DIVISOR 10000.0
#define MAX_ACCEL (200)
#define ANGLE_Kp 450
#define ANGLE_Kd 30
#define ANGLE_Ki 0

#define VELOCITY_Kp 70
#define VELOCITY_Kd 0
#define VELOCITY_Ki 5

static float VELOCITY_MAX = 10.0f;
static float STEERING_MAX = 2.0f;
#define MAX_PACKET_SIZE 96
#define DIVISOR 10000.0

char packet[MAX_PACKET_SIZE];
uint8_t packet_size = 0;

long pid_settings[6] = {
    ANGLE_Kp, ANGLE_Kd, ANGLE_Ki,
    VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki};

void sendPid();

// #define ACCESSPOINT

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
  WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
#endif
  // HTTP handler assignment
  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", index_html_gz, sizeof(index_html_gz));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response); });

  events.onConnect([](AsyncEventSourceClient *client)
                   {
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
    sendPid(); });

  // start server
  webserver.addHandler(&events);
  webserver.begin();
  server.listen(82);
  Serial.print("Is server live? ");
  Serial.println(server.available());
}

double convertToPercentage(int input)
{
  double percentage = (static_cast<double>(input) / 255); // * 100;
  // double newPercentage = percentage - 100;
  // return newPercentage;
  return percentage;
}

// handle http messages
void handle_message(WebsocketsMessage msg)
{
  double speed, steer, value;
  int _speed, _steer;
  String parameter = "";
  String response = "";

  commaIndex = msg.data().indexOf('#');

  if (commaIndex >= 0)
  {
    commaIndex = msg.data().indexOf(',');
    parameter = msg.data().substring(1, commaIndex);
    value = msg.data().substring(commaIndex + 1).toInt();
    response = "";

    if (parameter == "akp")
    {
      response = "P";
    }
    else if (parameter == "aki")
    {
      response = "I";
    }
    else if (parameter == "akd")
    {
      response = "D";
    }
    else if (parameter == "vkp")
    {
      response = "p";
    }
    else if (parameter == "vki")
    {
      response = "i";
    }
    else if (parameter == "vkd")
    {
      response = "d";
    }

    if (response != "")
    {
      response += (int)value;
      response += "\n\r";
      Serial.print(response);
    }
    else
      return;
  }
  else
  {
    commaIndex = msg.data().indexOf(',');
    _speed = msg.data().substring(0, commaIndex).toInt();
    _steer = msg.data().substring(commaIndex + 1).toInt();

    speed = convertToPercentage(_speed);
    steer = convertToPercentage(_steer);

    float velocity = speed * VELOCITY_MAX;
    float steering = steer * STEERING_MAX;

    response = "c";
    response += (int)(velocity * DIVISOR);
    response += ";";
    response += (int)(steering * DIVISOR);
    response += "\n\r";

    Serial.print(response);
  }
}

void send_float_array(float *a, uint8_t size)
{
  for (int i = 0; i < size; i++)
  {
    Serial.print((long)(a[i] * DIVISOR));
    if (i < size - 1)
    {
      Serial.print(';');
    }
  }
  Serial.print("\r\n");
}

void parse_float_array(char *p, uint8_t p_size, long *dest)
{
  char buf[16];
  long value;
  uint8_t buf_size = 0;
  uint8_t index = 0;
  for (uint8_t i = 0; i < p_size; i++)
  {
    if ((p[i] >= '0' && p[i] <= '9') || p[i] == '+' || p[i] == '-')
    {
      buf[buf_size++] = p[i];
    }
    else if (p[i] == ';')
    {
      buf[buf_size] = '\0';
      buf_size = 0;
      value = atol(buf);
      dest[index++] = value; //((float)value) / DIVISOR;
    }
  }
  buf[buf_size] = '\0';
  value = atol(buf);
  dest[index] = value; //((float)value) / DIVISOR;
}

void parse_settings(char *p, uint8_t p_size)
{
  parse_float_array(p, p_size, pid_settings);
  // anglePID.setSettings(pid_settings[0], pid_settings[1], pid_settings[2]);
  // velocityPID.setSettings(pid_settings[3], pid_settings[4], pid_settings[5]);
}

bool handle_packet(char *p, uint8_t p_size)
{
  switch (p[0])
  {
  case 'r':
    // send_float_array(pid_settings, 6);
    break;
  case 's':
    parse_settings(&p[1], p_size - 1);

    return true;
    break;
  case 'c':
    // parse_control(&p[1], p_size - 1);
    //  send_float_array(joystick, 2);
    break;
  }
  return false;
}

void sendPid()
{
  char tmp[sizeof(long) + 1];
  int i = 0;
  events.send("ping", NULL, millis());
  ltoa((long)(pid_settings[i]), tmp, 10);
  events.send(String(tmp).c_str(), "akp", millis());
  i++;
  ltoa((long)(pid_settings[i]), tmp, 10);
  events.send(String(tmp).c_str(), "aki", millis());
  i++;
  ltoa((long)(pid_settings[i]), tmp, 10);
  events.send(String(tmp).c_str(), "akd", millis());
  i++;
  ltoa((long)(pid_settings[i]), tmp, 10);
  events.send(String(tmp).c_str(), "vkp", millis());
  // events.send(tmp,"akp",millis());
  i++;
  ltoa((long)(pid_settings[i]), tmp, 10);
  events.send(String(tmp).c_str(), "vki", millis());
  i++;
  ltoa((long)(pid_settings[i]), tmp, 10);
  events.send(String(tmp).c_str(), "vkd", millis());
}

void loop()
{
  auto client = server.accept();
  client.onMessage(handle_message);
  while (client.available())
  {
    client.poll();

    while (Serial.available())
    {
      Serial.println("Sono qui");
      int c = Serial.read();
      if (c == '\n')
      {
        continue;
      }
      else if (c == '\r')
      {
        Serial.println(packet);
        bool packet_ok = handle_packet(packet, packet_size);
        packet_size = 0;
        // s450;30;30;40;50;60
        if (packet_ok)
        {
          sendPid();
        }
      }
      else
      {
        packet[packet_size++] = (uint8_t)c;
      }
    }
  }
}