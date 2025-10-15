#ifndef ESP32HARDWARE_H
#define ESP32HARDWARE_H

/*
  Wifi library compatible with ESP32
*/
#include "WiFi.h"

class Esp32Hardware
{
public:
  /*
    ESP32 Hardware constructor
  */
  Esp32Hardware()
  {
  }

  /*
    ESP32 setting connection with wifi
  */
  void setConnection(IPAddress &server, int port)
  {
    this->server = server;
    this->serverPort = port;
  }

  IPAddress getLocalIP()
  {
    return tcp.localIP();
  }

  /*
    ESP32 wifi init
  */
  void init()
  {
    this->tcp.connect(this->server, this->serverPort);
  }

  /*
    ESP32 wifi read data
  */
  int read()
  {
    if (this->tcp.connected())
    {
      return tcp.read();
    }
    else
    {
      this->tcp.connect(this->server, this->serverPort);
    }
    return -1;
  };

  /*
    ESP32 wifi write data
  */
  void write(const uint8_t *data, size_t length)
  {
    tcp.write(data, length);
  }

  unsigned long time() { return millis(); }

protected:
  WiFiClient tcp;
  IPAddress server;
  uint16_t serverPort = 11411;
};

#endif // ESP32HARDWARE_H