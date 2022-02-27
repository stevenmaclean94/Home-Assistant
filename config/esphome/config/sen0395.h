#include "esphome.h"
// #include "sensor.h"

class SEN0395 : public Component, public UARTDevice  {
 public:
  SEN0395(UARTComponent *parent) : UARTDevice(parent) {};

  BinarySensor *presence_sensor = new BinarySensor();

  String pkt_str = "";
  // Flag for packet cap status
  bool pkt_read_ok = false;

  bool presence_prev = 0;
  void setup() override {
    // nothing to do here

  }
  void loop() override {

    // While data comes in and we don't have a pending packet to process...
    while (available() && pkt_read_ok != true)
    {

      // Pull the bytes off the stream
      char inChar = read();

      // And build up the packet
      pkt_str += inChar;

      // Until we hit the end
      if (inChar == '\n')
      {
        pkt_read_ok = true;
      }
    }

    if (pkt_read_ok)
    {
      String header = pkt_str.substring(0,5);
    
      if (strcmp(header.c_str(), "$JYBSS"))
      {
        String r = pkt_str.substring(7, 8);
        bool presence = static_cast<bool>(r.toInt());
        if (presence != presence_prev)
        {
          presence_sensor->publish_state(presence);
          presence_prev = presence;
        }
      } else
      {
        // log string
        // ESP_LOGD("custom", pkt_str.c_str());
      }
      pkt_str = "";
      pkt_read_ok = false;
    
    }
  }

};
