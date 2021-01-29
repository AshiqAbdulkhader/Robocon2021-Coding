# Bot Connectivity-MQTT

## Setup
1. Install paho mqtt on raspberry pi
      "pip install paho-mqtt"
      
2.Install esp8266 board library for arduino IDE
2.1 In your Arduino IDE, go to File> Preferences
2.2 Enter "http://arduino.esp8266.com/stable/package_esp8266com_index.json" into the “Additional Boards Manager URLs” field as shown in the figure below.Then, click the “OK”
2.3 Open the Boards Manager. Go to Tools > Board > Boards Manager
2.4 Search for ESP8266 and press install button for the “ESP8266 by ESP8266 Community“
      
3. add the SSID and Password of the WiFi router and the IP address of the RaspberryPi to the variables inside the sketch

4. Upload the sketch

5. start the server side program from the RaspberryPi


## MQTT
MQTT is a publish/subscribe protocol that allows edge-of-network devices to publish to a broker. Clients connect to this broker, which then mediates communication between the two devices. Each device can subscribe, or register, to particular topics. When another client publishes a message on a subscribed topic, the broker forwards the message to any client that has subscribed.

Here the RaspberryPi is the publisher and the edge device is the esp8266 (Nodemcu). The server side program( MQTT-Server.py) publishes the message and the client which has subscribed to the topic that is published receives the message.This model is ideal for controlling as the RaspberryPi can send same commands to defferent devices just by changing the topic and only one of them will receive it. it can also catogarise commands by having different topics for different commands.
