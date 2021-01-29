import paho.mqtt.client as mqtt                          #import mqtt library

mqttc=mqtt.Client()                                      #setting up the mqtt object
mqttc.connect("localhost",1883,60)                       # connect(host, port=1883, keepalive=60, bind_address="")
mqttc.loop_start()

while(1):
        c=int(input("Enter a state(0/1) : "))
        if(c==1):
                mqttc.publish("esp8266/4","1")          #publish(topic, payload=None, qos=0, retain=False)
        elif(c==0):
                mqttc.publish("esp8266/4","0")
	
