from paho.mqtt import client as mqtt_client
import sys
import os
import random
import psutil




broker = 'localhost'
port = 1883
topic = "TURTLEBOT1"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
# username = 'emqx'
# password = 'public'


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    #client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        if (msg.payload.decode() == "lane") :
            feedback = os.system ("python3 /home/ubuntu/OneDrive/ROS2/Ros1_driving/laneFollowingWithStreamingCam.py")
        if (msg.payload.decode() == "processes") :
            for proc in psutil.process_iter():
                if(proc.name()== "python") :
                    print ("found process")
                    
    client.subscribe(topic)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()