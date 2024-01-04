import random
import json as js
from paho.mqtt import client as mqtt_client
from pymongo import MongoClient
import time

#=========================== CONFIGURATION ====================================

# MQTT configuration
broker = 'iot.ac.uma.es'
port   = 1883
client_id = f'subscribe-{random.randint(0, 100)}'
username = 'II15'
password = 'qX5nQDDd'

# Topics
waypointstopic  = "II15/Waypoints"                                                  
roverstopic     = "II15/Rovers"

# MongoDB configuration
uri         =  "mongodb://II15:qX5nQDDd@iot.ac.uma.es:27017/II15"
mongo       =  MongoClient(uri)    
db          =  mongo['II15']                           # Database
collection  =  db.Central                              # Collection


#========================== MQTT FUNCTIONS ====================================

#-------------------------- CONNECT MQTT ---------------------------------------
def connect_mqtt() :
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    
    return client

#-------------------------- SUBSCRIBE MQTT -------------------------------------
def subscribe(client):
    
    def on_message(client, userdata, msg):
        
        print(f"{msg.payload}")
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        
        try:
            
            #Insert received data in MongoDB collection
            print("Inserting data in MongoDB")
            mqtt_data = js.loads(msg.payload.decode())
            mqtt_data["Timestamp"] = time.time()
            collection.insert_one(mqtt_data)
        
        except:
            print("Error inserting data in MongoDB")
       
    # Subscribe to topics
    #client.subscribe(waypointstopic)
    client.subscribe([[waypointstopic,0],[roverstopic,0]])
    client.on_message = on_message
    

#===============================================================================
    

#================================= MAIN ===========================================
def main():
    client = connect_mqtt()         # Connect to MQTT Broker
    subscribe(client)               # Subscribe to MQTT Topic
    client.loop_forever()           # Loop to receive messages


# Run main function
if __name__ == '__main__':
    main()

#===============================================================================