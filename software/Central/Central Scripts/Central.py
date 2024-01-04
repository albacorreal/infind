#!/usr/bin/python

# ****************************************************** IMPORTS ***********************************************************

# ------------ GENERAL IMPORTS -----------
import threading
import subprocess
from   typing import Any
import logging
import time
import numpy  as np
import pandas as pd
import json   as js
import mysql.connector


# ------- IMPORTS FOR CENTRAL THREAD ------
import requests
import random 
import paho.mqtt.client as mqtt_client
from   pymongo     import MongoClient
from   tabulate    import tabulate
import pymysql


# ------ IMPORTS SERIAL THREAD ------------
import serial


# **************************************************** GLOBAL DATA **********************************************************


# ---------------------- MQTT CONFIGURATION ---------------------------

# MQTT CONFIGURATION
broker     =   'iot.ac.uma.es'
port       =   1883                     
client_id  =   f'central-{random.randint(0, 100)}'
username   =   'II15'
password   =   'qX5nQDDd'

# TOPICS
cmdtopic       = "II15/Commands"
emergencytopic = "II15/Emergency"




# --------------------- MONGODB CONFIGURATION --------------------------

uri         =  "mongodb://II15:qX5nQDDd@iot.ac.uma.es:27017/II15"
mongo       =  MongoClient(uri)    
db          =  mongo['II15']                           # Database
collection  =  db.Central                              # Collection

# ----------------------- SERIAL CONFIGURATION -------------------------

serialport = '/dev/ttyUSB0'  
baudrate   = 115200  


# ------------------- SHARED VARIABLES DICTIONARY -----------------------

Shared = {}
Shared['roversinfo']   =  None
Shared['newroverinfo'] =  False
Shared['status']       =  None
Shared['sensorcmd']    =  None
Shared['ack']          =  True


#========================================================================================================================================================
#                                                               CLASS HANDLE MQTT
#======================================================================================================================================================== 

class HandleMqtt:

    # ------------------------------------------------------- CONNECT MQTT ----------------------------------------------------------------------
    def connect_mqtt(self,broker,port):

        """
        This method connects to the MQTT broker.
        
        """

        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(client_id)
        client.username_pw_set(username,password)
        client.on_connect = on_connect
        client.connect(broker, port)
    
        return client
    
    # -------------------------------------------------- PUBLISH MQTT -----------------------------------------------------------------------------
    def publish(self,client,topic,msg):

        """
        This method publish a message on the MQTT topic.
        """
        
        # Serialize message if it is a dictionary
        try:
            json = js.dumps(msg)
        except:
            json = msg

        # Publish message
        result = client.publish(topic, json)

        # Inform
        status = result[0]
        if status == 0:
            print(f"Send message to topic succesfully\n")
        else:
            print(f"Failed to send message to topic\n")
    


    # -------------------------------------------------- DISCONNECT MQTT --------------------------------------------------------------------------
    def disconnect_mqtt(self,client):

        """
        This method disconnects from the MQTT broker.
        """
        client.disconnect()



#========================================================================================================================================================
#                                                        CLASS HANDLE SERIAL
#======================================================================================================================================================== 
class HandleSerial:


    # -------------------------------------------------- PROCESS DATA ----------------------------------------------------------------

    def process_data(self,data):
        
        if Shared['ack'] == True:
            
            try:
                
                # Serialize string as a dictionary
                datos = js.loads(data) 
                
                Shared['newroverinfo'] = (Shared['status'] != datos['Status'])
                
                if (Shared['newroverinfo'] == True): 
                    
                    print("NEW DATA RECEIVED!\n")
                    Shared['ack'] == False
                
            except:
                datos = data
                print("ERROR DESERIALIZING DATA\n")
                Shared['newroverinfo'] = False

            return datos
        
        else:
            return Shared['roversinfo']


    # ---------------------------------------------------------- READ SERIAL PORT ---------------------------------------------------------------------
    def read_serial(self,port, baudrate):
        
        """
        This method reads data from serial port.
        
        """

  
        with serial.Serial(port, baudrate, timeout=1) as ser:
    
            buffer =  ""  
            data   =  ""
            ok     =  True

            while ok:

                # Read byte from serial port
                byte = ser.read(1).decode()

                # Check if byte is end of line
                if byte == '\r':
                
                    # Process read data
                    data = self.process_data(buffer)

                    # Stop reading
                    ok = False

                    # Clean buffer
                    buffer = ""
                else:
                    # Add byte to buffer
                    buffer += byte

  
        return data

    # ------------------------------------------------------------ SEND DATA ------------------------------------------------------------------------
    def send_data(self,serport, baudrate,data):
    
      """
      This method sends data to serial port if data is not empty. It takes as an argument data to send, port name and baudrate.
      """
    
      with serial.Serial(serport, baudrate, timeout=1) as ser:

            if data != "":  
                 
                # Send data
                ser.write(data.encode())
                print(f"Sent over serial port: {data}\n")


#===============================================================================================================================
#                                                    CLASS SEQUENCE PLANNER
#===============================================================================================================================


class SequencePlanner:

    # ----------------------------------------------------- CLASS CONSTRUCTOR ----------------------------------------------------
    def __init__(self):
        
        """
        Constructor of SequencePlanner Class.
        
        """
 
        # ------------------ LISTS INITIALIZATION ----------------------------
        
        # Sequencer: TaskId, Priority, Latitude, Longitude, RoverId, Sender,Timeout, Timestamp, Distance
        self.Sequencer  =  []
        
        # Rovers Id: 
        self.RoversId   =  []

        # Register of rovers task in Sequencer (RoverId, Current Task Id, Current Task Latitude, Current Task Longitude)
        self.RoversTask =  []

        # Explored map (RoverId, Latitude, Longitude, Last time explored, Timestamp)
        self.Explored   =  []
        
        # Locations of rovers (RoverId, Current Latitude, Current Longitude)
        self.Locations  =  []
    
        # ----------------- VARIABLES INIZIALIZATION --------------------------

        # Map settings
        self.max_lat   =   90.0                                 # Maximum latitude           
        self.max_long  =  180.0                                 # Maximum longitude
        self.min_lat   =  -90.0                                 # Minimum latitude
        self.min_long  = -180.0                                 # Minimum longitude

        # Home settings
        self.home_lat  =  36.7                                   # Home latitude                         
        self.home_long =  -4.42                                  # Home longitude                                       
        
        # Trayectory settings
        self.direction =  1                                     # Direction of the trayectory (1: North, -1: South)      

        # Variables initialization
        self.usercmd    =  None                                 # User command
        self.sensorcmd  =  None                                 # Sensor data
        self.status     =  None                                 # Rover status               
        self.autocmds   =  []                                   # Automatic commands  

        # Boolean variables for updating data
        self.new_usercmd   =  False                             # New user command
        self.new_sensorcmd =  False                             # New sensor data
        self.new_status    =  False                             # New sensor status
        self.new_autocmd   =  False                             # New automatic command

        # Boolean variable for pending commnads
        self.pending       =  False                             # Pending commands

        # Task ids
        self.task_id = 0                                        # Tasks id 
        
        # Serial Module
        self.SerialModule = HandleSerial()                      # Serial Module

        # MQTT
        self.Mqtt   = HandleMqtt()                              # Mqtt handler
        
    
    # ********************************************* PRINCIPAL METHODS *************************************************
    
    # ----------------------------------------------- UPDATE ROVERS ----------------------------------------------------
    def updaterovers(self):
        
        """ 
        This function updates central data. 
        
        """
        # ----------------- GET ROVERS IDS --------------------------
        
      
        # SQL query location table
        cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')
        
        # Cursor object
        cursor = cnx.cursor()
        
        # Query rovers id in mysql db 
        query = "SELECT RoversId FROM Rovers;"
        cursor.execute(query)
        results = cursor.fetchall()
        
        # Initial numbers of rovers id
        initial = len(self.RoversId)
        
        if (len(results) > initial):
            
            # Add missing rovers ids
            for row in results:
                if row[0] not in self.RoversId:
                    self.RoversId.append(row[0])
        
        elif (len(results) < initial):
            
            # Delete rovers ids
            self.RoversId = []
            
            for row in results:
                if row[0] not in self.RoversId:
                    self.RoversId.append(row[0])

        # Inform
        if (len(self.RoversId) != initial):
            print(f"Rovers Ids: {self.RoversId}\n")
            print(f"Number of rovers: {len(self.RoversId)}\n")
    
    
    # ----------------------------------------------- PROCESS LOCATION --------------------------------------------------
    def process_location(self):
        
        """
            This method process current location received from rovers.
        """
        # Get locations from rovers
        for rover in self.RoversId:
           
            # Check if location already exists in the list
            matches = [d for d in self.Locations if (d[0] == rover)]
            
            # Get last location from mongodb
            query = collection.find_one({'Sender': rover},sort=[('Timestamp', -1)])
           
            #  Save new location
            if (query != None):
                
                if (len (matches) == 0):
                    
                    # Add new location
                    self.Locations.append([query['Sender'], query['Latitude'], query['Longitude']])
                else:
                    
                    # Update location
                    index = [i for i,x in enumerate(self.Locations) if (x[0] == rover)][0]
                    self.Locations[index][1] = query['Latitude']
                    self.Locations[index][2] = query['Longitude']
            
            else:
                
                # Add home location
                if (len (matches) == 0):
                    self.Locations.append([rover, self.home_lat, self.home_long])
            
        # ----------------------- SERIAL MESSAGES (TABLE FORMAT) -----------------------------
        print ("\nLOCATIONS:\n")
       
        # Locations table
        print('\n' + tabulate(self.Locations, headers=["RoverId", "Lat", "Long"]))

        # ------------------------------------------------------------------------------------
    
    # --------------------------------------------------- GET DATA ----------------------------------------------------
    def get_data(self):

        """
        This method will:
        - Get user commands via MQTT (from NodeRed)
        - Get data from SensorRover (LoRa)
        - Get status from SensorRover (LoRa)
        - Get status from ActuatorRover (LoRa)
        - Check if data received is new data and update data if it is
        
        """
    

        # ---------------------- GET DATA ------------------------

        # Inform
        print("\n...GETTING DATA...\n")

       # Get user commands from MongoDB collection          
        usercmd = collection.find_one({'Sender': 'User'},sort=[('Timestamp', -1)])
        
        
        # Get Rovers info (via LoRa and reading from serial port)
        roversinfo = Shared["roversinfo"]
       

        # ------------------- CHECK DATA ----------------------------
        
        # Inform
        print("\n....CHECKING DATA.....\n")

        # Check if data received is new data and classify rover info as command or status
        self.check_data(usercmd,roversinfo)
        
        # User commands
        if (self.new_usercmd):
                 
            del usercmd["_id"]
            self.usercmd = usercmd
        
            # Inform
            print("New user command received: ", self.usercmd)
        
        else:
            # Inform
            print("No new user command received\n")

        # Sensor commands
        if (self.new_sensorcmd):
            self.sensorcmd      = roversinfo
            Shared['status']    = roversinfo['Status']

            # Inform
            print("New sensor command received: ", self.sensorcmd)
        else:
            # Inform
            print("No new sensor command received")

        # Status
        if (self.new_status):
            self.status      = roversinfo
            Shared['status'] = roversinfo['Status']

            # Inform
            print("New status received: ", self.status)
        
        else:
            # Inform
            print("No new rover status received\n")


    # ------------------------------------------------ AUTO PLAN -----------------------------------------------------------
    def auto_plan(self):

        """
        This method will:
        
        - Generate automatic plan based on coordinates
        - Check if automatic plan is inside the map
    
        """
        
        # ---------------------- UPDATES --------------------------
        self.update_rover_task(rover=None,cmd=None)

        
        # -------------- CHECK AUTO COMMANDS ------------------------
        
        # Get Rovers with no auto commands
        autocmds =  [d for d in self.Sequencer if (d[5] == 'Auto')]                                       # Find auto commands
        matches  =  [d[4] for d in autocmds]                                                              # Get RoverIds of the auto commands
        missing  =  [rover_id for rover_id in self.RoversId if rover_id not in matches]                   # Find RoverIds with no auto commands
        
        

        # Check if there are Rovers with no auto commands
        if (len(missing)>0):

            # ---------------- GENERATE AUTO PLAN -------------------------- 
           
            print("ROVERS WITH NO AUTOMATIC PLAN CURRENTLY: " + str(missing)+'\n')
            print("... GENERATING AUTOMATIC COMMANDS...\n")

            latitudes  = []
            longitudes = []

            # Get coordinates of current tasks of Rovers with no auto commands
            for rover in missing:    
                
                print("... CREATING AUTO PLAN FOR " + str(rover) + "...\n")
                print("...GETTING " + str(rover) + " COORDINATES...\n")

                # Check if rover has a current task assigned
                matches  = [d for d in self.RoversTask if (d[0] == rover)]                                  

                # Check if rover has other tasks assigned
                if (len(matches)> 0 and all(x[1] is not None for x in matches)): 
                    
                    # Get current Task Id
                    current_id =  matches[0][1]
                    print("Current task of rover " + str(rover) + ": " + str(current_id))
                    
                    index      =  [i for i,x in enumerate(self.RoversTask) if (x[1] == current_id[0])][0]
                    
                    # Coordinates
                    latitudes.append(self.RoversTask[index][2])                                   # Latitude of current task
                    longitudes.append(self.RoversTask[index][3])                                  # Longitude of current task
                    
                
                else:
                    
                    # Inform 
                    print("Info: Rover " + str(rover) + " has no current task assigned\n")
                    print ("Info: Getting last location received from " + str(rover)+'\n')

                    # Get last location received from rover
                    index = [i for i,x in enumerate(self.Locations) if (x[0] == rover)][0]
                    
                    latitudes.append(self.Locations[index][1])                                    # Latitude of current location
                    longitudes.append(self.Locations[index][2])                                   # Longitude of current location             
                                     
            
            # Generate automatic plan based on coordinates. Close latitude and same longitude. 
            auto_latitudes  = [x + self.direction * np.random.uniform(0, 1) for x in latitudes if x is not None]            
            auto_longitudes = longitudes                                                                                    

            # Check if automatic plan is inside the map
            outer_latmax = [d for d,x in enumerate(auto_latitudes) if  (x > self.max_lat)]
            outer_latmin = [d for d,x in enumerate(auto_latitudes) if  (x < self.min_lat)]
            outer_lng    = [d for d,x in enumerate(auto_longitudes) if (x > self.max_long)]
        

            if (len(outer_latmax) > 0): 
                
                # Change direction
                self.direction = -1                                                                       
                
                # Set latitude inside the map and compute different longitude
                auto_longitudes [outer_latmax] = [x + self.direction * np.random.uniform(0, 1) for x in longitudes[outer_latmax]]  
                auto_latitudes  [outer_latmax] = self.max_lat                                             
                                                               
            elif (len(outer_latmin) > 0):

                # Change direction
                self.direction = 1                                                                      
                
                # Set latitude inside the map and compute different longitude
                auto_longitudes [outer_latmin] = [x + self.direction * np.random.uniform(0, 1) for x in longitudes[outer_latmax]]       
                auto_latitudes  [outer_latmax] = self.min_lat                                           
                      
            if (len(outer_lng)> 0):  
                    
                    # Set longitude inside the map     
                    auto_longitudes[outer_lng] = self.min_long                                         

          
            # Check if automatic plan has been explored recently (last 5 days)
            for d in range(len(auto_latitudes)):
                
                while ((auto_latitudes[d], auto_longitudes[d]) in self.Explored):
                    matches = [i for i in self.Explored if (i[2]< 5)]
                    
                    # Generate different latitude
                    if (matches != []):
                        auto_latitudes[d] = latitudes[d] + self.direction * np.random.uniform(0, 1)        

           
            # Create list of the auto commands to add to the Sequencer: 
            # [Id,Priority,Latitude, Longitude, RoverId, Sender, Timeout, Timestamp, Distance]
            autocmds = []
            
            for i in range(len(missing)):
                autocmds.append([None,None, auto_latitudes[i], auto_longitudes[i], missing[i], 'Auto', None, None, None])
                
                # Inform
                message = "{'Latitude':"+str(auto_latitudes[i])+", 'Longitude':"+str(auto_longitudes[i])+"}\n"
                print ("NEW AUTOMATIC COMMAND FOR " + str(missing[i])+ ": "+ message + '\n')
            
            
            # Save new automatic commands
            self.autocmds   = autocmds    
            
            # Activate new automatic commands
            self.new_autocmd = True

        else:

            # No new automatic command
            self.new_autocmd = False
            autocmds         = None

            # Inform
            print("...ALL ROVERS HAVE CURRENTLY AN AUTOMATIC COMMAND ALREDY...\n")
            print("...NO NEW AUTOMATIC COMMAND WILL BE GENERATED...\n")
        

        
    # ------------------------------------------- COMPUTE PRIORITY ----------------------------------------------------
    def compute_priority(self,client):

        """
        This method computes tasks priority of all Rovers
        PRIORITY ORDER: EMERGENCY,USER COMMAND, AUTOMATIC COMMAND

        """
        # ======================================= ROVERS PRIORITY COMPUTATION ==========================================


        # --------------------------------------------------------------------------------------------------------------
        # ID | Priority |  Latitude  | Length  |  Rover ID    |  timeout  |  Sender   |   Timestamp |    Distance
        # --------------------------------------------------------------------------------------------------------------
        # 5  |    1	    |    25.2    |   0.2   |   Sensor     |  125 sec  |   User    |   166598    |       34
        # --------------------------------------------------------------------------------------------------------------
        # 2  |    2	    |    25.2    |   0.2   |   Sensor     |  125 sec  |   User    |   177509    |       67
        # --------------------------------------------------------------------------------------------------------------
        # 0  |    3	    |    25.2    |   0.2   |   Sensor     |  125 sec  |   Auto    |   189067    |       23
        # --------------------------------------------------------------------------------------------------------------
        # 3  |    1	    |    25.2    |   0.2   |   Actuator   |  125 sec  |   User    |   166598    |       123
        # --------------------------------------------------------------------------------------------------------------
        # 6  |    2	    |    25.2    |   0.2   |   Actuator   |  125 sec  |   User    |   177509    |       234
        # --------------------------------------------------------------------------------------------------------------
        # 1  |    3	    |    25.2    |   0.2   |   Actuator   |  125 sec  |   Auto    |   189067    |       56
        # --------------------------------------------------------------------------------------------------------------

        


        # --------------------------------------- PRIORITY 1: RETURN HOME ------------------------------------------------ 
        
        # Check if new critical status is received
        if (self.new_status and self.status['Status']=='Emergency state'):

            # Inform
            print("Info: Rover " + str(self.status['Sender']) + " is in emergency. Computing task as most prioritary...\n") 

            # Current rover location
            index = [i for i,x in enumerate(self.Locations) if (x[0] == self.status['Sender'])]
           
            lat_now = self.Locations[index[0]][1]
            lng_now = self.Locations[index[0]][2]

            # Task ID
            self.task_id =  self.task_id + 1  

            # Return home command
            id        =  self.task_id                                                              # (0) Task ID
            priority  =  1                                                                         # (1) Most priority
            latitude  =  self.home_lat                                                             # (2) Latitude (Home)
            longitude =  self.home_long                                                            # (3) Longitude (Home)
            roverid   =  self.status['Sender']                                                     # (4) Rover ID
            sender    =  'Central'                                                                 # (5) Sender
            timeout   =  self.compute_timeout([[lat_now, lng_now],[latitude, longitude]])          # (6) Timeout 
            timestamp =  self.status['Timestamp']                                                  # (7) Timestamp
            distance  =  0                                                                         # (8) Distance (not needed)
            
            # Command structure
            command = [id, priority, latitude, longitude, roverid, sender, timeout, timestamp, distance]

            print("Info: Adding return home command to Sequencer: " + str(command)+'\n')

            try:
                # Find actual most prioritary command of rover in Sequencer
                index = [i for i,x in enumerate(self.Sequencer) if (x[4] == roverid)][0] 
                self.Sequencer.insert(index, command)   
            
            except:
                
                # Add at the end of Sequencer
                self.Sequencer.append(command) 

            # Update priority
            self.update_priority(roverid, priority,delete=False)


            # Inform
            print("Info: Return home command added to Sequencer\n")

        # -----------------------------------------------------------------------------------------------------------
        
        
        
        # ------------------------------------------ PRIORITY 2: USER COMMAND ---------------------------------------- 
        
        #Check if a new user command is received
        if (self.new_usercmd):
            
            # Inform
            print("Rover " + str(self.usercmd['RoverId']) + " received a user command. Computing priority of task...\n") 

            # Task ID
            self.task_id =  self.task_id + 1     

            # User command
            id         =  self.task_id                                                                   # (0) Task ID
            priority   =  None                                                                           # (1) Priority
            latitude   =  self.usercmd['Latitude']                                                       # (2) Latitude
            longitude  =  self.usercmd['Longitude']                                                      # (3) Longitude
            roverid    =  self.usercmd['RoverId']                                                        # (4) Rover ID
            sender     =  self.usercmd['Sender']                                                         # (5) Sender
            timeout    =  None                                                                           # (6) Timeout (Computed later)
            timestamp  =  self.usercmd['Timestamp']                                                      # (7) Timestamp
            distance   =  0                                                                              # (8) Distance (Computed later)


            # Computing priority based on distance to current rover task location
            try:

                # ID of current rover task
                current_id  = [d for d in self.RoversTask if (d[0] == self.usercmd['RoverId'])][0][1]

                # Index of current task in Sequencer
                index = [i for i,x in enumerate(self.Sequencer) if (x[0] == current_id)][0]

                # Compute distance to current task location
                lat_now    =  self.Sequencer[index][2]                                                   # Current task latitude
                long_now   =  self.Sequencer[index][3]                                                   # Current task longitude

            except:

                # Get last location 
                lat_now   = [d[1] for d in self.Locations if (d[0] == self.usercmd['RoverId'])][0]       # Current latitude
                long_now  = [d[1] for d in self.Locations if (d[0] == self.usercmd['RoverId'])][0]       # Current longitude

            # Compute timeout and distance
            timeout    =  self.compute_timeout([[lat_now, long_now],[latitude, longitude]])               # (6) Timeout 
            distance   =  self.compute_distance([[latitude, longitude],[lat_now, long_now]])              # (8) Distance 

            # Get tasks in Sequencer for rover received as user commands
            matches   = [d for d in self.Sequencer if (d[4] == roverid and d[5] == sender)]

            # Get indexes in Sequencer for user commands
            indexes   = [i for i,x in enumerate(self.Sequencer) if x in matches]

            # Get current distances of user commands
            distances = [d[8] for d in matches]
            distances.append(distance)
        
            # Sort sublist based on distance
            distancesorted = sorted(distances)
       
            # Get index of distance actual user command in sorted sublist
            index = [i for i,x in enumerate(distancesorted) if (x == distance)][0]
          
            # Compute index in Sequencer
            if (indexes != []):
                
                # Index in Sequencer
                newindex = index + indexes[0]
                
                # Compute priority
                priority = self.Sequencer[indexes[0]][1] + index
            
            else:
                
                # Get more priotary tasks for the same rover
                tasks = [d for d in self.Sequencer if (d[4] == roverid and d[5] == 'Central')]
                
                # Check if there are more prioritary tasks for the same rover
                if(tasks != []):
                    
                    # Index in Sequencer (user command with most prio   )
                    newindex = indexes[-1] + 1
                    
                    # Compute priority
                    priority = self.Sequencer[indexes[-1]][1] + 1
                
                # No more prioritary tasks
                else:
                    # Check if there are any command for rover in Sequencer
                    matches = [d for d in self.Sequencer if (d[4] == roverid)]

                    # Get indexes of the commands
                    indexes = [i for i,x in enumerate(self.Sequencer) if x in matches]
                    

                    if (indexes != []):
                        # Index in Sequencer (command with most priority for rover)
                        newindex = indexes[0]
                    
                    else:
                        
                        # Index in Sequencer (add to the bottom of the Sequencer)
                        newindex = len(self.Sequencer)
                    
                    # Compute priority
                    priority = 1

          
            # Command structure
            command = [id,priority, latitude, longitude, roverid, sender, timeout, timestamp,distance]

            # Add  command
            self.Sequencer.insert(newindex,command)

            # Update priority
            self.update_priority(roverid,priority,delete=False)

            # Reset usercmd
            self.new_usercmd = False


        # -------------------------------------------------------------------------------------------------------------------
        

        # ------------------------------------------ PRIORITY 3: SENSOR COMMAND (CMD) --------------------------------------- 
        
        # Check if new sensor data is received
        if (self.new_sensorcmd):
            
            print("Rover " + str(self.sensorcmd['Sender']) + " found an interesting location. Computing priority of task and assigning it...\n") 

            # Task ID
            self.task_id =  self.task_id + 1     
           
            # Sensor command
            id         =  self.task_id                                                                   # (0) Task ID
            priority   =  3                                                                              # (0) Priority
            latitude   =  self.sensorcmd['Latitude']                                                     # (1) Latitude
            longitude  =  self.sensorcmd['Longitude']                                                    # (2) Longitude
            roverid    =  self.compute_roverid(self.sensorcmd['Status'])                                 # (3) Rover ID
            sender     =  self.sensorcmd['Sender']                                                       # (4) Sender
            timeout    =  None                                                                           # (5) Timeout (Compute later)
            timestamp  =  self.sensorcmd['Timestamp']                                                    # (6) Timestamp

            try:

                # ID of current rover task
                current_id  = [d for d in self.RoversTask if (d[0] == self.sensorcmd['RoverId'])][0][1]

                # Index of current task in Sequencer
                index = [i for i,x in enumerate(self.Sequencer) if (x[0] == current_id)][0]

                # Compute distance to current task location
                lat_now    =  self.Sequencer[index][2]                                                     # Current task latitude
                long_now   =  self.Sequencer[index][3]                                                     # Current task longitude
        
            except:

                # Get last location 
                lat_now   = [d[1] for d in self.Locations if (d[0] == roverid)][0]                          # Current latitude
                long_now  = [d[1] for d in self.Locations if (d[0] == roverid)][0]                          # Current longitude
            
            
            # Compute timeout and distance
            timeout    =  self.compute_timeout([[lat_now, long_now],[latitude, longitude]])                  # (6) Timeout 
            distance   =  self.compute_distance([[latitude, longitude],[lat_now, long_now]])                 # (8) Distance  
        
        
            # Get tasks and indexes in Sequencer for rover received as user commands
            matches   = [d for d in self.Sequencer if (d[4] == roverid and d[5] == sender)]

            # Get indexes in Sequencer for user commands
            indexes   = [i for i,x in enumerate(self.Sequencer) if x in matches]

            # Get distances
            distances = [d[8] for d in matches] + distance


            # Sort sublist based on distance
            distancesorted = sorted(distances)

            # Get index of distance actual user command in sorted sublist
            index = [i for i,x in enumerate(distancesorted) if (x == distance)][0]
          
            
            # Compute index in Sequencer
            if (indexes != []):

                # Index in Sequencer
                newindex = index + indexes[0]
                
                # Compute priority
                priority = self.Sequencer[indexes[0]][1] + index
            
            # No sensor commands in Sequencer for this rover
            else:
                
                matches = [d for d in self.Sequencer if (d[4] == roverid)]
                indexes = [i for i,x in enumerate(self.Sequencer) if x in matches]
                #Get automatic tasks for rover
                task    = [d for d in self.Sequencer if (d[4] == roverid and d[5] == 'Auto')]
                
                # Check if there are automatic commands
                if(task != []):

                    # Get index of first automatic command for rover
                    index = [i for i,x in enumerate(self.Sequencer) if x in task][0]
                    
                    # Index in Sequencer
                    newindex = index
                   
                   # Compute priority
                    priority = self.Sequencer[index][1]
                
                # No automatic commands yet
                else:
                    # Check if rover has other tasks in Sequencer
                    matches = [d for d in self.Sequencer if (d[4] == roverid)]
                    indexes = [i for i,x in enumerate(self.Sequencer) if x in matches]
                    
                    if (indexes != []):
                        # Index in Sequencer (added to the bottom of the commands for the rover)
                        newindex = indexes[-1] + 1
                        
                        # Compute priority
                        priority = self.Sequencer[indexes[-1]][1] 
                    
                    # Rover has no tasks in Sequencer yet
                    else:

                        # Index in Sequencer
                        newindex = len(self.Sequencer)
                        
                        # Compute priority
                        priority = 1

            # Command structure
            command = [id,priority, latitude, longitude, roverid, sender, timeout, timestamp,distance]

            # Add  command
            self.Sequencer.insert(newindex,command)

            # Update priority
            self.update_priority(roverid,priority,delete=False)

            # Reset sensor comand to false (alredy processed information)
            self.new_sensorcmd = False


        # -------------------------------------------------------------------------------------------------------------------


        # ------------------------------------------ PRIORITY 4: AUTOMATIC COMMAND ---------------------------------------- 
        
        # Check if a new automatic command is generated (LEAST PRIORITY)
        if (self.new_autocmd):
            
            print("Computing priority of automatics commands generated...\n") 

            # Compute priority of all automatics commands generated
            for command in self.autocmds:

             
                # Current rover location
                index = [i for i,x in enumerate(self.Locations) if (x[0] == command[4])][0]
                lat_now = self.Locations[index][1]
                lng_now = self.Locations[index][2]

                # Task ID
                self.task_id =  self.task_id + 1  

                # New automatic command
                id         =  self.task_id                                                                # (0) Task ID
                priority   =  None                                                                        # (1) Priority
                latitude   =  command[2]                                                                  # (2) Latitude
                longitude  =  command[3]                                                                  # (3) Longitude
                roverid    =  command[4]                                                                  # (4) Rover ID                                                                   
                sender     =  "Auto"                                                                      # (5) Sender
                timeout    =  self.compute_timeout([[lat_now, lng_now],[latitude, longitude]])            # (6) Timeout (Compute later)
                timestamp  =  time.time()                                                                 # (7) Timestamp
                distance   =  0                                                                           # (8) Distance (not needed)
                                                                                    
                
            
                # Get tasks in Sequencer for rover
                matches  = [d for d in self.Sequencer if (d[4] == command[4])]
               
                # Compute priority
                if (matches != []):

                    # Compute priority as the least prioritary task for rover in Sequencer
                    priority = [d[1] for d in matches][-1] + 1
                
                # No tasks found
                else:
                    
                    # Compute priority as the most prioritary task for rover in Sequencer
                    priority = 1
               
                # Command structure
                command    =  [id, priority, latitude, longitude, roverid, sender, timeout, timestamp, distance]
                
                try:
                    # Get index of last task in Sequencer for the rover
                    index = [d for d,x in enumerate(self.Sequencer) if (x[4] == command[4])][-1] + 1
                    
                    # Insert command in Sequencer
                    self.Sequencer.insert(index,command)
                
                except:
                    # Add command to the bottom of Sequencer
                    self.Sequencer.append(command)
            
            # Reset autocmd
            self.new_autocmd = False

        # Update rovers task
        self.update_rover_task(rover=None,cmd = None)


        # ------------------- SAVE SEQUENCER IN MYSQL DB ----------------------------
        
        # Conexion to mysql database                          
        cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

        # Cursor
        cursor = cnx.cursor()
        
        # Get Sequencer rows
        data = [tuple(d) for d in self.Sequencer]
        
        # Delete last Sequencer
        drop_query = "DROP TABLE IF EXISTS Sequencer;"
        cursor.execute(drop_query)

        # Commit
        cnx.commit()

        # Create a new table
        create_query = "CREATE TABLE Sequencer (TaskId INT, Priority INT, Latitude FLOAT, Longitude FLOAT, RoverId VARCHAR(255), Sender VARCHAR(255), Timeout FLOAT, Timestamp FLOAT, Distance FLOAT);"
        cursor.execute(create_query)

        # Insert Sequencer
        query = "INSERT INTO Sequencer (TaskId, Priority, Latitude, Longitude, RoverId, Sender, Timeout, Timestamp, Distance) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s);"
        cursor.executemany(query, data)

        # Commit
        cnx.commit()

        # Close conection
        cnx.close()

        # -------------------------------- SEND SEQUENCER (MQTT) ------------------------------

        # -------------- INITIAL MESSAGE -----------------
        msg = {}
        msg['Info'] = "Sending"
        
        # Serialize json
        msg = js.dumps(msg)

        # Publish
        self.Mqtt.publish(client,cmdtopic,msg)

        
        # -------------- SEND SEQUENCER -------------------

        # List for saving Sequencer in json
        data = []

        # Transform list to dictionary
        for d in self.Sequencer:
            seq = dict(zip(["Priority", "Latitude", "Longitude", "RoverId", "Sender", "Timeout", "Timestamp"], [d[1], d[2], d[3], d[4], d[5], d[6], d[7]]))
            data.append(seq)

        # Mesage structure for Sequencer
        msg = {}
        msg['documents'] = data

        # Serialize json
        json = js.dumps(msg)
        
        # Publish
        self.Mqtt.publish(client,cmdtopic,json)


        # ----------------------- SERIAL MESSAGES (TABLE FORMAT) -----------------------------
        print("\nSEQUENCER")
       
        # Sequencer table
        print('\n' + tabulate(self.Sequencer, headers=["ID", "Priority", "Lat", "Long", "RoverID", "Sender", "Timeout", "Timestamp", "Distance"])+'\n' )

        # ------------------------------------------------------------------------------------

        
    # ----------------------------------------------- SEND COMMANDS ----------------------------------------------------------
    def send_cmd(self, rover, status):

        """
        This method sends coommands to rovers.
        It sends:
        - The most prioritary commands to the rover when they are available
        - Return home command when rover is in critical status

        """
        
        if (status == 'Emergency state' or status == 'Task finished. Available'):
            
            print('Sending command to rover\n')


            # --------------------------- UPDATE SEQUENCER ----------------------------
            
            if (status == 'Task finished. Available'):
                
                # Update Explored Map
                self.update_explored(rover)

                # Update Sequencer
                self.update_sequencer(rover)


                # Inform
                print((rover) +" is available. Sending next command...\n")
            
            else:
                
                # Inform
                print((rover) +" is in emergency state. Sending it home...\n")


            # ------------------------ GET COMMAND ----------------------------------
            
            try:
                # Find most prioritary command of the rover
                cmd  =  [d for d in self.Sequencer if (d[4] == rover and d[1] == 1)][0]
            
            except:

                # No command available. Wait for computing priority again
                self.pending = True
                
                # Updating rovers task to none
                cmd = [None,None,None,None,None,None,None,None,None]
                self.update_rover_task(rover,cmd)

                return None

            # Serialize command
            cmddata   =  self.serialize_message(cmd)

            # ------------------ UPDATE TASKS AND DISTANCE ----------------------------
            
            # Update rovers tasks
            self.update_rover_task(rover,cmd)

            # Update distances
            self.update_distances(rover)


            # ----------------------- SEND COMMAND --------------------------------------

            # Send command to Rover over serial port
            self.SerialModule.send_data(serialport, baudrate, cmddata)

            # Set pending flag to false
            self.pending   =  False

            # Reset status
            self.new_status = False

            #  *********************** API VERSION  *****************************************
            # |                                                                             |
            # | Send data to SensorRover (HTTP POST)                                        |
            # | response = requests.post('http://127.0.0.1:5000/LoRa/command', json=data))  |
            # |                                                                             |
            # |  # Check if data was sent successfully                                      |
            # |      if response.status_code == 200:                                        |
            # |          print("Data sent to SensorRover successfully")                     |
            # |          ok = True                                                          |
            # |      else:                                                                  |
            # |          print("Failed to send data to SensorRover")                        |
            # |          ok = False                                                         |
            # |                                                                             |
            # |                                                                             |
            # |------------------------------------------------------------------------------
    
    # -------------------------------------------------- SEND TIMEOUT----------------------------------------------------------------------
    def send_timeout(self,client):

        """
        This method checks if timeout has ocurred and sends the timeout of the most prioritary command.

        """
        # Current time
        timenow = time.time()

        # Check if timeout has ocurred in rover task
        for i in self.RoversId:

            # ID of current rover task
            current_id  = [d[1] for d in self.RoversTask if (d[0] == i)]
           
            if (current_id == []):
                current_id = current_id[0]
            
                # Index of current task in Sequencer
                index = [i for i,x in enumerate(self.Sequencer) if (x[0] == current_id)][0]
            
                # Timeout  and timetstamp of current task
                timeout     = self.Sequencer[index][6]
                timestamp   = self.Sequencer[index][6]

                # Check if timeout has ocurred
                if (timenow > timestamp + timeout):

                    # Structure mesage for timeout
                    index = [i for i,x in enumerate(self.Locations) if (x[0] == i)][0]

                    msg = {}
                    msg['Sender']    =  i
                    msg['Latitude']  =  self.Locations[index][1]
                    msg['Longitude'] =  self.Locations[index][2]
                    msg['Status']    =  'Timeout'

                    # Serialize to json
                    msg = js.dumps(msg)
                   
                   
                    # Publish to MQTT topic
                    self.Mqtt.publish(client,emergencytopic,msg)

         

 # ****************************************************** AUXILIARY METHODS ****************************************************

    
    # ------------------------------------------------------- CHECK DATA -------------------------------------------------------  
    def check_data(self,usercmd,roversinfo):

        """
        This method will:
        - Check if new user command is received
        - Check if new rover info is received and computes if it is a command or status
               
        """

        # ------------ CHECK IF NEW USER COMMAND IS RECEIVED --------------------   
        

        if (usercmd is not None): 
            
            if (self.usercmd is None or usercmd['Timestamp'] > self.usercmd['Timestamp']):
                self.new_usercmd = True
            else:
                self.new_usercmd = False
       
        else:
            self.new_usercmd = False
        
        # ------------- CHECK IF NEW ROVER INFO IS RECEIVED -----------------------
        
        # Check if new rover info is received and check if it is a command or status
       
        if (roversinfo is not None and Shared['newroverinfo']):
                                
            if ("Interesting location" in roversinfo["Status"]):    # Command message
                self.new_sensorcmd =  True
               
            else:
                self.new_status    =  True
            
            Shared['ack']      =  True

        else:
            self.new_status    =  False
            self.new_sensorcmd =  False
            

    # -------------------------------------------------- SERIALIZE MESSAGE ---------------------------------------------------------- 
    def serialize_message(self,command):

        """
        This method will create the message to be sent and serialize it as a json
        """
        # --------------- MESSAGE STRUCTURE ---------------------------
        msg = {}
        msg["Sender"]    =  command[5]
        msg["RoverId"]   =  command[4]
        msg["Timestamp"] =  int(time.time())
        msg["Latitude"]  =  round(command[2],4)
        msg["Longitude"] =  round(command[3],4)
        msg["Priority"]  =  command[1]
            

        # --------------------- COMPUTE TIMEOUT -------------------------
        try:

            # ID of current rover task
            current_id =  [d[1] for d in self.RoversTask if (d[0] == command[4])][0]

            # Index of current task in Sequencer
            index      =  [i for i,x in enumerate(self.Sequencer) if (x[0] == current_id)][0]
        
        except:

            # Index of most priority task
            index      =  [i for i,x in enumerate(self.Sequencer) if (x[4] == command[4])][0]

        
        # ------------------ ADD TIMEOUT ----------------------------------
        msg["Timeout"] =  self.compute_timeout([[command[2],command[3]],[self.Sequencer[index][2],self.Sequencer[index][3]]])

       
        # ------------- SERIALIZE MESSAGE TO JSON -------------------------
        json = js.dumps(msg) 
        
        
        return json

   
    # -------------------------------------------- COMPUTE DISTANCE---------------------------------------------------------
    def compute_distance(self,coordinates):

        """
        This method computes the distance between two coordinates.
        The distance is computed based on the Haversine formula.
        
        """
        
        longitude = []
        latitude  = []

        # Convert coordinates to radians
        longitude.append(np.radians(float(coordinates[0][1])))
        latitude.append(np.radians(float(coordinates[0][0])))
        longitude.append(np.radians(float(coordinates[1][1])))
        latitude.append(np.radians(float(coordinates[1][0])))

        # Planet radius      
        Radius = 1737                  

        # Delta longitude and latitude
        dlongitude = np.subtract(longitude[1], longitude[0])
        dlatitude  = np.subtract(latitude[1], latitude[0])

        # Distance formula
        a = np.add(np.power(np.sin(np.divide(dlatitude, 2)), 2), np.multiply(np.cos(latitude[0]),np.multiply(np.cos(latitude[1]), np.power(np.sin(np.divide(dlongitude, 2)), 2))))
        
        c = np.multiply(2, np.arcsin(np.sqrt(a)))

        return c*Radius
    

    # --------------------------------------------- TIMEOUT --------------------------------------------------------------------------
    def compute_timeout(self, coordinates):

        """
        This method computes the timeout of the command.
        The timeout is computed based on the distance between the previous command and the current command.
        
        """

        # Minimum linear speed
        speed = 0.5

        # Distance to command place
        distance = self.compute_distance(coordinates)

        # Timeout
        timeout  = distance/speed + 20

        return timeout

    # -------------------------------------------------- COMPUTE ROVER ID -----------------------------------------------------------------
    def compute_roverid(self,status):

        """
        This method returns the roverid that is gonna do the task
        """
        flags = []

        # Get flags from status
        status.replace('Interesting location.','')
        flags.append(status.split(' '))

        # Conexion to mysql database                          
        cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

        # Cursor
        cursor = cnx.cursor()
    
        # Get roversid that can do the task 
        results = []
        for flag in flags:
            query = "SELECT Rovers FROM AsignedRovers WHERE Flag = " + flag + ";"
            cursor.execute(query)
            results.append(cursor.fetchall())

            # Reset cursor
            cursor = cnx.cursor()

        # Close connection
        cnx.close()

        # Eliminate duplicates
        rovers = []

        for rover in results:
            if rover not in rovers:
                rovers.append(rover)    
     
        # Send rovers that can do the task
        
        distances = []

        if (rovers != []):

            for i in results:
                
                matches = [d for d in self.RoversTask if (d[0] == i)]

                # Send rover with no task
                if (len(matches) == 0):
                    rover = i

                    # Inform 
                    print("Rover " + str(rover) + " is going to do the task\n")

                    return rover
               
                # Send rover closer to the task location
                else:
                    distances.append(self.compute_distance([[matches[0][2],matches[0][3]],[self.sensorcmd['Latitude'],self.sensorcmd['Longitude']]]))
                    continue
            
            index = distances.index(min(distances))
            rover = results[index][1]

            # Inform
            print("Rover " + str(rover) + " is going to do the task\n")

            return  rover
            
        else:

            # Inform
            print("Can't assign a rover for this task\n")
        
      
            

    # ******************************************************** UPDATES **************************************************************************
    
    # ---------------------------------------------------- UPDATE ROVER TASK --------------------------------------------------------------------
    def update_rover_task(self, rover,cmd):

        """
        This method updates the register of the indexes of the tasks in the Sequencer
        that belongs to each Rover.
        
        """

        # Update RoversId in RoversTask with tasks in the Sequencer 
        newroversid = []

        for i in range(len(self.Sequencer)):
            if (self.Sequencer[i][4] not in newroversid):
                newroversid.append(self.Sequencer[i][4])
        
        lastroversid =  [d[0] for d in self.RoversTask]
        missing      =  [d for d in newroversid if (d not in lastroversid)] 
            
        if (missing != []):
            for i in missing:
                self.RoversTask.append([i,None,None,None]) 
        
          
        latitude  = []
        longitude = []

        if (rover is not None):

            # Update current task id of rover which has just finished a task
            index = [d for d,x in enumerate(self.RoversTask) if (x[0] == rover)][0]
            self.RoversTask[index][1] = cmd[0]
            self.RoversTask[index][2] = cmd[2]
            self.RoversTask[index][3] = cmd[3]
            
            # Coordinates of current task
            latitude.append(cmd[2])
            longitude.append(cmd[3])
        
        else:

            latitude  = None
            longitude = None
        
        # ---------------------- SAVE IN MYSQL DB -----------------------------
        
        # Get rows to insert
        data = [tuple(d) for d in self.RoversTask]
        
        # Insert data
        # Conexion to mysql database                           
        cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

        # Cursor
        cursor = cnx.cursor()

        # Delete last Sequencer
        drop_query = "DROP TABLE IF EXISTS Task;"
        cursor.execute(drop_query)

        # Create a new table
        create_query = "CREATE TABLE Task (RoverId VARCHAR(255),TaskId INT,Latitude FLOAT, Longitude FLOAT);"
        cursor.execute(create_query)

        # Insert data
        query = "INSERT INTO Task (RoverId, TaskId, Latitude, Longitude) VALUES (%s, %s, %s, %s);"
        cursor.executemany(query, data)

        # Commit
        cnx.commit()

        # Close conection
        cnx.close()
        
        # ------------------- SERIAL MESSAGES -----------------------------------   
        print("\nROVERS TASKS:\n")
        print('\n' + (tabulate(self.RoversTask, headers=["RoverId", "Current Task Id", "Latitude", "Longitude"]))) 
        print("--------------------------------------")


    # ------------------------------------------------------- UPDATE DISTANCES ------------------------------------------------------------------
    def update_distances(self,rover):
        
        """
        This method updates distances to the new current task. 
        """
        current_id = [d[1] for d in self.RoversTask if (d[0] == rover)]
      
        if (len(current_id) > 0):
            
            # ID of current rover task
            current_id = current_id[0]
            
            # Index of current task in Sequencer
            index = [i for i,x in enumerate(self.Sequencer) if (x[0] == current_id)]
           
            # Get coordinates of current command
            lat_now  = self.Sequencer[index[0]][2]
            long_now = self.Sequencer[index[0]][3]

            # Get tasks and indexes in Sequencer for rover received as user commands or sensor commands
            matches = [d for d in self.Sequencer if (d[4] == rover and (d[5] == 'User' or d[5] == 'SensorRover'))]  
            indexes = [i for i,x in enumerate(self.Sequencer) if x in matches]

            # Update distances
            for i in indexes:
                lat  = self.Sequencer[i][2]             
                long = self.Sequencer[i][3]

                # Update distance
                self.Sequencer[i][8] = self.compute_distance([[lat_now, long_now],[lat, long]])

            # Matches with updated distances
            matches = [d for d in self.Sequencer if (d[4] == rover and (d[5] == 'User' or d[5] == 'SensorRover'))] 

            # Sort sublist based on distance
            matches = sorted(matches, key=lambda x: x[8])

            # Replace Sequencer with sorted sublist
            for i in range(len(indexes)):
                self.Sequencer[indexes[i]] = matches[i]


    # ------------------------------------------------------- UPDATE SEQUENCER ------------------------------------------------------------------

    def update_sequencer(self,rover):

        """
        This method updates the Sequencer.  
        Deletes alredy finished tasks. 
        """
        try:
            # ID of current rover task
            id = [d[1] for d in self.RoversTask if (d[0] == rover)][0]

            # Index of current task in Sequencer
            index = [i for i,x in enumerate(self.Sequencer) if (x[0] == id)][0]

            # Update Sequencer (Delete task alredy finished)
            self.Sequencer.pop(index) 
            
            # Update prioritys
            self.update_priority(rover,1,delete=True)
        
        except:
            pass
        
        # -------------------- SAVE SEQUENCER IN MY SQL DATA BASE ----------------------------
        
        # Conexion to mysql database                         
        cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

        # Cursor
        cursor = cnx.cursor()
        
        # Get Sequencer rows
        data = [tuple(d) for d in self.Sequencer]

        # Delete last Sequencer
        drop_query = "DROP TABLE IF EXISTS Sequencer"
        cursor.execute(drop_query)

        # Create a new table
        create_query = "CREATE TABLE Sequencer (TaskId INT, Priority INT, Latitude FLOAT, Longitude FLOAT, RoverId VARCHAR(255), Sender VARCHAR(255), Timeout FLOAT, Timestamp FLOAT, Distance FLOAT);"
        cursor.execute(create_query)

        # Insert Sequencer
        query = "INSERT INTO Sequencer (TaskId, Priority, Latitude, Longitude, RoverId, Sender, Timeout, Timestamp, Distance) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s);"
        cursor.executemany(query, data)

        # Commit
        cnx.commit()

        # Close conection
        cnx.close()
        

        # ----------------------- SERIAL MESSAGES (TABLE FORMAT) -----------------------------
        print ("\n UPDATED SEQUENCER:\n")
       
        # Sequencer table
        print('\n' + tabulate(self.Sequencer, headers=["ID", "Priority", "Lat", "Long", "RoverID", "Sender", "Timeout", "Timestamp", "Distance"]))

        # ------------------------------------------------------------------------------------

  
    # ------------------------------------------------------- UPDATE EXPLORED MAP --------------------------------------------------------------
    def update_explored(self, rover):

        """
        This method updates the explored map.
        It takes as an argument the name of the rover which has finished its task. 
        
        """
        
        # Actual timestamp
        now = time.time() 
       
        # Update last time explored of alredy explored coordinates
        for i in range(len(self.Explored)):

            # Compute days since last time explored
            last = (now - self.Explored[i][4])/(3600*24)  

            # Update last time (days)
            self.Explored[i][3] = self.Explored[i][3] + last                           

        
        # Update Explored list and Sequencer
        try:    
            # ID of current rover task
            id = [d[1] for d in self.RoversTask if (d[0] == rover)][0]

            # Index of current task in Sequencer
            index = [i for i,x in enumerate(self.Sequencer) if (x[0] == id)][0]
            
            # Explored list : (Latitude, Longitude, RoverId, Last time (days), Timestamp )
            self.Explored.append([self.Sequencer[index][4], self.Sequencer[index][2], self.Sequencer[index][3], 0, now])

            print("Explored map updated\n")

        except:
           print("Explored map not updated\n")

        # -------------------- SAVE EXPLORED MAP IN SHARED REGISTER-------------
            
        # Conexion to mysql database                           
        cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

        # Cursor
        cursor = cnx.cursor()
        
        # Get Sequencer rows
        data = [tuple(d) for d in self.Explored]

        # Delete last Sequencer
        drop_query = "DROP TABLE IF EXISTS Explored;"
        cursor.execute(drop_query)

        # Create a new table
        create_query = "CREATE TABLE Explored (RoverId VARCHAR(255), Latitude FLOAT, Longitude FLOAT, LastTime FLOAT, Timestamp FLOAT);"
        cursor.execute(create_query)

        # Insert Sequencer
        query = "INSERT INTO Explored ( RoverId, Latitude, Longitude, LastTime, Timestamp) VALUES (%s, %s, %s, %s, %s);"
        cursor.executemany(query, data)

        # Commit
        cnx.commit()

        # Close conection
        cnx.close()
        

        #-------------------- SERIAL MESSAGES ------------------------------------
        print("EXPLORED MAP: \n")
        print('\n' + tabulate(self.Explored, headers=["Latitude", "Longitude", "RoverId", "Last time (days)", "Timestamp"]))
        #---------------------------------------------------------------------------       
    
    # ------------------------------------------------------- UPDATE PRIORITY ------------------------------------------------------------------
    def update_priority(self, rover, priority, delete):

        """
        Update priority of less prioritized task. If delete is true, the priority is decreased.

        """
        # Find indexes of tasks for rover
        indexes = [i for i,x in enumerate(self.Sequencer) if (x[4] == rover and x[1] >= priority)]

        if (not delete):
            
            # Update priority
            for i in indexes[1:]:
                self.Sequencer[i][1] = self.Sequencer[i][1] + 1
        
        else:
            
            # Update priority
            for i in indexes:
                self.Sequencer[i][1] = self.Sequencer[i][1] - 1





#========================================================================================================================================================
#                                                                THREADS
#======================================================================================================================================================== 
        
# -------------------------------------------------------------- CENTRAL THREAD --------------------------------------------------------------------------
def Central():

    #Create SequencePlanner object
    CentralModule = SequencePlanner()

    # Connect to MQTT Broker
    client = CentralModule.Mqtt.connect_mqtt(broker,port)

    # Inform
    print("......STARTING CENTRAL MODULE......")

    # ----------------------------- CENTRAL LOOP ----------------------------------------------

    # Loop 
    while True:

        # Update Central Information
        CentralModule.updaterovers()

        # Process Location
        CentralModule.process_location()

        # Get data
        CentralModule.get_data()

        # Generate automatic plan
        CentralModule.auto_plan()

        # Compute priority
        CentralModule.compute_priority(client)

      
        # Check rovers availability
        status     =  CentralModule.status
        new_status =  CentralModule.new_status
        pending    =  CentralModule.pending 
        
        print("....NEW STATUS RECEIVED: ", new_status)

        if (new_status or pending):
            
            # Send next command to Rover
            CentralModule.send_cmd(status['Sender'],status['Status'])
            
            # Inform
            print('Sent command to ' + str(status['Sender']))
            

        else:
            
            # Check if timeout has ocurred and send timeout
            CentralModule.send_timeout(client)
            
            # Inform
            print("No Rovers avilable. Waiting...\n")

        time.sleep(3)


# -------------------------------------------------------------- SERIAL THREAD -------------------------------------------------------------------------
def SerialRead ():
    
    # Create Serial object
    SerialModule = HandleSerial()
    
    # ---------------------- SERIAL LOOP ----------------------------

    # Loop 
    while True:

        # Read serial port
        Shared["roversinfo"] = SerialModule.read_serial(serialport, baudrate)




#========================================================================================================================================================
#                                                               MAIN
#======================================================================================================================================================== 
def main():



    # ------------------ CREATE THREADS --------------------
    
    CentralThread  = threading.Thread(target=Central)                          # Central thread
    SerialThread   = threading.Thread(target=SerialRead)                       # Serial thread
   
    
    # ------------------ START THREADS ---------------------
   
    CentralThread.start()
    SerialThread.start()
   
    # LOGGING INFO
    print("THREADS STRATED")

    

# MAIN EXECUTION
if __name__ == "__main__":
    main()