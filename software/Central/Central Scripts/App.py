# **************************************************** FLASK APP *****************************************************************

import numpy as np
import pandas as pd
import math
import utm
import random 
import warnings
import plotly.io as pio
import plotly.graph_objects as go
import colorlover as cl
from   IPython.display import HTML
import urllib.request, json
from   IPython.display import HTML
from   dash import Dash, html, dcc, Input, Output
import plotly.express as px
import dash
import  paho.mqtt.client as mqtt_client
from   dash.dependencies import Input, Output
from   flask import Flask
import pymysql
import dash_bootstrap_components as dbc
from   flask_ngrok import run_with_ngrok
import requests
import json as js


# --------------------------------------------- 3D RENDER PLANET -------------------------------------------------------------


# --------------- GET [X Y] ------------------


def getxy(latitudes,longitudes,planetbase):


    # Initialize empty lists for x and y
    x = []
    y = []
    epsilon = 1e-10
    
   
    for latitude, longitude in zip(latitudes, longitudes):
        
        if longitude is not None and latitude is not None:
            
            xvalue = utm.from_latlon(latitude, longitude)[0]/1000
            yvalue = utm.from_latlon(latitude, longitude)[1]/1000
            
            x.append(xvalue)

            if (abs(yvalue) > 1000):
                y.append(yvalue)
            else:
                y.append(yvalue+3000)
                
    
    return x, y


# -------------- DATA FRAME OPTIONS ----------------------

pd.set_option('display.max_rows'   , 500) 
pd.set_option('display.max_columns', 500)
warnings.filterwarnings("ignore")


# ----------------- DATA IMPORT ----------------------

# HTTP POST (get earth coordinates (x,y,z))
url = "https://raw.githubusercontent.com/Nov05/playground-fireball/master/data/earth.json"
response = urllib.request.urlopen(url)
json_data = json.loads(response.read())

#------------------- DATA ANALYSIS -------------------
#  PARALLEL PLANET MAP
x0 = json_data['data'][0]['x']

y0 = json_data['data'][0]['y']
z0 = json_data['data'][0]['z']

#  MERIDIANS PLANET MAP
x1 = json_data['data'][1]['x']
y1 = json_data['data'][1]['y']
z1 = json_data['data'][1]['z']

#  BASE PLANET MAP
x2 = json_data['data'][2]['x']
y2 = json_data['data'][2]['y']
z2 = json_data['data'][2]['z']


# List for x and y on planet base
xbase = []
xbase.append(x0)
xbase.append(x1)


ybase = []
ybase.append(x0)
ybase.append(x1)


planetbase = np.array([xbase[0],ybase[0]]).T


# --------------- COLORS ------------------------------
# Colors palette
colors = ["rgb(0,0,255)","rgb(255, 255, 255)","rgb(12, 52, 61)","rgb(106, 168, 79)","rgb(229.5,229.5,25.5)","rgb(106, 168, 79)","rgba(31, 119, 180, 0.34)","rgb(31, 119, 180)" ]
HTML(cl.to_html(colors))
color_background = 'black'                         # Background
color_lines      = 'darkgrey'                      # Parallels and Meridians
color_base       = 'dimgrey'                       # Base surface

opacity_base = 0.1
opacity_land = 0.8
size_lines   = 1

# -------------------- DATA PLOT -----------------------
# PRALLELS
parallels = go.Scatter3d(x=x0, y=y0, z=z0,mode='lines',marker=dict(size=size_lines,color=color_lines, opacity=opacity_base,showscale=False,line=dict(width=size_lines,color=color_base)),showlegend=False,name='Parallel')

# MERIDIANS
meridians = go.Scatter3d(x=x1,y=y1,z=z1,mode='lines',marker=dict(size=size_lines,color=color_lines,opacity=opacity_base,showscale=False,line=dict(width=size_lines,color=color_base,)),showlegend=False,name='Meridian')

# BASE
base      = go.Scatter3d(x=x2,y=y2,z=z2,mode='markers',marker=dict(size=2,color=color_base,opacity=opacity_base,showscale=False,line=dict(width=1, color=color_base)),showlegend=False,name='Base',)

# PLANET DATA
data = [parallels, meridians, base]

# ------------------- LAYOUT ---------------------------
# Scene
scene = dict(
        xaxis = dict(title="x axis",color=color_background,backgroundcolor=color_background,showaxeslabels=False,showline=False,showgrid=False,zeroline=False,visible = False),
        yaxis = dict(title="y axis",color=color_background,backgroundcolor=color_background,showaxeslabels=False,showline=False,showgrid=False,zeroline=False,visible = False),
        zaxis = dict(title="z axis",color=color_background,backgroundcolor=color_background,showaxeslabels=False,showline=False,showgrid=False,zeroline=False,visible = False),
        )

# Layout
layout = go.Layout(autosize=False,width=930,height=600,margin=dict( l=0,r=0,b=0,t=0),title=dict(text='',x=0.55, y=0.95,font=dict(size=20,color='white',family='Times New Roman')),scene = scene,paper_bgcolor=color_background,plot_bgcolor=color_background,showlegend = False,hovermode = 'closest',
        images=[dict(
        source="https://cdn.pixabay.com/photo/2017/08/30/01/05/milky-way-2695569_1280.jpg", 
        yref="paper",
        x=0,
        y=1,
        sizex=1,
        sizey=1,
        sizing="stretch",
        opacity=0.5,
        layer="below")])


# --------------------- HTML --------------------------------
# Render the plot in html
fig = go.Figure(data=data, layout=layout)

pio.write_html(fig, '3DRenderPlanet.html')



# ------------------------ SERVER ------------------------------

server = Flask(__name__)

# ---------------------- APP DASH ----------------------------

external_stylesheets = [
    'https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/css/bootstrap.min.css',
    {
        'href': 'https://use.fontawesome.com/releases/v5.8.1/css/all.css',
        'rel': 'stylesheet',
        'integrity': 'sha384-50oBUHEmvpQ+1lW4y57PTFmhCaXp0ML5d60M1M7uH2+nqUivzIebhndOJK28anvf',
        'crossorigin': 'anonymous'
    }
]

app3D = Dash(__name__, server = server,external_stylesheets=[dbc.themes.BOOTSTRAP, 'https://use.fontawesome.com/releases/v5.8.1/css/all.css'])

# Run APP with ngrok
run_with_ngrok(server)

df = px.data.iris()


# ---------------------- APP LAYOUT -------------------------------

# Botones
button1 = dbc.Button(["Sequencer ", html.Span(className="fa fa-table", style={'margin-left': '6px'})], id="btn-1", n_clicks=0,style = {'borderRadius': '15px','width': '260px', 'height': '80px','margin': '5px', 'fontFamily': 'Times New Roman','color': 'black', 'border': 'none', 'padding': '15px 27px', 'textAlign': 'center','textDecoration': 'none', 'display': 'inline-block', 'fontSize': '22px', 'background-color': '#CDC5BF'})
button2 = dbc.Button(["Current Tasks ", html.Span(className="fa fa-truck-pickup", style={'margin-left': '4px'})], id="btn-2", n_clicks=0,style = {'borderRadius': '15px','width': '260px', 'height': '80px','margin': '5px', 'fontFamily': 'Times New Roman','color': 'black', 'border': 'none', 'padding': '15px 27px', 'textAlign': 'center','textDecoration': 'none', 'display': 'inline-block', 'fontSize': '22px', 'background-color': '#CDC5BF'})
button3 = dbc.Button(["Explored map ", html.Span(className="fa fa-compass", style={'margin-left': '4px'})], id="btn-3", n_clicks=0,style = {'borderRadius': '15px','width': '260px', 'height': '80px','margin': '5px', 'fontFamily': 'Times New Roman','color': 'black', 'border': 'none', 'padding': '15px 27px', 'textAlign': 'center','textDecoration': 'none', 'display': 'inline-block', 'fontSize': '22px', 'background-color': '#CDC5BF'})


# Layout
app3D.layout = html.Div(style={'backgroundColor':'black','width': '100%', 'height': '100%', 'margin': '0 auto'}, children=[
    html.Div([
        html.H1('CENTRAL MODULE',  style={'textAlign': 'center', 'margin': '0px', 'fontFamily': 'Times New Roman', 'color': 'black','fontSize': '25px','backgroundColor': 'darkgrey'})]),
    html.Div(style = {'display': 'flex', 'justify-content': 'center', 'flex-direction': 'row'}, children=[
        button1, button2, button3,
    ]),
    html.Div(style={'borderTop': '2px solid white'}),
    html.Div([
        dcc.Graph(id ='3DPlanet', figure=fig),
        dcc.Interval(id='interval-component',interval=15*1000, n_intervals=0)
    ])
])

# ------------------------------------------ CALLBACK TO UPDATE PLOT ---------------------------------------------------
@app3D.callback(
    Output('3DPlanet', 'figure'),
    Input ('btn-1', 'n_clicks'),
    Input ('btn-2', 'n_clicks'),
    Input ('btn-3', 'n_clicks'),
    [Input('interval-component', 'n_intervals')])
def update(btn1,btn2,btn3,n_clicks):

    
    
    # ------------------------- GET UPDATED DATA -----------------------------

   
    # -------------- SEQUENCER -------------------
 
    # Create a connection to MySQL
    cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

    # Create a cursor object
    cursor = cnx.cursor()

    # Define the select query
    select_query = "SELECT RoverId, Latitude, Longitude FROM Sequencer;"

    # Execute the select query
    cursor.execute(select_query)

    # Fetch all the rows
    data = cursor.fetchall()

    # Close the connection
    cnx.close()
    
    latitudes  =  []
    longitudes =  []
    labels1    =  []
    
    # Now 'data' is a tuple of tuples where each tuple represents a row from the table.
    for row in data:
       latitudes.append(row[1])
       longitudes.append(row[2])
       labels1.append((row[0]) + ' has this location as task assigned')

       
    x1,y1 = getxy(latitudes,longitudes,planetbase)
    z1    = list((6400/90)*latitud for latitud in latitudes)
    
    sequencer = go.Scatter3d(x=x1, y=y1, z=z1,mode='markers',marker=dict(size=5,color='blue',opacity=1,showscale=False,line=dict(width=1, color='blue')),text = labels1)
  

    # -------------- EXPLORED -------------------

    # Create a connection to MySQL
    cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

    # Create a cursor object
    cursor = cnx.cursor()

    # Define the select query
    select_query = "SELECT RoverId, Latitude, Longitude FROM Explored;"

    # Execute the select query
    cursor.execute(select_query)

    # Fetch all the rows
    data = cursor.fetchall()

    # Close the connection
    cnx.close()
    
    latitudes  = []
    longitudes = []
    labels    = []
    
    # Now 'data' is a tuple of tuples where each tuple represents a row from the table.
    for row in data:
       latitudes.append(row[1])
       longitudes.append(row[2])
       labels.append((row[0]) + ' explored this place')

       
    x2,y2 = getxy(latitudes,longitudes,planetbase)
    z2    = list((6400/90)*latitud for latitud in latitudes)
    
    explored = go.Scatter3d(x=x2, y=y2, z=z2,mode='markers',marker=dict(size=5,color='green',opacity=1,showscale=False,line=dict(width=1, color='green')),text = labels,name='Explored')
    
    # ----------------- CURRENT TASK -------------------

    # Create a connection to MySQL
    cnx = pymysql.connect(host='localhost', user='root', password='centralrovers', database='CentralInfo')

    # Create a cursor object
    cursor = cnx.cursor()

    # Define the select query
    select_query = "SELECT RoverId, Latitude, Longitude FROM Task;"

    # Execute the select query
    cursor.execute(select_query)

    # Fetch all the rows
    data = cursor.fetchall()

    # Close the connection
    cnx.close()
    
    latitudes  = []
    longitudes = []
    labels     = []
    
    # Get coordinates
    for row in data:
       latitudes.append(row[1])
       longitudes.append(row[2])
       labels.append((row[0]) + ' now going to this place')

       
    x3,y3 = getxy(latitudes,longitudes,planetbase)
    z3    = list((6400/90)*latitud for latitud in latitudes if latitud is not None)
    
    task = go.Scatter3d(x=x3, y=y3, z=z3,mode='markers',marker=dict(size=4,color='red',opacity=1,showscale=False,line=dict(width=1, color='red')),text = labels,name='Current Task')
    
    # ----------------- PLOT ------------------------

    # Plot is updated by time intervals and button clicks

    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]                 # Click on button
    
    if 'btn-1' in changed_id:
        
        # Addind sequencer trace to original figure
        fig.data = fig.data[0:3]
        fig.add_trace(sequencer)


    elif 'btn-2' in changed_id:
        
        # Addind explored trace to original figure
        fig.data = fig.data[0:3]
        fig.add_trace(task)
        

    elif 'btn-3' in changed_id:
        
        # Addind  trace to original figure
        fig.data = fig.data[0:3]
        fig.add_trace(explored)

    # Adding all traces 
    else: 
        if (sequencer is not None and sequencer not in fig.data):
            fig.add_trace(sequencer)
        
        if (explored is not None and explored not in fig.data):
            fig.add_trace(explored)
       
        if (task is not None and task not in fig.data):
            fig.add_trace(task)
    
    return fig


# =========================================== MQTT =========================================================

# MQTT CONFIGURATION
broker     =   'iot.ac.uma.es'
port       =   1883                     
client_id  =   f'render-{random.randint(0, 100)}'
username   =   'II15'
password   =   'qX5nQDDd'

# TOPICS
urltopic   = "II15/URL"



# ------------------------------------------- CONNECT MQTT ---------------------------------------------------
def connect_mqtt(broker,port):
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

# ------------------------------------------- PUBLISH MQTT -----------------------------------------------------
def publish(client,topic,msg):
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
    

# ====================================== MAIN =======================================

if __name__ == "__main__":
    
    
    # ------------- SAVE DYNAMIC URL -------------------
    # Get the ngrok tunnels
    response = requests.get("http://localhost:4040/api/tunnels")

    # Parse the JSON response
    data = json.loads(response.text)

    # Get the URL of the first tunnel
    ngrok_url = data['tunnels'][0]['public_url']

    # Message
    msg = {"url": ngrok_url}
    msg = js.dumps(msg)

    # Connect to Mqtt Broker
    client = connect_mqtt(broker,port)

    # Publish URL
    publish(client,urltopic,msg)

    
    # ----------------- RUN SERVER -------------------
    server.run()
