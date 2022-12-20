#!/usr/bin/env python3

##################################################################
# Name:             arduino.py
# Author:           Tal Akerman
# Version:          1.0 (Dec 2022)
# Description:      This script runs 3 proccesses:
#                   1. connects to the Arduino board for 
#                       periodic read
#                   2. connects to the Arduino board for 
#                       one time read via Socket
#                   3. RPi fan control - refer to PCB creation
#
##################################################################


################
#   Imports
################
import socket
from threading import Thread
from multiprocessing import Process,Event
import serial
import time
import sys
import json
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime
import RPi.GPIO as IO
import subprocess
import signal

################
#   Globals
################

CONFIG_FILE = 'config.json'
try:
    with open(CONFIG_FILE) as f:
        CONFIG = json.load(f)
except Exception as e:
    print("Cannot open configuration file " + str(CONFIG_FILE) + ": " + str(e))
    sys.exit(1)

# Command timeout to Arduino
TIMEOUT = CONFIG["timeout"]
# Socket receive max length
MAX_LENGTH = 4096
# Every X seconds to get sensors data from Arduino 
INTERVALS = CONFIG["intervals"]
# Socket connection 
PORT = CONFIG["socket_port"]
HOST = CONFIG["socket_host"]
# Arduino USB connection 
TTY = CONFIG["tty"]
BAUD = CONFIG["baud"]
PERIODIC_COMMAND = CONFIG["periodic_command"]
#Influx DB connection
INFLUX_DB_SERVER = CONFIG["influx_db_server"]
INFLUX_DB_SENSORS = CONFIG["influx_db_sensors"]
INFLUX_DB_EVENTS = CONFIG["influx_db_events"]
LOCATION = CONFIG["location"]
# JSON file with sensors calibrations 
CALIBRATION_FILE = CONFIG["calibrations_file"]
# RPi GPIO Pin the fan is connected to
FAN_PIN = CONFIG["fan_gpio_pin"]
# Silence error events for X times
SHUT_UP_INTERVAL = CONFIG["shut_up_interval"]
CURRENT_SHUT_UP_COUNT = {}


##################################################################

# Make Arduino response pretty, and convert to JSON
def get_response(response):
    response = response[:-3]
    response = response.replace("'", "\"" )
    response_json = json.loads(response)
    return response,response_json



##################################################################

# Convert response from Arduino to numbers where it can
# And add calibrations from calibration file data
def convert_to_numbers(response_json):
    for key in response_json:
        try:
            response_json[key] = float(response_json[key])
            if(key in calibrations):
                response_json[key] = round((response_json[key] + calibrations[key]),2)
        except:
            pass
    return response_json

##################################################################

# Write to DB
def write_to_db(domain,response_json,measurement):
    if(domain == INFLUX_DB_SENSORS):
        response_json = convert_to_numbers(response_json)
    print_debug("Writing to DB: " + str(response_json))
    loaded = [
                {
                    "measurement": measurement,
                    "tags": {
                        "Ort": LOCATION,
                        "domain": domain
                    },
                    "time": datetime.utcnow(),
                    "fields": response_json
                }
            ]
    write_api.write(bucket=domain, record=loaded)
    return

##################################################################

# Check if Arduino response succeed
def check_response(response_json):
    if(response_json["ok"] == "1"):
        if("sht31a_temp" in response_json):
            write_to_db(INFLUX_DB_SENSORS,response_json,CONFIG["influx_db_sensors"])
        else:
            write_to_db(INFLUX_DB_EVENTS,response_json,CONFIG["influx_db_events"])
    elif("error" in response_json):
        write_to_db(INFLUX_DB_EVENTS,response_json,CONFIG["influx_db_events"])
    else:
        print_error_to_db("check_response","Something went wrong")
        return 1
    return 0


##################################################################

# Check if Arduino response timed-out 
def check_timeout(now,cmd_started,treshold):
    if((now - cmd_started) >= treshold):
        print_error_to_db("check_timeout","Timeout or Command not supported")
        return 1
    return 0


##################################################################

# Wait for a response from Arduino
# Get the response and print it 
def wait_and_print():
    cmd_started = time.time()
    now = time.time()
    while(arduino.inWaiting()==0 and (now - cmd_started) < TIMEOUT): 
            now = time.time()
            pass
    if(check_timeout(now,cmd_started,TIMEOUT)):
        return 1
    while(arduino.inWaiting()>0):
        response = arduino.readline().decode('utf-8').rstrip()
    response,response_json = get_response(response)
    if(check_response(response_json)):
        return 1
    arduino.flushInput()
    return 0


##################################################################

# Send command to Arduino
def send_command(command):
    try:
        arduino.write(str(command + "#").encode('utf-8'))
        time.sleep(0.1)
        wait_and_print()
    except:
        pass
    return


##################################################################

# Handle socket request
def handle(e,clientsocket):
    while 1:
        buf = clientsocket.recv(MAX_LENGTH).decode("utf-8")
        if(buf == ''):
            return
        send_command(buf)


##################################################################

# Process 1 - get periodic sensors reading
def periodic_read(e):
    while(True):
        e.wait()
        send_command(PERIODIC_COMMAND)
        time.sleep(INTERVALS)
        

##################################################################

# Process 2 - handle socket connection and requests
# This is multithreaded - can open multiple connections 
def one_time_read(e):
    serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serversocket.bind((HOST, PORT))
    serversocket.listen(10)
    while(True):
        e.wait()
        (clientsocket, address) = serversocket.accept()
        ct = Thread(target=handle, args=(e,clientsocket,))
        ct.start()
        e.set()


##################################################################

# Get date/time and convert to string
def get_current_time():
    now = datetime.now()
    dt_string = now.strftime("[%d-%m-%Y %H:%M:%S] ")
    return now,dt_string

##################################################################

# Print Debug data
def print_debug(message):
    if(isDebug):
        now,dt_string = get_current_time()
        print(dt_string + message)
    return


##################################################################

# Load sensors calibrations JSON file 
def load_calibrations_file():
    try:
        with open(CALIBRATION_FILE) as f:
            d = json.load(f)
        if(d):
            print_debug("Using calibrations: " + str(d))
    except Exception as e:
        print_debug("Cannot open calibrations file " + str(CALIBRATION_FILE) + ": " + str(e))
        sys.exit(1)
    return d


##################################################################

# Connect to InfluxDB
def connect_to_db():
    dbClient = InfluxDBClient(INFLUX_DB_SERVER, org=CONFIG["influx_db_org"])
    write_api = dbClient.write_api(write_options=SYNCHRONOUS)
    print_debug("Connected to InfluxDB at " + str(INFLUX_DB_SERVER))
    return dbClient,write_api


##################################################################

# print errors to DB, consider the snooze count 
def print_error_to_db(function_type,message):
    if(function_type not in CURRENT_SHUT_UP_COUNT):
        CURRENT_SHUT_UP_COUNT[function_type] = 10
    if(CURRENT_SHUT_UP_COUNT[function_type] >= SHUT_UP_INTERVAL):
        write_to_db(INFLUX_DB_EVENTS,json.loads('{"ok" : "0", "error" : "[' + str(function_type) + '] ' + str(message) + '"}'),CONFIG["influx_db_events"])
        CURRENT_SHUT_UP_COUNT[function_type] = 0
    else:
        CURRENT_SHUT_UP_COUNT[function_type] = CURRENT_SHUT_UP_COUNT + 1
    return

##################################################################

# Read CPU temperatureit as a float in degrees celcius
def get_temp():                              
    output = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True)
    temp_str = output.stdout.decode()
    try:
        return float(temp_str.split('=')[1].split('\'')[0])
    except:
        print_error_to_db("get_temp","Cannot read CPU temp")
        return float(0)
        pass


##################################################################

# Process 3 to control the RPi fan
def fan_control(e):
    IO.setwarnings(False)
    IO.setmode (IO.BCM)
    IO.setup(FAN_PIN,IO.OUT)
    fan = IO.PWM(FAN_PIN,CONFIG["fan_pwm"])
    fan.start(100)
    currentDuty = 100
    while(True):
        temp = get_temp()
        write_to_db(INFLUX_DB_SENSORS,json.loads('{"ok" : "1", "cpu_temp" : "' + str(temp) + '", "current_fan_speed" : "' + str(currentDuty) + '"}'),CONFIG["influx_db_sensors"])
        if temp > CONFIG["high_cpu_temp"]["temp"]:
            fan.ChangeDutyCycle(CONFIG["high_cpu_temp"]["fan_speed"])
            currentDuty = CONFIG["high_cpu_temp"]["fan_speed"]
        elif temp > CONFIG["mid_high_cpu_temp"]["fan_speed"]:
            fan.ChangeDutyCycle(CONFIG["mid_high_cpu_temp"]["fan_speed"])
            currentDuty = CONFIG["mid_high_cpu_temp"]["fan_speed"]
        elif temp > CONFIG["mid_low_cpu_temp"]["fan_speed"]:
            fan.ChangeDutyCycle(CONFIG["mid_low_cpu_temp"]["fan_speed"])
            currentDuty = CONFIG["mid_low_cpu_temp"]["fan_speed"]
        else:
            fan.ChangeDutyCycle(CONFIG["low_cpu_temp"]["fan_speed"])
            currentDuty = CONFIG["low_cpu_temp"]["fan_speed"]
        time.sleep(10) 


##################################################################


# CTRL+C Handler
def signal_handler(sig, frame):
    close()


##################################################################

# Close socket and exit
def close():
    print("Closing...")
    process1.terminate()
    process2.terminate()
    process3.terminate()
    sys.exit(0)

##################################################################
# Main 
# Connect to Arduino and start processes
signal.signal(signal.SIGINT, signal_handler)
e = Event()

# Check if debug mode
isDebug = False
isConnected = False
try:
    if(sys.argv[1] == "debug"):
        isDebug = True
        print_debug("Debug mode")
except:
    pass
try: 
    if(CONFIG["debug"] == True):
        isDebug = True
        print_debug("Debug mode")
except:
    pass

    
calibrations = load_calibrations_file()
dbClient,write_api = connect_to_db()

with serial.Serial(TTY, BAUD, timeout=1) as arduino:
    time.sleep(0.1)
    if arduino.isOpen():
        isConnected = True
        print_debug("Connected to Arduino")
        if(wait_and_print()):
            sys.exit(1)
        process1 = Process(target=periodic_read, args=(e,))
        process1.start()

        process2 = Process(target=one_time_read, args=(e,))
        process2.start()

        process3 = Process(target=fan_control, args=(e,))
        process3.start()

        e.set()
        process1.join()
        process2.join()
        process3.join()


if(not isConnected):
    print_debug("Error connecting to Arduino")
    sys.exit(1)
