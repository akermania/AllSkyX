#!/usr/bin/python

##################################################################
# Name:             pwn_fan.py
# Author:           Tal Akerman
# Version:          1.0 (Dec 2022)
# Description:      Test your PWM fan by selecting speeds 
#                   
#
##################################################################


################
#   Imports
################
import RPi.GPIO as IO 
import subprocess
import signal
import sys

# CTRL+C Handler
def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
IO.setwarnings(False)
# GPIO14
IO.setmode (IO.BCM)
IO.setup(14,IO.OUT) 
# Set GPIO14 as a PWM output, with 100Hz frequency 
# (this should match your fans specified PWM frequency)
fan = IO.PWM(14,100)
fan.start(100)


while True:
    command = input("Select Speed:")
    fan.ChangeDutyCycle(int(command))


sys.exit(0)