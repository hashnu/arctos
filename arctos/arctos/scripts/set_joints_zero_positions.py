#!/usr/bin/env python3
from sshkeyboard import listen_keyboard, stop_listening
import time
import can

canId = "can0"
canbitrate = 500000
jointId = 0x02
jointStep = 1
jointDirection = 1 
timeout = 1
canCommands = {}
canSuccess = {}
canCommands["screen_auto_off_on"]=[0x87, 0x01]
canCommands["CW"]=[0x86, 0x00]
canCommands["CCW"]=[0x86, 0x01]
canCommands["release_lock"]=[0x3D, 0x3E]
canCommands["enable_motor"]=[0xF3, 0x01]
canSuccess["CCW"] = [0x86, 0x1]
canSuccess["enable_motor"] = [0xF3,0x01]

def get_CRC(id,data):
    return id + sum(data) & 0xFF

def check_CRC(id,data):
    sum = id 
    for x in data[:-1]:
        sum += x
    if data[-1] == sum & 0xFF:
        return True
    else:
        return False

#Can interface definition
bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)

can_resp = False
notifier = can.Notifier(bus,[])
def receive_message(message):
    global can_resp
    can_resp = message                                                          

def sendcommand(joint,command):
    global can_resp
    global notifier
    global receive_message
    can_resp = False
    notifier.add_listener(receive_message)
    msg = can.Message(arbitration_id=joint,data=bytearray(command) + bytes([get_CRC(joint,command)]),is_extended_id=False)
    bus.send(msg)
    ts = time.time()
    while not can_resp and time.time()-ts < timeout:
        time.sleep(0.001)   
    notifier.remove_listener(receive_message)
    if not can_resp or not check_CRC(joint,can_resp.data) or joint !=can_resp.arbitration_id:
        print("ERROR - No message received from Arctos arm")

def  relative_motion_by_pulses(joint,pulses,speed,acceleration,direction):
    sendcommand(joint,[0xFD,direction + ((speed >> 8) & 0b1111),speed & 0xFF,acceleration, (pulses >> 16) & 0xFF,(pulses >> 8) & 0xFF,(pulses >> 0) & 0xFF,])
    return can_resp.data[1]

def absolute_motion_by_axis(absolute_axis,speed,acceleration):
    sendcommand(joint,[0xF5,((speed >> 8) & 0b1111),speed & 0xFF,acceleration,(absolute_axis >> 16) & 0xFF,(absolute_axis >> 8) & 0xFF,(absolute_axis >> 0) & 0xFF,])
    return can_resp.data[1]

try:
    #Setting of joints
    print("\n--------------------------")
    print(" Setting of Y joint")
    print("--------------------------")

    encoder_resolution_degree = 360/(0x4000)
    degre_per_pulses = 1.8/16/150
    step_in_degree = 1
    pulses = round(step_in_degree/degre_per_pulses) #round(1/degre_per_pulses)
    joint = 2

    #Add keyboard listening
    print(" Please push Left (Counter Clockwise) or Right (Clockwise) key until zero is reached.\n Endstop may need to be manualy reset (plug / unplug or with a magnet).\n Then press Escape")

    def space_press(key):
        global zero_encoder_value
        if key == "left":
            relative_motion_by_pulses(joint,pulses,100,1,0)
        if key == "right":
            relative_motion_by_pulses(joint,pulses,100,1,0x80)
        if key == "esc":
            stop_listening()

    listen_keyboard(on_press=space_press,)

    print(" Please confirm zero pospython3ition by pressing Escape")
    listen_keyboard(on_press=space_press,)

    zero_encoder_value = int.from_bytes(can_resp.data[2:8],byteorder="big")

    print(" Setting zero position")
    sendcommand(joint,[0x92])

finally:
    bus.shutdown()
