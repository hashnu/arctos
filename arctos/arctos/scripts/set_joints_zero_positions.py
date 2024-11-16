#!/usr/bin/env python3
from sshkeyboard import listen_keyboard, stop_listening
import time
import can

canId = "can0"
canbitrate = 500000
timeout = 1
gear_names = ["X joint","Y joint","Z joint","A joint","B joint","C_joint"]
gear_ratios = [13.5,150.,150.,48.,67.82,67.82]

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
bus = can.Bus(interface='socketcan', channel=canId, bitrate=canbitrate)

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

def sendcommandandforget(joint,command):
    msg = can.Message(arbitration_id=joint,data=bytearray(command) + bytes([get_CRC(joint,command)]),is_extended_id=False)
    bus.send(msg)

def  relative_motion_by_pulses(joint,pulses,speed,acceleration,direction):
    sendcommandandforget(joint,[0xFD,direction + ((speed >> 8) & 0b1111),speed & 0xFF,acceleration, (pulses >> 16) & 0xFF,(pulses >> 8) & 0xFF,(pulses >> 0) & 0xFF,])
    #return can_resp.data[1]

def absolute_motion_by_axis(absolute_axis,speed,acceleration):
    sendcommand(joint,[0xF5,((speed >> 8) & 0b1111),speed & 0xFF,acceleration,(absolute_axis >> 16) & 0xFF,(absolute_axis >> 8) & 0xFF,(absolute_axis >> 0) & 0xFF,])
    return can_resp.data[1]

try:
    for i in range(0,len(gear_names)): 
        #Setting of joints
        print("\n--------------------------")
        print(" Setting of %s"%gear_names[i])
        print("--------------------------")

        encoder_resolution_degree = 360/(0x4000)
        degre_per_pulses = 1.8/16/gear_ratios[i]
        step_in_degree = 1
        pulses = round(step_in_degree/degre_per_pulses) #round(1/degre_per_pulses)
        joint = i+1

        #Add keyboard listening
        print(" Please push Left (Counter Clockwise) or Right (Clockwise) key until zero is reached.\n Endstop may need to be manualy reset (plug / unplug or with a magnet).\n Then press Escape")

        def space_press(key):
            global zero_encoder_value
            if key == "left":
                relative_motion_by_pulses(joint,pulses*1,100 if i!=0 else 10,1,0)
            if key == "right":
                relative_motion_by_pulses(joint,pulses*1,100 if i!=0 else 10,1,0x80)
            if key == "esc":
                stop_listening()

        listen_keyboard(on_press=space_press,)

        print(" Please confirm zero position by pressing Escape")
        listen_keyboard(on_press=space_press,)

        print(" Setting zero position")
        sendcommand(joint,[0x92])

finally:
    bus.shutdown()
