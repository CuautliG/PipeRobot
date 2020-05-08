import RPi.GPIO as GPIO
import servgear
import gyro
from guizero import App, Text, TextBox, PushButton, Slider, Box, CheckBox
import math
from time import sleep
import tkinter as tk
import cv2
from PIL import Image, ImageTk
import os
import numpy as np

gyro.MPU_Init(0x68)
gyro.MPU_Init(0x69)

global last_frame                                      #creating global variable
last_frame = np.zeros((480, 640, 3), dtype=np.uint8)
global cap
cap = cv2.VideoCapture(0)

def show_vid():                                        #creating a function
    if not cap.isOpened():                             #checks for the opening of camera
        print("cant open the camera")
    flag, frame = cap.read()
    frame = cv2.flip(frame, 1)
    if flag is None:
        print ("error")
    elif flag:
        global last_frame
        last_frame = frame.copy()

    pic = cv2.cvtColor(last_frame, cv2.COLOR_BGR2RGB)     #we can change the display color of the frame gray,black&white here
    img = Image.fromarray(pic)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(10, show_vid)

def calculate_ret(addres):
    return gyro.calculate(addres)

def updatef():
    a = calculate_ret(0x68)
    pos.value = str(round(a.posx,2))+"           "+str(round(a.posy,2))
    pos.after(1000,updatef)

def updates():
    a = calculate_ret(0x69)
    pos1.value = str(round(a.posx,2))+"           "+str(round(a.posy,2))
    pos1.after(1000,updates)

def angleba(value):
    servgear.servo(3,int(value))
def anglebb(value):
    servgear.servo(2,int(value))
def anglebc(value):
    servgear.servo(1,int(value))
def anglebd(value):
    servgear.servo(0,int(value))

def anglefa(value):
    servgear.servo(12,int(value))
def anglefb(value):
    servgear.servo(13,int(value))
def anglefc(value):
    servgear.servo(14,int(value))
def anglefd(value):
    servgear.servo(15,int(value))

def moveba(value):
    servgear.motor(dirba.value,7,int(value))
def movebb(value):
    servgear.motor(dirbb.value,6,int(value))
def movebc(value):
    servgear.motor(dirbc.value,5,int(value))
def movebd(value):
    servgear.motor(dirbd.value,4,int(value))

def movefa(value):
    servgear.motor(dirfa.value,8,int(value))
def movefb(value):
    servgear.motor(dirfb.value,9,int(value))
def movefc(value):
    servgear.motor(dirfc.value,10,int(value))
def movefd(value):
    servgear.motor(dirfd.value,11,int(value))

def moveall(value):
    servgear.motor(dirall.value,4,int(value))
    servgear.motor(dirall.value,5,int(value))
    servgear.motor(dirall.value,6,int(value))
    servgear.motor(dirall.value,7,int(value))
    servgear.motor(dirall.value,8,int(value))
    servgear.motor(dirall.value,9,int(value))
    servgear.motor(dirall.value,10,int(value))
    servgear.motor(dirall.value,11,int(value))

def movealls(value):
    servgear.servo(0,int(value))
    servgear.servo(1,int(value))
    servgear.servo(2,int(value))
    servgear.servo(3,int(value))
    servgear.servo(12,int(value))
    servgear.servo(13,int(value))
    servgear.servo(14,int(value))
    servgear.servo(15,int(value))

def stop():
    for i in range(4,7):
        servgear.motor(0,i,0)
        servgear.motor(0,i+4,0)

def reset():
    mba.tk.set(0)
    mbb.tk.set(0)
    mbc.tk.set(0)
    mbd.tk.set(0)
    mfa.tk.set(0)
    mfb.tk.set(0)
    mfc.tk.set(0)
    mfd.tk.set(0)
    sba.tk.set(0)
    sbb.tk.set(0)
    sbc.tk.set(0)
    sbd.tk.set(0)
    sfa.tk.set(0)
    sfb.tk.set(0)
    sfc.tk.set(0)
    sfd.tk.set(0)

def routine1():
    sfa.tk.set(170)
    sfc.tk.set(170)
    sba.tk.set(170)
    sbc.tk.set(170)
    sfb.tk.set(150)
    sfd.tk.set(150)
    sbb.tk.set(150)
    sbd.tk.set(150)

app = App(title="Multi-Body Autonomous Inspection Robot for Small Diamter Pipes", layout="grid", height=760, width=760)

title_box = Box(app, width="fill", border=True,grid=[0,1,2,1])
title = Text(title_box, text="GUI for Robot",size="30")

Bmodule_box = Box(app, width="fill", height="fill", layout="grid", border=True,grid=[0,2])
title_module1 = Text(Bmodule_box, text="Back Module",align="right", grid=[0,1,2,1])
sbc = Slider(Bmodule_box, command=anglebc, start=0, end=190, grid=[1,2])
sbb = Slider(Bmodule_box, command=anglebb, start=0, end=190, grid=[0,5])
sba = Slider(Bmodule_box, command=angleba, start=0, end=190, grid=[1,8])
sbd = Slider(Bmodule_box, command=anglebd, start=0, end=190, grid=[2,5])

mbc = Slider(Bmodule_box, command=movebc, start=0, end=100, grid=[1,3])
mbb = Slider(Bmodule_box, command=movebb, start=0, end=100, grid=[0,6])
mba = Slider(Bmodule_box, command=moveba, start=0, end=100, grid=[1,9])
mbd = Slider(Bmodule_box, command=movebd, start=0, end=100, grid=[2,6])

dirbc = CheckBox(Bmodule_box, text="Check for Forward",grid=[1,4])
dirbb = CheckBox(Bmodule_box, text="Check for Forward",grid=[0,7])
dirba = CheckBox(Bmodule_box, text="Check for Forward",grid=[1,10])
dirbd = CheckBox(Bmodule_box, text="Check for Forward",grid=[2,7])

Fmodule_box = Box(app, width="fill", height="fill", layout="grid", border=True,grid=[1,2])
title_module2 = Text(Fmodule_box, text="Front Module", align="right", grid=[0,1,2,1])
sfc = Slider(Fmodule_box, command=anglefc, start=0, end=190, grid=[1,2])
sfb = Slider(Fmodule_box, command=anglefb, start=0, end=190, grid=[0,5])
sfa = Slider(Fmodule_box, command=anglefa, start=0, end=190, grid=[1,8])
sfd = Slider(Fmodule_box, command=anglefd, start=0, end=190, grid=[2,5])

mfc = Slider(Fmodule_box, command=movefc, start=0, end=100, grid=[1,3])
mfb = Slider(Fmodule_box, command=movefb, start=0, end=100, grid=[0,6])
mfa = Slider(Fmodule_box, command=movefa, start=0, end=100, grid=[1,9])
mfd = Slider(Fmodule_box, command=movefd, start=0, end=100, grid=[2,6])

dirfc = CheckBox(Fmodule_box, text="Check for Forward",grid=[1,4])
dirfb = CheckBox(Fmodule_box, text="Check for Forward",grid=[0,7])
dirfa = CheckBox(Fmodule_box, text="Check for Forward",grid=[1,10])
dirfd = CheckBox(Fmodule_box, text="Check for Forward",grid=[2,7])

Fmodule_angles = Box(app, width="fill", height="fill", layout="grid", border=True,grid=[0,3])
text_gyro = Text(Fmodule_angles, text="Position Gyro Back", align="top", grid=[0,1,2,1])
text_posx = Text(Fmodule_angles, text="Position X   Position Y", align="top", grid=[0,2])
pos = Text(Fmodule_angles, text="XX", align="top", grid=[0,3])
pos.after(1000,updatef)

Smodule_angles = Box(app, width="fill", height="fill", layout="grid", border=True,grid=[1,3])
text_gyro = Text(Smodule_angles, text="Position Gyro Front", align="top", grid=[0,1,2,1])
text_posx = Text(Smodule_angles, text="Position X   Position Y", align="top", grid=[0,2])
pos1 = Text(Smodule_angles, text="XX", align="top", grid=[0,3])
pos1.after(1000,updates)

Buttons = Box(app, width="fill", height="fill", layout="grid",align="bottom", border=True,grid=[0,4,2,4])
text_buttons = Text(Buttons, text="Functions", align="top", grid=[0,1,2,1])
button = PushButton(Buttons, text="STOP", command=stop, grid=[0,2,])
button = PushButton(Buttons, text="RESET", command=reset, grid=[1,2])
button = PushButton(Buttons, text="ROUTINE 1", command=routine1, grid=[2,2])

mall = Slider(Buttons, command=moveall, start=0, end=100, grid=[0,3])
dirall = CheckBox(Buttons, text="Check for Forward",grid=[2,3])

sall = Slider(Buttons, command=movealls, start=0, end=190, grid=[0,4])

#show_vid()
app.display()
#cap.realease()
