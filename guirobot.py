import RPi.GPIO as GPIO
import servgear
import gyro
from guizero import App, Text, TextBox, PushButton, Slider, Box, CheckBox
import math
from time import sleep
from tkinter import *
import tkinter as tk
from tkinter import filedialog
from tkinter.messagebox import showinfo
import cv2
from PIL import Image, ImageTk
import os
import numpy as np

gyro.MPU_Init(0x68)
gyro.MPU_Init(0x69)

class Root(Tk):
    def __init__(self):
        super(Root,self).__init__()
        self.title("Multi-Body Autonomous Inspection Robot for Small Diameter Pipes")
        self.minsize(1200,600)
        #Creation of Subtitle
        self.subtitle = tk.Label(self, text = "GUI for Robot",font = "Verdana 20 bold")
        self.subtitle.grid(columnspan=3, row = 1)
        #Frame creation for Back Module
        self.backmodule = tk.LabelFrame(self,text = "Back Module")
        self.backmodule.grid(column = 1, row = 2, padx = 20, pady = 20)
        #Frame creation for Front Module
        self.frontmodule = tk.LabelFrame(self,text = "Front Module")
        self.frontmodule.grid(column = 2, row = 2, padx = 20, pady = 20)
        #Variables for sliders
        self.sbc = DoubleVar()
        self.sbb = DoubleVar()
        self.sba = DoubleVar()
        self.sbd = DoubleVar()
        self.mbc = DoubleVar()
        self.mbb = DoubleVar()
        self.mba = DoubleVar()
        self.mbd = DoubleVar()
        self.sfc = DoubleVar()
        self.sfb = DoubleVar()
        self.sfa = DoubleVar()
        self.sfd = DoubleVar()
        self.mfc = DoubleVar()
        self.mfb = DoubleVar()
        self.mfa = DoubleVar()
        self.mfd = DoubleVar()
        #Labels and Check Creations
        self.createLabels()
        self.dirbc = BooleanVar()
        self.dirbb = BooleanVar()
        self.dirba = BooleanVar()
        self.dirbd = BooleanVar()
        self.dirfc = BooleanVar()
        self.dirfb = BooleanVar()
        self.dirfa = BooleanVar()
        self.dirfd = BooleanVar()
        self.default = BooleanVar()
        self.dir = BooleanVar()
        self.createChecks()
        self.createSliders()
        self.createSensors()
        #Functions
        self.functions = tk.LabelFrame(self,text = "Functions")
        self.functions.grid(column = 1, row = 3, padx = 20, pady = 20)
        self.functionsbtn()
        #Trajectory upload
        self.labelbrowse = tk.LabelFrame(self,text = "Open a File")
        self.labelbrowse.grid(column = 2, row = 3, padx = 20, pady = 20)
        self.trajectorybtn()
        #Initial Setup
        self.initsetup = tk.LabelFrame(self,text = "Initial Setup")
        self.initsetup.grid(columnspan=3, row = 4, padx = 20, pady = 20)
        self.initangle()

    def createSensors(self):
        #IMU
        self.sensorb = tk.Label(self.backmodule, text="XX")
        self.sensorb.grid(columnspan = 3, row = 1)
        self.sensorb.after(1000, self.updateb)
        self.sensorf = tk.Label(self.frontmodule, text="XX")
        self.sensorf.grid(columnspan = 3, row = 1)
        self.sensorf.after(1000, self.updatef)
        #Encoders back module
        self.encoderbc = tk.Label(self.backmodule, text="0")
        self.encoderbc.grid(column = 4, row = 1)
        self.encoderbb = tk.Label(self.backmodule, text="0")
        self.encoderbb.grid(column = 2, row = 5)
        self.encoderba = tk.Label(self.backmodule, text="0")
        self.encoderba.grid(column = 4, row = 9)
        self.encoderbd = tk.Label(self.backmodule, text="0")
        self.encoderbd.grid(column = 6, row = 5)
        #Encoders front module
        self.encoderbc = tk.Label(self.frontmodule, text="0")
        self.encoderbc.grid(column = 4, row = 1)
        self.encoderbb = tk.Label(self.frontmodule, text="0")
        self.encoderbb.grid(column = 2, row = 5)
        self.encoderba = tk.Label(self.frontmodule, text="0")
        self.encoderba.grid(column = 4, row = 9)
        self.encoderbd = tk.Label(self.frontmodule, text="0")
        self.encoderbd.grid(column = 6, row = 5)

    def createSliders(self):
        #Movement for the back module
        sbc = Scale(self.backmodule, from_=0,to=190, orient=HORIZONTAL,command=self.anglebc, var=self.sbc, label="Servomotor").grid(column = 4, row = 3)
        sbb = Scale(self.backmodule, from_=0,to=190, orient=HORIZONTAL,command=self.anglebb, var=self.sbb, label="Servomotor").grid(column = 2, row = 7)
        sba = Scale(self.backmodule, from_=0,to=190, orient=HORIZONTAL,command=self.angleba, var=self.sba, label="Servomotor").grid(column = 4, row = 11)
        sbd = Scale(self.backmodule, from_=0,to=190, orient=HORIZONTAL,command=self.anglebd, var=self.sbd, label="Servomotor").grid(column = 6, row = 7)
        mbc = Scale(self.backmodule, from_=0,to=100, orient=HORIZONTAL,command=self.movebc , var=self.mbc, label="Motor").grid(column = 3, row = 3)
        mbb = Scale(self.backmodule, from_=0,to=100, orient=HORIZONTAL,command=self.movebb , var=self.mbb, label="Motor").grid(column = 1, row = 7)
        mba = Scale(self.backmodule, from_=0,to=100, orient=HORIZONTAL,command=self.moveba , var=self.mba, label="Motor").grid(column = 3, row = 11)
        mbd = Scale(self.backmodule, from_=0,to=100, orient=HORIZONTAL,command=self.movebd , var=self.mbd, label="Motor").grid(column = 5, row = 7)
        #Movement for the front module
        sfc = Scale(self.frontmodule, from_=0,to=190, orient=HORIZONTAL,command=self.anglefc, var=self.sfc, label="Servomotor").grid(column = 4, row = 3)
        sfb = Scale(self.frontmodule, from_=0,to=190, orient=HORIZONTAL,command=self.anglefb, var=self.sfb, label="Servomotor").grid(column = 2, row = 7)
        sfa = Scale(self.frontmodule, from_=0,to=190, orient=HORIZONTAL,command=self.anglefa, var=self.sfa, label="Servomotor").grid(column = 4, row = 11)
        sfd = Scale(self.frontmodule, from_=0,to=190, orient=HORIZONTAL,command=self.anglefd, var=self.sfd, label="Servomotor").grid(column = 6, row = 7)
        mfc = Scale(self.frontmodule, from_=0,to=100, orient=HORIZONTAL,command=self.movefc , var=self.mfc, label="Motor").grid(column = 3, row = 3)
        mfb = Scale(self.frontmodule, from_=0,to=100, orient=HORIZONTAL,command=self.movefb , var=self.mfb, label="Motor").grid(column = 1, row = 7)
        mfa = Scale(self.frontmodule, from_=0,to=100, orient=HORIZONTAL,command=self.movefa , var=self.mfa, label="Motor").grid(column = 3, row = 11)
        mfd = Scale(self.frontmodule, from_=0,to=100, orient=HORIZONTAL,command=self.movefd , var=self.mfd, label="Motor").grid(column = 5, row = 7)

    def createLabels(self):
        #Labels for Back Module
        tk.Label(self.backmodule, text="Actuators in Arm C").grid(column=3, row = 1)
        tk.Label(self.backmodule, text="Actuators in Arm B").grid(column=1, row = 5)
        tk.Label(self.backmodule, text="Actuators in Arm A").grid(column=3, row = 9)
        tk.Label(self.backmodule, text="Actuators in Arm D").grid(column=5, row = 5)
        #Labels for Front Module
        tk.Label(self.frontmodule, text="Actuators in Arm C").grid(column=3, row = 1)
        tk.Label(self.frontmodule, text="Actuators in Arm B").grid(column=1, row = 5)
        tk.Label(self.frontmodule, text="Actuators in Arm A").grid(column=3, row = 9)
        tk.Label(self.frontmodule, text="Actuators in Arm D").grid(column=5, row = 5)

    def createChecks(self):
        #Check button for Back Module
        dirbc = Checkbutton(self.backmodule, text = "Direction", state = "normal", var=self.dirbc).grid(column = 3, row = 4)
        dirbb = Checkbutton(self.backmodule, text = "Direction", state = "normal", var=self.dirbb).grid(column = 1, row = 8)
        dirba = Checkbutton(self.backmodule, text = "Direction", state = "normal", var=self.dirba).grid(column = 3, row = 12)
        dirbd = Checkbutton(self.backmodule, text = "Direction", state = "normal", var=self.dirbd).grid(column = 5, row = 8)
        #Check button for Back Module
        dirfc = Checkbutton(self.frontmodule, text = "Direction", state = "normal", var=self.dirfc).grid(column = 3, row = 4)
        dirfb = Checkbutton(self.frontmodule, text = "Direction", state = "normal", var=self.dirfb).grid(column = 1, row = 8)
        dirfa = Checkbutton(self.frontmodule, text = "Direction", state = "normal", var=self.dirfa).grid(column = 3, row = 12)
        dirfd = Checkbutton(self.frontmodule, text = "Direction", state = "normal", var=self.dirfd).grid(column = 5, row = 8)

    def functionsbtn(self):
        self.bstop = tk.Button(self.functions, text = "STOP",command = self.stop)
        self.bstop.grid(column = 1, row = 1)
        self.breset = tk.Button(self.functions, text = "RESET", command = self.reset)
        self.breset.grid(column = 2, row = 1)
        self.brou1 = tk.Button(self.functions, text = "Routine 1")
        self.brou1.grid(column = 3, row = 1)
        self.sliders = Scale(self.functions, from_=0,to=190, orient=HORIZONTAL,command=self.angle, label="Servomotors").grid(column = 1, row = 2)
        self.sliderm = Scale(self.functions, from_=0,to=100, orient=HORIZONTAL,command=self.move , label="Motors").grid(column = 2, row = 2)
        dir = Checkbutton(self.functions, text = "Direction", state = "normal", var=self.dir).grid(column = 3, row = 2)
#comments
    def initangle(self):
        self.button = tk.Button(self.initsetup,text = "Calculate",command = self.calculate)
        self.button.grid(column = 2, row = 5)
        self.iangle = DoubleVar()
        self.rho = DoubleVar()
        self.arml = DoubleVar()
        self.w = DoubleVar()
        default = Checkbutton(self.initsetup, text = "Default Values", state = "normal", var=self.default, command=self.defvalues).grid(column = 2, row = 1)
        tk.Label(self.initsetup, text="Initial Angle (Deg):").grid(column=1, row=2)
        iangle = tk.Entry(self.initsetup,textvariable=self.iangle).grid(column = 2, row =2)
        tk.Label(self.initsetup, text="Wheel radius (cm):").grid(column=1,row=3)
        rho = tk.Entry(self.initsetup,textvariable=self.rho).grid(column = 1, row =4)
        tk.Label(self.initsetup, text="Arm Length (cm):").grid(column=2,row=3)
        arml = tk.Entry(self.initsetup,textvariable=self.arml).grid(column = 2, row =4)
        tk.Label(self.initsetup, text="Distance between joints (cm):").grid(column=3,row=3)
        w = tk.Entry(self.initsetup,textvariable=self.w).grid(column = 3, row =4)

    def defvalues(self):
        if self.default.get() == 1:
            self.rho.set(1.25)
            self.arml.set(11.6)
            self.w.set(5.1)
        else:
            self.rho.set(0)
            self.arml.set(0)
            self.w.set(0)

    def calculate(self):
        diameter = (self.rho.get()*2)+self.w.get()+(self.arml.get()*2*math.sin(math.radians(self.iangle.get())))
        showinfo("Setup",'Set the Robot with a distance of '+'{:.2f}'.format(diameter)+' cm')

    def trajectorybtn(self):
        self.bbrowse = tk.Button(self.labelbrowse, text = "Browse a File",command = self.fileDialog)
        self.bbrowse.grid(column = 1, row = 1)
        self.bplay = tk.Button(self.labelbrowse,text = "Play Trajectory",command = self.playTrajectory)
        self.bplay.grid(column = 2, row = 1)
        self.bstop = tk.Button(self.labelbrowse,text = "Stop Inspection",command = self.stopTrajectory)
        self.bstop.grid(column = 3, row = 1)


    def fileDialog(self):
        global animate
        self.filename = filedialog.askopenfilename(initialdir = "/Home",title = "Select file",filetypes = (("text","*.txt"),("all files","*.*"))) 
        self.label = tk.Label(self.labelbrowse, text = "")
        animate = False
        self.line = open(self.filename)
        self.label.grid(column = 1, row = 2)
        self.label.configure(text = self.filename)

    def stopTrajectory(self):
        global animate
        self.stop()
        self.reset()
        self.after_cancel(animate)
        animate = False

    def playTrajectory(self):
        global animate
        servos = []
        motors = []
        try: # Read and print the entire file line by line
            temp = self.line.readline()
            if 'S' in temp:
                for word in temp.split():
                    try:
                        servos.append(float(word))
                    except ValueError:
                        pass
                print(servos)
                """
                sfac = (servos[0]+servos[1])/2
                sbac = (servos[2]+servos[3])/2
                self.anglefa(int(sfac))
                self.anglefc(int(sfac))
                self.anglefd(int(servos[0]))
                self.anglefb(int(servos[1]))
                self.angleba(int(sbac))
                self.anglebc(int(sbac))
                self.anglebd(int(servos[2]))
                self.anglebb(int(servos[3]))
                """
            elif 'G' in temp:
                for word in temp.split():
                    try:
                        motors.append(float(word))
                    except ValueError:
                        pass
                print(motors)
                """
                mfac = (motors[0]+motors[1])/2
                mbac = (motors[2]+motors[3])/2
                self.movefa(int(mfac))
                self.movefc(int(mfac))
                self.movefd(int(motors[0]))
                self.movefb(int(motors[1]))
                self.moveba(int(mbac))
                self.movebc(int(mbac))
                self.movebd(int(motors[2]))
                self.movebb(int(motors[3]))
                """
                sleep(0.25)
            animate= self.after(1,self.playTrajectory)
        except ValueError:
            self.line.close()

    def angleba(self,value):
    	servgear.servo(3,int(value))
    def anglebb(self,value):
        servgear.servo(2,int(value))
    def anglebc(self,value):
        servgear.servo(1,int(value))
    def anglebd(self,value):
        servgear.servo(0,int(value))

    def anglefa(self,value):
        servgear.servo(12,int(value))
    def anglefb(self,value):
        servgear.servo(13,int(value))
    def anglefc(self,value):
        servgear.servo(14,int(value))
    def anglefd(self,value):
        servgear.servo(15,int(value))

    def moveba(self,value):
        servgear.motor(self.dirba.get(),7,int(value))
    def movebb(self,value):
        servgear.motor(self.dirbb.get(),6,int(value))
    def movebc(self,value):
        servgear.motor(self.dirbc.get(),5,int(value))
    def movebd(self,value):
        servgear.motor(self.dirbd.get(),4,int(value))

    def movefa(self,value):
        servgear.motor(self.dirfa.get(),8,int(value))
    def movefb(self,value):
        servgear.motor(self.dirfb.get(),9,int(value))
    def movefc(self,value):
        servgear.motor(self.dirfc.get(),10,int(value))
    def movefd(self,value):
        servgear.motor(self.dirfd.get(),11,int(value))

    def calculate_ret(self,addres):
        return gyro.calculate(addres)

    def updateb(self):
        a = self.calculate_ret(0x68)
        self.sensorb.configure(text= str(round(a.posx,2))+"           "+str(round(a.posy,2)))
        self.sensorb.after(1000,self.updateb)

    def updatef(self):
        a = self.calculate_ret(0x69)
        self.sensorf.configure(text= str(round(a.posx,2))+"           "+str(round(a.posy,2)))
        self.sensorf.after(1000,self.updatef)

    def move(self,value):
        for i in range(4,8):
            servgear.motor(self.dir.get(),i,int(value))
            servgear.motor(self.dir.get(),i+4,int(value))

    def angle(self,value):
        for i in range(0,4):
            servgear.servo(i,int(value))
            servgear.servo(i+12,int(value))

    def stop(self):
        self.mbc.set(0)
        self.mbb.set(0)
        self.mba.set(0)
        self.mbd.set(0)
        self.mfc.set(0)
        self.mfb.set(0)
        self.mfa.set(0)
        self.mfd.set(0)
        for i in range(4,8):
            servgear.motor(0,i,0)
            servgear.motor(0,i+4,0)

    def reset(self):
        self.sbc.set(0)
        self.sbb.set(0)
        self.sba.set(0)
        self.sbd.set(0)
        self.sfc.set(0)
        self.sfb.set(0)
        self.sfa.set(0)
        self.sfd.set(0)
        for i in range(0,4):
            servgear.servo(i,0)
            servgear.servo(i+12,0)

    def routine1(self):
        sfa.tk.set(170)
        sfc.tk.set(170)
        sba.tk.set(170)
        sbc.tk.set(170)
        sfb.tk.set(150)
        sfd.tk.set(150)
        sbb.tk.set(150)
        sbd.tk.set(150)

class Timer:
    def __init__(self, parent):
        # variable storing time
        self.seconds = 0
        # label displaying time
        self.label = tk.Label(parent, text="0 s", font="Arial 30", width=10)
        # start the timer
        self.label.after(1000, self.refresh_label)

    def refresh_label(self):
        """ refresh the content of the label every second """
        # increment the time
        self.seconds += 1
        # display the new time
        self.label.configure(text="%i s" % self.seconds)
        # request tkinter to call self.refresh after 1s (the delay is given in ms)
        self.label.after(1000, self.refresh_label)


if __name__ == '__main__':
    root = Root()
#    timer = Timer(root)
    root.mainloop()
