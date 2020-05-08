import cv2   #open source computer vision library
from tkinter import *  #import only what necessary
from PIL import Image, ImageTk
import RPi.GPIO as pin    #import gpio control library

pin.setwarnings(False)

msg =''  #message variable

#width, height = 800, 500  #setting widht and height

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 350)

root = Tk()
root.title("RPI Robot Control Panel by Ashraf Minhaj")

#main label for showing the feed
imagel = Label(root)
imagel.pack()

# font 
font = cv2.FONT_HERSHEY_SIMPLEX 
# org 
org = (30, 20) 
# fontScale 
fontScale = 0.7
# Blue color in BGR 
color = (255, 255, 255)  
# Line thickness of 2 px 
thickness = 2

notif = ''

#load background and button images

msg = ''

def msg_default():
    global msg
    msg = ''
    #return msg
  
def notification():
    global notif    #notification variable (global)
    

    return notif

def check_faces(f):  #check face upon given frame
    
    faces = face_cascade.detectMultiScale(f, scaleFactor = 1.5, minNeighbors = 5)
    #print(faces)
    for(x, y, w, h) in faces:

        print('Face found\n')
        #print(x, y, w, h)
        roi_f = f[y: y+h, x: x+w]  #region of interest is face
        
        #*** Drawing Rectangle ***
        color = (255, 0, 0)
        stroke = 2
        end_cord_x = x+w
        end_cord_y = y+h
        cv2.rectangle(f, (x,y), (end_cord_x, end_cord_y), color, stroke)
        
        return f
    

def get_frame():
    """get a frame from the cam and return it."""
    print("chhobi lagbo vai.")
    ret, frame = cap.read()
    
    return frame

def update():
    """update frames."""
    global msg
    print("dak porse vai")

    frame = get_frame()
    
    #if notification() != None:
    cv2.putText(frame, msg, org, font, fontScale, color, thickness, cv2.LINE_AA)
    
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    #manipulate image here (if needed)
    #x = notification()
    #cv2.putText(frame, 'yahoo', org, font, fontScale, color, thickness, cv2.LINE_AA)
    
    

    try:
        cv2.image = check_faces(frame)
    except:
        pass

    img = Image.fromarray(cv2image)
    
    imgtk = ImageTk.PhotoImage(image=img)

    imagel.imgtk = imgtk
    imagel.configure(image=imgtk)
        
    msg_default()
    
    
    imagel.after(500, update)

msg = ''
msg_default()

update()

root.resizable(0, 0)

root.mainloop()
pin.cleanup()

