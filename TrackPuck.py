from Tkinter import *
import math
import cv2
import numpy as np

def init(data):
    # load data.xyz as appropriate
    pass

def mousePressed(event, data):
    # use event.x and event.y
    pass

def keyPressed(event, data):
    # use event.char and event.keysym
    pass

def timerFired(data):
    pass

def redrawAll(canvas, data):
    # draw in canvas
    pass

def run(width=500, height=550): # 15-112 Notes
    def nothing(temp): # OpenCV Starter Code
        return  # OpenCV Starter Code
    rgbMax = 255     # OpenCV Starter Code
    rgbMin = 0     # OpenCV Starter Code
    
    sliderForPuck = "Slider for Puck"
    cv2.namedWindow(sliderForPuck)      # OpenCV Starter Code
    
    cam_index = 0 # Default camera is at index 0.      # OpenCV Starter Code
    cap = cv2.VideoCapture(cam_index) # Video capture object      # OpenCV Starter Code

    sliders = ['R_MAX', 'G_MAX', 'B_MAX', 'R_MIN', 'G_MIN', 'B_MIN']      # OpenCV Starter Code
    
    for slider in sliders:      # OpenCV Starter Code
        cv2.createTrackbar(slider, sliderForPuck, rgbMin, rgbMax, nothing) # OpenCV Starter Code

    # Sliders for tracking Puck. Set default values after getting the puck!!
    #cv2.setTrackbarPos("R_MAX", sliderForPuck, 93) # OpenCV Docs
    #cv2.setTrackbarPos("R_MIN", sliderForPuck, 36) # OpenCV Docs
    #cv2.setTrackbarPos("G_MAX", sliderForPuck, 188) # OpenCV Docs
    #cv2.setTrackbarPos("G_MIN", sliderForPuck, 51) # OpenCV Docs
    #cv2.setTrackbarPos("B_MAX", sliderForPuck, 198) # OpenCV Docs
    #cv2.setTrackbarPos("B_MIN", sliderForPuck, 73) # OpenCV Docs
            
    cap.open(cam_index) # Enable the camera  # OpenCV Starter Code
              
    def openCVPart():
        ret, frame = cap.read()     # OpenCV Starter Code

        thresholds = []      # OpenCV Starter Code
        for slider in sliders:      # OpenCV Starter Code
            thresholds.append(cv2.getTrackbarPos(slider, sliderForPuck))      # OpenCV Starter Code

        upperBound = np.array(thresholds[0:3])     # OpenCV Starter Code
        lowerBound = np.array(thresholds[3:6])     # OpenCV Starter Code
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)     # OpenCV Starter Code
        threshold = cv2.inRange(hsv, lowerBound, upperBound)     # OpenCV Starter Code

        res = cv2.bitwise_and(frame, frame, mask = threshold) # OpenCV docs
        
        im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # OpenCV docs

        biggestRectangleArea = 0
        biggestRectangleCoordinates = (0,0,0,0)        
        for contour in contours: # OpenCV Docs
            x,y,w,h = cv2.boundingRect(contour) # OpenCV Docs
            if(w * h > biggestRectangleArea): 
                biggestRectangleCoordinates = (x, y, w, h)
                biggestRectangleArea = w*h
        x = biggestRectangleCoordinates[0]
        y = biggestRectangleCoordinates[1]
        w = biggestRectangleCoordinates[2]
        h = biggestRectangleCoordinates[3]  
        cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),2) #OpenCV Docs
        centerOfPuck = ((x+w)//2, (y+h)//2)
        print(centerOfPuck)
   
        #displays all windows
    #    cv2.imshow('Input', frame) # OpenCV Starter Code
        cv2.imshow("Puck", res) # OpenCV Starter Code
    
    def redrawAllWrapper(canvas, data): # 15-112 Notes
        openCVPart()
        canvas.delete(ALL) # 15-112 Notes
        redrawAll(canvas, data) # 15-112 Notes
        canvas.update()    # 15-112 Notes

    def mousePressedWrapper(event, canvas, data): # 15-112 Notes
        mousePressed(event, data) # 15-112 Notes
        redrawAllWrapper(canvas, data) # 15-112 Notes

    def keyPressedWrapper(event, canvas, data): # 15-112 Notes
        keyPressed(event, data) # 15-112 Notes
        redrawAllWrapper(canvas, data)# 15-112 Notes

    def timerFiredWrapper(canvas, data): # 15-112 Notes
        timerFired(data) # 15-112 Notes
        redrawAllWrapper(canvas, data) # 15-112 Notes
        # pause, then call timerFired again # 15-112 Notes
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data) # 15-112 Notes
    # Set up data and call init # 15-112 Notes
    class Struct(object): pass # 15-112 Notes
    data = Struct() # 15-112 Notes 
    data.width = width # 15-112 Notes
    data.height = height  # 15-112 Notes
    data.timerDelay = 100 # milliseconds # 15-112 Notes
    root = Tk() # 15-112 Notes
    init(data) # 15-112 Notes
    # create the root and the canvas # 15-112 Notes
    canvas = Canvas(root, width=data.width, height=data.height) # 15-112 Notes
    canvas.pack() # 15-112 Notes
    # set up events # 15-112 Notes
    root.bind("<Button-1>", lambda event: # 15-112 Notes
                            mousePressedWrapper(event, canvas, data)) # 15-112 Notes
    root.bind("<Key>", lambda event: # 15-112 Notes
                            keyPressedWrapper(event, canvas, data)) # 15-112 Notes
    timerFiredWrapper(canvas, data) # 15-112 Notes
    # and launch the app # 15-112 Notes
    root.mainloop()  # blocks until window is closed # 15-112 Notes
    cap.release() # OpenCV Starter Code
    cv2.destroyAllWindows() # OpenCV Starter Code
    print("bye!") # 15-112 Notes
    
run(500, 550) # 15-112 Notes