import cv2
import numpy as np

def nothing(temp): 
    return  

def track():
    rgbMax = 255      
    rgbMin = 0      
    
    sliderForPuck = "Slider for Puck"
    cv2.namedWindow(sliderForPuck)       
    
    cam_index = 0 # Default camera is at index 0.       
    cap = cv2.VideoCapture(cam_index) # Video capture object       
    
    sliders = ['R_MAX', 'G_MAX', 'B_MAX', 'R_MIN', 'G_MIN', 'B_MIN']       
    
    for slider in sliders:       
        cv2.createTrackbar(slider, sliderForPuck, rgbMin, rgbMax, nothing)  
    
    # Sliders for tracking Puck. Set default values after getting the puck!!
    #cv2.setTrackbarPos("R_MAX", sliderForPuck, 93) 
    #cv2.setTrackbarPos("R_MIN", sliderForPuck, 36) 
    #cv2.setTrackbarPos("G_MAX", sliderForPuck, 188) 
    #cv2.setTrackbarPos("G_MIN", sliderForPuck, 51) 
    #cv2.setTrackbarPos("B_MAX", sliderForPuck, 198) 
    #cv2.setTrackbarPos("B_MIN", sliderForPuck, 73)             

    cap.open(cam_index) # Enable the camera               
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            cap.release()
            break
        ret, frame = cap.read()      
    
        thresholds = []       
        for slider in sliders:       
            thresholds.append(cv2.getTrackbarPos(slider, sliderForPuck))       
    
        upperBound = np.array(thresholds[0:3])      
        lowerBound = np.array(thresholds[3:6])      
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)      
        threshold = cv2.inRange(hsv, lowerBound, upperBound)      
    
        res = cv2.bitwise_and(frame, frame, mask = threshold) 
        
        im2, contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    
        biggestRectangleArea = 0
        biggestRectangleCoordinates = (0,0,0,0)        
        for contour in contours: 
            x,y,w,h = cv2.boundingRect(contour) 
            if(w * h > biggestRectangleArea): 
                biggestRectangleCoordinates = (x, y, w, h)
                biggestRectangleArea = w*h
        x = biggestRectangleCoordinates[0]
        y = biggestRectangleCoordinates[1]
        w = biggestRectangleCoordinates[2]
        h = biggestRectangleCoordinates[3]  
        cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),2) 
        centerOfPuck = ((x+w)//2, (y+h)//2)
        print(centerOfPuck)
    
        #displays all windows
    #    cv2.imshow('Input', frame)  
        cv2.imshow("Puck", res)  

track()