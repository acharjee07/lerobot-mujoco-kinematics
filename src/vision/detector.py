import cv2
import numpy as np
from src.config import camera_index

# --- HSV Color Ranges (RED) ---
# Red wraps around 0/180 in HSV, so we need two masks.
LOWER_RED1 = np.array([0, 120, 70])
UPPER_RED1 = np.array([10, 255, 255])

LOWER_RED2 = np.array([170, 120, 70])
UPPER_RED2 = np.array([180, 255, 255])

def get_red_object(frame, draw_debug=True):
    """
    Finds the largest red object in the frame.
    
    Returns:
        cx, cy (int): Centroid coordinates
        area (float): Size of the object
        found (bool): True if object detected
        frame (img):  The frame with debug drawings
    """
    if frame is None:
        return 0, 0, 0, False, None

    # 1. Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 2. Create Masks
    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 3. Clean Noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # 4. Find Contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cx, cy, area = 0, 0, 0
    found = False
    
    if len(contours) > 0:
        # Get largest blob
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        
        # Minimum area filter (avoids noise)
        if area > 500:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                found = True
                
                if draw_debug:
                    # Draw visual feedback
                    cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Pos: {cx},{cy}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return cx, cy, area, found, frame

if __name__ == "__main__":
    # --- CAMERA TEST MODE ---
    print("Starting Camera Test... Press 'q' to quit.")
    cap = cv2.VideoCapture(camera_index) # Use config camera index
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        _, _, _, _, debug_frame = get_red_object(frame)
        cv2.imshow("Red Object Detector", debug_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'): break
            
    cap.release()
    cv2.destroyAllWindows()