import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils
# mp_hands loads the MediaPipe hands module, which provides the hand detection and tracking functionality.
# hands = mp_hands.Hands() creates an instance of the hands solution, which processes video frames to detect and track hands.
# mp_drawing = mp.solutions.drawing_utils provides functions for drawing landmarks on the image

cap = cv2.VideoCapture(0)
# This opens the default webcam (index 0) using OpenCV. c

while cap.isOpened():
    ret, frame = cap.read() # cap.read() reads a frame from the webcam
    if not ret:
        break
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # BGR -> RGB
    results = hands.process(frame_rgb)
    
    # do you have hands
    if results.multi_hand_landmarks: 
        for hand_landmarks in results.multi_hand_landmarks:
            # Drawing landmarks on the image
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Get the coordinates of the wrist (landmark 0)
            wrist_landmark = hand_landmarks.landmark[0]
            frame_height, frame_width, _ = frame.shape
            wrist_x = int(wrist_landmark.x * frame_width)
            wrist_y = int(wrist_landmark.y * frame_height)
            
            # Calculate the offset of the wrist 
            offset_x = wrist_x - (frame_width // 2)
            offset_y = wrist_y - (frame_height // 2)
            
            # Send offset to ROS
            print(f"Offset from center: ({offset_x}, {offset_y})")

    cv2.imshow('Hand Tracking', frame)

    # press q to caccel program
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()