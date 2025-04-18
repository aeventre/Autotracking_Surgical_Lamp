import cv2

cap = cv2.VideoCapture(32)
while True:
    ret, frame = cap.read()
    print(f"ret = {ret}")
    if not ret:
        break
    cv2.imshow("Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
