import cv2

cap = cv2.VideoCapture(4)
cap.set(0, cv2.CAP_DSHOW)
cap.set(3, 2500)
cap.set(4, 1900)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FPS, 30)

count = 0

while True:
    ret, im = cap.read()
    cv2.imshow("frame", im)
    key = cv2.waitKey(1)

    if key == ord('r'):
        count += 1
        cv2.imwrite(str(count) + ".jpg", im)
