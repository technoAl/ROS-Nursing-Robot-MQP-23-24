import cv2
from apriltag import apriltag

cap = cv2.VideoCapture(0)
cap.set(3, 3840)
cap.set(4, 2160)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FPS, 30)
count = 0
while True:
    ret, im = cap.read()
    if ret:
        cv2.imshow("d", im)
        cv2.waitKey(1)
        # gray_image = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        # detector = apriltag(family="tag36h11")
        # detections = detector.detect(gray_image)  # , estimate_tag_pose=True, camera_params=PARAMS, tag_size=TAG_SIZE)
        # if len(detections) > 0:
        #     print("found")
        count+=1
        print(count)




