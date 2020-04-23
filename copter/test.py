import cv2
from yolo import*

cap = cv2.VideoCapture(0)
my_yolo = YOLO()        
while(True):
 
  ret, frame = cap.read()
  image = Image.fromarray(frame)
  image = my_yolo.detect_image(image)
  print(my_yolo.yolo_result)
  result = np.asarray(image)    
  cv2.imshow("result", result)   
  if cv2.waitKey(1) & 0xFF == 27:
    break

# 釋放攝影機
cap.release()

# 關閉所有 OpenCV 視窗
cv2.destroyAllWindows()