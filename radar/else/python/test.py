'''
Author: your name
Date: 2021-10-09 12:12:27
LastEditTime: 2021-10-27 22:11:38
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /yolov5/test.py
'''
import torch
import cv2 as cv
from torch._C import Size
import time
# Model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='runs/train/exp44/weights/best.pt')  # default
# cap = cv.VideoCapture('/home/joyce/视频/川大vs广工.MOV')
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 800)
font = cv.FONT_HERSHEY_SIMPLEX
while True:
  start = time.time()
  ret,img = cap.read()
  # img = cv.resize(img, (640, 400))
  img1 = img
  results = model(img, size=640)
  _i = 0
  tl = [ ]
  br = [ ]
  flag = 0
  confidence = []
  for i in results.xyxy[0]:
      j = 0
      tl.append( (int(results.xyxy[0][_i][j]), int(results.xyxy[0][_i][j+1])) ) 
      br.append( (int(results.xyxy[0][_i][j+2]), int(results.xyxy[0][_i][j+3])) ) 
      confidence.append(float(results.xyxy[0][_i][j+4]))
      flag = int(results.xyxy[0][_i][j+5])
      if float(results.xyxy[0][_i][4]) > 0:
        if flag == 4 or flag == 0:
          img = cv.rectangle(img, tl[_i], br[_i],(0, 0, 255), 3, 8)
          img1 = cv.putText(img, str(round(float(results.xyxy[0][_i][4]), 2)), tl[_i], font, 1.2, (255, 255, 255), 2)
      _i += 1
  cv.imshow('face', img1)
  k = cv.waitKey(1)  
  if (k & 0xff == ord('q')):  
      break
  end = time.time()
  seconds = end - start
  fps = 1 / seconds
  print(fps)
