

import numpy as np
import matplotlib as mp
import pygame
import cv2
from FeatureExtractor import *
import random

from Oxford_dataset.ReadCameraModel import *
import g2o
import pangolin

from skimage import data
from skimage.color import rgb2gray
from skimage.feature import match_descriptors, ORB, plot_matches
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform
from skimage.transform import EssentialMatrixTransform
import glob
from Display import *
from Oxford_dataset.UndistortImage import *

import time


reducedH=800
fx , fy , cx , cy , G_camera_image , LUT = ReadCameraModel ( "Oxford_dataset/model" )
print(fx , fy , cx , cy )
cy=cy*(980.0-reducedH)/980
#init pygame
W=1920//2
H=reducedH//2
F=282
K=np.array(([fx,0,cx//2],[0,fy,cy//2],[0,0,1]))

print (cv2.__version__ )


storeVideo=False
displayFeature=True
siftOrOrb=False








screen=pygame.display.set_mode((W,H))
print("Start")














 

fe=FeatureExtractor(displayFeature,siftOrOrb,W,H,K)


p1=None


def process_frame(frame):
	#print(frame.shape)
	# print("Processing")

	

	img=cv2.resize(frame,(W,H))
	
	ret=np.array(fe.extract(img))
	
	if len(ret)==0:
		return
		
	# p1 = multiprocessing.Process(target=display.paint, args=(img, ))
	# p1.start()
	# p1.join()
	display.paint(img)



display=Display(W,H)
cap=cv2.VideoCapture("vid3.avi")
frameCounter=0
prev_time=time.time()
avg_time=0
waiting=True

missFrames=20

while cap.isOpened():
	prev_time=time.time()

	ret,frame=cap.read()
	if ret:
		frame=frame[0:reducedH,:]
	
	frameCounter+=1
	if ret==True   and   frameCounter>missFrames:
		process_frame(frame)
		waiting=False
		
	else:
		#cv2.destroyAllWindows()
		#break
		print("Waiting")
		if ret==False:
			break
	if waiting==False:
		avg_time=(avg_time*(frameCounter-1-missFrames)+(time.time()-prev_time))/(frameCounter-missFrames)
		print("Avg Frame rate is ",1/avg_time," fps")
	



#importing frames fro vid2



if storeVideo:


	path="/home/pranav/SLAM/Oxford_dataset/stereo/centre/*"
	print(path)
	files =glob.glob(path)
	print(len(files))





	path="./Oxford_dataset/stereo/centre/*"
	print(path)
	files =glob.glob(path)
	print(len(files))

	files=np.sort(files)
	print(files[0:5])
	#print(files)

	v2 = []
	print("IN LIST")



	temp=cv2.imread(files[0])
	print(temp.shape)
	size=(temp.shape[1],temp.shape[0])	

	out = cv2.VideoWriter('vid3.avi',cv2.VideoWriter_fourcc(*'DIVX'), 25, size)


	for f in files:
		f=cv2.imread(f)
		f=UndistortImage(f,LUT)
		#cv2.imshow("f",f)
		#cv2.waitKey(0)

		out.write(f)

	    

	print(len(v2))



	out.release()


