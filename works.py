import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pygame
import cv2
from skimage import data
from skimage.color import rgb2gray
from skimage.feature import match_descriptors, ORB, plot_matches
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform
from skimage.transform import EssentialMatrixTransform
import torch
import random



class OdomPlotter(object):
	def __init__(self,axislimit):
		self.map = plt.figure()
		self.map_ax = Axes3D(self.map)
		self.map_ax.autoscale(enable=True, axis='both', tight=True)
		 
		# # # Setting the axes properties
		self.map_ax.set_xlim3d([-axislimit,axislimit])
		self.map_ax.set_ylim3d([-axislimit,axislimit])
		self.map_ax.set_zlim3d([-axislimit,axislimit])
		 
		self.hl, = self.map_ax.plot3D([0], [0], [0])


	def update_line(self, new_data):
		xdata, ydata, zdata = self.hl._verts3d
		self.hl.set_xdata(list(np.append(xdata, new_data[0])))
		self.hl.set_ydata(list(np.append(ydata, new_data[1])))
		self.hl.set_3d_properties(list(np.append(zdata, new_data[2])))
		plt.draw()
		plt.pause(0.0002)
		plt.show(block=False)

 

class Frame(object):
	kp=None
	des=None
	parent=None
	def __init__(img):
		kp,des=fe.get(img)













def add_ones(x):
		return np.hstack((x,np.ones((len(x),1)) ))

class FeatureExtractor(object):
	GX=16
	GY=16
	displayFeature=None
	siftOrOrb=None
	last=None
	f_est_avg=[]
	def __init__(self,displayFeature,siftOrOrb,W,H,K):
		self.orb = cv2.ORB_create(scoreType=cv2.ORB_FAST_SCORE)
		self.sift=cv2.xfeatures2d.SIFT_create()

		self.false_counter=0

		self.bf=cv2.BFMatcher()
		self.displayFeature=displayFeature
		self.siftOrOrb=siftOrOrb
		self.W=W
		self.H=H
		self.K=K
		self.Kinv=np.linalg.inv(self.K)
		
		if siftOrOrb==True:
			self.FLANN_INDEX_KDTREE = 1
			self.index_params = dict(algorithm = self.FLANN_INDEX_KDTREE, trees = 5)
			self.search_params = dict(checks=50)
		else:
			self.FLANN_INDEX_LSH=6
			self.index_params= dict(algorithm = self.FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
			self.search_params = dict(checks=100)


		self.flann = cv2.FlannBasedMatcher(self.index_params,self.search_params)

		self.lastRt = np.zeros((3,4))
		self.lastRt[0:3,0:3]=np.eye(3)
		self.RtI=np.zeros((3,4));self.RtI[0:3,0:3]=np.eye(3)

		self.odomPlotter=OdomPlotter(500)





		
 


		pass

	
	def normalize(self,pts):
		#print(pts.shape)
		norm=np.dot(self.Kinv,add_ones(pts).T).T
		#print(norm.shape,norm[0])
		#assert 1==0
		return norm[:,0:2]

	def denormalizept(self,pt):
		ret= np.dot(self.K,np.array([pt[0],pt[1],1.0]).T)
		
		#ret=np.float32(ret)
		#print(ret)
		#ret/=ret[2]
		#print(ret)
		return ((ret[0]),(ret[1]))
		#return int(round(pt[0]+self.W)),int(round(pt[1]+self.H))


	def denormalizepts(self,pts):
		ret= np.dot(self.K,add_ones(pts).T)
		
		#ret=np.float32(ret)
		#print(ret)
		#ret/=ret[2]
		#print(ret)
		return ret[:,0:2]
		#return int(round(pt[0]+self.W)),int(round(pt[1]+self.H))


	def drawKps(self,img,pts):
		print()
		for (u,v) in pts:

			cv2.circle(img,(int(u),int(v)),color=(0,0,255),radius=3)

	def drawMatches(self,img,pts1,pts2):
		for i in range(len(pts1)):
			x1,y1=(pts1[i])
			x2,y2=(pts2[i])

			r,g,b = np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255)
			r=0
			g=0
			b=255

			cv2.circle(img,(int(x1),int(y1)),color=(r,g,b),radius=3)
			cv2.circle(img,(int(x2),int(y2)),color=(r,g,b),radius=3)
			cv2.line(img, (int(x1),int(y1)), (int(x2),int(y2)), (r,g,b), 1)

	def EssentialMatrixFromFundamentalMatrix(self,F, K):

	    E = K.T@F@K
	    #print(E)
	    U, S, V_T = np.linalg.svd(E)

	    E = np.dot(U, np.dot(np.diag([1, 1, 0]), V_T))
	    return E


	def ExtractCameraPose(self,E):
	    W=np.array([[0,-1,0], [1,0,0] , [0,0,1] ])
	    U,D,Vt=np.linalg.svd(E)
	    C1=U[:,2]
	    R1=U@W@Vt
	    
	    C2=-U[:,2]
	    R2=U@W@Vt
	    
	    
	    C3=U[:,2]
	    R3=U@W.T@Vt
	    
	    C4=-U[:,2]
	    R4=U@W.T@Vt
	    
	    # if(np.linalg.det(R1)<-0):
	    #     C1=-C1
	    #     R1=-R1
	        
	    
	    # if(np.linalg.det(R2)<-0):
	    #     C2=-C2
	    #     R2=-R2
	        
	    
	    # if(np.linalg.det(R3)<-0):
	    #     C3=-C3
	    #     R3=-R3
	        
	    
	    # if(np.linalg.det(R4)<-0):
	    #     C4=-C4
	    #     R4=-R4
	        
	    return C1,R1,C2,R2,C3,R3,C4,R4



	



	def ExtractCameraPoseGeorge(self,E):
		W=np.array([[0,-1,0], [1,0,0] , [0,0,1] ])
		U,D,Vt=np.linalg.svd(E)
		if np.linalg.det(U)<0:
		 	print("#####      det U <0      #########")
		 	assert np.linalg.det(U)>0
		if np.linalg.det(Vt)<0:
			Vt=-1.0*Vt
		R=np.dot(np.dot(U,W),Vt)

		if np.sum(np.diagonal(R))<0:
			R=np.dot(np.dot(U,W.T),Vt)

		t=U[:,2]
		return t,R





	def LinearTriangulation(self,K, C1, R1, C2, R2, x1, x2):       ########## can be torch

		I = np.identity(3)
		#print(x1.shape)

		sz = x1.shape[0]
		C1 = np.reshape(C1, (3, 1))
		C2 = np.reshape(C2, (3, 1))
		P1 = np.dot(K, np.dot(R1, np.hstack((I, -C1))))
		P2 = np.dot(K, np.dot(R2, np.hstack((I, -C2))))
		#print(P1)
		#print(P2)
		#     print(P2.shape)
		X1 = np.hstack((x1, np.ones((sz, 1))))
		X2 = np.hstack((x2, np.ones((sz, 1))))


		X = np.zeros((sz, 3))

		for i in range(sz):
		    skew1 = self.skew(X1[i, :])
		    skew2 = self.skew(X2[i, :])
		    A = np.vstack((np.dot(skew1, P1), np.dot(skew2, P2)))
		    _, _, v = np.linalg.svd(A)
		    x = v[-1] / v[-1, -1]
		    x = np.reshape(x, (len(x), -1))
		    X[i, :] = x[0:3].T

		return X
    

        
	
	def skew(self,x):                               ################ can be torch

		return np.array([[0, -x[2], x[1]], [x[2], 0, x[0]], [x[1], x[0], 0]])

	def DisambiguateCameraPose(self,C1,R1,C2,R2,C3,R3,C4,R4,X1,X2,X3,X4):
	    #print()
	    C=[]
	    R=[]
	    X=[]
	    C.append(C1);C.append(C2);C.append(C3);C.append(C4);
	    R.append(R1);R.append(R2);R.append(R3);R.append(R4);
	    X.append(X1);X.append(X2);X.append(X3);X.append(X4);
	    
	    bestC=[]
	    bestR=[]
	    bestX=[]
	    max=-1
	    for i in range(len(C)):
	        counter=0
	        C_=C[i]
	        R_=R[i]
	        X_=X[i]
	        for x in X_:   #######Check how many pts in front
	            if R_[2]@(x-C_) >0   and    x[2]>=0:
	                counter+=1
	                
	            
	        if max<counter:
	            max=counter
	            bestR=R_
	            bestC=C_
	            bestX=X_
	            
	    
	    if max==0:
        	print("NONONO")

	    return bestC,bestR,bestX



	

	


	def extract(self,img):

		while True:


			kp=None
			des=None
			if self.siftOrOrb:
				kp,des= self.sift.detectAndCompute(img,None)

			else:
				# kp,des= self.orb.detectAndCompute(img,None)
				gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
				pts = cv2.goodFeaturesToTrack(gray, 19000, qualityLevel=0.1, minDistance=7)

				# extraction
				kp = [cv2.KeyPoint(x=f[0][0], y=f[0][1], _size=40) for f in pts]
				kp, des = self.orb.compute(img, kp)
				


			tempPoints=[]

				
			matches=None
			pts1,pts2=[0],[0]
			
			ret=[]
			if self.last is not None:
				if self.siftOrOrb:
					#matches = self.flann.knnMatch(des,self.last['des'],k=2)
					matches = self.flann.knnMatch(des,self.last['des'],k=2)
				else:
					matches = self.bf.knnMatch(des,self.last['des'],k=2)
					

					# Sort them in the order of their distance.
					#matches = sorted(matches, key = lambda x:x.distance)
			
				print(len(matches))
				good = []
				pts1 = []
				pts2 = []
				

				#print(len(matches))

				# ratio test as per Lowe's paper
				for i,(m,n) in enumerate(matches):
				    if m.distance < 0.7*n.distance:
				        kp1=kp[m.queryIdx].pt
				        kp2=self.last['kp'][m.trainIdx].pt
				   
				       
				        ret.append((kp1,kp2))  ###1 is curr


				
				
			else:
				self.last={'kp':kp,'des':des}
				return ret

			





			if len(ret)==0:
				print("No Return")
			else:
				try:

					ret=np.array(ret)
				
					
					#pts1=self.normalize(ret[:,0])
					#pts2=self.normalize(ret[:,1])
					pts1 = np.int32(ret[:,0,:])
					pts2 = np.int32(ret[:,1,:])


					pts1_n=self.normalize(pts1)
					pts2_n=self.normalize(pts2)
					


					model,inliers=ransac( (pts1_n,pts2_n),     ##########pts2,pts1
										#FundamentalMatrixTransform,
										EssentialMatrixTransform,
										min_samples=8,
										residual_threshold=0.0001,
										max_trials=50
										) 

									                                ###1 is curr i.e. is 0





					ret=ret[inliers]
					mask=1
					F=model.params

			
					
					# F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)
					# ###We select only inlier points
					# ret=ret[mask.ravel()==1]
					#print("aftr",ret.shape)				
					#print("Try")
					#print(F)


					if F is None   or mask is None    or   F.shape!=(3,3):

						print("Problem in F")
						print(F.shape)
						assert 1==0                                   ##############Error can be here


					#E=self.EssentialMatrixFromFundamentalMatrix(F,self.K)
					E=F
					self.drawMatches(img,ret[:,0],ret[:,1])
					
					Cbest,Rbest=None,None

		
					Cbest,Rbest=self.ExtractCameraPoseGeorge(E)          ################Error can be here

					Rt1=np.zeros((3,4));Rt1[0:3,0:3]=Rbest;Rt1[0:3,3]=Cbest
					X1=cv2.triangulatePoints(self.RtI,Rt1,ret[:,1].T,ret[:,0].T).T			
					X1=np.asarray([y[0:3]/y[3] for y in X1])
					self.false_counter=0
					

				except:
					

					self.false_counter+=1


					if self.false_counter<3:
						continue

					else:
						print("Changing kps and des")
						self.last={'kp':kp,'des':des}
						self.false_counter=0
						return ret

					


					C1,R1,C2,R2,C3,R3,C4,R4=self.ExtractCameraPose(E)

					Rt1=np.zeros((3,4));Rt1[0:3,0:3]=R1;Rt1[0:3,3]=C1
					Rt2=np.zeros((3,4));Rt2[0:3,0:3]=R2;Rt2[0:3,3]=C2
					Rt3=np.zeros((3,4));Rt3[0:3,0:3]=R3;Rt3[0:3,3]=C3
					Rt4=np.zeros((3,4));Rt4[0:3,0:3]=R4;Rt4[0:3,3]=C4
					

					

					# X1=cv2.triangulatePoints(self.RtI,Rt1,ret[:,1].T,ret[:,0].T).T			
					# X1=np.asarray([y[0:3]/y[3] for y in X1])
					# #print(X1.shape,X1[0:5])
					# #X1=X1[:,0:3]

					# X2=cv2.triangulatePoints(self.RtI,Rt2,ret[:,1].T,ret[:,0].T).T
					# X2=np.asarray([y[0:3]/y[3] for y in X2])
					# #X2=X2[:,0:3]
					

					# X3=cv2.triangulatePoints(self.RtI,Rt3,ret[:,1].T,ret[:,0].T).T
					# X3=np.asarray([y[0:3]/y[3] for y in X3])
					# #X3=X3[:,0:3]
					

					# X4=cv2.triangulatePoints(self.RtI,Rt4,ret[:,1].T,ret[:,0].T).T
					# X4=np.asarray([y[0:3]/y[3] for y in X4])
					#X4=X4[:,0:3]
					

					X1=self.LinearTriangulation(self.K, np.zeros((3,1)), np.eye(3), C1, R1, ret[:,1], ret[:,0])				
					X2=self.LinearTriangulation(self.K, np.zeros((3,1)), np.eye(3), C2, R2, ret[:,1], ret[:,0])
					X3=self.LinearTriangulation(self.K, np.zeros((3,1)), np.eye(3), C3, R3, ret[:,1], ret[:,0])
					X4=self.LinearTriangulation(self.K, np.zeros((3,1)), np.eye(3), C4, R4, ret[:,1], ret[:,0])	






					Cbest,Rbest,Xbest=self.DisambiguateCameraPose(C1,R1,C2,R2,C3,R3,C4,R4,X1,X2,X3,X4)
					#self.lastRt[0:3,0:3]=Rbest
					#self.lastRt[0:3,3]=Cbest
					


			
				#Rt=np.zeros((3,4));Rt[0:3,0:3]=Rbest;Rt[0:3,3]=Cbest
				#pose=np.eye(4);pose[0:3,0:3]=self.lastRt[0:3,0:3];pose[0:3,3]=self.lastRt[0:3,3]

				
				Rt=np.zeros((3,4));Rt[0:3,0:3]=self.lastRt[0:3,0:3];Rt[0:3,3]=self.lastRt[0:3,3]
				pose=np.eye(4);pose[0:3,0:3]=Rbest;pose[0:3,3]=Cbest




				temp=np.dot(Rt,pose)
				print(temp)
				print("#################################################")




				trans=temp[:,3]
				self.odomPlotter.update_line((trans[0],trans[1], trans[2]))
						

				

				###########################################################

				self.lastRt=temp


				self.last={'kp':kp,'des':des}


				###########################################################


				


			return ret
