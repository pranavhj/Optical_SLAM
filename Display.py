import numpy as np
import matplotlib as mp
import pygame
import cv2
import sdl2.ext
class Display(object):
	def __init__(self,W,H):
		sdl2.ext.init()
		self.window=sdl2.ext.Window('twitchslam',size=(W,H))
		self.window.show()
		self.W=W
		self.H=H

	def paint(self,img):
		events=sdl2.ext.get_events()
		for event in events:
			if event.type==sdl2.SDL_QUIT:
				running=False
				exit(0)
				break
		#print(dir(window))
		surf=sdl2.ext.pixels3d(self.window.get_surface())
		surf[:,:,0:3]=img.swapaxes(0,1)

		self.window.refresh()

