
	
import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import multiprocessing 
import copy
import queue
import threading
 
 
def update_line(hl, new_data):
	xdata, ydata, zdata = hl._verts3d
	hl.set_xdata(list(np.append(xdata, new_data[0])))
	hl.set_ydata(list(np.append(ydata, new_data[1])))
	hl.set_3d_properties(list(np.append(zdata, new_data[2])))
	plt.draw()
 
 
# map = plt.figure()
# map_ax = Axes3D(map)
# map_ax.autoscale(enable=True, axis='both', tight=True)
 
# # # # Setting the axes properties
# map_ax.set_xlim3d([-100.0, 100.0])
# map_ax.set_ylim3d([-100.0, 100.0])
# map_ax.set_zlim3d([-100.0, 100.0])
 
# hl, = map_ax.plot3D([0], [0], [0])
 
# update_line(hl, (2,2, 1))
# plt.show(block=False)
# plt.pause(1)
 
# update_line(hl, (5,5, 5))
# plt.show(block=False)
# plt.pause(2)
 
# update_line(hl, (85,1, 4))
# plt.show(block=True)






def DISPLAY():
	import numpy as np
	import OpenGL.GL as gl
	import pangolin

	pangolin.CreateWindowAndBind('Main', 640, 480)
	gl.glEnable(gl.GL_DEPTH_TEST)

	# Define Projection and initial ModelView matrix
	scam = pangolin.OpenGlRenderState(
	    pangolin.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
	    pangolin.ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin.AxisDirection.AxisY))
	handler = pangolin.Handler3D(scam)

	# Create Interactive View in window
	dcam = pangolin.CreateDisplay()
	dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0/480.0)
	dcam.SetHandler(handler)
	while not pangolin.ShouldQuit():
	    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
	    gl.glClearColor(1.0, 1.0, 1.0, 1.0)
	    dcam.Activate(scam)
	    
	    # Render OpenGL Cube
	    pangolin.glDrawColouredCube()

	    # Draw Point Cloud
	    points = np.random.random((100000, 3)) * 10
	    gl.glPointSize(2)
	    gl.glColor3f(1.0, 0.0, 0.0)
	    pangolin.DrawPoints(points)

	    pangolin.FinishFrame()




threading.Thread(target=DISPLAY,daemon=True).start()

while True:
	print("Hello")





