'''
Created on Aug 4, 2011

@author: kcaluwae
'''
import numpy as np

def create_prism(num_struts,diam,height,rotation):
    '''
    Creates a regular tensegrity prism with num_struts struts (>=3).
    Diam is the diameter of the ground face
    Height is obviousy the height of the prism
    Rotation is the rotation of the top plane wrt the ground plane
    returns: points,bars,springs
    '''
    #create points
    points = np.zeros((3,2*num_struts))
    angles = np.arange(0,2*np.pi,2*np.pi/num_struts)
    points[0,:num_struts] = np.sin(angles)*diam
    points[1,:num_struts] = np.cos(angles)*diam
    points[2,:num_struts] = -height*0.5
    points[0,num_struts:] = np.sin(angles+rotation)*diam
    points[1,num_struts:] = np.cos(angles+rotation)*diam
    points[2,num_struts:] = height*0.5
    #create bars
    bars = np.zeros((num_struts,2*num_struts))
    bars[:,:num_struts] = -np.eye(num_struts)
    bars[:,num_struts:] = np.eye(num_struts)
    #create springs
    springs = np.zeros((num_struts*3,2*num_struts))
    springs[:num_struts,:num_struts] = -np.eye(num_struts)+np.roll(np.eye(num_struts),1,1) #ground layer
    springs[num_struts:2*num_struts,num_struts:] = -np.eye(num_struts)+np.roll(np.eye(num_struts),1,1) #top layer
    springs[2*num_struts:,:num_struts] = -np.eye(num_struts) #connections between layers
    springs[2*num_struts:,num_struts:] = np.roll(np.eye(num_struts),1,1)
    return points,bars,springs

def create_prism_stack(num_struts, height, diam, orientation,  levels):
	'''
	'levels' tensegrity prisms are stacked
	'''
	#points are numbered per layer 
	num_points = num_struts*levels*2
	points_per_layer = num_struts
	B = []
	for i in range(levels):
		b = np.hstack((np.eye(points_per_layer),-np.eye(points_per_layer)))
		b_zero = np.zeros((num_struts,2*points_per_layer*(levels-1)))
		b_full = np.roll(np.hstack((b,b_zero)), 2*points_per_layer*i, axis=1)
		B.append(b_full)

	B = np.vstack(B).astype(int)

	C = []
	A = []
	for i in range(levels):
		#break
		num_springs = num_struts #inter level connections
		c = np.hstack((np.eye(points_per_layer),-np.roll(np.eye(points_per_layer),1,axis=1)))
		c_zero = np.zeros((num_struts,2*points_per_layer*(levels-1)))
		c_full = np.roll(np.hstack((c,c_zero)), 2*points_per_layer*i,axis=1)
		C.append(c_full)

	for i in range(1,levels):
		#break
		num_springs = 2*num_struts #connections to lower level
		
		c1 = np.hstack((np.eye(points_per_layer),-np.roll(np.eye(points_per_layer),0,axis=1)))
		c2 = np.hstack((np.eye(points_per_layer),-np.roll(np.eye(points_per_layer),-1,axis=1)))
		c_zero = np.zeros((num_struts,2*points_per_layer*(levels-1)))
		
		c1_full = np.roll(np.hstack((c1,c_zero)), 2*points_per_layer*i-points_per_layer,axis=1)
		c2_full = np.roll(np.hstack((c2,c_zero)), 2*points_per_layer*i-points_per_layer,axis=1)
		C.append(c1_full)
		C.append(c2_full)

	for i in range(levels-1):
		break
		c1 = np.hstack((np.eye(points_per_layer),np.zeros((points_per_layer,points_per_layer)),-np.eye(points_per_layer)))
		c_zero = np.zeros((num_struts,2*points_per_layer*(levels-1)-points_per_layer))
		
		c1_full = np.roll(np.hstack((c1,c_zero)), 2*points_per_layer*i-1,axis=1)
		c2_full = np.roll(c1_full.copy(), points_per_layer,axis=1)

		C.append(c1_full)
		C.append(c2_full)
	for i in range(2,levels): #actuators...
		#break
		c = np.zeros((points_per_layer,num_points))
		c[:,points_per_layer*i*2:points_per_layer*(i*2+1)] = np.eye(points_per_layer)
		c[:,points_per_layer*(i*2-3):points_per_layer*(i*2-2)] = -np.roll(np.eye(points_per_layer),1,axis=1)
		A.append(c)

	
	#connections for top and bottom
	#c_end = np.eye(num_struts)-np.roll(np.eye(num_struts),1,axis=1)
	#c_zero = np.zeros((num_struts,2*points_per_layer*(levels-1)+points_per_layer))
	#c_bottom = np.hstack((c_end,c_zero))
	#c_top = np.hstack((c_zero,c_end))

	#C.append(c_bottom)
	#C.append(c_top)
	for i in range(levels):
		c = np.eye(num_struts)-np.roll(np.eye(num_struts),1,axis=1)
		#c = np.hstack((c,))
		c_zero = np.zeros((num_struts,2*points_per_layer*(levels-1)+points_per_layer))
		c_tot = np.hstack((c,c_zero))
		c_tot = np.roll(c_tot,2*points_per_layer*i,axis=1)
		C.append(c_tot)
		c_tot2 = c_tot.copy()
		c_tot2 = np.roll(c_tot2,points_per_layer,axis=1)
		C.append(c_tot2)

	C = np.vstack(C).astype(int)
	A = np.vstack(A).astype(int)	

	N = np.zeros((3,num_points))
		
	angles = np.arange(0,2*np.pi,2*np.pi/num_struts)
	for i in range(levels):
		N[0,i*2*points_per_layer:(i*2+1)*points_per_layer] = np.sin(angles+i*orientation+np.pi/num_struts*i)*diam #bottom layer of each level
		N[1,i*2*points_per_layer:(i*2+1)*points_per_layer] = np.cos(angles+i*orientation+np.pi/num_struts*i)*diam
		
		N[2,i*2*points_per_layer:(i*2+1)*points_per_layer] = height*i
		
		N[0,(i*2+1)*points_per_layer:(i*2+2)*points_per_layer] = np.sin(angles+(i+1)*orientation+np.pi/num_struts*i)*diam #top layer of each level
		N[1,(i*2+1)*points_per_layer:(i*2+2)*points_per_layer] = np.cos(angles+(i+1)*orientation+np.pi/num_struts*i)*diam
		
		N[2,(i*2+1)*points_per_layer:(i*2+2)*points_per_layer] = height*(i+1)
		
	return N,B,C,A	    
