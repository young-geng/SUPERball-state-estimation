import scipy.optimize as so
import numpy as np
import scipy.spatial as ss
import theano
import theano.tensor as T

'''
    TODO: enable/disable terms
    TODO: add support for distance constraints
    TODO: add support for accelerometers
'''

class Calibration(object):
	def __init__(self,num_nodes_base,num_nodes_floating):
		self.num_base = num_nodes_base
		self.num_floating = num_nodes_floating
		#create variables for positions
		self.X_base = T.dmatrix('X_base')
		self.X_floating = T.dmatrix('X_floating')
		#create variables for distance measurements
		self.dist_B_B = T.dvector('dist_B_B')
		self.dist_B_F = T.dvector('dist_B_F')
		self.dist_F_F = T.dvector('dist_F_F')
		#create variables for relative costs (types of measurements/constraints)
		self.relative_cost_B_B = T.dscalar('relative_cost_B_B')
		self.relative_cost_B_F = T.dscalar('relative_cost_B_F')
		self.relative_cost_F_F = T.dscalar('relative_cost_F_F')
		self.relative_cost_F_F_constraints = T.dscalar('relative_cost_F_F_constraints')
		#create variables to premultiply (enable/disable) distance measurement costs
		self.mult_B_B = T.dvector('mult_B_B')
		self.mult_B_F = T.dvector('mult_B_F')
		self.mult_F_F = T.dvector('mult_F_F')
		#create variables for distance constraints
		#TODO: do we need distance constraints between base nodes?
		self.dist_F_F_constraints = T.dvector('dist_constraints_F_F')
		#create variables for inclination measurements
		self.inclination = T.dvector('inclination')

	def setup(self,
		measurements_B_B=None,
		measurements_B_F=None,
		measurements_F_F=None,
		constraints_F_F=None):
		'''
			measurements_X_Y: array of node pairs (B = base, F = floating) for distance measurements
		'''
		#distance and calculations based on estimated positions
		self.cost_B_B = T.constant(0.)
		self.cost_B_F = T.constant(0.)
		self.cost_F_F = T.constant(0.)
		self.cost_F_F_constraints = T.constant(0.)
		print "Computing mutual distances and cost function"
		if(not measurements_B_B is None):
			#compute all mutual distances
			X = self.X_base
			Y = self.X_base
			all_dists = ((X ** 2).sum(1).reshape((X.shape[0], 1)) + (Y ** 2).sum(1).reshape((1, Y.shape[0])) - 2 * X.dot(Y.T)).flatten() #flat
			#select the elements we need
			selected = measurements_B_B[:,0]*self.num_base+measurements_B_B[:,1]
			dists = T.sqrt(all_dists[selected])
			self.dists_B_B = dists
			#compute cost
			self.cost_B_B = T.mean(self.mult_B_B*((dists-self.dist_B_B)**2))#MSE on distances: equivalent to optimizing log-likelihood when assuming gaussian noise 
		if(not measurements_B_F is None):
			#compute all mutual distances
			X = self.X_base
			Y = self.X_floating
			all_dists = ((X ** 2).sum(1).reshape((X.shape[0], 1)) + (Y ** 2).sum(1).reshape((1, Y.shape[0])) - 2 * X.dot(Y.T)).flatten() #flat
			#select the elements we need
			selected = measurements_B_F[:,0]*self.num_floating+measurements_B_F[:,1]
			dists = T.sqrt(all_dists[selected])
			self.dists_B_F = dists
			#compute cost
			self.cost_B_F = T.mean(self.mult_B_F*((dists-self.dist_B_F)**2))
		if(not measurements_F_F is None):
			#compute all mutual distances
			X = self.X_floating
			Y = self.X_floating
			all_dists = ((X ** 2).sum(1).reshape((X.shape[0], 1)) + (Y ** 2).sum(1).reshape((1, Y.shape[0])) - 2 * X.dot(Y.T)).flatten() #flat
			#select the elements we need
			selected = measurements_F_F[:,0]*self.num_floating+measurements_F_F[:,1]
			dists = T.sqrt(all_dists[selected])
			self.dists_F_F = dists
			#compute cost
			self.cost_F_F = T.mean(self.mult_F_F*((dists-self.dist_F_F)**2))
		if(not constraints_F_F is None):
			#compute all mutual distances
			X = self.X_floating
			Y = self.X_floating
			all_dists = ((X ** 2).sum(1).reshape((X.shape[0], 1)) + (Y ** 2).sum(1).reshape((1, Y.shape[0])) - 2 * X.dot(Y.T)).flatten() #flat
			#select the elements we need
			selected = constraints_F_F[:,0]*self.num_floating+constraints_F_F[:,1]
			dists = T.sqrt(all_dists[selected])
			self.dists_F_F_constraints = dists
			#compute cost
			self.cost_F_F_constraints = T.mean((dists-self.dist_F_F_constraints)**2)
		#full cost function
		self.cost = self.cost_B_B*self.relative_cost_B_B + self.cost_B_F*self.relative_cost_B_F + self.cost_F_F*self.relative_cost_F_F + self.cost_F_F_constraints*self.relative_cost_F_F_constraints 
		#compute gradients
		print "Computing gradients"
		self.grad_X_base = T.grad(self.cost,self.X_base,disconnected_inputs='ignore')
		self.grad_X_floating = T.grad(self.cost,self.X_floating,disconnected_inputs='ignore')
		print "Compiling functions"
		self.f_cost = theano.function([self.X_base,self.X_floating,self.dist_B_B,self.dist_B_F,self.dist_F_F,self.dist_F_F_constraints,self.relative_cost_B_B,self.relative_cost_B_F,self.relative_cost_F_F,self.relative_cost_F_F_constraints,self.mult_B_B,self.mult_B_F,self.mult_F_F],self.cost,mode="FAST_RUN",on_unused_input='ignore')
		self.f_grad_X_base = theano.function([self.X_base,self.X_floating,self.dist_B_B,self.dist_B_F,self.dist_F_F,self.dist_F_F_constraints,self.relative_cost_B_B,self.relative_cost_B_F,self.relative_cost_F_F,self.relative_cost_F_F_constraints,self.mult_B_B,self.mult_B_F,self.mult_F_F],self.grad_X_base,mode="FAST_RUN",on_unused_input='ignore')
		self.f_grad_X_floating = theano.function([self.X_base,self.X_floating,self.dist_B_B,self.dist_B_F,self.dist_F_F,self.dist_F_F_constraints,self.relative_cost_B_B,self.relative_cost_B_F,self.relative_cost_F_F,self.relative_cost_F_F_constraints,self.mult_B_B,self.mult_B_F,self.mult_F_F],self.grad_X_floating,mode="FAST_RUN",on_unused_input='ignore')


	def calibrate(self,
		distances_B_B,
		distances_B_F,
		distances_F_F,
		distances_F_F_constraints,
		relative_cost_B_B=1.,
		relative_cost_B_F=1.,
		relative_cost_F_F=1.,
		relative_cost_F_F_constraints=10.,
		mult_B_B=1,
		mult_B_F=1,
		mult_F_F=1,
		cut_off=0.01,
		max_iter=10):
		'''
			Run gradient descent to locate base and floating nodes
		'''
		if(np.isscalar(mult_B_B) and not distances_B_B is None):
			mult_B_B = np.ones(distances_B_B.shape[0])*mult_B_B
		if(np.isscalar(mult_B_F) and not distances_B_F is None):
			mult_B_F = np.ones(distances_B_F.shape[0])*mult_B_F
		if(np.isscalar(mult_F_F) and not distances_F_F is None):
			mult_F_F = np.ones(distances_F_F.shape[0])*mult_F_F
		final_cost = cut_off + 1
		#helper functions to compute cost and gradient
		def cost_fun(x):
			base = x[:self.num_base*3].reshape((self.num_base,3))
			floating = x[self.num_base*3:].reshape((self.num_floating,3))
			return self.f_cost(base,floating,distances_B_B,distances_B_F,distances_F_F,distances_F_F_constraints,relative_cost_B_B,relative_cost_B_F,relative_cost_F_F,relative_cost_F_F_constraints,mult_B_B,mult_B_F,mult_F_F)
		def cost_grad(x):
			base = x[:self.num_base*3].reshape((self.num_base,3))
			floating = x[self.num_base*3:].reshape((self.num_floating,3))
			base_grad = self.f_grad_X_base(base,floating,distances_B_B,distances_B_F,distances_F_F,distances_F_F_constraints,relative_cost_B_B,relative_cost_B_F,relative_cost_F_F,relative_cost_F_F_constraints,mult_B_B,mult_B_F,mult_F_F)
			base_grad[0] = 0
			base_grad[1,1:] = 0
			base_grad[2,2] = 0
			floating_grad = self.f_grad_X_floating(base,floating,distances_B_B,distances_B_F,distances_F_F,distances_F_F_constraints,relative_cost_B_B,relative_cost_B_F,relative_cost_F_F,relative_cost_F_F_constraints,mult_B_B,mult_B_F,mult_F_F)
			return np.hstack((base_grad.ravel(),floating_grad.ravel()))
		iteration = 0
		best_cost = final_cost
		while(final_cost>cut_off and iteration<max_iter):
			#initial guesses
			X_base = np.random.random((self.num_base,3))*10.
			X_base[0] = 0 		#origin
			X_base[1,1:] = 0 	#first axis
			X_base[2,2] = 0		#second axis
			X_floating = np.random.random((self.num_floating,3))*10.
			X_initial = np.hstack((X_base.ravel(),X_floating.ravel()))
			#run optimization
			res = so.fmin_bfgs(cost_fun,X_initial,fprime=cost_grad,disp=0,gtol=1e-7)
			#result
			X_base_res = res[:self.num_base*3].reshape(X_base.shape)
			X_floating_res = res[self.num_base*3:].reshape(X_floating.shape)
			final_cost = self.f_cost(X_base_res,X_floating_res,distances_B_B,distances_B_F,	distances_F_F,distances_F_F_constraints,relative_cost_B_B,relative_cost_B_F,relative_cost_F_F,relative_cost_F_F_constraints,mult_B_B,mult_B_F,mult_F_F)
			print "cost"
			print final_cost
			if(final_cost<best_cost):
				X_base_res_final = X_base_res.copy()
				X_floating_res_final = X_floating_res.copy()
			iteration += 1
		return X_base_res_final,X_floating_res_final

	def localize(self,
		distances_B_F,
		distances_F_F,
		distances_F_F_constraints,
		X_base,
		relative_cost_B_F=1.,
		relative_cost_F_F=1.,
		relative_cost_F_F_constraints=10.,
		mult_B_F=1,
		mult_F_F=1,
		cut_off=0.01,
		max_iter=10):
		'''
			Run gradient descent to locate floating nodes
		'''
		relative_cost_B_B=0.#not used
		mult_B_B = None
		distances_B_B = None
		if(np.isscalar(mult_B_F) and not distances_B_F is None):
			mult_B_F = np.ones(distances_B_F.shape[0])*mult_B_F
		if(np.isscalar(mult_F_F) and not distances_F_F is None):
			mult_F_F = np.ones(distances_F_F.shape[0])*mult_F_F
		final_cost = cut_off + 1
		#helper functions to compute cost and gradient
		def cost_fun(x):
			base = X_base
			floating = x.reshape((self.num_floating,3))
			return self.f_cost(base,floating,distances_B_B,distances_B_F,distances_F_F,distances_F_F_constraints,relative_cost_B_B,relative_cost_B_F,relative_cost_F_F,relative_cost_F_F_constraints,mult_B_B,mult_B_F,mult_F_F)
		def cost_grad(x):
			base = X_base
			floating = x.reshape((self.num_floating,3))
			floating_grad = self.f_grad_X_floating(base,floating,distances_B_B,distances_B_F,distances_F_F,distances_F_F_constraints,relative_cost_B_B,relative_cost_B_F,relative_cost_F_F,relative_cost_F_F_constraints,mult_B_B,mult_B_F,mult_F_F)
			return floating_grad.ravel()
		iteration = 0
		best_cost = final_cost
		while(final_cost>cut_off and iteration<max_iter):
			#initial guesses
			X_floating = np.random.random((self.num_floating,3))*10.
			X_initial = X_floating.ravel()
			#run optimization
			res = so.fmin_bfgs(cost_fun,X_initial,fprime=cost_grad,disp=0,gtol=1e-7)
			#result
			X_floating_res = res.reshape(X_floating.shape)
			final_cost = self.f_cost(X_base,X_floating_res,distances_B_B,distances_B_F,	distances_F_F,distances_F_F_constraints,relative_cost_B_B,relative_cost_B_F,relative_cost_F_F,relative_cost_F_F_constraints,mult_B_B,mult_B_F,mult_F_F)
			print "cost"
			print final_cost
			if(final_cost<best_cost):
				X_floating_res_final = X_floating_res.copy()
			iteration += 1
		return X_floating_res_final


if __name__ == "__main__":
	import scipy.spatial as ss
	import sys
	#sys.setrecursionlimit(2000) #seriously, needed for naive implementation :)
	#test calibration module
	n_b = 4  #>= 4
	robot_positions = 3
	n_f = 12*robot_positions
	X_base = np.random.random((n_b,3))*10.
	X_base[0] = 0 		#origin
	X_base[1,1:] = 0 	#first axis
	X_base[2,2] = 0		#second axis
	X_float = np.zeros((n_f,3))
	import prism
	N,B,C = prism.create_prism(6,1.8,1.8,0.3)
	N = N.T 
	N[:,2] += N[:,2].min()
	robot_shape = N#np.random.random((12,3))*1.8
	#compute bar constraints
	meas_cons_f_f = np.zeros((B.shape[0],2),dtype=np.int)
	dist_cons_f_f = np.zeros(B.shape[0])
	for i in xrange(B.shape[0]):
		meas_cons_f_f[i] = (B[i].argmin(),B[i].argmax())
		dist_cons_f_f[i] = np.sqrt(np.sum((robot_shape[meas_cons_f_f[i][0]]-robot_shape[meas_cons_f_f[i][1]])**2))	
	for i in xrange(robot_positions):
		X_float[i*12:(i+1)*12] = robot_shape
		X_float[i*12:(i+1)*12,:] += np.random.random((1,3))
	#compute mutual distances
	import itertools
	meas_b_b = np.array(list(itertools.combinations(np.arange(n_b),2)))
	meas_b_f = np.array(list(itertools.product(np.arange(n_b),np.arange(n_f))))
	meas_f_f = np.array(list(itertools.combinations(np.arange(n_f),2)))
	dist_b_b = np.zeros(meas_b_b.shape[0])
	dist_b_f = np.zeros(meas_b_f.shape[0])
	dist_f_f = np.zeros(meas_f_f.shape[0])
	for i in xrange(meas_b_b.shape[0]):
		start = meas_b_b[i,0]
		stop = meas_b_b[i,1]
		dist_b_b[i] = np.sqrt(np.sum((X_base[start]-X_base[stop])**2))
	for i in xrange(meas_b_f.shape[0]):
		start = meas_b_f[i,0]
		stop = meas_b_f[i,1]
		dist_b_f[i] = np.sqrt(np.sum((X_base[start]-X_float[stop])**2))
	for i in xrange(meas_f_f.shape[0]):
		start = meas_f_f[i,0]
		stop = meas_f_f[i,1]
		dist_f_f[i] = np.sqrt(np.sum((X_float[start]-X_float[stop])**2))
	#add noise to distances
	noise_level_B_B = 0.05
	noise_level_B_F = 0.05
	noise_level_F_F = 0.05
	dist_b_b += np.random.randn(dist_b_b.shape[0])*noise_level_B_B
	dist_b_f += np.random.randn(dist_b_f.shape[0])*noise_level_B_F
	dist_f_f += np.random.randn(dist_f_f.shape[0])*noise_level_F_F

	#create calibration object (select which measurements are available)
	cal = Calibration(n_b,n_f)
	#cal.setup(None,meas_b_f,None)
	cal.setup(meas_b_b,meas_b_f,meas_f_f,meas_cons_f_f)#,relative_cost_B_B=10.)	
	#cal.setup(None,meas_b_f,meas_f_f)
	print "Calibrating"
	base_est, float_est = cal.calibrate(dist_b_b, dist_b_f,dist_f_f,dist_cons_f_f,1,1,1,10)	
	for i in xrange(3):
		if(np.mean(np.abs(base_est[:,i]-X_base[:,i]))>np.mean(np.abs(base_est[:,i]+X_base[:,i]))):
			base_est[:,i] *=-1
		if(np.mean(np.abs(float_est[:,i]-X_float[:,i]))>np.mean(np.abs(float_est[:,i]+X_float[:,i]))):
                        float_est[:,i] *=-1
	print "Reconstruction error:"
	print "Base:"
	print np.mean(np.sqrt(np.sum((base_est-X_base)**2,1)))
	print "Floating:"
	print np.mean(np.sqrt(np.sum((float_est-X_float)**2,1)))
	#check constraint violation	
	cons_measured = np.zeros(dist_cons_f_f.shape)
	for i in xrange(B.shape[0]):
		cons_measured[i] = np.sqrt(np.sum((float_est[meas_cons_f_f[i][0]]-float_est[meas_cons_f_f[i][1]])**2))	
	print "Constraint violations"
	print cons_measured-dist_cons_f_f	

	#create calibration object (select which measurements are available)
	cal = Calibration(n_b,n_f)
	#cal.setup(None,meas_b_f,None)
	cal.setup(meas_b_b,meas_b_f,meas_f_f)#,relative_cost_B_B=10.)	
	#cal.setup(None,meas_b_f,meas_f_f)
	print "Calibrating"
	base_est, float_est = cal.calibrate(dist_b_b, dist_b_f,dist_f_f,None,1,1,1,100)
        for i in xrange(3):
                if(np.mean(np.abs(base_est[:,i]-X_base[:,i]))>np.mean(np.abs(base_est[:,i]+X_base[:,i]))):
                        base_est[:,i] *=-1
		if(np.mean(np.abs(float_est[:,i]-X_float[:,i]))>np.mean(np.abs(float_est[:,i]+X_float[:,i]))):
                        float_est[:,i] *=-1	
	print "Reconstruction error:"
	print "Base:"
	print np.mean(np.sqrt(np.sum((base_est-X_base)**2,1)))
	print "Floating:"
	print np.mean(np.sqrt(np.sum((float_est-X_float)**2,1)))
	
	cons_measured = np.zeros(dist_cons_f_f.shape)
	for i in xrange(B.shape[0]):
		cons_measured[i] = np.sqrt(np.sum((float_est[meas_cons_f_f[i][0]]-float_est[meas_cons_f_f[i][1]])**2))	
	print "Constraint violations"
	print cons_measured-dist_cons_f_f	

	#test localization module
	meas_b_f_loc = np.array(list(itertools.product(np.arange(n_b),np.arange(12))))
	meas_f_f_loc = np.array(list(itertools.combinations(np.arange(12),2)))
	X_test = np.random.random((12,3))*10. #test "robot" to localize
	X = T.dmatrix('x')
	Y = T.dmatrix('y')
	all_dists = T.sqrt((X ** 2).sum(1).reshape((X.shape[0], 1)) + (Y ** 2).sum(1).reshape((1, Y.shape[0])) - 2 * X.dot(Y.T))
	f_xy = theano.function([X,Y],all_dists)
	BF = f_xy(X_base,X_test)
	FF = f_xy(X_test,X_test)
	FF_constraints = FF
	loc = Calibration(n_b,12)
	loc.setup(None,meas_b_f_loc,meas_f_f_loc,meas_f_f_loc[::10])	
	test_est = loc.localize(BF.ravel()[meas_b_f_loc[:,0]*12+meas_b_f_loc[:,1]]+noise_level_B_F*np.random.randn(meas_b_f_loc.shape[0]),
				FF.ravel()[meas_f_f_loc[:,0]*12+meas_f_f_loc[:,1]]+noise_level_F_F*np.random.randn(meas_f_f_loc.shape[0]),FF_constraints.ravel()[meas_f_f_loc[:,0]*12+meas_f_f_loc[:,1]][::10],X_base)

        for i in xrange(3):
		if(np.mean(np.abs(test_est[:,i]-X_test[:,i]))>np.mean(np.abs(test_est[:,i]+X_test[:,i]))):
                        test_est[:,i] *=-1	

	print "Test (with constraints):"
	print np.mean(np.sqrt(np.sum((test_est-X_test)**2,1)))
	loc.setup(None,meas_b_f_loc,meas_f_f_loc,None)
	test_est = loc.localize(BF.ravel()[meas_b_f_loc[:,0]*12+meas_b_f_loc[:,1]]+noise_level_B_F*np.random.randn(meas_b_f_loc.shape[0]),
				FF.ravel()[meas_f_f_loc[:,0]*12+meas_f_f_loc[:,1]]+noise_level_F_F*np.random.randn(meas_f_f_loc.shape[0]),None,X_base)

        for i in xrange(3):
		if(np.mean(np.abs(test_est[:,i]-X_test[:,i]))>np.mean(np.abs(test_est[:,i]+X_test[:,i]))):
                        test_est[:,i] *=-1	

	print "Test (no constraints):"
	print np.mean(np.sqrt(np.sum((test_est-X_test)**2,1)))
	
