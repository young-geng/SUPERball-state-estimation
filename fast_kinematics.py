import numpy as np
import theano
import theano.tensor as T
import scipy.optimize as so
import cPickle as pickle
import time

#structure parameters
barLength = 1.67;
totalSUPERballMass = 21;    
barSpacing = barLength/4;
lims = 2.5*barLength;
Kp = 998;                   
Ka = 3150;                  
preTension = 100;                   
Cp = 70;                    
Ca = 70;                    
stringStiffness = np.vstack([Ka*np.ones((12,1)), Kp*np.ones((12,1))]);   
barStiffness = 100000*np.ones((6,1));
nodes = np.array([
    [barSpacing,      0,               -barLength*0.5],
    [barSpacing,      0,                barLength*0.5],
    [0,               barLength*0.5,    barSpacing],
    [0,              -barLength*0.5,    barSpacing],
    [barLength*0.5,   barSpacing,       0],
    [-barLength*0.5,   barSpacing,       0],
    [-barSpacing,      0,                barLength*0.5],
    [-barSpacing,      0,               -barLength*0.5],      
    [0,               barLength*0.5,   -barSpacing],
    [0,              -barLength*0.5,   -barSpacing],
    [barLength*0.5,  -barSpacing,       0],
    [-barLength*0.5,  -barSpacing,       0]
    ])

bars = np.vstack((np.arange(0,12,2),
    np.arange(1,12,2))).T
strings = np.vstack((np.array((1,  2, 3, 4, 5, 6, 7, 8,  9, 10, 11, 12,  1, 1, 11, 11, 10, 10, 3, 3, 7,  7, 6, 6,)),
           np.array((11, 5, 7, 2, 9, 3, 6, 12, 8, 1,  10, 4,   9, 5, 2,  4,  12, 8,  5, 2, 4, 12, 8, 9,)))).T-1

Cs = np.zeros((strings.shape[0],nodes.shape[0]))
Cb = np.zeros((bars.shape[0],nodes.shape[0]))
for i in xrange(strings.shape[0]):
    Cs[i,strings[i,0]] = -1
    Cs[i,strings[i,1]] = 1
for i in xrange(bars.shape[0]):
    Cb[i,bars[i,0]] = -1
    Cb[i,bars[i,1]] = 1
    
ks = stringStiffness.flatten()
kb = barStiffness.flatten()
C = np.vstack((Cs,Cb))
k = np.vstack((stringStiffness,barStiffness)).flatten()
l0bar = np.linalg.norm(Cb.dot(nodes),axis=1)

#theano variables
l0string = T.vector() #input
N = T.matrix() #output

#theano functions
Lstring = T.sqrt(T.sum(T.dot(Cs,N)**2,1)) #string length
Lbar = T.sqrt(T.sum(T.dot(Cb,N)**2,1)) #bar length
Fbar = kb*(Lbar-l0bar)
Fstring = ks*T.maximum(0,Lstring-l0string)
Ubar = 0.5*kb*(Lbar-l0bar)**2
Ustring = 0.5*ks*T.maximum(0,Lstring-l0string)**2
U = T.sum(Ubar)+T.sum(Ustring)
f_Fbar = theano.function([N],Fbar) #bar forces
f_Fstring = theano.function([N,l0string],Fstring) #string forces
f_U = theano.function([N,l0string],U,mode='FAST_RUN') #potential energy
dU_N = T.grad(U,N)
f_dU_N = theano.function([N,l0string],dU_N,mode='FAST_RUN') #derivative of potential energy wrt nodes



def estimate_forces_and_potential_energy(N,l0_string):
    cost_fun = lambda x: f_U(x.reshape((12,3)),l0_string)
    jac_cost = lambda x: f_dU_N(x.reshape((12,3)),l0_string).flatten()
    #gradient descent on the potential energy
    res = so.minimize(cost_fun,N.ravel(),jac=jac_cost,method='L-BFGS-B',tol=1e-5)
    return f_Fstring(res.x.reshape((12,3)),l0_string),f_U(res.x.reshape((12,3)),l0_string),res.x.reshape((12,3))


class SUPERballKinematics(object):
    def __init__(self,N0,C_passive,C_actuated,C_bar,k_passive,k_actuated,k_bar,l0_passive,l0_bar,lhard_passive=0.279,lhard_actuated=0.060,spring_hard_limit=True):
        self.N0 = N0.ravel()
        self.C_passive = C_passive
        self.C_actuated = C_actuated
        self.C_bar = C_bar
        self.k_passive = k_passive
        self.k_actuated = k_actuated
        self.k_bar = k_bar
        self.l0_passive = l0_passive
        self.l0_bar = l0_bar
        self.lhard_passive = lhard_passive
        self.lhard_actuated = lhard_actuated
        self.spring_hard_limit = spring_hard_limit

        #variables
        self.N = T.vector() #output, reshape to 12,3
        self.l0_actuated = T.vector() #input
        self.N_m = self.N.reshape((12,3))

        #helper functions
        self.L_passive = T.sqrt(T.sum(T.dot(self.C_passive,self.N_m)**2,1)) #string length passive cables
        self.L_actuated = T.sqrt(T.sum(T.dot(self.C_actuated,self.N_m)**2,1)) #string length actuated cables
        self.L_bar = T.sqrt(T.sum(T.dot(self.C_bar,self.N_m)**2,1)) #bar lengths

        if (not self.spring_hard_limit):
            self.F_passive = self.k_passive*T.maximum(0,self.L_passive-self.l0_passive) #string forces actuated cables
            self.F_actuated = self.k_actuated*T.maximum(0,self.L_actuated-self.l0_actuated) #string forces actuated cables
        else:
            _l0 = T.maximum(0,self.L_passive-self.l0_passive)
            _l1 = T.maximum(0,self.L_passive-self.l0_passive-self.lhard_passive)
            self.F_passive = self.k_passive*(_l0 + T.exp(_l1)-1)
            _l0 = T.maximum(0,self.L_actuated-self.l0_actuated)
            _l1 = T.maximum(0,self.L_actuated-self.l0_actuated-self.lhard_actuated)
            self.F_actuated = self.k_actuated*(_l0 + T.exp(_l1)-1)
        self.F_bar = self.k_bar*T.maximum(0,self.L_bar-self.l0_bar) #bar forces

        #potential energy functions
        if (not self.spring_hard_limit):
            self.U_passive = 0.5*self.k_passive*T.maximum(0,self.L_passive-self.l0_passive)**2
            self.U_actuated = 0.5*self.k_actuated*T.maximum(0,self.L_actuated-self.l0_actuated)**2
        else:
            _l0 = T.maximum(0,self.L_passive-self.l0_passive)
            _l1 = T.maximum(0,self.L_passive-self.l0_passive-self.lhard_passive)
            self.U_passive = 0.5*self.k_passive*_l0**2 + self.k_passive*(T.exp(_l1)-_l1-1)
            _l0 = T.maximum(0,self.L_actuated-self.l0_actuated)
            _l1 = T.maximum(0,self.L_actuated-self.l0_actuated-self.lhard_actuated)
            self.U_actuated = 0.5*self.k_actuated*_l0**2 + self.k_actuated*(T.exp(_l1)-_l1-1)


        self.U_bar = 0.5*self.k_bar*(self.L_bar-self.l0_bar)**2 #potential energy per bar
        self.U = T.sum(self.U_passive)+T.sum(self.U_actuated)+T.sum(self.U_bar) #total potential energy
        self.dU_N = T.grad(self.U,self.N)

        print "Compiling Theano functions..."
        #compiled functions
        self.f_U = theano.function([self.N,self.l0_actuated],self.U,mode='FAST_RUN') 
        self.f_dU_N = theano.function([self.N,self.l0_actuated],self.dU_N,mode='FAST_RUN') 
        self.f_F_passive = theano.function([self.N],self.F_passive,mode='FAST_RUN') 
        self.f_F_actuated = theano.function([self.N,self.l0_actuated],self.F_actuated,mode='FAST_RUN') 
        self.f_F_bar = theano.function([self.N],self.F_bar,mode='FAST_RUN') 

    def find_equilibrium(self,l0_actuated,tol=1e-5,method='L-BFGS-B'):
        res = so.minimize(self.f_U,self.N0,jac=self.f_dU_N,args=(l0_actuated,),method=method,tol=tol)
        #compute forces
        F_actuated = self.f_F_actuated(res.x,l0_actuated)
        #compute energy
        U = res.fun
        #compute final nodal coordinates
        N = res.x.reshape((12,3))
        return F_actuated,U,N

#create instance of SUPERballKinematics that matches the robot
superball = SUPERballKinematics(nodes,
                            Cs[12:] #passive
                            ,Cs[:12], #actuated
                            Cb,
                            998,#passive stiffness
                            3150, #actuated stiffness
                            100000,#bar stiffness
                            .99, #passive l0
                            1.67,spring_hard_limit=True) #bar length
#inputs should be between 0.5m and 95m
def run_algorithm(iterations,l0=None):
    #iterations is ignored if l0 is not None
    if (l0 is None):
        l0 = np.random.random((iterations,12))*0.45+0.5
    result = map(superball.find_equilibrium,l0)
    return l0,result

def run_algorithm_save(iterations,path="",l0=None):
    #iterations is ignored if l0 is not None
    l0,result = run_algorithm(iterations,l0)
    iterations = l0.shape[0]
    #extract results into arrays
    F = np.zeros((iterations,12),dtype=np.float32)
    U = np.zeros(iterations,dtype=np.float32)
    N = np.zeros((iterations,12,3),dtype=np.float32)
    l0 = l0.astype(np.float32)
    for i in xrange(iterations):
        F[i] = result[i][0]
        U[i] = result[i][1]
        N[i] = result[i][2]
    date_string = time.strftime('%Y_%m_%d_%H_%M_%S', time.localtime())
    data = {'l0':l0,'Fstring':F,'U':U,'N':N}
    for k in data:
        fname = "%s/%s_%s.npy"%(path,k,date_string)
        np.save(fname,data[k])
        print fname