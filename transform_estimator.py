import numpy as np

def find_transformation_matrix(X,Y):
	'''
		Finds the optimal homogeneous transform between X and Y,
		such that X.dot(R) +T ~= Y (X and Y are Nx3 matrices).	
	'''
	Xm = X.mean(0)
	Ym = Y.mean(0)
	#center X and Y
	Xc = X-Xm
	Yc = Y-Ym
	#SVD to find rotation
	H = Xc.T.dot(Yc)
	V,S,WT = np.linalg.svd(H)
	d = np.sign(np.linalg.det(WT.T.dot(V.T)))
	R = WT.T.dot(np.diag([1,1,d])).dot(V.T).T
	T = -R.T.dot(Xm.reshape((-1,1)))+Ym.reshape((-1,1))
	return R,T.ravel()

def random_rot(dim):
    """Return a random rotation matrix, drawn from the Haar distribution
    (the only uniform distribution on SO(n)).
    The algorithm is described in the paper
    Stewart, G.W., 'The efficient generation of random orthogonal
    matrices with an application to condition estimators', SIAM Journal
    on Numerical Analysis, 17(3), pp. 403-409, 1980.
    For more information see
    http://en.wikipedia.org/wiki/Orthogonal_matrix#Randomization"""
    H = np.eye(dim)
    D = np.ones((dim,))
    for n in xrange(1, dim):
        x = np.random.normal(size=(dim-n+1,))
        D[n-1] = np.sign(x[0])
        x[0] -= D[n-1]*np.sqrt((x*x).sum())
        # Householder transformation

        Hx = np.eye(dim-n+1) - 2.*np.outer(x, x)/(x*x).sum()
        mat = np.eye(dim)
        mat[n-1:,n-1:] = Hx
        H = np.dot(H, mat)
    # Fix the last sign such that the determinant is 1
    D[-1] = -D.prod()
    H = (D*H.T).T
    if(np.linalg.det(H)<0):
        H = -H
    return H	
