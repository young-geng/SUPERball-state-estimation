#X = N x 12 x 3
X_o = X[:,0].copy() #origin
X_x = ((X[:,4]-X_o).T/np.linalg.norm(X[:,4]-X_o,axis=1)).T #x axis
X_z = np.cross(X_x,X[:,8]-X_o) #z axis
X_z=(X_z.T/np.linalg.norm(X_z,axis=1)).T 
X_y = np.cross(X_z,X_x) #y axis
COM_x = -np.sum(X_o*X_x,1) 
COM_y = -np.sum(X_o*X_y,1)
COM_z = -np.sum(X_o*X_z,1)

COM = np.vstack((COM_x,COM_y,COM_z)).T #COM
