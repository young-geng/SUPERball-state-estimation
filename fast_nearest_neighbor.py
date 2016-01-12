import numpy as np
import os
import pyflann

#FLANN stuff
pyflann.set_distance_type('euclidean')
f = pyflann.FLANN()
algo_options={'algorithm': 'composite',
 'branching': 32,
 'build_weight': 0.009999999776482582,
 'cb_index': 0.5,
 'centers_init': 'default',
 'checks': 32,
 'cores': 0,
 'eps': 0.0,
 'iterations': 20,
 'key_size_': 20L,
 'leaf_max_size': 4,
 'log_level': 'warning',
 'max_neighbors': -1,
 'memory_weight': 0.0,
 'multi_probe_level_': 2L,
 'random_seed': 38340929,
 'sample_fraction': 0.10000000149011612,
 'sorted': 1,
 'speedup': 0.0,
 'table_number_': 12L,
 'target_precision': 1.0,
 'trees': 5}

#data
print "loading data..."
datas_F = filter(lambda x:x[0]=='F',os.listdir('/Users/kcaluwae/Documents/nasa/multilateration/kinematics_data/'))
datas_l0 = filter(lambda x:x[0]=='l',os.listdir('/Users/kcaluwae/Documents/nasa/multilateration/kinematics_data/'))

datan_F = [np.load("/Users/kcaluwae/Documents/nasa/multilateration/kinematics_data/%s"%o) for o in datas_F]
datan_l0 = [np.load("/Users/kcaluwae/Documents/nasa/multilateration/kinematics_data/%s"%o) for o in datas_l0]

data = np.vstack(datan_l0)
print "building FLANN index (%d samples)..."%data.shape[0]
f.build_index(data,**algo_options)

#a = np.random.random(12)*0.45+0.5
#f.nn_index(a.astype(np.float32))