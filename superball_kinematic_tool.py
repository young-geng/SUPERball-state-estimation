import numpy as np
import os
import pyflann
import time

class KinematicDataProcessing(object):
	def __init__(self):
		self.l0_files = []
		self.F_files = []
		self.l0_datasets = []
		self.F_datasets = []

	def add_data(self,l0_file,F_file):
		print l0_file
		#print F_file
		self.l0_files.append(l0_file)
		self.F_files.append(F_file)

	def add_data_directory(self,path):
		#find all files starting with l0
		l0_files = filter(lambda x:x[0:2]=='l0',os.listdir(path))
		for l0_file in l0_files:
			#find matching F_file
			F_file = "Fstring%s"%(l0_file[2:])
			self.add_data("%s/%s"%(path,l0_file),"%s/%s"%(path,F_file))

	def filter_data(self,F_filter=lambda x: np.where(x.max(1)<250)[0]):
		#load files and filter out data matching the given filter
		self.l0_datasets = []
		self.F_datasets = []
		for idx,l0_file in enumerate(self.l0_files):
			#open data file
			F_file = self.F_files[idx]
			F_data = np.load(F_file)
			#filter data
			valid_indices = F_filter(F_data)
			self.F_datasets.append(F_data[valid_indices,:])
			del F_data
			l0_data = np.load(l0_file)
			self.l0_datasets.append(l0_data[valid_indices,:])
			print "loaded %s, %d/%d valid entries, %d total valid entries"%(
				l0_file,
				valid_indices.shape[0],
				l0_data.shape[0],
				np.sum([o.shape[0] for o in self.l0_datasets])) 


	def save_dataset(self,output_prefix):
		l0_output = np.vstack((self.l0_datasets))
		date_string = time.strftime('%Y_%m_%d_%H_%M_%S', time.localtime())
		np.save("%s/l0_filtered_%s.npy"%(output_prefix,date_string),l0_output)
		del l0_output
		F_output = np.vstack((self.F_datasets))
		np.save("%s/Fstring_filtered_%s.npy"%(output_prefix,date_string),F_output)
		print "%s/l0_filtered_%s.npy"%(output_prefix,date_string)


class KinematicMotorConstraints(object):
	def __init__(self,l0_filtered_file,distance_type='euclidean',algo_options={
				 'algorithm': 'composite',
				 'iterations': 20,
				 'target_precision': 1.0,
				 'trees': 5},index_file=None):
		print "Loading data"
		self.l0_filtered_file = l0_filtered_file
		#open file
		self.l0_data = np.load(self.l0_filtered_file) #need to keep this in memory to get the actual lengths
		#create FLANN index 
		pyflann.set_distance_type(distance_type)
		self.nn = pyflann.FLANN()
		if(index_file is None):
			print "building FLANN index"
			print self.nn.build_index(self.l0_data,**algo_options)
		else:
			print "loading FLANN index from file"
			self.nn.load_index(index_file,self.l0_data)

	def find_nearest_valid_values(self,l0):
		idx = self.nn.nn_index(l0.astype(np.float32))[0][0]
		return self.l0_data[idx]

	def save_index(self,fname):
		self.nn.save_index(fname)		
