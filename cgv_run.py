#######################################################################
# Process all the buildings in the data folder

from os.path import isfile, join
from PIL import Image
import os
import argparse
import numpy as np
import json
import subprocess
import sys

def main(data_dir, weight, algorithm, output_dir):
	# Create the output directory if not exists
	if not os.path.exists(output_dir):
		os.mkdir(output_dir)
	
	for cluster_folder in sorted(os.listdir(data_dir + "/BuildingClusters/")):
		print(cluster_folder)
		
		cluster_path = data_dir + "/BuildingClusters/" + cluster_folder
		
		# read metadata.json
		metadata = json.load(open(cluster_path + "/cluster_" + cluster_folder + "__metadata.json"))

		# set the cgv tool arguments
		cur_path = os.path.dirname(os.path.realpath(__file__))
		intput_file_path = cur_path + "/" + cluster_path + "/Slices/slice_000000.png"
		output_obj_path = cur_path + "/" + output_dir + "/" + cluster_folder + "_building.obj"
		output_topface_path = cur_path + "/" + output_dir + "/" + cluster_folder + "_building.txt"
		
		# run cgv tool
		subprocess.call(["cgv/LEGO_NOGUI", intput_file_path, weight, algorithm, str(metadata["position"][0]), str(metadata["position"][1]), "0", str(metadata["voxel_size"]), output_obj_path, output_topface_path], cwd = "cgv")

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("data_dir", help="path to site data folder (e.g., AOI-D1-WPAFB)")
	parser.add_argument("weight", help="weight (0 - 1)")
	parser.add_argument("algorithm", help="algorithm option (1 - All, 2 - DP)")
	parser.add_argument("output_dir", help="path to folder to save the output")
	args = parser.parse_args()
	
	if not os.path.exists(args.data_dir):
		print("Directory not found: " + args.data_dir)
		sys.exit(0)

	main(args.data_dir, args.weight, args.algorithm, args.output_dir)