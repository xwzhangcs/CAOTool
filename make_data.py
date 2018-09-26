#######################################################################
# Cut roof

from os.path import isfile, join
from PIL import Image
import os
import argparse
import numpy as np


def main(image_dir):
	prev_index = 1
	for file in sorted(os.listdir(image_dir)):
		print(file)
		
		filename, file_extension = os.path.splitext(file)
		
		name, id_str = filename.rsplit('_', 1)
		id = int(id_str)
		
		# read image
		im = Image.open(image_dir + "/" + file).convert(mode="RGB")
		
		for i in range(prev_index, id):
			im.save(image_dir + "/" + name + "_" + "{:0>3d}".format(i) + ".png")
			#print(image_dir + "/" + name + "_" + "{:0>3d}".format(i))
		
		prev_index = id + 1

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("image_dir", help="path to the image directory")
	args = parser.parse_args()

	main(args.image_dir)
