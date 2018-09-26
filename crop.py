#######################################################################
# Crop images

from os.path import isfile, join
from PIL import Image
import os
import argparse
import numpy as np


def main(image_dir, x1, y1, x2, y2, output_dir):
	if not os.path.exists(output_dir):
		os.mkdir(output_dir)
	
	for file in os.listdir(image_dir):
		print(file)
		
		# read image
		im = Image.open(image_dir + "/" + file)

		# crop
		im2 = im.crop((x1, y1, x2, y2))
		
		# save
		im2.save(output_dir + "/" + file)
		

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("image_dir", help="path to folder containing images")
	parser.add_argument("x1", type=int, help="x1")
	parser.add_argument("y1", type=int, help="y1")
	parser.add_argument("x2", type=int, help="x2")
	parser.add_argument("y2", type=int, help="y2")
	parser.add_argument("output_dir", help="path to folder to save the output")
	args = parser.parse_args()

	main(args.image_dir, args.x1, args.y1, args.x2, args.y2, args.output_dir)
