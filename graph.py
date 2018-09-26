import matplotlib.pyplot as plt
import numpy as np
import argparse

def main(record_file):
	errors = []
	num_primitive_shapes = []
	algorithms = []

	file = open(record_file, "r")
	for line in file.readlines():
		data = line.split(" ")
		errors.append(float(data[0]))
		num_primitive_shapes.append(int(data[1]))
		algorithms.append(int(data[2]))

	fig, axs = plt.subplots(1, 3, tight_layout=True)

	bins = np.linspace(0.0, 1.0, num=11)
	axs[0].hist(errors, facecolor='green', bins=bins, edgecolor='black', linewidth=1.2, alpha=0.75)
	axs[0].set_xlabel('Error (1-IOU)')
	axs[0].set_xlim([0, 1])
	axs[0].set_ylim([0, 200])
	bins = np.linspace(0, 100, num=11)
	axs[1].hist(num_primitive_shapes, facecolor='blue', bins=bins, edgecolor='black', linewidth=1.2, alpha=0.75)
	axs[1].set_xlabel('#Primitive shapes')
	axs[1].set_xlim([0, 100])
	axs[1].set_ylim([0, 200])
	axs[2].hist(algorithms, facecolor='red', edgecolor='black', linewidth=1.2, alpha=0.75)
	axs[2].set_xlabel('Selected Algorithm')
	axs[2].set_xticklabels(['', 'DP', 'RA', 'Curve'])
	
	#plt.grid(True)
	plt.show()
	
	
if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("record_file", help="path to record file")
	args = parser.parse_args()

	main(args.record_file)