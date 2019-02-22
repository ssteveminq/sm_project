import numpy as np
import matplotlib.pyplot as plt


def map_states(row):

	num_elements = len(row)

	for i in range(0,num_elements):
		if row[i] != 0:
			row[i] = row[i]-1

def main():

	data = np.genfromtxt('results.csv',delimiter=',', dtype = float)

	time = [row[0] for row in data]
	state = [row[1] for row in data]
	workload = [row[2]/30 for row in data]

	print workload

	print time 
	map_states(state)

	plt.figure(1)
	plt.plot(time, state)
	plt.plot(time, workload)
	plt.xlabel('time (s)')

	plt.grid()

	plt.show()


if __name__ == '__main__':
	main()


