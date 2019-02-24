import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm


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
	complete_work_with_robot = [row[3] for row in data]
	tries = [row[4] for row in data]
	success = [row[5] for row in data]
	obstacle1 = [row[6]*1 for row in data]
	obstacle2 = [row[7]*2 for row in data]
	human_at_start_of_check = [row[8] for row in data]

	print workload

	print time 
	map_states(state)

	plt.figure(1)
	plt.plot(time, state, 'k--', label='state')
	plt.scatter(time,state,c=cm.magma(complete_work_with_robot))
	plt.plot(time, workload, '--', label='workload')
	#plt.scatter(time,workload)
	plt.plot(time, obstacle1, 'o', label='obstacle1')
	plt.plot(time, obstacle2, 'o', label='obstacle2')
	plt.xlabel('time (s)')
	plt.legend()
	plt.grid()
	axes = plt.gca()
	axes.set_ylim([-.1,3.1])

	plt.show()


if __name__ == '__main__':
	main()


