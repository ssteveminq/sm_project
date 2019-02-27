import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm


def map_states(row):

	num_elements = len(row)

	for i in range(0,num_elements):
		if row[i] != 0:
			row[i] = row[i]-1

	row[0] = 0

	return row 

def main():

	data = np.genfromtxt('02_26_init_30_percent_2.csv',delimiter=',', dtype = float)

	time = [row[0]/60 for row in data]
	time[0] = 0
	state = [row[1] for row in data]
	workload = [row[2]/30 for row in data]
	complete_work_with_robot = [row[3] for row in data]
	tries = [row[4] for row in data]
	success = [row[5] for row in data]
	obstacle1 = [row[6]*1 for row in data]
	obstacle2 = [row[7]*2 for row in data]
	human_at_start_of_check = [row[8] for row in data]

	# print workload
	state = map_states(state)
	print state
	for i in range(0, len(state)):
		print state[i], complete_work_with_robot[i]

	# exit()



	# Plot 2
	plt.figure(1)
	plt.subplot(2, 1, 1)
	plt.plot(time[1:], state[1:], 'r', label='state')
	plt.scatter(time[1:],state[1:],c=cm.magma(complete_work_with_robot[1:]))
	plt.plot(time[1:], obstacle1[1:], 'o', label='obstacle1')
	plt.plot(time[1:], obstacle2[1:], 'o', label='obstacle2')
	axes = plt.gca()
	axes.set_xlim([-.1,6])
	axes.set_ylim([-.1,3.1])

	plt.grid()
	plt.subplot(2, 1, 2)

	plt.plot(time, workload, label='workload')
	# plt.scatter(time,workload)
	axes = plt.gca()
	axes.set_xlim([-.1,6])
	axes.set_ylim([.2,.9])

	plt.grid()
	plt.show()

# # Plot 1
# 	# plt.figure(1)
# 	plt.plot(time, state, 'r', label='state')
# 	# plt.plot(time, state, 'k--', label='state')
# 	plt.scatter(time,state,c='k')

# 	# plt.scatter(time,state,c=cm.magma(complete_work_with_robot))
# 	plt.plot(time, workload, label='workload')
# 	plt.scatter(time,workload)
# 	# plt.plot(time, obstacle1, 'o', label='obstacle1')
# 	# plt.plot(time, obstacle2, 'o', label='obstacle2')
# 	plt.plot()
# 	plt.xlabel('time (s)')
# 	# plt.legend()
# 	plt.grid()
# 	axes = plt.gca()
# 	axes.set_ylim([-.1,3.1])
# 	# axes.set_ylim([.2,.9])
# 	axes.set_xlim([0,34])

# 	plt.show()


if __name__ == '__main__':
	main()


