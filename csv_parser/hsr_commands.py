import csv
import numpy as np

f = open('states_and_vals.csv', 'rt')
reader = csv.reader(f)

row_count = sum(1 for row in reader)

f.seek(0)

col_count = len(next(reader))
print "columns"
print col_count

f.seek(0)

data = [row for row in reader] # list comprehension 

f.close()

string_col = data[0]


position = []

for col in range(0, col_count):
	if (string_col[col] == 'robot_state'):
		robot_state_col = col 

for row1 in range(1,row_count):
	this_row = data[row1]
	position.extend([this_row[robot_state_col]])


position = np.array(position)
position.resize(position.shape[0])

print position
