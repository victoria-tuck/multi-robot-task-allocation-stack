import csv
import ast

row_vals = [[],[],[],[],[],[]]
inits = [[],[],[],[],[],[]]
files = ['control_time_robot1_3.csv',  'control_time_robot2_3.csv', 'control_time_robot3_3.csv', 'control_time_robot4_3.csv', 'control_time_robot5_3.csv', 'control_time_robot6_3.csv']
for file in files:
    with open(file, 'r') as filename:
        reader = csv.reader(filename)
        multi = False
        ind = 0
        for row in reader:
            numeric_values = [float(value) for value in row if value.replace('.', '', 1).isdigit()]
            if multi:
                row_vals[ind].extend(numeric_values[1:])
                if len(numeric_values) > 0:
                    avg = sum(numeric_values[1:]) / len(numeric_values[1:])
                    inits[ind].append(numeric_values[0])
            else:
                row_vals[ind].extend(numeric_values[1:])
                if len(numeric_values) > 0:
                    inits[ind].append(numeric_values[0])
                    avg = sum(numeric_values) / len(numeric_values)
                else:
                    avg = 0
            # row_averages[ind].append(avg)
            multi = True
            ind += 1

print(inits)
print(row_vals)
print(f"Single agent control initialiation: {sum(inits[0])/len(inits[0])}")
print(f"Two agent control initialization: {sum(inits[1])/len(inits[1])}")
print(f"Three agent control initialization: {sum(inits[2])/len(inits[2])}")
print(f"Single agent control calculation average: {sum(row_vals[0])/len(row_vals[0])}")
print(f"Two agent control calculation average: {sum(row_vals[1])/len(row_vals[1])}")
print(f"Three agent control calculation average: {sum(row_vals[2])/len(row_vals[2])}")