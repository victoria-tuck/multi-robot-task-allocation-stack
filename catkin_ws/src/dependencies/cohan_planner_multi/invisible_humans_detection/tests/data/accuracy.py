import yaml

with open('random_data.yaml','r') as file:
	data = yaml.safe_load(file)

total = sum(data['total'])
fp = sum(data['false_postives'])
overlap = sum(data['overlap'])
acc = (total - (fp+overlap))/ total
acc_wo_overlap = (total - fp)/ total

print("Random accuracy without overlap is {0:.2f} and with overlap is {1:.2f}".format(acc*100, acc_wo_overlap*100))

with open('laas_adream_data.yaml','r') as file:
	data2 = yaml.safe_load(file)

total = sum(data2['total'])
fp = sum(data2['false_postives'])
overlap = sum(data2['overlap'])
acc = (total - (fp+overlap))/ total
acc_wo_overlap = (total - fp)/ total

print("Laas accuracy without overlap is {0:.2f} and with overlap is {1:.2f}".format(acc*100, acc_wo_overlap*100))

with open('bremen_data.yaml','r') as file:
	data3 = yaml.safe_load(file)

total = sum(data3['total'])
fp = sum(data3['false_postives'])
overlap = sum(data3['overlap'])
acc = (total - (fp+overlap))/ total
acc_wo_overlap = (total - fp)/ total

print("Bremen accuracy without overlap is {0:.2f} and with overlap is {1:.2f}".format(acc*100, acc_wo_overlap*100))


# Over all calculation
total = sum(data['total']) + sum(data2['total']) + sum(data3['total'])
fp = sum(data['false_postives']) + sum(data2['false_postives']) + sum(data3['false_postives'])
overlap = sum(data['overlap']) + sum(data2['overlap']) + sum(data3['overlap'])

acc = (total - (fp+overlap))/ total
acc_wo_overlap = (total - fp)/ total

print("Overall accuracy without overlap is {0:.2f} and with overlap is {1:.2f}".format(acc*100, acc_wo_overlap*100))
