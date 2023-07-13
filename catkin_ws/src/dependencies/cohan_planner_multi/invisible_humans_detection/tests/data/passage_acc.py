import yaml

with open('random_passage_data.yaml','r') as file:
  data = yaml.safe_load(file)

acc = sum(data['t_detect'])
acc2 =  sum(data['no_detect_lim'])

print("Random accuracy within limits {0:.2f} and with out of limits is {1:.2f}".format(acc, acc2))

with open('laas_passage_data.yaml','r') as file:
  data2 = yaml.safe_load(file)

acc = sum(data2['t_detect'])/50
acc2 = sum(data2['no_detect_lim'])/50

print("Laas accuracy within limits {0:.2f} and with out of limits is {1:.2f}".format(acc*100, acc2*100))

with open('bremen_passage_data.yaml','r') as file:
  data3 = yaml.safe_load(file)

acc = sum(data3['t_detect'])/50
acc2 = sum(data3['no_detect_lim'])/50

print("Bremen accuracy within limits {0:.2f} and with out of limits is {1:.2f}".format(acc*100, acc2*100))

acc = sum(data['t_detect']+data2['t_detect']+data3['t_detect'])/200 #len(data['detections'])
acc2 =  sum(data['no_detect_lim']+data2['no_detect_lim']+data3['no_detect_lim'])/200 #len(data['detections'])
#acc = acc + sum(data['f_detect']+data2['f_detect']+data3['f_detect'])/200 #len(data['detections'])
#acc = acc + sum(data['no_detect']+data2['no_detect']+data3['no_detect'])/200 #len(data['detections'])
#acc = sum(data['detections'])/100. #len(data['detections'])

print("Overall accuracy within limits {0:.2f} and with out of limits is {1:.2f}".format(acc*100, acc2*100))
