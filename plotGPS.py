import csv

with open('2_2_23_gps.csv', mode ='r') as f:
  csv_f = csv.reader(f)
  for line in csv_f:
        print(line, "\n")
