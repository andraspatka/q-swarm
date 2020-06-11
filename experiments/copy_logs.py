#!/usr/bin/env python3
import sys
import os

args = sys.argv

experiment_name = ""
r = ""
k = ""
c = ""

PATH_TO_LOGS = "logs/disease"
PATH_TO_DESTINATION = ""

if len(args) >= 4:
	experiment_name = args[1]
	r = args[2]
	k = args[3]
	PATH_TO_DESTINATION = f"results/covid/{experiment_name}/r_{r}_k_{k}"

if len(args) == 5:
	c = args[4]
	PATH_TO_DESTINATION = f"results/covid/{experiment_name}/r_{r}_k_{k}_c_{c}"
	

if not os.path.exists(PATH_TO_DESTINATION):
	os.mkdir(PATH_TO_DESTINATION)
	logs = f"{PATH_TO_DESTINATION}/logs"
	frames = f"{PATH_TO_DESTINATION}/frames"
	if not os.path.exists(logs):
		os.mkdir(logs)
	if not os.path.exists(frames):
		os.mkdir(frames)

print(PATH_TO_DESTINATION)

logs = os.listdir(PATH_TO_LOGS) 

for log in logs:
	os.rename(f"{PATH_TO_LOGS}/{log}", f"{PATH_TO_DESTINATION}/logs/{log}")
	



