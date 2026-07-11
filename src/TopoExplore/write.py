#!/usr/bin/python3.8
import os

file_path = "/home/oseasy/SMMR_topo_ws/src/SMMR-Explore-topoexplore/src/TopoExplore/data.txt"
if not os.path.exists(file_path):
    open(file_path, 'w').close()

for i in range(0,100):
    with open(file_path, 'a') as f:
         f.write("0\n")

