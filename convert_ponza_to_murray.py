import pandas as pd
import math
import os


def euclidian_dist(a, b):
    return math.sqrt( (pow(a[0]-b[0],2)) + (pow(a[1]-b[1],2)))


types = ['doublecenter', 'restricted','singlecenter', 'uniform']
nodes = [5,6,7,8,9,10,20,50,75,100,175,250 ]
variation = ['', 'alpha_1', 'alpha_3']

path =  "problems/ponza16/uniform/";
#name  = "uniform-alpha_1-51-n10"

files = os.listdir(path)

print(files)

for file in files:

    print(file)  
    dir_name = path + file[:-4]
    
    if file.find('txt') == -1:
        continue
    
    with open(path + file) as f:
        lines = f.readlines()
    
    truckSpeed = lines[1]
    droneSpeed = lines[3]
    nNodes = int(lines[5])
    
    try: 
        os.mkdir(dir_name) 
    except OSError as error: 
        print("skip directory creation")
    
    nodes = []
        
    ####  Creating Nodes.csv  ####
    f = open(dir_name + '/nodes.csv', "w")
      
    depot = lines[7].split()
    nodes.append(depot)
    
    f.write('0,' + depot[0] + ',' + depot[1] + ',' + droneSpeed )
    
    for row in range(nNodes-1):
        costumer = lines[row+9].split()
        nodes.append(costumer)    
        f.write(str(row+1) + ',' + costumer[0] + ',' + costumer[1] + ',0\n')
        
    f.write(str(nNodes) + ',' + depot[0] + ',' + depot[1] + ',' + '0' )
    f.close()
    
    nodes.append(depot)  
    nodes.reverse()
    nodes =  [[float(x[0]), float(x[1])] for x in nodes]
    
    
    ####  Creating tau.csv  ####
    f = open(dir_name + '/tau.csv', "w")
    fp = open(dir_name + '/tauprime.csv', "w")
    
    truck_distance = "euclidian" #manhatam
    drone_distance = "euclidian" #manhatam
    
    for i in range(nNodes):
        for j in range(nNodes):
            if (truck_distance == "euclidian"):
                f.write( str(euclidian_dist(nodes[i], nodes[j])/float(truckSpeed)) + ',')
            else:
                f.write( str((abs(nodes[i][0]-nodes[j][0]) + abs(nodes[i][1]-nodes[j][1]))/float(truckSpeed)) + ',')
        
            if drone_distance == "euclidian": #manhatam    
                fp.write( str(euclidian_dist(nodes[i], nodes[j])/float(droneSpeed)) + ',') 
            else:
                fp.write( str((abs(nodes[i][0]-nodes[j][0]) + abs(nodes[i][1]-nodes[j][1]))/float(droneSpeed)) + ',') 
                
        
        if (truck_distance == "euclidian"):
            f.write( str(euclidian_dist(nodes[i], nodes[nNodes])/float(truckSpeed)) + '\n')
        else:
            f.write( str((abs(nodes[i][0]-nodes[nNodes][0]) + abs(nodes[i][1]-nodes[nNodes][1]))/float(truckSpeed)) + '\n')
            
        if drone_distance == "euclidian":
            fp.write( str(euclidian_dist(nodes[i], nodes[nNodes])/float(droneSpeed)) + '\n')
        else:
            fp.write( str((abs(nodes[i][0]-nodes[nNodes][0]) + abs(nodes[i][1]-nodes[nNodes][1]))/float(droneSpeed)) + '\n')
     
    for j in range(nNodes):
         f.write( '0,')
         fp.write( '0,')
    
    f.write( '0')
    f.close()
    fp.write( '0')
    fp.close()


    
        


