# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 08:55:34 2021

@author: andre
"""

import pandas as pd
import math
import os

path =  "problems/ponza16/uniform/";
files = os.listdir(path)

distribution = 'uniform'
alpha = ''
nodes = '10'

problems = []

for file in files:
    
    if file.find(distribution) != -1 and file.find(alpha) != -1 and file.find('n' + nodes) != -1:
        if file.find('txt') == -1 and file.find('n' + nodes + '0') == -1:
            problems.append(file)
            
print (problems)