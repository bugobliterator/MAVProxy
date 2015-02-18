import logging
import os
import os.path
import threading
import types
import numpy
import math,random

def ellipsoid_fitness(param,fi,p):
    a = param[0]
    b = param[1]
    c = param[2]
    o1 = param[3]
    o2 = param[4]
    o3 = param[5]
    D = numpy.array([[a,0.,0.],
                    [0.,b,0.],
                    [0.,0.,c]],dtype = float)
    U = numpy.array([o1,o2,o3],dtype=float)
    s=0
    for i in range(999):
      val = numpy.dot(numpy.dot((data[i]-U).T,D),(data[i]-U))
      s = s+math.pow((1.-val),2)
    return s

def sphere_fitness(data,param):
    fi = numpy.array([0 for i in range(100)],dtype=float)[numpy.newaxis]
    a = 1/math.pow(param[0,0],2)
    o1 = -param[0,1]
    o2 = -param[0,2]
    o3 = -param[0,3]
    D = numpy.array([[a,0.,0.],
                    [0.,a,0.],
                    [0.,0.,a]],dtype = float)
    U = numpy.array([o1,o2,o3],dtype=float)[numpy.newaxis]
    s=0
    for i in range(100):
      #val = numpy.dot(numpy.dot((data[:,i]-U),D),(data[:,i]-U).T)
      fi[0,i] = 1 - a*((data[0,i,0] - o1)*(data[0,i,0] - o1) +
                       (data[0,i,1] - o2)*(data[0,i,1] - o2) +
                        (data[0,i,2] - o3)*(data[0,i,2] - o3))
    return fi

def calc_jacobian(data,param):
    
    J = numpy.zeros(shape=(4,100),dtype=float)
    cur = sphere_fitness(data,param)
    for i in range(4):
        param[0,i] = param[0,i] + 0.000000001
        J[i] = cur - sphere_fitness(data,param)
        param[0,i] = param[0,i] - 0.000000001
        
    return J
    
def generate_sqsum(fi):
    s=0
    for i in range(100):
       s = s+math.pow(fi[0,i],2)
    return s
     
def evaluatelm(param,data,goal):
    I = numpy.matrix(numpy.identity(4), copy=False)
    l = 1
    best_past_fitness = generate_sqsum(sphere_fitness(data,param))
    best_param = param
    for i in range(20):
        past_fitness = generate_sqsum(sphere_fitness(data,param))
        J = calc_jacobian(data,param)
        JTJ = numpy.dot(J,J.T)
        JTJ = JTJ + l*I
        JTFI = numpy.dot(J,sphere_fitness(data,param).T)
        dparam = numpy.dot(JTFI.T,numpy.linalg.inv(JTJ.T))
        param = param + dparam[0,:]
        if best_past_fitness < 1:
            break
        if generate_sqsum(sphere_fitness(data,param)) < past_fitness:
            l = l/10.
        if generate_sqsum(sphere_fitness(data,param)) > past_fitness:
            l = l*10.
        if best_past_fitness > generate_sqsum(sphere_fitness(data,param)):
            best_param = param
            best_past_fitness = generate_sqsum(sphere_fitness(data,param))
    return best_param,best_past_fitness

def magcalib(param,data):    
    print 'Processing, Please Wait     '
    m=1
    best_past_fitness = generate_sqsum(sphere_fitness(data,param))
    best_past_param = param
    sat = 0
    while(best_past_fitness > 1 and sat < 3):
        param,cur_fitness = evaluatelm(param,data,m)
        if cur_fitness == best_past_fitness:
            sat += 1
        else:
            sat = 0
        for j in range(4):
            if param[0,j] > 1000 or param[0,j] < -1000:
                return best_past_param,best_past_fitness
        best_past_param = param
        best_past_fitness = cur_fitness
        print best_past_param,best_past_fitness
    return best_past_param,best_past_fitness