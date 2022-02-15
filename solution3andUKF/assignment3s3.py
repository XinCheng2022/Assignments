import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import csv
import math
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
from Scr.BicycleModel import KinematicBicycleModel



def main():
    with open('data_measurements.csv', 'r') as f:
    
        reader = csv.reader(f, delimiter=',')
        headers = next(reader)
        data = np.array(list(reader)).astype(float)
        delta = data[:, 2]
        v1r = data[:, 5]
        a1r = data[:, 6]
        theta1dot = data[:, 3]
        theta = data[:, 1]

    with open('data_ground_truth.csv', 'r') as f:
        reader = csv.reader(f, delimiter=',')
        headers = next(reader)
        data = np.array(list(reader)).astype(float)
        L2 = 8.3
        px = data[:, 1]
        py = data[:, 2]
        pxr = px - L2 * np.cos(data[:, 6] - data[:, 3])
        pyr = py - L2 * np.sin(data[:, 6] - data[:, 3])
    

    #f = 5
    #t = np.arange(0,1/f,dt)
    #delta = np.sin(2*np.pi*5*t)


    # delta1 = np.full(1000,np.deg2rad(3))
    # delta2 = np.full(2000,np.deg2rad(-3))
    # delta3 = np.zeros(2000)
    # delta4 = np.full(100,np.deg2rad(-30))
    # args = (delta1,delta2,delta3,delta4,delta3)

    deltatemp0 = np.zeros(1000)
    deltatmp1 = np.arange( 0, np.deg2rad(5),np.deg2rad(0.01))
    deltatemp2 = np.full(1000,np.deg2rad(5))
    deltatmp3 = np.arange(np.deg2rad(5), 0, np.deg2rad(-0.01))
    deltatemp4 = np.full(1000,np.deg2rad(20))
    deltatemp5 = np.zeros(1000)
    args = (deltatemp0,deltatmp1,deltatemp2,deltatmp3,deltatemp4, deltatemp5)

    #delta = np.concatenate(args)

    #v1r = 10
    #a1r = 0
    xp = 0
    yp = 0
    #theta = 0
    theta1 = 0
    theta2 = 0
    theta2Sim = 0
    car = KinematicBicycleModel()

    ## debug variables
    latP = 0
    longP = 0
    latR = 0
    longR = 0

    Theta = 0
    Theta11 = 0
    Theta22 = 0
    Theta11dot = 0
    Theta22dot = 0
    BetaPP = 0
    v1r11 = 0
    Theta2Sim = 0
    ##debug eng
    j=0
    for i in delta:
        
        if v1r[j] > 100  :
            xp, yp, xr, yr, theta1, theta2, v1rT, a1rT = car.kinematic_model( xp, yp, theta1, theta2, i, 100, a1r[j], theta1dot[j], theta[j])
        elif v1r[j] < -20: 
            xp, yp, xr, yr, theta1, theta2, v1rT, a1rT = car.kinematic_model( xp, yp, theta1, theta2, i, -20, a1r[j], theta1dot[j], theta[j])
        else:
            xp, yp, xr, yr, theta1, theta2, v1rT, a1rT = car.kinematic_model( xp, yp, theta1, theta2, i, v1r[j], a1r[j], theta1dot[j], theta[j])
        j += 1
        #debug info
        latP = np.append(latP,xp)
        longP = np.append(longP,yp)
        latR = np.append(latR,xr)
        longR = np.append(longR,yr)
        #Theta = np.append(Theta,theta)
        Theta11 = np.append(Theta11,theta1)
        Theta22 = np.append(Theta22,theta2)
        #Theta22Sim = np.append(Theta2Sim,theta2Sim)
        v1r11 = np.append(v1r11,a1r)
        #Theta11dot = np.append(Theta11dot,theta1dot)
        #Theta22dot = np.append(Theta22dot,theta2dot)
        #BetaPP = np.append(BetaPP,betaP)
    
    plt.scatter(latP[1:],longP[1:],s = 6,marker='o',label = 'simFront')
    plt.scatter(latR[1:],longR[1:],s = 6, marker='s',label = 'simRear') # squre is 2r
    plt.scatter(px,py,s = 2 ,label = 'GTFront')
    plt.scatter(pxr,pyr,s = 2 ,marker='s',label = 'GTREAR')
    #plt.plot(v1r11)
    #plt.plot(Theta11)
    # plt.plot(Theta22)
    # plt.plot(Theta22Sim,label='simplified')
    #plt.plot(Theta22dot)
    #plt.plot(Theta11dot)
    #plt.plot(BetaPP)
    #plt.plot(Theta)

    # plt.plot(data[:, 6]-data[:, 3], label= 'GT')
    # plt.plot(Theta22)
    # plt.scatter(px - L2 * np.cos(Theta22[1:]),py - L2 * np.sin(Theta22[1:]),s = 6,marker='o',label = 'simFront')
    # plt.scatter(px,py,s = 1 ,label = 'GTFront')
    # plt.scatter(pxr,pyr,s = 1 ,label = 'GTREAR')
    plt.legend()
    plt.show()
 

        
    
    

if __name__ == '__main__':
    main()
