import numpy as np
from Scr.normalise_angle import normalise_angle
import matplotlib.pyplot as plt
import math
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
import csv

''' 
    x[0] new_xp
    x[1] new_yp
    x[2] new_xr
    x[3] new_yr
    x[4] new_theta1
    x[5] new_theta2
    x[6] new_v1r # err 0.05 
    x[7] new_a1r # err 0.02
    x[8] delta  # err 0.01
    X[9] theta  # err 0.01
'''

def transf(x, dt):
    L1 = 4.15
    L2 = 8.3
    B = 0.35
    theta1dot = x[6] * np.tan(x[8]) /L1
    theta2dot = (x[6]/L2) * (np.sin(x[9]) + np.cos(x[9]) * B * x[8] / L1 )
    F = np.array([x[0] + x[6] * np.cos(x[4]) * dt, 
                  x[1] + x[6] * np.sin(x[4]) * dt,
                  x[0] + x[6] * np.cos(x[4]) * dt - L2 * np.cos(x[5])   ,
                  x[1] + x[6] * np.sin(x[4]) * dt - L2 * np.sin(x[5])  ,
                  normalise_angle(x[4] + theta1dot * dt ), 
                  normalise_angle(x[5] + theta2dot * dt ), 
                  x[6] + x[7],
                  x[7],
                  x[8], 
                  x[4] - x[5]    ])
    return F.T
      
def hf(x):
    return np.array([x[6], x[7], x[8], x[9] ])

if __name__ == '__main__':
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
        deltaGT = data[:, 5]
        thetaGT = data[:, 3]
        theta1GT = data[:, 6]
        theta2GT = data[:, 6] -data[:, 3]
        v1rGT = np.multiply(data[:, 9] , np.cos(deltaGT)) 


    dt = 0.01
    points = MerweScaledSigmaPoints(10, alpha=0.001, beta=3.0, kappa=-1) # alpha

    ukf = UKF(dim_x = 10, dim_z = 4, fx = transf, hx = hf, dt = dt, points = points)

    ukf.x = np.array([0, 0, 0, 0,0,0,0,0,0,0])
    ukf.P *= 0.05
    ukf.R = np.diag([0.05, 0.02, 0.01, 0.01  ])
    

    M = np.matrix([
        [dt*dt/2 ,0.01 ],
        [ dt*dt/2, 0.01],
        [ dt*dt/2, 0.015],
        [ dt*dt/2, 0.015],
        [ dt, 0.01],
        [ dt, 0.01],
        [ dt, 0],
        [ 1,0 ],
        [ 0,1 ],
        [ dt ,0.001 ]
        ])
    q = np.array([[0.01, 0],[0, 0.01]])
    Q = M * q * M.T
    ukf.Q = Q * dt

    ukfv = [] 

 
    j = 0
    for i in delta:
        ukf.predict()
        ukf.update([v1r[j], a1r[j], i, theta[j] ])
        j += 1
        ukfv.append(ukf.x.copy())
    ukfv = np.asarray(ukfv)
    plt.figure(0)
    plt.scatter(ukfv[:,0], ukfv[:,1], s = 6,marker='o',label = 'ukf-Front')
    plt.scatter(ukfv[:,2], ukfv[:,3], s = 6,marker='s',label = 'ukf-Rear')
    plt.scatter(px,py,s = 2 ,label = 'GTFront')
    plt.scatter(pxr,pyr,s = 2 ,marker='s',label = 'GTREAR')
    plt.grid()
    plt.legend()
    plt.figure(1)
    plt.plot(ukfv[:,8], linewidth=3,alpha=0.9, label = 'ukf delta')
    plt.plot(delta, linewidth=1, alpha=0.3, label = 'Sensor delta')
    plt.plot(deltaGT, linewidth=2, alpha=1, label = 'GT delta')
    plt.legend()
    plt.figure(2)
    plt.plot(ukfv[:,6], linewidth=3,alpha=0.9, label = 'ukf v1r')
    plt.plot(v1r, linewidth=1, alpha=0.3, label = 'Sensor v1r')
    plt.plot(v1rGT, linewidth=2, alpha=1, label = 'GT v1r')
    plt.legend()
    plt.figure(3)
    plt.plot(ukfv[:,9], linewidth=3,alpha=0.9, label = 'ukf theta')
    plt.plot(theta, linewidth=1, alpha=0.3, label = 'Sensor theta')
    plt.plot(thetaGT, linewidth=2, alpha=1, label = 'GT theta')
    plt.legend()

    fig, axs = plt.subplots(2, 2)
    axs[0,0].plot (ukfv[:,0], linewidth=3,alpha=0.9, label = 'ukf xp')
    axs[0,0].plot(px, linewidth=2, alpha=1, label = 'GT xp')
    axs[0,0].legend(loc="upper left")
    axs[0,0].grid()
    axs[0,1].plot(ukfv[:,1], linewidth=3,alpha=0.9, label = 'ukf yp')
    axs[0,1].plot(py, linewidth=2, alpha=1, label = 'GT yp')
    axs[0,1].legend(loc="upper left")
    axs[0,1].grid()
    axs[1,0].plot(ukfv[:,2], linewidth=3,alpha=0.9, label = 'ukf xr')
    axs[1,0].plot(pxr, linewidth=2, alpha=1, label = 'GT xr')
    axs[1,0].legend(loc="upper left")
    axs[1,0].grid()
    axs[1,1].plot(ukfv[:,3], linewidth=3,alpha=0.9, label = 'ukf yr')
    axs[1,1].plot(pyr, linewidth=2, alpha=1, label = 'GT yr')
    plt.legend()
    plt.grid()



    plt.figure(6)
    plt.plot(ukfv[:,4], linewidth=3,alpha=0.9, label = 'ukf theta1')
    plt.plot(theta1GT, linewidth=2, alpha=1, label = 'GT theta1')
    plt.legend()

    plt.figure(7)
    plt.plot(ukfv[:,5], linewidth=3,alpha=0.9, label = 'ukf theta2')
    plt.plot(theta2GT, linewidth=2, alpha=1, label = 'GT theta2')
    plt.legend()


    plt.show()
'''
    x[0] new_xp
    x[1] new_yp
    x[2] new_xr
    x[3] new_yr
    x[4] new_theta1
    x[5] new_theta2
    x[6] new_v1r # err 0.05 
    x[7] new_a1r # err 0.02
    x[8] delta  # err 0.01
    X[9] theta  # err 0.01
'''
  