import numpy as np
from libs.normalise_angle import normalise_angle
import matplotlib.pyplot as plt
from scipy.optimize import fsolve


class KinematicBicycleModel():

    def __init__(self, L1: float=4.15, L2: float=8.3, B: float=0.35, max_steer: float=np.deg2rad(33), dt: float=0.01):
       
        self.dt = dt
        self.L1 = L1 
        self.L2 = L2
        self.B = B
        self.max_steer = max_steer

    def kinematic_model(self, xp: float, yp: float, vp: float, theta1: float, theta2: float, delta: float, v1r: float, a1r: float):
        
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        betaP = np.arctan( (self.B / self.L1) * np.tan(delta) )

        A = np.matrix([[1, 0, 0], [0, 1, 0], [0, self.dt * v1r / self.L2, 1 - self.dt * v1r / self.L2 ]] )
        B = np.matrix([[0, self.dt, 0], [self.dt * v1r / self.B, 0, 0], [self.dt * v1r / self.L2, 0, 0] ])
        C = A * np.array([ [vp],[theta1],[theta2] ]) + B * np.array([ [betaP],[a1r],[v1r] ])
        new_vp = float(C[0])
        new_theta1 = float(C[1])
        new_theta2 = float(C[2])
        new_xp = xp + new_vp * np.cos(theta1 + betaP) * self.dt
        new_yp = yp + new_vp * np.sin(theta1 + betaP) * self.dt
        new_xr = new_xp - self.L2 * np.cos(theta2)
        new_yr = new_yp - self.L2 * np.sin(theta2)
        new_theta1 = normalise_angle(theta1 + new_theta1 * self.dt )
        new_theta2 = normalise_angle(theta2 + new_theta2 * self.dt )

  

        return new_xp, new_yp, new_vp, new_xr, new_yr, new_theta1 ,new_theta2

def main():

    dt = 0.01
    vp = 0
    xp = 0
    yp = 0
    theta1 = 0
    theta2 = 0
    v1r = 0.5
    a1r = 0.2
    

    #f = 5
    #t = np.arange(0,1/f,dt)
    #delta = np.sin(2*np.pi*5*t)


    #delta1 = np.full(1000,np.deg2rad(2))
    #delta2 = np.full(2000,np.deg2rad(-3))
    #delta3 = np.zeros(2000)
    #delta4 = np.full(100,np.deg2rad(-30))
    #args = (delta1,delta2,delta3,delta4,delta3)
    #args = (delta1,delta1)

    deltatemp0 = np.zeros(1000)
    deltatmp1 = np.arange( 0, np.deg2rad(5),np.deg2rad(0.01))
    deltatemp2 = np.full(1000,np.deg2rad(5))
    deltatmp3 = np.arange(np.deg2rad(5), 0, np.deg2rad(-0.01))
    deltatemp4 = np.zeros(2000)
    args = (deltatemp0,deltatmp1,deltatemp2,deltatmp3,deltatemp4)

    #input v1r and delta    
    delta = np.concatenate(args)
 
    car = KinematicBicycleModel()
    car.dt = dt



    latP = 0
    longP = 0
    latR = 0
    longR = 0

    ## debug variables
    Theta = 0
    Theta11 = 0
    Theta22 = 0
    Theta11dot = 0
    Theta22dot = 0
    BetaPP = 0
    ##debug eng
    for i in delta:
        xp, yp, vp, xr, yr, theta1 ,theta2 = car.kinematic_model( xp, yp,vp, theta1, theta2, i, v1r, a1r)
        #debug info
        latP = np.append(latP,xp)
        longP = np.append(longP,yp)
        latR = np.append(latR,xr)
        longR = np.append(longR,yr)
        #Theta = np.append(Theta,theta)
        Theta11 = np.append(Theta11,theta1)
        Theta22 = np.append(Theta22,theta2)
        #Theta11dot = np.append(Theta11dot,theta1dot)
        #Theta22dot = np.append(Theta22dot,theta2dot)
        #BetaPP = np.append(BetaPP,betaP)
    
    plt.figure(0)
    plt.scatter(latP[1:],longP[1:],s = 6,marker='o', label = 'front')
    plt.scatter(latR[1:],longR[1:], alpha= 0.1, s = 2, marker='s', label = 'rear') # 
    plt.legend()

    plt.figure(1)
    plt.plot(Theta11, label = 'theta1')
    plt.plot(Theta22, label = 'theta2')
    #plt.plot(Theta22dot)
    #plt.plot(Theta11dot)
    #plt.plot(BetaPP)
    #plt.plot(Theta)
    plt.legend()
    plt.show()
 

if __name__ == '__main__':
    main()