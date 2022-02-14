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
    
    vp = 0
    betaP = 0
    theta1Dot = 0
    theta1 = 0
    theta2 = 0

    def baseCal(self, delta, v1r):

        delta = np.clip(delta, -self.max_steer, self.max_steer)
        self.vp = v1r * np.sqrt( np.power(( self.B * np.tan(delta)/ self.L1 ), 2 )   + 1)
        self.betaP = np.arctan( (self.B / self.L1) * np.tan(delta) )
        self.theta1Dot = v1r * np.tan(delta) / self.L1

        

    def f_theta2 (self, theta2Dot):
        return self.vp * np.sin( self.theta1 - self.theta2 + self.betaP) / self.L2 - theta2Dot

    def kinematic_model(self, xp: float, yp: float, theta: float, theta1: float, theta2: float, delta: float, v1r: float):


        self.baseCal( delta, v1r)
        theta2Dot = fsolve(self.f_theta2,1)



        # Compute the final state using the discrete time model
        new_v1r = v1r
        #new_xp = xp + self.vp * np.cos(theta1 + self.betaP) * self.dt
        #new_yp = yp + self.vp * np.sin(theta1 + self.betaP) * self.dt
        new_xp = xp + v1r * np.cos(theta1 ) * self.dt
        new_yp = yp + v1r * np.sin(theta1 ) * self.dt
        new_xr = new_xp - self.L2 * np.cos(theta2)
        new_yr = new_yp - self.L2 * np.sin(theta2)
        new_theta1 = normalise_angle(theta1 + self.theta1Dot * self.dt )
        new_theta2 = normalise_angle(theta2 + theta2Dot * self.dt )
        new_theta =  new_theta1- new_theta2 
        self.theta1 = new_theta1
        self.theta2 = new_theta2

        return new_xp, new_yp,new_theta, new_theta1, new_theta2 ,new_v1r, new_xr,new_yr, self.theta1Dot, theta2Dot, self.betaP

def main():

    dt = 0.01

    
    

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
    deltatemp4 = np.zeros(2000)
    args = (deltatemp0,deltatmp1,deltatemp2,deltatmp3,deltatemp4)

    #input v1r and delta    
    delta = np.concatenate(args)
    v1r = 10

    car = KinematicBicycleModel()

    #ini for state vector
    xp = 0
    yp = 0
    theta = 0
    theta1 = 0
    theta2 = 0
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
        xp, yp, theta, theta1, theta2, v1r, xr, yr,theta1dot, theta2dot, betaP = car.kinematic_model( xp, yp, theta, theta1, theta2, i, v1r)
        #debug info
        latP = np.append(latP,xp)
        longP = np.append(longP,yp)
        latR = np.append(latR,xr)
        longR = np.append(longR,yr)
        Theta = np.append(Theta,theta)
        Theta11 = np.append(Theta11,theta1)
        Theta22 = np.append(Theta22,theta2)
        Theta11dot = np.append(Theta11dot,theta1dot)
        Theta22dot = np.append(Theta22dot,theta2dot)
        BetaPP = np.append(BetaPP,betaP)
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