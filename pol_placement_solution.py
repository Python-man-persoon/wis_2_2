# -*- coding: utf-8 -*-
"""
Created on Fri Nov 28 12:54:50 2025

@author: ericn
"""

#import systems
import wis_2_2_utilities as util
import wis_2_2_systems as systems
import plot_csv as plot

import control as ct
import numpy as np
import math

#set timestep
timestep = 1e-3

g = 9.81

#pendulum 
rho_wood = 700 #kg/m3
l = 0.6 #pendulum1_length
d = 0.2
mp=rho_wood * l * d**2
print(mp)

#flywheel 
rho_iron = 7870 #kg/m3
flywheel_radius = 0.1
flywheel_thickness = 0.01 
mc = rho_iron*(math.pi * flywheel_radius**2 * flywheel_thickness)
Ic = 0.5 * mc * flywheel_radius**2
print (mc)

#hoekversenelling
Icm = (1/12) * mp* (d**2 + l**2)
I = Icm + (mp*(l/2)**2)
Icyl = (1/2) * mc * flywheel_radius**2
print (Icyl)
print (Icm)
print (I)

#termen
T1 = (I + mc *(l**2))
T2 = ((mp * g * l)/(2 * (T1))) + ((mc * g * l)/(T1))

matrix_A = np.array([[0,1,0,0],
                     [0, 0, -T2, 0],
                     [0,0,0,1],
                     [0, 0, T2, 0]])

matrix_B = np.array([[0],[(1/Icyl)],[0],[(-1/T1)]])

#print('eigenvalues of A are:')
#print(np.linalg.eigvals(matrix_A))



#matrix_K = ct.place(matrix_A,matrix_B,list_poles)
#print(' ')
#print('Gain matrix K is:')
#print(matrix_K)
#print(' ')
#print('eigenvalues of (A-BK) are:')
#print(np.linalg.eigvals((matrix_A-matrix_B @ matrix_K)))


class controller():
    def __init__(self, A, B, poles, target=0):
        #self.matrix_gain=np.array([[-1.25031373e+01, -4.34851760e+00, -5.91529008e+03, -1.19596823e+03]])
        self.matrix_gain= ct.place(A, B, poles)
        print(f'Gain matrix: {self.matrix_gain}')
    def feedBack(self, observe): 
        u= -self.matrix_gain @ observe
        return u
    
def main():
  model=systems.flywheel_inverted_pendulum()
  poles = [complex(-4, 0.1), complex(-4, -0.1), -15, -20]
  control = controller(matrix_A, matrix_B, poles)
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  simulation.max_duration = 15 #seconds
  simulation.GIF_toggle = True #set to false to avoid frame and GIF creation

  while simulation.vis.Run():
      if simulation.time<simulation.max_duration:
        simulation.step()
        u = control.feedBack(simulation.observe())
        simulation.control(u)
        simulation.log()
        simulation.refreshTime()
      else:
        print('Ending visualisation...')
        simulation.vis.GetDevice().closeDevice()
  print('Simulation cost end: ', simulation.cost_input, simulation.cost_state)      
  simulation.writeData()
  #plot.csv()
        

if __name__ == "__main__":
  main()