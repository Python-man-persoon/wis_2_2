# -*- coding: utf-8 -*-
"""
Created on Mon Jul  3 14:33:49 2023

@author: Rik
"""

#_______
# DOOR MEET MODUS
#_______

import wis_2_2_utilities_nochrono as util
import wis_2_2_systems_nochrono as systems
#import random
import numpy as np

#set timestep
timestep = 2e-3

  
class pp_controller():
  def __init__(self, target=0):
    self.matrix_gain=np.array([[0, 0, 0, 0]])
    
  def feedBack(self, observe):
    u= -self.matrix_gain @ observe
    return u  
  
class controller():
  def __init__(self, target=0):
    pass
    
  def feedBack(self, observe):
    u=0
    return u


def main():
  model =systems.stacked_inverted_pendulum(
    state0 = [0, 0, 0, 0],
    u0 = 0.1
)
  control = controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  #simulation.max_duration = 600 #seconde
  simulation.data_mode = 'direct'
  simulation.GIF_toggle = False #set to false to avoid frame and GIF creation

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
        
  simulation.writeData()
        

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("stacked_inverted_pendulum.csv")


#angular velocity of the first pendulum
plt.plot(df.iloc[:, 0], df.iloc[:, 4]) 
plt.title('angular velocity of the first pendulum')
plt.xlabel("time")
plt.ylabel("angular velocity")
plt.show()


# angle of the first pendulum'
plt.plot(df.iloc[:, 0], df.iloc[:, 3])
plt.title('angle of the first pendulum')
plt.xlabel("time")
plt.ylabel("angle")
plt.show()
if __name__ == "__main__":
  main()




if __name__ == "__main__":
  main()