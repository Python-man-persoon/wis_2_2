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
import plot_csv
import scipy.linalg

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

def calculate_A():
  epsilon = 1e-6
  n_states = 4
  A = np.zeros((n_states, n_states))
  for i in range(n_states):
    # Create a state where only state 'i' is slightly offset
    x_before = np.zeros(n_states)
    x_before[i] = epsilon
    
    model = systems.stacked_inverted_pendulum(state0=x_before, u0=0)

    model.updateSystem(timestep)
    x_after = model.getState() 
    
    # Calculate the derivative for this column
    # (Difference in state) / (time * perturbation size)
    A[:, i] = (x_after - x_before) / (timestep * epsilon)
  return A

def calculate_B():
  epsilon = 1e-6
  n_states = 4
  A = np.zeros((n_states, n_states))
  B = np.zeros((n_states, 1)) # Assuming 1 input (the motor)
  # Fresh model for B: start at zero state, but give a small nudge to input u
  model_b = systems.stacked_inverted_pendulum(state0=[0,0,0,0], u0=epsilon)
  model_b.updateSystem(timestep)
  x_after_b = model_b.getState()
  
  # B = (change in state) / (time * input_size)
  # Note: x_before here is just [0,0,0,0]
  B[:, 0] = (x_after_b - np.zeros(n_states)) / (timestep * epsilon)  
  return B

A = calculate_A()
B = calculate_B()
print("Matrix A:")
print(A)
print("Eigenvalue A:")
print(np.linalg.eigvals(A))
Co = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B])
rank = np.linalg.matrix_rank(Co)
print("Rank of Controllability Matrix:", rank) # Should be 4 for full controllability


def main():

  model =systems.stacked_inverted_pendulum()
  control = controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  simulation.max_duration = 10 #seconde
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
  plot_csv.csv()


if __name__ == "__main__":
  main()