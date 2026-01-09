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
import numpy as np
import plot_csv
from scipy import signal
import control as ct
import matplotlib.pyplot as plt

#set timestep
timestep = 2e-3


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
eigvals_A = np.linalg.eigvals(A)
print("Eigenvalue A:")
print(eigvals_A)
print("Matrix B:")
print(B)
# Controllability check
Con = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B])
rank = np.linalg.matrix_rank(Con)
print("\nControllability Matrix:", Con)
print("Rank of Controllability Matrix:", rank) # Should be 4 for full controllability

#Pendulum 1 angle & Pendulum 2 angle
C1 = np.array([
  [1,0,0,0],
  [0,0,1,0]
])
# Extra sensor
C2 = np.array([
  [1,1,0,0],
  [0,0,1,0]
])

# Gain matrix from pole placement
target_poles = np.array([-30,-11,-30,-12])
placement = signal.place_poles(A.T, C1.T, target_poles)
L = placement.gain_matrix.T
print("Gain matrix L from pole placement:")
print(L)
check_poles = np.linalg.eigvals(A - L @ C1)
print("\nActual observer poles:", check_poles)

Q = np.diag([1,1,1,1])
R = np.array([[0.1]])
K = ct.lqr(A, B, Q, R)[0]
print("LQR Gain matrix K:")
print(K)
  
class LQR_controller():
  def __init__(self, target=0):
    self.matrix_gain = K
    pass
    
  def feedBack(self, observe):
    u= -self.matrix_gain @ observe
    return u

class LQR_observer_controller():
  def __init__(self, A, B, C, L, K, dt, target=0):
    self.A = A
    self.B = B
    self.C = C
    self.L = L
    self.K = K
    self.dt = dt

    self.x_hat = np.zeros((4,1)) # Initial estimate
    self.u_prev = 0.0

  def feedBack(self, observe):
    y_raw = np.array(observe)
    y = np.array([[y_raw[0]], [y_raw[2]]])  # Change this according to C matrix

    prediction = self.A @ self.x_hat + self.B * self.u_prev
    innovation = y - (self.C @ self.x_hat)
    corrention = self.L @ innovation

    x_hat_dot = prediction + corrention
    self.x_hat += x_hat_dot * self.dt

    u_matrix = -self.K @ self.x_hat
    u = float(u_matrix.item())
    self.u_prev = u
    return u

class controller_no_control:
  def __init__(self, A, B, C, L, K, dt, target=0):
    self.A = A
    self.B = B
    self.C = C
    self.L = L
    self.K = K
    self.dt = dt

    self.x_hat = np.zeros((4,1)) # Initial estimate
    self.u_prev = 0.0

  def feedBack(self, observe):
    y_raw = np.array(observe)
    y = np.array([[y_raw[0]], [y_raw[2]]])  # Change this according to C matrix

    prediction = self.A @ self.x_hat + self.B * self.u_prev
    innovation = y - (self.C @ self.x_hat)
    corrention = self.L @ innovation

    x_hat_dot = prediction + corrention
    self.x_hat += x_hat_dot * self.dt
    u=0
    return u


def main():

  model =systems.stacked_inverted_pendulum(high_kick=1.5)
  #control = LQR_observer_controller(A, B, C1, L, K, timestep)
  control = controller_no_control(A, B, C1, L, K, timestep)
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

        real_state = simulation.model.getState()
        real_angle1_history.append(real_state[0])
        real_angle2_history.append(real_state[2])
        est_angle1_history.append(control.x_hat[0].item())
        est_angle2_history.append(control.x_hat[2].item())
        time_history.append(simulation.time)

      else:
        print('Ending visualisation...')
        simulation.vis.GetDevice().closeDevice()
        
  simulation.writeData()
  plot_csv.csv()

  plt.figure(figsize=(10, 6))
  plt.plot(time_history, real_angle1_history, label='Real Angle 1 (Sensor)', color='blue', linewidth=2)
  plt.plot(time_history, est_angle1_history, label='Observer Estimate 1', color='orange', linestyle='--')
  plt.plot(time_history, real_angle2_history, label='Real Angle 2 (Sensor)', color='green', linewidth=2)
  plt.plot(time_history, est_angle2_history, label='Observer Estimate 2', color='red', linestyle='--')
  plt.title('Observer Performance: Real vs. Estimated Pendulum Angles')
  plt.xlabel('Time (s)')
  plt.ylabel('Angle (rad)')
  plt.legend()
  plt.grid(True)
  plt.show()



time_history = []
real_angle1_history = []
real_angle2_history = []
est_angle1_history = []
est_angle2_history = []



if __name__ == "__main__":
  main()