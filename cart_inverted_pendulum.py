# -*- coding: utf-8 -*-
"""
Created on Sun Nov 23 12:59:21 2025

@author: Eric Nusser & Finn van Woensel
"""

import wis_2_2_utilities as util
import wis_2_2_systems as systems
#import random
import numpy as np
import plot_csv as plot

#set timestep
timestep = 4e-3


class controller():
    def __init__(self, target=0):
        self.target = target

        self.Integral1 = 0.0
        self.Integral2 = 0.0
        # PID gains for cart position
        #P1: -6225, I1: -100, D1: -38

        self.K_P1 = 200
        self.K_I1 = 100
        self.K_D1 = 500
        # PID gains for pendulum angle
        self.K_P2 = 1000
        self.K_I2 = 100
        self.K_D2 = 800
        
    def feedBack(self, observe):
        # update integral term
        self.Integral1 += observe[0] * timestep
        self.Integral2 += observe[2] * timestep

        # calculate feedback
        u1 = (
            self.K_P1 * observe[0]
            + self.K_I1 * self.Integral1
            + self.K_D1 * observe[1]
        )
        u2 = (
            self.K_P2 * observe[2]
            + self.K_I2 * self.Integral2
            + self.K_D2 * observe[3]
        )

        u = u1 + u2

        return u


def main():
  model=systems.cart_inverted_pendulum()
  control = controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  simulation.max_duration = 10 #seconds
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
  print('Simulation cost end: ', simulation.cost_input, simulation.cost_state)      
  simulation.writeData()
  plot.csv()
        

if __name__ == "__main__":
  main()