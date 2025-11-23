# -*- coding: utf-8 -*-
"""
Created on Sun Nov 23 12:59:21 2025

@author: ericn
"""

import wis_2_2_utilities as util
import wis_2_2_systems as systems
#import random
import numpy as np

#set timestep
timestep = 2e-3


class controller():
    def __init__(self, target=0):
        self.target = target
        self.Integral = 0.0
        self.K_P = 0
        self.K_I = 0
        self.K_D = 10
        
    def feedBack(self, observe):
        # update integral term
        self.Integral += observe[0] * timestep

        # calculate feedback
        u = (
            self.K_P * observe[0]
            + self.K_I * self.Integral
            + self.K_D * observe[1]
        )

        return u


def main():
  model=systems.cart_inverted_pendulum()
  control = controller()
  simulation = util.simulation(model=model,timestep=timestep)
  simulation.setCost()
  #simulation.max_duration = 600 #seconde
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
        
  




if __name__ == "__main__":
  main()