# -*- coding: utf-8 -*-
"""
Created on Mon Jul  3 14:32:57 2023

@author: Rik
"""
import numpy as np
from scipy.integrate import solve_ivp

# import pychrono as chrono
# pychrono.irrlicht as irr
import math as m
#from PIL import Image, ImageFile
#ImageFile.LOAD_TRUNCATED_IMAGES = True
#import os
#import glob
#import shutil

#repeated parameter
rho_wood = 700 #kg/m3
rho_iron = 7870 #kg/m3
g = 9.81

###############################################################################


def shiftAngle(angle):
    return (angle + m.pi) % (2 * m.pi) - m.pi

###############################################################################      
      
class mass_spring():
    def __init__(self,body_size1=0.2,spring_coef1=100,rest_length1=0,damping_coef1=0.0, second_body=False, body_size2=0.2,spring_coef2=10,rest_length2=0,damping_coef2=0.0):
        self.title = ('mass_spring_system')
        
        # set constants
        
        self.body_1_size = body_size1
        m1 = body_size1**3*rho_wood
        self.body_1_Pos = -1
        
        #actuatior on body_1
        self.u1 = np.array([[0]])
        
        k1 = spring_coef1
        self.rest_length1 = rest_length1 #not implemented
        c1 = damping_coef1
        
        self.second_body = second_body
        
        self.body_2_size = body_size2
        m2 = body_size2**3*rho_wood
        self.body_2_Pos = -2     
        k2 = spring_coef2
        self.rest_length2 = rest_length2 #not implemented
        c2 = damping_coef2
        
        self.position = 0
        self.velocity = 0
        
        self.state = [self.body_1_Pos,0]
        
        
        self.A = np.array([[0,1],[-k1/m1,-c1/m1]])
        self.B = np.array([[0],[1/m1]])
        
        if self.second_body:
          self.position2 = 0
          self.velocity2 = 0
          
          self.state = [self.body_1_Pos,0,self.body_2_Pos,0]
          
          self.A = np.array([[0,1,0,0],
                            [-(k1+k2)/m1,-(c1+c2)/m1,k2/m1,c2/m2],
                            [0,0,0,1],
                            [k2/m2,c2/m2,-k2/m2,-c2/m2]])
          self.B = np.array([[0],[1/m1],[0],[0]])
            
    def getState(self):
        #return the current state
        if self.second_body:
          return np.array([self.position,self.velocity,self.position2,self.velocity2])
        else:
          return np.array([self.position,self.velocity])
      
    def updateSystem(self, timestep = 1e-2):
        self.DoStepDynamics(timestep)
        self.position = self.state[0]
        self.velocity = self.state[1]
        if self.second_body:
          self.position2 = self.state[2]
          self.velocity2 = self.state[3]
          
        
        
    def updateActuator1(self, input1):
        #update actuator1
        self.u1[0,0] = input1
      
    def updateActuator2(self, input2):
        #update actuator2
        pass
    
    def dydx(self,time,state):
        state = np.array([state]).T
        dot_state = self.A @ state + self.B @ self.u1
        return dot_state.flatten().tolist()
    
    def DoStepDynamics(self, timestep = 1e-2):
        step_data = solve_ivp(self.dydx ,[0,timestep], self.state, t_eval=[timestep])
        self.state = step_data.y.flatten().tolist()
        
    
    

###############################################################################

class rotating_mass_spring():
    def __init__(self,body_size=0.3,spring_coef=50,rest_length=0,damping_coef=0.01,constant_rot = 0.0, prop_rot = 3.0, adjust = 0.1, rotor_length = 3):
        self.title = ('rotating_mass_spring_system')

        self.constant_factor = constant_rot
        self.proportional_factor = prop_rot        
        
        # set constants
        
        self.body_size = body_size
        self.mass = body_size**3*rho_wood
        self.body_Pos = -1
        
        #actuatior on body_1
        self.u1 = 0
        
        self.k1 = spring_coef
        self.rest_length1 = rest_length #not implemented
        self.c1 = damping_coef
        
        if prop_rot != 0:
            start_pos = self.k1/(self.mass*prop_rot**2)+adjust
        else:
            start_pos = adjust
        
        
        self.position = 0
        self.velocity = 0
        
        self.state = [start_pos,0]
        
        
            
    def getState(self):
        #return the current state
        
        return np.array([self.position,self.velocity])
      
    def updateSystem(self, timestep = 1e-2):
        self.DoStepDynamics(timestep)
        self.position = self.state[0]
        self.velocity = self.state[1]
          
        
        
    def updateActuator1(self, input1):
        #update actuator1
        self.u1 = input1
      
    def updateActuator2(self, input2):
        #update actuator2
        pass
    
    def dydx(self,time,state):
        dot_state = [state[1],self.u1/self.mass -self.c1/self.mass*state[1] - self.k1/self.mass*state[0] + (self.constant_factor**2+self.proportional_factor**2*state[0]*2)*state[0]]
        return dot_state
    
    def DoStepDynamics(self, timestep = 1e-2):
        step_data = solve_ivp(self.dydx ,[0,timestep], self.state, t_eval=[timestep])
        self.state = step_data.y.flatten().tolist()
        
            
    
# ###############################################################################

class balance_ball():
  def __init__(self,control_mode=3, start_pos = 1/32):
    print('model not implemented without pychrono, expect error')
#          self.system = chrono.ChSystemNSC()
#          self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
#          self.title = ('balance_ball_system')
         
#          # Set gravity
#          self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
#          self.camara_vector = chrono.ChVectorD(-0.8, 1.2, 1.5)

#          # Set the simulation parameters
#          self.beam_length = 1.0
#          self.beam_thickness = 0.05
#          self.ball_radius = 0.1
#          self.friction_coefficient = 1.0

#          self.control_mode = control_mode
#          self.starting_pos_x = self.beam_length * start_pos

#          # Create a material with the desired friction coefficient
#          self.material = chrono.ChMaterialSurfaceNSC()
#          self.material.SetFriction(self.friction_coefficient)

#          # Create the ground body
#          self.ground = chrono.ChBody()
#          self.ground.SetBodyFixed(True)
#          self.system.Add(self.ground)

#          # Create the beam
#          self.beam = chrono.ChBodyEasyBox(self.beam_thickness, self.beam_length, self.beam_thickness, rho_wood, True, True, self.material)
#          self.beam.SetPos(chrono.ChVectorD(0, 0, 0))
#          self.beam.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))
#          self.system.Add(self.beam)

#          # Create the ball
#          self.y_offset = self.ball_radius+self.beam_thickness / 2
#          self.body_1 = chrono.ChBodyEasySphere(self.ball_radius, rho_wood, True, True, self.material)
#          self.body_1.SetPos(chrono.ChVectorD(self.starting_pos_x, self.y_offset , 0))
#          self.body_1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
#          self.system.Add(self.body_1)

#          # Create a revolute joint between the beam and the ground
#          self.revolute = chrono.ChLinkLockRevolute()
#          self.revolute.Initialize(self.beam, self.ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))))
#          self.system.AddLink(self.revolute)

#          # Create the motor
#          positionA1 = chrono.ChVectorD(0, 0, 0) 
#          if self.control_mode == 1:
#            self.rotmotor1 = chrono.ChLinkMotorRotationAngle()
#          elif self.control_mode == 2:
#            self.rotmotor1 = chrono.ChLinkMotorRotationSpeed()
#          else:
#            self.rotmotor1 = chrono.ChLinkMotorRotationTorque()

#          # Connect the rotor and the stator and add the motor to the sys:
#          self.rotmotor1.Initialize(self.ground,                # body A (slave)
#                                self.beam,               # body B (master)
#                                chrono.ChFrameD(positionA1)  # motor frame, in abs. coords
#          )
#          self.system.Add(self.rotmotor1)
         
#      def getState(self):
#          #return the current state
#          try: 
#            return np.array([self.position,self.velocity,self.beam_angle,self.beam_angle_vel])
#          except:
#            return np.array([0,0,0,0])
       
#      def updateSystem(self, timestep = 1e-2):
#          #Do step
#          self.system.DoStepDynamics(timestep)
         
#          #fetch current angle
#          self.angle_x = shiftAngle(self.revolute.GetRelAngle())
         
#          #calculate state
#          self.position = self.body_1.GetPos().x/m.cos(self.angle_x)
#          self.velocity = self.body_1.GetPos_dt().x/m.cos(self.angle_x)
#          self.beam_angle = self.angle_x
#          self.beam_angle_vel = self.revolute.GetRelWvel().z
       
#      def updateActuator1(self, input1):
#          #update actuator1
#          f_input1 = chrono.ChFunction_Const(input1)
#          if self.control_mode == 1:
#            self.rotmotor1.SetAngleFunction(f_input1)
#          elif self.control_mode == 2:
#            self.rotmotor1.SetSpeedFunction(f_input1)
#          else:
#            self.rotmotor1.SetMotorFunction(f_input1)
           
#      def updateActuator2(self, input2):
#          #update actuator2
#          pass       
       
# ###############################################################################

class cart_inverted_pendulum():
      def __init__(self,second_pendulum = False, pendulum1_length = 0.6, pendulum2_length = 0.6, high_kick = 2.0, state0 = None, u0 = 0):
          self.title = ('cart_inverted_pendulum')
         

          self.second_pendulum = second_pendulum
          self.pendulum1_angle = 0.0
          self.pendulum2_angle = 0.0
          
          
          
          self.l = pendulum1_length
          d = 0.02
          self.m_p = rho_wood*d**2*self.l
          self.I_cm = self.m_p*(d**2+self.l**2)/12
          self.I_ep = self.I_cm + self.m_p*self.l**2/4 
          
          self.l2 = pendulum2_length
          self.m_p2 = rho_wood*d**2*self.l2
          self.I_cm2 = self.m_p2*(d**2+self.l2**2)/12
          self.I_ep2 = self.I_cm2 + self.m_p2*self.l2**2/4 
          
          self.u1 = u0
          self.m_c = rho_wood*0.2**2*0.4
          
          
          
      
          if state0 is None:          
            self.state = [0, high_kick, 0, -2*high_kick/self.l]
            
            if self.second_pendulum:
              # Set up second pendulum
              self.state = [0, high_kick, 0, -2*high_kick/self.l,0,0]
              

          else:
            self.state = state0
            
        
          

          

          

          # initiate state variables
          self.cart_position = 0. 
          self.cart_velocity = 0.
         
          self.pend1_angle = 0.
          self.pend1_angle_vel = 0. 
         
          if self.second_pendulum!=False:
            self.pend2_angle = 0.
            self.pend2_angle_vel = 0. 
            
            b1,d1,l1,rho1, b2,d2,l2,rho2, m0 = [d,d,self.l,rho_wood,d,d,self.l2,rho_wood, self.m_c]
            
            m1 = rho1*b1*d1*l1
            I1 = (1/12)*m1*l1**2
            lc1 = l1/2

            m2 = rho2*b2*d2*l2
            I2 = (1/12)*m2*l2**2
            lc2 = l2/2

            self.p = np.array([m0, m1, I1, lc1, m2, I2, lc2])
            
            self.M_abs_from_local = (np.array([[1,0,0,0,0,0],
                                              [0,1,0,0,0,0],
                                              [0,0,1,0,0,0],
                                              [0,0,0,1,0,0],
                                              [0,0,1,0,1,0],
                                              [0,0,0,1,0,1]]))
            
            self.M_local_from_abs = np.linalg.inv(self.M_abs_from_local)
            
            
      def getState(self):
          #return the current state
          if self.second_pendulum != False:
            state = np.array([ self.cart_position,
                              self.cart_velocity,
                              self.pend1_angle,
                              self.pend1_angle_vel,
                              self.pend2_angle,
                              self.pend2_angle_vel])
          else:
            state = np.array([ self.cart_position,
                              self.cart_velocity,
                              self.pend1_angle,
                              self.pend1_angle_vel])
          return state
       
      def updateSystem(self, timestep = 1e-2):
          #Do step
          self.DoStepDynamics(timestep)
         
          #calculate state
          self.cart_position = self.state[0]
          self.cart_velocity = self.state[1]
          self.pend1_angle = shiftAngle(self.state[2])
          self.pend1_angle_vel = self.state[3]
          if self.second_pendulum:
            self.pend2_angle = shiftAngle(self.state[4])
            self.pend2_angle_vel = self.state[5]

          
      def updateActuator1(self, input1):
          #update actuator1
          try:
            self.u1 = input1[0]
          except:
            self.u1 = input1
          
            
      def updateActuator2(self, input2):
          #update actuator2
          pass    
        
      def dydx(self,time,state):
          
        if self.second_pendulum == False:
          #s = state[0] not used in equations
          v = state[1]
          theta = state[2]
          dtheta = state[3]
        
        
          h0 = self.m_p*self.l/2*np.cos(theta)
          h1 = self.I_cm+self.m_p*self.l**2/4
        
          LHS = np.array([[1,0,0,0],
                         [0,self.m_c+self.m_p,0,h0],
                         [0,0,1,0],
                         [0,h0,0,h1]])
          
          RHS = np.array([[v],
                          [self.u1 + self.m_p*self.l/2*np.sin(theta)*dtheta**2],
                          [dtheta],
                          [self.m_p*self.l*g/2*np.sin(theta)]])
        
          dot_state = np.linalg.inv(LHS) @ RHS
          
          
        # elif self.second_pendulum == 'Backup':
        #   #working in absolute coordinates
          
        #   #s = state[0] not used in equations
        #   v = state[1]
        #   theta1 = state[2]
        #   dtheta1 = state[3]
        #   theta2 = state[4]+state[2]
        #   dtheta2 = state[5]+state[3]
          
        #   l1 = self.l
        #   l2 = self.l2
          
        #   m1 = self.m_p
        #   m2 = self.m_p2
        #   mc = self.m_c
          
        #   I_1 = self.I_cm
        #   I_2 = self.I_cm2
          
        #   LHS = np.array([     [1,0,0,0,0,0],
        #                       [0,mc+m1+m2,0, (m1+2*m2)*l1/2*np.cos(theta1),0,2*m2*l2/2*np.cos(theta2)],
        #                       [0,0,1,0,0,0],
        #                       [0,(m1+2*m2)*l1*np.cos(theta1)/2,0,I_1+m1*l1**2/4 + m2*l1**2,0,m2*l1*l2/2*np.cos(theta1+theta2)],
        #                       [0,0,0,0,1,0],
        #                       [0, m2*l2/2*np.cos(theta2),0,m2*l2*l1/2*np.cos(theta1+theta2),0,I_2 + m2*l2**2/4]])
          
        #   RHS = np.array([[v],
        #                   [(m1+2*m2)*l1*np.sin(theta1)*dtheta1**2/2+m2*l2*np.sin(theta2)*dtheta2**2/2+self.u1],
        #                   [dtheta1],
        #                   [(m1+2*m2)*l1/2*np.sin(theta1)*dtheta1*v+m2*l1*l2/2*np.sin(theta1+theta2)*(dtheta1+dtheta2)*dtheta2
        #                    +g*(m1+2*m2)*l1/2*np.sin(theta1)-(m1+2*m2)*l1/2*np.sin(theta1)*v*dtheta1-m2*l1*l2/2*np.sin(theta1+theta2)*dtheta1*dtheta2],
        #                   [dtheta2],
        #                   [m2/2*(l1*l2*np.sin(theta1+theta2)*(dtheta1+dtheta2)*dtheta1+l2*np.sin(theta2)*dtheta2*v)
        #                    +g*m2*l2/2*np.sin(theta2)-m2/2*(l1*l2*np.sin(theta1+theta2)*dtheta1*dtheta2+l2*np.sin(theta2)*dtheta2*v)]])
          
          
          
          
          
          
          
        #   dot_state = np.linalg.inv(self.M_local_from_abs) @ np.linalg.inv(LHS) @ RHS
        
        
  
        # elif self.second_pendulum == 'Chat':
                    
        #         stateL = self.M_abs_from_local @ self.state
            
        #         x, dx, th1, dth1, th2, dth2 = stateL

        #         m0, m1, I1, lc1, m2, I2, lc2 = self.p

        #         ddx = (dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2) - lc2*m2*(lc2*m2*(2*dth1**2*lc1*m.sin(th1 - th2) + g*m.sin(th2)) - lc2*m2*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))*m.cos(th2)/(m0 + m1 + m2) - (lc1*(-2*dth2**2*lc2*m2*m.sin(th1 - th2) + g*m1*m.sin(th1) + 2*g*m2*m.sin(th1)) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))/(m0 + m1 + m2))*(-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2)))*m.cos(th2)/(I2 - lc2**2*m2**2*m.cos(th2)**2/(m0 + m1 + m2) + (1/2)*m2*(2*lc2**2*m.sin(th2)**2 + 2*lc2**2*m.cos(th2)**2) - (-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))**2/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2))) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*(lc1*(-2*dth2**2*lc2*m2*m.sin(th1 - th2) + g*m1*m.sin(th1) + 2*g*m2*m.sin(th1)) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))/(m0 + m1 + m2) - (-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))*(lc2*m2*(2*dth1**2*lc1*m.sin(th1 - th2) + g*m.sin(th2)) - lc2*m2*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))*m.cos(th2)/(m0 + m1 + m2) - (lc1*(-2*dth2**2*lc2*m2*m.sin(th1 - th2) + g*m1*m.sin(th1) + 2*g*m2*m.sin(th1)) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))/(m0 + m1 + m2))*(-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2)))/(I2 - lc2**2*m2**2*m.cos(th2)**2/(m0 + m1 + m2) + (1/2)*m2*(2*lc2**2*m.sin(th2)**2 + 2*lc2**2*m.cos(th2)**2) - (-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))**2/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2))))/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2)))/(m0 + m1 + m2)
        #         ddth1 = (lc1*(-2*dth2**2*lc2*m2*m.sin(th1 - th2) + g*m1*m.sin(th1) + 2*g*m2*m.sin(th1)) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))/(m0 + m1 + m2) - (-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))*(lc2*m2*(2*dth1**2*lc1*m.sin(th1 - th2) + g*m.sin(th2)) - lc2*m2*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))*m.cos(th2)/(m0 + m1 + m2) - (lc1*(-2*dth2**2*lc2*m2*m.sin(th1 - th2) + g*m1*m.sin(th1) + 2*g*m2*m.sin(th1)) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))/(m0 + m1 + m2))*(-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2)))/(I2 - lc2**2*m2**2*m.cos(th2)**2/(m0 + m1 + m2) + (1/2)*m2*(2*lc2**2*m.sin(th2)**2 + 2*lc2**2*m.cos(th2)**2) - (-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))**2/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2))))/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2))
        #         ddth2 = (lc2*m2*(2*dth1**2*lc1*m.sin(th1 - th2) + g*m.sin(th2)) - lc2*m2*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))*m.cos(th2)/(m0 + m1 + m2) - (lc1*(-2*dth2**2*lc2*m2*m.sin(th1 - th2) + g*m1*m.sin(th1) + 2*g*m2*m.sin(th1)) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*(dth1**2*lc1*m1*m.sin(th1) + 2*dth1**2*lc1*m2*m.sin(th1) + dth2**2*lc2*m2*m.sin(th2))/(m0 + m1 + m2))*(-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2)))/(I2 - lc2**2*m2**2*m.cos(th2)**2/(m0 + m1 + m2) + (1/2)*m2*(2*lc2**2*m.sin(th2)**2 + 2*lc2**2*m.cos(th2)**2) - (-lc2*m2*(lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))*m.cos(th2)/(m0 + m1 + m2) + (1/2)*m2*(4*lc1*lc2*m.sin(th1)*m.sin(th2) + 4*lc1*lc2*m.cos(th1)*m.cos(th2)))**2/(I1 + (1/2)*m1*(2*lc1**2*m.sin(th1)**2 + 2*lc1**2*m.cos(th1)**2) + (1/2)*m2*(8*lc1**2*m.sin(th1)**2 + 8*lc1**2*m.cos(th1)**2) - (lc1*m1*m.cos(th1) + 2*lc1*m2*m.cos(th1))**2/(m0 + m1 + m2)))

        #         dot_state = self.M_local_from_abs @ np.array([dx, ddx, dth1, ddth1, dth2, ddth2])
        
        
        else:
        
            l1 = self.l
            l2 = self.l2
            
            m1p = self.m_p
            m2p = self.m_p2
            mc = self.m_c
            
            I1cm = self.I_cm
            I2cm = self.I_cm2
            
            u = self.u1
            
            stateA = self.M_abs_from_local @ self.state
        
            x, dx, th1, dth1, th2, dth2 = stateA
        
            
            A = np.array([[1.0*m1p + 1.0*m2p + 1.0*mc, 0.5*l1*m1p*m.cos(th1) + 1.0*l1*m2p*m.cos(th1), 0.5*l2*m2p*m.cos(th2)], [0.5*l1*m1p*m.cos(th1) + 1.0*l1*m2p*m.cos(th1), 1.0*I1cm + 0.25*l1**2*m1p + 1.0*l1**2*m2p, 0.5*l1*l2*m2p*m.cos(th1 - th2)], [0.5*l2*m2p*m.cos(th2), 0.5*l1*l2*m2p*m.cos(th1 - th2), 1.0*I2cm + 0.25*l2**2*m2p]])

             
            b = np.array([[0.5*dth1**2*l1*m1p*m.sin(th1) + 1.0*dth1**2*l1*m2p*m.sin(th1) + 0.5*dth2**2*l2*m2p*m.sin(th2) + 1.0*u], [l1*(-0.5*dth2**2*l2*m2p*m.sin(th1 - th2) + 0.5*g*m1p*m.sin(th1) + 1.0*g*m2p*m.sin(th1))], [0.5*l2*m2p*(dth1**2*l1*m.sin(th1 - th2) + g*m.sin(th2))]])
            

            acc = np.linalg.solve(A, b)[:,0]
            
            dot_state = self.M_local_from_abs @ np.array([dx, acc[0], dth1, acc[1], dth2, acc[2]])
            
        
        return dot_state.flatten().tolist()
        
      
      def DoStepDynamics(self, timestep = 1e-2):
          step_data = solve_ivp(self.dydx ,[0,timestep], self.state, t_eval=[timestep])
          self.state = step_data.y.flatten().tolist()        
            
###############################################################################

class balance_plate():
      def __init__(self, speed = 10, width = 0.1):
        print('model not implemented without pychrono, expect error')
#          self.system = chrono.ChSystemNSC()
#          self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
#          self.title = ('balance_plate_system')
         
#          # Set gravity
#          self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
#          self.camara_vector = chrono.ChVectorD(-0.8, 1.2, 1.5)

#          # Set the simulation parameters
#          self.beam_length = 2.0
#          self.beam_thickness = 0.1
#          self.ball_radius = 0.1
#          self.friction_coefficient = 1.0
#          self.starting_pos_x = 0.0

#          # Create a material with the desired friction coefficient
#          self.material = chrono.ChMaterialSurfaceNSC()
#          self.material.SetFriction(self.friction_coefficient)

#          # Create the ground body
#          self.ground = chrono.ChBody()
#          self.ground.SetBodyFixed(True)
#          self.system.Add(self.ground)
         
#          # Create the connect body
#          self.connect = chrono.ChBody()
#          self.system.Add(self.connect)

#          # Create the beam
#          self.beam = chrono.ChBodyEasyBox(self.beam_thickness / 2, self.beam_length / 2, self.beam_length  / 2, rho_wood, True, True, self.material)
#          self.beam.SetPos(chrono.ChVectorD(0, 0, 0))
#          self.beam.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))
#          self.system.Add(self.beam)

#          # Create the ball
#          self.y_offset = self.ball_radius+self.beam_thickness / 2
#          self.body_1 = chrono.ChBodyEasySphere(self.ball_radius, rho_wood, True, True, self.material)
#          self.body_1.SetPos(chrono.ChVectorD(self.starting_pos_x, self.y_offset , 0))
#          self.body_1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
#          self.system.Add(self.body_1)

#          # Create a revolute joint between the beam and the connect
#          self.revolute1 = chrono.ChLinkLockRevolute()
#          self.revolute1.Initialize(self.beam, self.connect, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0))) #, chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 0))
#          self.system.AddLink(self.revolute1)

#          positionA1 = chrono.ChVectorD(0, 0, 0) 
#          self.rotmotor1 = chrono.ChLinkMotorRotationSpeed()

#          # Connect the rotor and the stator and add the motor to the sys:
#          self.rotmotor1.Initialize(self.connect,                # body A (slave)
#                                self.beam,               # body B (master)
#                                chrono.ChFrameD(positionA1)  # motor frame, in abs. coords
#          )
#          self.system.Add(self.rotmotor1)
         
#          # Create a revolute joint between the connect and the ground
#          self.revolute2 = chrono.ChLinkLockRevolute()
#          self.revolute2.Initialize(self.connect, self.ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))
#          self.system.AddLink(self.revolute2)
         
#          self.rotmotor2 = chrono.ChLinkMotorRotationSpeed()

#          # Connect the rotor and the stator and add the motor to the sys:
#          self.rotmotor2.Initialize(self.ground,                # body A (slave)
#                                self.connect,               # body B (master)
#                                chrono.ChFrameD(positionA1,chrono.Q_ROTATE_Z_TO_X)  # motor frame, in abs. coords
#          )
#          self.system.Add(self.rotmotor2)
         
#          self.angle_x = 0
#          self.angle_z = 0
         
#          self.width = width 
#          self.speed = speed
         
#      def getState(self):
#          #return the current state
#          try: 
#            return np.array([self.position_x,self.velocity_x,self.position_z,self.velocity_z])
#          except:
#            return np.array([0,0,0,0])
       
#      def updateSystem(self, timestep = 1e-2):
#          #Do step
#          self.system.DoStepDynamics(timestep)
         
#          #fetch current angle
#          self.angle_x = shiftAngle(self.revolute1.GetRelAngle())
#          self.angle_z = shiftAngle(self.revolute2.GetRelAngle())
         
#          #calculate state
#          self.position_x = self.body_1.GetPos().x/m.cos(self.angle_x)
#          self.velocity_x = self.body_1.GetPos_dt().x/m.cos(self.angle_x)
#          self.position_z = self.body_1.GetPos().z/m.cos(self.angle_z)
#          self.velocity_z = self.body_1.GetPos_dt().z/m.cos(self.angle_z)
         
#          ### note ###
#          #
#          # These positions and velocities are only approximate projection.
#          # The angles affect eachother, which creates a non linear interaction.
#          # These projects are accurate around 0.
         
#      def updateActuator1(self, input1):
#          #update actuator1
#          d_theta = input1 - self.angle_x
#          f_input1 = chrono.ChFunction_Const(-self.speed*m.tanh(d_theta/self.width))
#          self.rotmotor1.SetSpeedFunction(f_input1)
         
#      def updateActuator2(self, input2):
#          #update actuator2
#          d_theta = input2 - self.angle_z
#          f_input2 = chrono.ChFunction_Const(-self.speed*m.tanh(d_theta/self.width))
#          self.rotmotor2.SetSpeedFunction(f_input2)     

# ###############################################################################

class rotation_inverted_pendulum():
      def __init__(self,second_pendulum = False, pendulum1_length = 0.6, pendulum2_length = 0.6, high_kick = 300.0, y0 = None):
        print('model not implemented without pychrono, expect error')

#          self.title = ('rotation_inverted_pendulum')
         
#          # Set gravity
#          self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
#          self.camara_vector = chrono.ChVectorD(-0.8, 1.2, 1.5)

         

#          self.second_pendulum = second_pendulum
#          self.pendulum1_angle = 0.0
#          self.pendulum2_angle = 0.0
         
#          self.arm_length = 0.3

          
#          # Set up ground
#          self.ground = chrono.ChBodyEasyCylinder(1 ,self.arm_length *.8, 0.1, rho_wood,True,False)
#          self.ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
#          self.ground.SetBodyFixed(True)
#          linebox = chrono.ChBoxShape(2*self.arm_length *0.8+0.01, 0.105, .01)
#          linebox.SetColor(chrono.ChColor(0.6, 0, 0))
#          self.ground.AddVisualShape(linebox, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
#          self.system.Add(self.ground)
#          self.table_color = chrono.ChColor(0.1 ,.3, .3)
#          self.ground.GetVisualShape(0).SetColor(self.table_color)
         
#          # Set up pivot
#          self.pivot = chrono.ChBodyEasyCylinder(1 , self.arm_length *.2, .2, rho_wood,True,False)
#          self.system.AddBody(self.pivot)
#          self.pivot.SetBodyFixed(True)
#          self.pivot.SetPos(chrono.ChVectorD(0, 0, 0))
#          self.pivot.GetVisualShape(0).SetColor(self.table_color)

#          # Set up arm
#          self.arm = chrono.ChBodyEasyBox(self.arm_length, 0.03, 0.03, rho_wood, True, False)
#          self.arm.SetPos(chrono.ChVectorD(self.arm_length/2, 0.1, 0))
#          self.arm.SetRot_dt(chrono.Q_from_AngY(chrono.CH_C_PI_2)*high_kick)
#          self.system.Add(self.arm)        

#          # Set up first pendulum
#          self.pendulum1 = chrono.ChBodyEasyBox(0.02, pendulum1_length, 0.02, rho_wood, True, False)
#          self.pendulum1.SetPos(chrono.ChVectorD(self.arm_length, pendulum1_length / 2 + 0.115, 0))
#          self.pendulum1.SetRot(chrono.Q_from_AngAxis(self.pendulum1_angle, chrono.ChVectorD(0, 0, 1)))
#          self.system.Add(self.pendulum1)

#          # Set up revolute joint between arm and first pendulum
#          self.revolute1 = chrono.ChLinkLockRevolute()
#          self.revolute1.Initialize(self.arm, self.pendulum1, chrono.ChCoordsysD(chrono.ChVectorD(self.arm_length, 0.115, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
#          self.system.Add(self.revolute1)


#          if self.second_pendulum:
#            # Set up second pendulum
#            self.pendulum2 = chrono.ChBodyEasyBox(0.02, pendulum2_length, 0.02, rho_wood, True, False)
#            self.pendulum2.SetPos(chrono.ChVectorD(self.arm_length, pendulum2_length / 2 + pendulum1_length + 0.115, 0))
#            self.pendulum2.SetRot(chrono.Q_from_AngAxis(self.pendulum2_angle, chrono.ChVectorD(0, 0, 1)))
#            self.system.Add(self.pendulum2)
           
#            # Set up revolute joint between first and second pendulum
#            self.revolute2 = chrono.ChLinkLockRevolute()
#            self.revolute2.Initialize(self.pendulum1, self.pendulum2, chrono.ChCoordsysD(chrono.ChVectorD(self.arm_length, pendulum1_length + 0.115, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
#            self.system.Add(self.revolute2)

#          # Create a revolute joint between the arm and the ground
#          self.revolute = chrono.ChLinkLockRevolute()
#          self.revolute.Initialize(self.arm, self.ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),chrono.Q_ROTATE_Z_TO_Y))
#          self.system.AddLink(self.revolute)

#          # Create the motor
#          self.rotmotor1 = chrono.ChLinkMotorRotationTorque()
#          positionA1 = chrono.ChVectorD(0, 0, 0) 
           
#          # Connect the rotor and the stator and add the motor to the sys:
#          self.rotmotor1.Initialize(self.ground,                # body A (slave)
#                                self.arm,               # body B (master)
#                                chrono.ChFrameD(positionA1,chrono.Q_ROTATE_Z_TO_Y)  # motor frame, in abs. coords,
#          )
#          self.system.Add(self.rotmotor1)
         
#          # initiate state variables
#          self.arm_angle = 0. 
#          self.arm_angle_vel = 0.
         
#          self.pend1_angle = 0.
#          self.pend1_angle_vel = 0. 
         
#          if self.second_pendulum:
#            self.pend2_angle = 0.
#            self.pend2_angle_vel = 0. 
           
#      def getState(self):
#          #return the current state
#          if self.second_pendulum:
#            state = np.array([ self.arm_angle,
#                               self.arm_angle_vel,
#                               self.pend1_angle,
#                               self.pend1_angle_vel,
#                               self.pend2_angle,
#                               self.pend2_angle_vel])
#          else:
#            state = np.array([ self.arm_angle,
#                               self.arm_angle_vel,
#                               self.pend1_angle,
#                               self.pend1_angle_vel])
#          return state
       
#      def updateSystem(self, timestep = 1e-2):
#          #Do step
#          self.system.DoStepDynamics(timestep)
         
#          #calculate state
#          self.arm_angle = shiftAngle(self.revolute.GetRelAngle())
#          self.arm_angle_vel = self.revolute.GetRelWvel().z
         
#          self.pend1_angle = shiftAngle(self.revolute1.GetRelAngle())
#          self.pend1_angle_vel = self.revolute1.GetRelWvel().z
         
#          if self.second_pendulum:
#            self.pend2_angle = shiftAngle(self.revolute2.GetRelAngle())
#            self.pend2_angle_vel = self.revolute2.GetRelWvel().z
         
#      def updateActuator1(self, input1):
#          #update actuator1
#          f_input1 = chrono.ChFunction_Const(input1)
#          self.rotmotor1.SetMotorFunction(f_input1)
#          pass
           
#      def updateActuator2(self, input2):
#          #update actuator2
#          pass      
             
# ###############################################################################

class flywheel_inverted_pendulum():
      def __init__(self,second_pendulum = False, pendulum1_length = 0.6, pendulum2_length = 0.6, high_kick = 100.0, state0 = None, u0 = 0):


          self.title = ('flywheel_inverted_pendulum')
          # Set gravity


          self.second_pendulum = second_pendulum
          d = 0.02
          
          if self.second_pendulum:
            
            self.second_pendulum = True
            self.l2 = pendulum2_length
            
            self.m2p = rho_wood*d**2*self.l2
            self.I2cm = self.m2p*(d**2+self.l2**2)/12
            self.I2ep = self.I2cm + self.m2p*self.l2**2/4 
            
            self.A_from_L = np.array([[1,0,1,0,0,0],
                                 [0,1,0,1,0,0],
                                 [0,0,1,0,0,0],
                                 [0,0,0,1,0,0],
                                 [0,0,1,0,1,0],
                                 [0,0,0,1,0,1]])
            
            self.L_from_A = np.linalg.inv(self.A_from_L)
          
          self.pendulum1_angle = 0.0
          self.pendulum2_angle = 0.0
          self.flywheel_radius = 0.1
          self.flywheel_thickness = 0.01 
          
          self.l = pendulum1_length
          self.l1=self.l
          
          self.m_p = rho_wood*d**2*self.l
          self.I_cm = self.m_p*(d**2+self.l**2)/12
          self.I_ep = self.I_cm + self.m_p*self.l**2/4 
          
          self.u1 = u0
          
          self.m_cyl = rho_iron*self.flywheel_thickness*self.flywheel_radius**2*m.pi
          self.I_cyl = self.m_cyl*self.flywheel_radius**2/2
          
          if state0 is None:
            if second_pendulum:
                self.state = [0, high_kick*self.m_p/(self.m_p+self.m_cyl), 0, -high_kick*self.m_p/(self.m_p+self.m_cyl), 0, -high_kick*self.m_p/(self.m_p+self.m_cyl)]
            else:
                self.state = [0, high_kick*self.m_p/(self.m_p+self.m_cyl), 0, -high_kick*self.m_p/(self.m_p+self.m_cyl)]
              
              
          else:
            self.state = state0
            

          # initiate state variables
          self.fly_angle = 0. 
          self.fly_angle_vel = 0.
          
          self.pend1_angle = 0.
          self.pend1_angle_vel = 0. 
          
          if self.second_pendulum:
            self.pend2_angle = 0.
            self.pend2_angle_vel = 0. 
                
          
      def getState(self):
          #return the current state
          if self.second_pendulum:
            state = np.array([ self.fly_angle,
                                self.fly_angle_vel,
                                self.pend1_angle,
                                self.pend1_angle_vel,
                                self.pend2_angle,
                                self.pend2_angle_vel])
          else:
            state = np.array([ self.fly_angle,
                                self.fly_angle_vel,
                                self.pend1_angle,
                                self.pend1_angle_vel])
          return state
        
      def updateSystem(self, timestep = 1e-2):
          #Do step
          self.DoStepDynamics(timestep)
          
          #calculate state
          self.fly_angle = shiftAngle(self.state[0])
          self.fly_angle_vel = self.state[1]
          self.pend1_angle = shiftAngle(self.state[2])
          self.pend1_angle_vel = self.state[3]
          if self.second_pendulum:
            self.pend2_angle = shiftAngle(self.state[4])
            self.pend2_angle_vel = self.state[5]
          
      def updateActuator1(self, input1):
            #update actuator1
            try:
              u = input1[0]
            except:
              u = input1
            self.u1 = u
              
      def updateActuator2(self, input2):
            #update actuator2
            pass   
      
      def dydx(self,time,state):
          
        if self.second_pendulum:
            lstate = np.array(state)  
            astate = self.A_from_L@lstate
            
            #working in absolute coordinates
            #phi = astate[0]    #flywheel angle, not used in equations
            dphi  = astate[1]
            theta1  = astate[2] #pendulum1 angle
            dtheta1 = astate[3]
            theta2  = astate[4] #pendulum2 angle
            dtheta2 = astate[5]
            
            LHS = np.array([[1,0,0,0,0,0],
                            [0,self.I_cyl,0,0,0,0],
                            [0,0,1,0,0,0],
                            [0,0,0,self.I_ep+self.l**2*self.m2p+self.l1**2*self.m_cyl,0,0.5*self.l1*self.l2*self.m2p*np.cos(theta1 - theta2)],  
                            [0,0,0,0,1,0],
                            [0,0,0,0.5*self.l1*self.l2*self.m2p*np.cos(theta1 - theta2),0,self.I2cm+ 0.25*self.l2**2*self.m2p]    
                            ])
            
            RHS = np.array([[dphi],
                            [self.u1],
                            [dtheta1],
                            [0.5*g*self.l1*self.m_p*np.sin(theta1) + g*self.l1*self.m2p*np.sin(theta1) + g*self.l1*self.m_cyl*np.sin(theta1)  + 0.5*self.l1*self.l2*self.m2p*np.sin(theta1 - theta2)*dtheta2**2  - self.u1],
                            [dtheta2],
                            [ 0.5*g*self.l2*self.m2p*np.sin(theta2) + 0.5*self.l1*self.l2*self.m2p*np.sin(theta1 - theta2)*dtheta1**2]
                            ])
            
            dot_state = self.L_from_A@np.linalg.inv(LHS) @ RHS
            flat_dot_state = dot_state.flatten().tolist()
            
            
            
        else:
            #working in local coordinates
            #phi = state[0]    #flywheel angle, not used in equations
            dphi  = -state[1]
            theta  = state[2] #pendulum angle
            dtheta = state[3]
          
          
            h0 = self.I_ep+self.m_cyl*self.l**2
          
            LHS = np.array([[1,0,0,0],
                           [0,0,0,h0],
                           [0,0,1,0],
                           [0,self.I_cyl,0,-self.I_cyl]])
            
            RHS = np.array([[dphi],
                            [-self.u1 +  (self.m_p*g*self.l/2  + self.m_cyl*g*self.l)*np.sin(theta)],
                            [dtheta],
                            [-self.u1]])
          
            dot_state = np.linalg.inv(LHS) @ RHS
            flat_dot_state = dot_state.flatten().tolist()
            flat_dot_state[0]=-flat_dot_state[0]
            flat_dot_state[1]=-flat_dot_state[1]
          
          
        return flat_dot_state
        
      
      def DoStepDynamics(self, timestep = 1e-2):
          step_data = solve_ivp(self.dydx ,[0,timestep], self.state, t_eval=[timestep])
          state=step_data.y.flatten().tolist()
          self.state = state
          
           


  
    
            
###############################################################################

class stacked_inverted_pendulum():
      def __init__(self,num_pendulum = 2, pendulum_length = 0.6, high_kick = 2.0, state0 = None, u0 = 0):
          self.title = ('stacked_inverted_pendulum')

          self.second_pendulum = (num_pendulum>1)
          
          self.third_pendulum = (num_pendulum>2)
          
          if num_pendulum>3:
            print('more then 3 pendula not implemented without pychrono, expect errors')

          self.pend1_angle = 0.
          self.pend1_angle_vel = 0. 
          
          self.l = pendulum_length
          d = 0.02
          self.m_p = rho_wood*d**2*self.l
          self.m = self.m_p
          self.I_cm = self.m_p*(d**2+self.l**2)/12
          self.Icm = self.I_cm
          self.I_ep = self.I_cm + self.m_p*self.l**2/4
          self.I = self.I_ep
          self.u1 = u0
          
          if self.third_pendulum:
              self.A_from_L = np.array([ [1,0,0,0,0,0],
                                         [0,1,0,0,0,0],
                                         [1,0,1,0,0,0],
                                         [0,1,0,1,0,0],
                                         [1,0,1,0,1,0],
                                         [0,1,0,1,0,1]])
              
              self.L_from_A = np.linalg.inv(self.A_from_L)
              
          elif self.second_pendulum:
              self.A_from_L = np.array([ [1,0,0,0],
                                         [0,1,0,0],
                                         [1,0,1,0],
                                         [0,1,0,1]])
              
              self.L_from_A = np.linalg.inv(self.A_from_L)
              
                                       
          
          
          if state0 is None:
            if self.third_pendulum:
              self.state = [0, high_kick, 0, -high_kick, 0, -high_kick]
              
              self.pend2_angle = 0.
              self.pend2_angle_vel = 0. 
              
              self.pend3_angle = 0.
              self.pend3_angle_vel = 0. 
            elif self.second_pendulum:
              self.state = [0, high_kick, 0, -high_kick]  
              
              self.pend2_angle = 0.
              self.pend2_angle_vel = 0. 
            else:
              self.state = [0, high_kick]
              
              self.pend2_angle = 0.
              self.pend2_angle_vel = 0. 
          else:
            self.state = state0
            self.pend1_angle = shiftAngle(self.state[0])
            self.pend1_angle_vel = self.state[1]
            if self.second_pendulum:
              self.pend2_angle = shiftAngle(self.state[2])
              self.pend2_angle_vel = self.state[3]
              if self.third_pendulum:
                self.pend3_angle = shiftAngle(self.state[4])
                self.pend3_angle_vel = self.state[5]
            
            # if self.third_pendulum:
            #   self.pend3_angle = 0.
            #   self.pend3_angle_vel = 0.       
          
      def getState(self):
          #return the current state
          # if self.fourth_pendulum:
          #   state = np.array([ self.pend1_angle,
          #                       self.pend1_angle_vel,
          #                       self.pend2_angle,
          #                       self.pend2_angle_vel,
          #                       self.pend3_angle,
          #                       self.pend3_angle_vel,
          #                       self.pend4_angle,
          #                       self.pend4_angle_vel])
          
          if self.third_pendulum:
            state = np.array([  self.pend1_angle,
                                self.pend1_angle_vel,
                                self.pend2_angle,
                                self.pend2_angle_vel,
                                self.pend3_angle,
                                self.pend3_angle_vel])
          
          elif self.second_pendulum:
            state = np.array([  self.pend1_angle,
                                self.pend1_angle_vel,
                                self.pend2_angle,
                                self.pend2_angle_vel])
          else:
            state = np.array([  self.pend1_angle,
                                self.pend1_angle_vel])
          return state
        
      def updateSystem(self, timestep = 1e-2):
          #Do step
          self.DoStepDynamics(timestep)
          
          #calculate state
          self.pend1_angle = shiftAngle(self.state[0])
          self.pend1_angle_vel = self.state[1]
          if self.second_pendulum:
            self.pend2_angle = shiftAngle(self.state[2])
            self.pend2_angle_vel = self.state[3]
            if self.third_pendulum:
              self.pend3_angle = shiftAngle(self.state[4])
              self.pend3_angle_vel = self.state[5]
            #   if self.fourth_pendulum:
            #     self.pend4_angle = shiftAngle(self.revolute4.GetRelAngle())
            #     self.pend4_angle_vel = self.revolute4.GetRelWvel().z
          
      def updateActuator1(self, input1):
          #update actuator1
          try:
            self.u1 = input1[0]
          except:
            self.u1 = input1
          
            
      def updateActuator2(self, input2):
          #update actuator2
          pass    
        
      def dydx(self,time,state):
          
        if self.third_pendulum:
          lstate = np.array(state)
          astate = self.A_from_L@lstate
          
          theta_1 = astate[0]
          dtheta_1 = astate[1]
          theta_2 = astate[2]
          dtheta_2 = astate[3]
          theta_3 = astate[4]
          dtheta_3 = astate[5]
          
          LHS = np.array([[1,0,0,0,0,0],
                          [0,self.I+ 2.0*self.l**2*self.m ,0, 1.5*self.l**2*self.m*np.cos(theta_1 - theta_2) ,0, 0.5*self.l**2*self.m*np.cos(theta_1 - theta_3)], 
                          [0,0,1,0,0,0],
                          [0,1.5*self.l**2*self.m*np.cos(theta_1 - theta_2) ,0,self.Icm+ 1.25*self.l**2*self.m ,0,0.5*self.l**2*self.m*np.cos(theta_2 - theta_3) ],
                          [0,0,0,0,1,0],
                          [0, 0.5*self.l**2*self.m*np.cos(theta_1 - theta_3),0,0.5*self.l**2*self.m*np.cos(theta_2 - theta_3) ,0,self.Icm+ 0.25*self.l**2*self.m]])
          
          RHS = np.array([[dtheta_1],
                          [ 2.5*g*self.l*self.m*np.sin(theta_1) - 1.5*self.l**2*self.m*np.sin(theta_1 - theta_2)*dtheta_2**2 - 0.5*self.l**2*self.m*np.sin(theta_1 - theta_3)*dtheta_3**2 + 1.0*self.u1],
                          [dtheta_2],
                          [1.5*g*self.l*self.m*np.sin(theta_2) + 1.5*self.l**2*self.m*np.sin(theta_1 - theta_2)*dtheta_1**2 - 0.5*self.l**2*self.m*np.sin(theta_2 - theta_3)*dtheta_3**2],
                          [dtheta_3],
                          [0.5*g*self.l*self.m*np.sin(theta_3) + 0.5*self.l**2*self.m*np.sin(theta_1 - theta_3)*dtheta_1**2 + 0.5*self.l**2*self.m*np.sin(theta_2 - theta_3)*dtheta_2**2]])
        
          dot_state = self.L_from_A @ np.linalg.inv(LHS) @ RHS
          
          
        elif self.second_pendulum:
          lstate = np.array(state)
          astate = self.A_from_L@lstate
          
          theta_1 = astate[0]
          dtheta_1 = astate[1]
          theta_2 = astate[2]
          dtheta_2 = astate[3]
        
          LHS = np.array([[1,0,0,0],
                         [0,self.I + self.l**2*self.m  ,0, 0.5*self.l**2*self.m*np.cos(theta_1 - theta_2) ],
                         [0,0,1,0],
                         [0,0.5*self.l**2*self.m*np.cos(theta_1 - theta_2) ,0,self.Icm+ 0.25*self.l**2*self.m ]])
          
          RHS = np.array([[dtheta_1],
                          [1.5*g*self.l*self.m*np.sin(theta_1) - 0.5*self.l**2*self.m*np.sin(theta_1 - theta_2)*dtheta_2**2   + self.u1],
                          [dtheta_2],
                          [0.5*g*self.l*self.m*np.sin(theta_2) + 0.5*self.l**2*self.m*np.sin(theta_1 - theta_2)*dtheta_1**2]])
        
          dot_state = self.L_from_A @ np.linalg.inv(LHS) @ RHS
        else:
          print('single pendulum not implemented without pychrono, expect error')
          
        return dot_state.flatten().tolist()
        
      
      def DoStepDynamics(self, timestep = 1e-2):
          step_data = solve_ivp(self.dydx ,[0,timestep], self.state, t_eval=[timestep])
          self.state = step_data.y.flatten().tolist()
          
                
###############################################################################

class cylinder_stability():
      def __init__(self, length = 2, width = 0.3, radius = 0.4):
        print('model not implemented without pychrono, expect error')
#          self.system = chrono.ChSystemNSC()
#          self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
#          self.title = ('cylinder_stability')
         
#          # Set gravity
#          self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
#          self.camara_vector = chrono.ChVectorD(2+length, 0.9, 1.2)

#          # Set the simulation parameters
#          self.beam_length = length
#          self.beam_thickness = width
#          self.cyl_radius = radius
#          self.cyl_height = 2*width
#          self.friction_coefficient = 1.0
#          self.table_color = chrono.ChColor(0.1 ,.1, .1)

#          # Create a material with the desired friction coefficient
#          self.material = chrono.ChMaterialSurfaceNSC()
#          self.material.SetFriction(self.friction_coefficient)

#          # Create the beam
#          self.beam = chrono.ChBodyEasyBox(self.beam_length,self.beam_thickness,  self.beam_thickness , rho_wood, True, True, self.material)
#          self.beam.SetPos(chrono.ChVectorD(0, self.cyl_radius+self.beam_length/2, 0.001))
#          self.beam.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))
#          self.system.Add(self.beam)

#          # Create the cylinder
#          self.cyl = chrono.ChBodyEasyCylinder(0, self.cyl_radius, self.cyl_height, rho_wood,True,True,self.material)
#          self.cyl.SetPos(chrono.ChVectorD(0, 0, 0))
#          self.cyl.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
#          self.cyl.SetBodyFixed(True)
#          self.system.Add(self.cyl)
         
#      def getState(self):
#          #return the current state
#          try: 
#            return np.array([self.beam_angle,self.beam_angle_vel])
#          except:
#            return np.array([0,0])
       
#      def updateSystem(self, timestep = 1e-2):
#          #Do step
#          self.system.DoStepDynamics(timestep)

#          #calculate state
#          self.beam_angle = chrono.Q_to_Euler123(self.beam.GetRot()).y 
#          self.beam_angle_vel = chrono.Q_to_Euler123(self.beam.GetRot_dt()).y
       
#      def updateActuator1(self, input1):
#          #update actuator1
#          pass
           
#      def updateActuator2(self, input2):
#          #update actuator2
#          pass       

# ###############################################################################

class balance_slide():
      def __init__(self, body_size = 0.2, start_pos = 1/32):
        print('model not implemented without pychrono, expect error')
#          self.system = chrono.ChSystemNSC()
#          self.system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
#          self.title = ('balance_slide_system')
         
#          # Set gravity
#          self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
#          self.camara_vector = chrono.ChVectorD(-0.8, 1.2, 1.5)

#          # Set the simulation parameters
#          self.beam_length = 2.0
#          self.beam_thickness = 0.05

#          self.starting_pos_x = self.beam_length * start_pos
         
#          # Set parameters
#          self.body_1_size = body_size
         
#          # Create the beam
#          self.beam = chrono.ChBodyEasyBox(self.beam_thickness, self.beam_length, self.beam_thickness, rho_wood, True, False)
#          self.beam.SetPos(chrono.ChVectorD(0, 0, 0))
#          self.beam.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))
#          self.system.Add(self.beam)

#          # Create the box
#          self.body_1 = chrono.ChBodyEasyBox(self.body_1_size, self.body_1_size, self.body_1_size, rho_wood, True, False)
#          self.body_1.SetPos(chrono.ChVectorD(self.starting_pos_x, 0 , 0))
#          self.body_1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
#          self.system.Add(self.body_1)

#          # Create the ground body
#          self.ground = chrono.ChBody()
#          self.ground.SetBodyFixed(True)
#          self.system.Add(self.ground)
         
#          # Create a revolute joint between the beam and the ground
#          self.revolute = chrono.ChLinkLockRevolute()
#          self.revolute.Initialize(self.beam, self.ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))))#
#          self.system.AddLink(self.revolute)

#          # Create the motor
#          positionA1 = chrono.ChVectorD(0, 0, 0) 
#          self.rotmotor1 = chrono.ChLinkMotorRotationTorque()

#          # Connect the rotor and the stator and add the motor to the sys:
#          self.rotmotor1.Initialize(self.ground,                # body A (slave)
#                                self.beam,               # body B (master)
#                                chrono.ChFrameD(positionA1)  # motor frame, in abs. coords
#          )
#          self.system.Add(self.rotmotor1)
          
#          # Create prismatic link
#          self.prismatic1 = chrono.ChLinkLockPrismatic()
#          self.prismatic1.Initialize(self.beam, self.body_1, chrono.ChCoordsysD(
#              chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
#          self.system.AddLink(self.prismatic1)

#          # Create the motor
#          positionA1 = chrono.ChVectorD(0, 0, 0) 
#          self.rotmotor1 = chrono.ChLinkMotorRotationTorque()

#          # Connect the rotor and the stator and add the motor to the sys:
#          self.rotmotor1.Initialize(self.ground,                # body A (slave)
#                                self.beam,               # body B (master)
#                                chrono.ChFrameD(positionA1,chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))  # motor frame, in abs. coords ,
#          )
#          self.system.Add(self.rotmotor1)
         
#          self.angle_x = 0
         
#      def getState(self):
#          # Return the current state
#          try: 
#            return np.array([self.position,self.velocity,self.beam_angle,self.beam_angle_vel])
#          except:
#            return np.array([0,0,0,0])
       
#      def updateSystem(self, timestep = 1e-2):
#          #Do step
#          self.system.DoStepDynamics(timestep)
         
#          #fetch current angle
#          self.angle_x = shiftAngle(self.revolute.GetRelAngle())
         
#          #calculate state
#          self.position = self.body_1.GetPos().x/m.cos(self.angle_x)
#          self.velocity = self.body_1.GetPos_dt().x/m.cos(self.angle_x)
#          self.beam_angle = self.angle_x
#          self.beam_angle_vel = self.revolute.GetRelWvel().z
         
#          if abs(self.position)>self.beam_length / 2 and self.prismatic1.IsBroken() == False:
#            self.prismatic1.SetBroken(True)
     
#      def updateActuator1(self, input1):
#          #update actuator1
#          f_input1 = chrono.ChFunction_Const(input1)
#          self.rotmotor1.SetMotorFunction(f_input1)         
           
#      def updateActuator2(self, input2):
#          #update actuator2
#          pass              


# ###############################################################################