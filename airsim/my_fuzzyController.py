# original author : Tzung-Hsien Huang
# code rewrited by LiaoSteve (refference -> paper: https://arxiv.org/ftp/arxiv/papers/1705/1705.04114.pdf)

import numpy as np
import time
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt 
import matplotlib.ticker as ticker

class FuzzyControl(object):
    def __init__(self):
        # Input range
        self.rules = []
        self.x_universe  = np.arange(-320, 320, 1)    # in pixel  640*480 image
        self.y_universe  = np.arange(-240, 240, 1)    # in pixel
        self.distance_universe  = np.arange(0, 1500, 1)      # distance in cm

        # Output range
        self.vx_universe  = np.arange(-3, 3, 0.1)     # forward and backward velocity [m/s]
        self.vy_universe  = np.arange(-1, 1, 0.1)     # left and right velocity [m/s]
        self.vz_universe = np.arange(-0.3, 0.3, 0.1)    # up and down velocity [m/s]
        self.turn_yaw_universe = np.arange(-60,60,1)  # turn_yaw [degree]
        
        # Input
        self.x_input = ctrl.Antecedent(self.x_universe, 'x')  # left and right in pixel 
        self.y_input = ctrl.Antecedent(self.y_universe, 'y')  # up and down in pixel
        self.distance_input = ctrl.Antecedent(self.distance_universe, 'distance_input')

        # Output
        self.vx         = ctrl.Consequent(self.vx_universe, 'vx')
        self.vy         = ctrl.Consequent(self.vy_universe, 'vy')
        self.vz         = ctrl.Consequent(self.vz_universe, 'vz')
        self.turn_yaw  = ctrl.Consequent(self.turn_yaw_universe, 'turn_yaw')

    def Membership(self):
        # Membership function of inputs
        self.x_input['right'] = fuzz.zmf(self.x_input.universe, -150, 0)
        self.x_input['mid']   = fuzz.trimf(self.x_input.universe, (-45, 0, 45))
        self.x_input['left']  = fuzz.smf(self.x_input.universe, 0, 150)

        self.y_input['bottom'] = fuzz.zmf(self.y_input.universe, -240, 0)
        self.y_input['mid']    = fuzz.trapmf(self.y_input.universe, [-100, -50, 50, 100])
        self.y_input['top']    = fuzz.smf(self.y_input.universe, 0, 240)

        self.distance_input['far']  = fuzz.trapmf(self.distance_input.universe, [380, 900, 1500, 1500])        
        self.distance_input['near']  = fuzz.trapmf(self.distance_input.universe, [0, 0, 200, 750])

        # Membership function of outputs. 
        self.vx['fast'] = fuzz.pimf(self.vx.universe, -0.3, 1.2, 3, 3)
        self.vx['slow'] = fuzz.zmf(self.vx.universe, -1.5, 1.0)

        self.vy['left']  = fuzz.zmf(self.vy.universe, -1, -0.4)
        self.vy['mid']   = fuzz.trimf(self.vy.universe, (-0.5, 0, 0.5))
        self.vy['right'] = fuzz.smf(self.vy.universe, 0.4, 1)

        self.vz['top']    = fuzz.zmf(self.vz.universe, -0.2, -0.15)
        self.vz['mid']    = fuzz.trapmf(self.vz.universe, [-0.15, -0.05, 0.05, 0.1])
        self.vz['bottom'] = fuzz.trapmf(self.vz.universe, [0.05, 0.1, 0.2, 0.2])               
        
        self.turn_yaw['turn_right']  = fuzz.zmf(self.turn_yaw.universe, -50, 0)
        self.turn_yaw['turn_mid']   = fuzz.trimf(self.turn_yaw.universe, (-30, 0, 30))
        self.turn_yaw['turn_left'] = fuzz.smf(self.turn_yaw.universe, 0, 50)

    def viewGraphic(self):
        self.x_input.view()         #1
        self.y_input.view()         #2
        self.distance_input.view()  #3

        self.vx.view()              #4
        self.vy.view()              #5
        self.vz.view()              #6
        self.turn_yaw.view()        #7        
        plt.show()
    
    def FuzzyRule(self):
        self.Membership()        
        self.rule1  = ctrl.Rule(self.x_input['left'] & self.y_input['top'] & self.distance_input['near']    , consequent=(self.vy['right'], self.vz['bottom'], self.vx['slow'], self.turn_yaw['turn_right']))
        self.rule2  = ctrl.Rule(self.x_input['left'] & self.y_input['top'] & self.distance_input['far']     , consequent=(self.vy['right'], self.vz['mid'], self.vx['fast'], self.turn_yaw['turn_mid']))
        self.rule3  = ctrl.Rule(self.x_input['left'] & self.y_input['mid'] & self.distance_input['near']    , consequent=(self.vy['right'], self.vz['top'], self.vx['slow'], self.turn_yaw['turn_right']))
        self.rule4  = ctrl.Rule(self.x_input['left'] & self.y_input['mid'] & self.distance_input['far']     , consequent=(self.vy['right'], self.vz['mid'], self.vx['fast'], self.turn_yaw['turn_mid']))
        self.rule5  = ctrl.Rule(self.x_input['left'] & self.y_input['bottom'] & self.distance_input['near'] , consequent=(self.vy['right'], self.vz['top'], self.vx['slow'], self.turn_yaw['turn_right']))
        self.rule6  = ctrl.Rule(self.x_input['left'] & self.y_input['bottom'] & self.distance_input['far']  , consequent=(self.vy['right'], self.vz['mid'], self.vx['fast'], self.turn_yaw['turn_mid']))

        self.rule7  = ctrl.Rule(self.x_input['mid'] & self.y_input['top'] & self.distance_input['near']     , consequent=(self.vy['mid'], self.vz['bottom'], self.vx['slow'], self.turn_yaw['turn_mid']))
        self.rule8  = ctrl.Rule(self.x_input['mid'] & self.y_input['top'] & self.distance_input['far']      , consequent=(self.vy['mid'], self.vz['mid'], self.vx['fast'], self.turn_yaw['turn_mid']))
        self.rule9  = ctrl.Rule(self.x_input['mid'] & self.y_input['mid'] & self.distance_input['near']     , consequent=(self.vy['mid'], self.vz['top'], self.vx['slow'], self.turn_yaw['turn_mid']))
        self.rule10 = ctrl.Rule(self.x_input['mid'] & self.y_input['mid'] & self.distance_input['far']      , consequent=(self.vy['right'], self.vz['mid'], self.vx['fast'], self.turn_yaw['turn_mid']))
        self.rule11 = ctrl.Rule(self.x_input['mid'] & self.y_input['bottom'] & self.distance_input['near']  , consequent=(self.vy['mid'], self.vz['top'], self.vx['slow'], self.turn_yaw['turn_mid']))
        self.rule12 = ctrl.Rule(self.x_input['mid'] & self.y_input['bottom'] & self.distance_input['far']   , consequent=(self.vy['mid'], self.vz['mid'], self.vx['fast'], self.turn_yaw['turn_mid']))

        self.rule13 = ctrl.Rule(self.x_input['right'] & self.y_input['top'] & self.distance_input['near']   , consequent=(self.vy['left'], self.vz['bottom'] , self.vx['slow'], self.turn_yaw['turn_left']))
        self.rule14 = ctrl.Rule(self.x_input['right'] & self.y_input['top'] & self.distance_input['far']    , consequent=(self.vy['left'], self.vz['mid'] , self.vx['fast'], self.turn_yaw['turn_mid']))
        self.rule15 = ctrl.Rule(self.x_input['right'] & self.y_input['mid'] & self.distance_input['near']   , consequent=(self.vy['left'], self.vz['top'] , self.vx['slow'], self.turn_yaw['turn_left']))
        self.rule16 = ctrl.Rule(self.x_input['right'] & self.y_input['mid'] & self.distance_input['far']    , consequent=(self.vy['left'], self.vz['mid'] , self.vx['fast'], self.turn_yaw['turn_mid']))
        self.rule17 = ctrl.Rule(self.x_input['right'] & self.y_input['bottom'] & self.distance_input['near'], consequent=(self.vy['left'], self.vz['top'] , self.vx['slow'], self.turn_yaw['turn_left']))
        self.rule18 = ctrl.Rule(self.x_input['right'] & self.y_input['bottom'] & self.distance_input['far'] , consequent=(self.vy['left'], self.vz['mid'] , self.vx['fast'], self.turn_yaw['turn_mid']))
        
        return self.rule1, self.rule2, self.rule3, self.rule4, self.rule5, self.rule6, self.rule7, self.rule8, self.rule9, self.rule10, self.rule11, self.rule12, self.rule13, self.rule14, self.rule15, self.rule16, self.rule17, self.rule18
    def pre_fzprocess(self):
        self.rules     = self.FuzzyRule()
        self.fzcontrol = ctrl.ControlSystem(self.rules)
        self.fzmotion  = ctrl.ControlSystemSimulation(self.fzcontrol)
    
    def fzprocess(self, delta_x, delta_y, distance):                   
        self.fzmotion.input['x'] = delta_x
        self.fzmotion.input['y'] = delta_y
        self.fzmotion.input['distance_input'] = distance
        time_s = time.time()
        self.fzmotion.compute()
        time_e = time.time()
        time_cost = time_e - time_s
        
        return self.fzmotion.output['vx'], self.fzmotion.output['vy'], self.fzmotion.output['vz'], self.fzmotion.output['turn_yaw'], time_cost
        
if __name__ == '__main__':

    fz = FuzzyControl()   
    fz.pre_fzprocess()
    vx, vy, vz, turn_yaw, cost_t = fz.fzprocess(delta_x=300, delta_y=0, distance=10)        
    print('\nv_forward: ',round(vx,2))
    print('v_right: ',round(vy,2))
    print('v_up: ',round(vz,2))
    print('turn_yaw',round(turn_yaw,2))
    print('time',cost_t,'sec') 
    
    fz.viewGraphic()