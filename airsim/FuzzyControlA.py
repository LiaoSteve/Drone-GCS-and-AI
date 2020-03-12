# Update : 2020/01/04
#          Optimize the algorithm
# Note : x -> Horizontal
#        y -> Vertical
#        z -> Forward/Backward
# Author : Tzung-Hsien Huang

import numpy as np
import time, sys
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt 
import matplotlib.ticker as ticker

class FuzzyControl(object):
    def __init__(self, mode='far_to_waypoint'):
        self.rules = []
        self.mode = mode
        self.x_universe  = np.arange(-200, 200, 1)    # in pixel 
        self.y_universe  = np.arange(-240, 240, 1)    # in pixel

        if self.mode == 'far_to_waypoint':
            self.z_universe  = np.arange(0, 1500, 1)      # distance in cm
        elif self.mode == 'near_to_waypoint':
            self.z_universe  = np.arange(0, 300, 1)      # distance in cm
        else:
            print('mode name error')
            self.mode == 'far_to_waypoint'
            self.z_universe  = np.arange(0, 1500, 1)      # distance in cm

        self.v_universe  = np.arange(-4, 4, 0.1)
        self.vz_universe = np.arange(-2, 1.5, 0.1)      # m/s, output
    
        # Input
        self.x_input = ctrl.Antecedent(self.x_universe, 'x')
        self.y_input = ctrl.Antecedent(self.y_universe, 'y')
        self.z_input = ctrl.Antecedent(self.z_universe, 'z')
        # Output
        self.vx = ctrl.Consequent(self.v_universe, 'vx')
        self.vy = ctrl.Consequent(self.v_universe, 'vy')
        self.vz = ctrl.Consequent(self.vz_universe, 'vz')
    
    def Membership(self):
        # Membership function of inputs
        self.x_input['right'] = fuzz.zmf(self.x_input.universe, -150, 0)
        self.x_input['mid']   = fuzz.trimf(self.x_input.universe, (-45, 0, 45))
        self.x_input['left']  = fuzz.smf(self.x_input.universe, 0, 150)

        self.y_input['bottom'] = fuzz.zmf(self.y_input.universe, -240, 0)
        self.y_input['mid']    = fuzz.trapmf(self.y_input.universe, [-100, -50, 50, 100])
        self.y_input['top']    = fuzz.smf(self.y_input.universe, 0, 240)

        if self.mode == 'far_to_waypoint':
            self.z_input['far']  = fuzz.trapmf(self.z_input.universe, [380, 900, 1500, 1500])            
            self.z_input['near']  = fuzz.trapmf(self.z_input.universe, [0, 0, 200, 750])
        else:
            self.z_input['far']  = fuzz.trapmf(self.z_input.universe, [200, 280, 300, 500])            
            self.z_input['near']  = fuzz.trapmf(self.z_input.universe, [0, 0, 100, 300])

        # Membership function of outputs. 
        self.vx['left']  = fuzz.trapmf(self.vx.universe, [-4, -4, -2, 0])
        self.vx['mid']   = fuzz.trapmf(self.vx.universe, [-1.0, -0.5, 0.5, 1.0])
        self.vx['right'] = fuzz.trapmf(self.vx.universe, [0, 2, 4, 4])

        self.vy['top']    = fuzz.trapmf(self.vy.universe, [-4, -4, -2, 0])
        self.vy['mid']    = fuzz.trapmf(self.vy.universe, [-1.2, -0.8, 0.3, 0.7])
        self.vy['bottom'] = fuzz.trapmf(self.vy.universe, [0, 2, 4, 4])

        self.vz['fast'] = fuzz.pimf(self.vz.universe, -0.3, 1.2, 1.5, 1.5)
        self.vz['slow'] = fuzz.zmf(self.vz.universe, -1.5, 1.0) 
    
    def viewGraphic(self):
        self.x_input.view() #1
        self.y_input.view() #2
        self.z_input.view() #3
        self.vx.view()      #4
        self.vy.view()      #5
        self.vz.view()      #6
        plt.show()
    
    def FuzzyRule(self):
        self.Membership()
        self.rule1  = ctrl.Rule(self.x_input['left'] & self.y_input['top'] & self.z_input['near']    , consequent=(self.vx['right'], self.vy['bottom'], self.vz['slow']))
        self.rule2  = ctrl.Rule(self.x_input['left'] & self.y_input['top'] & self.z_input['far']     , consequent=(self.vx['right'], self.vy['mid'], self.vz['fast']))
        self.rule3  = ctrl.Rule(self.x_input['left'] & self.y_input['mid'] & self.z_input['near']    , consequent=(self.vx['right'], self.vy['top'], self.vz['slow']))
        self.rule4  = ctrl.Rule(self.x_input['left'] & self.y_input['mid'] & self.z_input['far']     , consequent=(self.vx['right'], self.vy['mid'], self.vz['fast']))
        self.rule5  = ctrl.Rule(self.x_input['left'] & self.y_input['bottom'] & self.z_input['near'] , consequent=(self.vx['right'], self.vy['top'], self.vz['slow']))
        self.rule6  = ctrl.Rule(self.x_input['left'] & self.y_input['bottom'] & self.z_input['far']  , consequent=(self.vx['right'], self.vy['mid'], self.vz['fast']))

        self.rule7  = ctrl.Rule(self.x_input['mid'] & self.y_input['top'] & self.z_input['near']     , consequent=(self.vx['mid'], self.vy['bottom'], self.vz['slow']))
        self.rule8  = ctrl.Rule(self.x_input['mid'] & self.y_input['top'] & self.z_input['far']      , consequent=(self.vx['mid'], self.vy['mid'], self.vz['fast']))
        self.rule9  = ctrl.Rule(self.x_input['mid'] & self.y_input['mid'] & self.z_input['near']     , consequent=(self.vx['mid'], self.vy['top'], self.vz['slow']))
        self.rule10 = ctrl.Rule(self.x_input['mid'] & self.y_input['mid'] & self.z_input['far']      , consequent=(self.vx['right'], self.vy['mid'], self.vz['fast']))
        self.rule11 = ctrl.Rule(self.x_input['mid'] & self.y_input['bottom'] & self.z_input['near']  , consequent=(self.vx['mid'], self.vy['top'], self.vz['slow']))
        self.rule12 = ctrl.Rule(self.x_input['mid'] & self.y_input['bottom'] & self.z_input['far']   , consequent=(self.vx['mid'], self.vy['mid'], self.vz['fast']))

        self.rule13 = ctrl.Rule(self.x_input['right'] & self.y_input['top'] & self.z_input['near']   , consequent=(self.vx['left'], self.vy['bottom'] , self.vz['slow']))
        self.rule14 = ctrl.Rule(self.x_input['right'] & self.y_input['top'] & self.z_input['far']    , consequent=(self.vx['left'], self.vy['mid'] , self.vz['fast']))
        self.rule15 = ctrl.Rule(self.x_input['right'] & self.y_input['mid'] & self.z_input['near']   , consequent=(self.vx['left'], self.vy['top'] , self.vz['slow']))
        self.rule16 = ctrl.Rule(self.x_input['right'] & self.y_input['mid'] & self.z_input['far']    , consequent=(self.vx['left'], self.vy['mid'] , self.vz['fast']))
        self.rule17 = ctrl.Rule(self.x_input['right'] & self.y_input['bottom'] & self.z_input['near'], consequent=(self.vx['left'], self.vy['top'] , self.vz['slow']))
        self.rule18 = ctrl.Rule(self.x_input['right'] & self.y_input['bottom'] & self.z_input['far'] , consequent=(self.vx['left'], self.vy['mid'] , self.vz['fast']))
        
        return self.rule1, self.rule2, self.rule3, self.rule4, self.rule5, self.rule6, self.rule7, self.rule8, self.rule9, self.rule10, self.rule11, self.rule12, self.rule13, self.rule14, self.rule15, self.rule16, self.rule17, self.rule18
    
    def pre_fzprocess(self):
        self.rules     = self.FuzzyRule()
        self.fzcontrol = ctrl.ControlSystem(self.rules)
        self.fzmotion  = ctrl.ControlSystemSimulation(self.fzcontrol)
    
    def fzprocess(self, delta_x, delta_y, distance):
        #self.rules = self.FuzzyRule()
        #self.fzcontrol = ctrl.ControlSystem(self.rules)
        #self.fzmotion  = ctrl.ControlSystemSimulation(self.fzcontrol)
                
        self.fzmotion.input['x'] = delta_x
        self.fzmotion.input['y'] = delta_y
        self.fzmotion.input['z'] = distance
        time_s = time.time()
        self.fzmotion.compute()
        time_e = time.time()
        time_cost = time_e - time_s
        
        return self.fzmotion.output['vx'], self.fzmotion.output['vy'], self.fzmotion.output['vz'], time_cost
        
if __name__ == '__main__':
    print('Testing for this file, it will output vx, vy, vz and cost time.')
    fz = FuzzyControl(mode='near_to_waypoint')   
    fz.pre_fzprocess()
    vx, vy, vz, cost_t = fz.fzprocess(delta_x=220, delta_y=0, distance=500)    
    if fz.mode == 'near_to_waypoint':
        vx = vx/2
        vy = vy/2
        vz = vz/2
    print(round(vx,2))
    print(round(vy,2))
    print(round(vz,2))
    print(cost_t)
    
    fz.viewGraphic()