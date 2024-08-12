#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time
import numpy as np
#Import files
import sim_interface
import control
import maps
import searchAstar as  search


map = maps.maps_dictionary[1]
map.goal = [9,9]


def main():
    if (sim_interface.sim_init()):
        #Obtain handles to sim elements
        sim_interface.get_handles()
        #Start simulation
        if (sim_interface.start_simulation()):
            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)
            
        
            robot_state = sim_interface.localize_robot()
            
            
            # for bill in bill_state:
            #     map.map_data[int(bill[0])][int(bill[1])] = maps.bill_id
            intrim_gs = []
            while intrim_gs == [] :    
                bill_state = sim_interface.localize_bills()
                intrim_gs = search.get_path(robot_state, map, bill_state, True)
                time.sleep(0.25)
            
            old_bill_state = bill_state.copy()
            print("new path  ",  intrim_gs)
            change_in_proximity = False
            i = 0
            V = 0 
            W = 0

            while not control.at_goal(robot_state, map.goal):

                if change_in_proximity :
                    intrim_gs = search.get_path(robot_state, map, bill_state, change_in_proximity)
                    i = 0
                    print("new path  ",  intrim_gs)
                    if control.at_goal(robot_state, map.goal):
                        print("reached")
                        break
                      
                if intrim_gs != []:

                    if np.linalg.norm(np.array(robot_state[0:2]) - np.array(intrim_gs[i])) <= 0.5 and i<len(intrim_gs)-1 :
                        i = i + 1
                    
                    [V,W] = control.gtg(robot_state, intrim_gs[i])
                    sim_interface.setvel_pioneers(V, W)
                    

                else :
                    sim_interface.setvel_pioneers(0,W)

                time.sleep(0.25)
                robot_state = sim_interface.localize_robot()
                bill_state = sim_interface.localize_bills()
                change_in_proximity = False
                for j in range(len(bill_state)):
                    bill =np.array( bill_state[j])
                    if np.linalg.norm(np.array(old_bill_state[j])-bill)>=0.75 and np.linalg.norm(robot_state[0:2]-bill)<= 2.5 :
                        change_in_proximity = True
                        old_bill_state = bill_state 
                
            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    sim_interface.setvel_pioneers(0.0, 0.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':
    main()                    
    print ('Program ended')            
