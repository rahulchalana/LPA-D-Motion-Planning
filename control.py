import robot_params
import numpy as np 

prev_heading_error = 0.0
total_heading_error = 0.0

def at_goal(robot_state, goal_state):    
    
    #check if we have reached goal point
    d = np.sqrt(((goal_state[0] - robot_state[0])**2) + ((goal_state[1] - robot_state[1])**2))
    
    if d <= robot_params.goal_threshold:
        print("Reached goal")
        print(robot_state)
        print(goal_state)
        
        return True
    else:
        return False

def gtg(robot_state, goal_state):  
    #The Go to goal controller
    
    global prev_heading_error
    global total_heading_error   
    
    #Controller parameters
    Kp = 0.00656
    Kd = 0.0001
    Ki = 0.0
    dt = 0.5

    #determine how far to rotate to face the goal point
    #PS. ALL ANGLES ARE IN RADIANS
    delta_theta = (np.arctan2((goal_state[1] - robot_state[1]), (goal_state[0] - robot_state[0]))) - robot_state[2]
    #restrict angle to (-pi,pi)
    delta_theta = ((delta_theta + np.pi)%(2.0*np.pi)) - np.pi
    
    #Error is delta_theta in degrees
    e_new = ((delta_theta*180.0)/np.pi)
    e_dot = (e_new - prev_heading_error)/dt 
    total_heading_error = (total_heading_error + e_new)*dt
    #control input for angular velocity
    W = (Kp*e_new) + (Ki*total_heading_error) + (Kd*e_dot)
    prev_heading_error = e_new
  
    #find distance to goal
    d = np.sqrt(((goal_state[0] - robot_state[0])**2) + ((goal_state[1] - robot_state[1])**2))
    
    #velocity parameters
    distThresh = 0.1#mm
    
    #control input for linear velocity
    V =  (0.2)*(np.arctan(d + distThresh)) 
    
    #request robot to execute velocity
    return[V,W]
                                       
                   
