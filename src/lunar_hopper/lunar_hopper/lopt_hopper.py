import mpopt_lulav as mp_l
import matplotlib.pyplot as plt
import numpy as np
import casadi as ca
import math
from scipy.interpolate import interp1d
from typing import Callable

###########################################################

def interpolate(t, array): #interpolating results
    t_new = np.linspace(0, int(np.around(t[len(t)-1])), num=int(np.around(t[len(t)-1]))+1)
    array_new = np.interp(t_new, t[:,0], array)
    #t_new = np.append (t_new, math.ceil(t_new[-1]))
    #array_new = np.append(array_new, array_new[-1])

    return (t_new, array_new)

def solve(ros_node,
          dyn_func : Callable =  lambda x, u, t: [x[1], u[0] - 1.5],
          term_cost: Callable = lambda xf, tf, x0, t0: tf,
          term_constr: Callable = lambda xf, tf, x0, t0: [],
          ocp_tf0: int = 10, ocp_x00: list = [], ocp_xf0: list = [], ocp_lbx: list = [], 
          ocp_ubx: list = [], ocp_lbu: list = [], ocp_ubu: list = [], ocp_btf: list = [], ocp_u00: list = []):

    
    '''
    Lulav Space optimal control problems solver.
    This is the main function providing single-phase optimal control simulation.
    Parameters
    ----------
    dyn_fuction : Function (x,u,t)
        A function with simulation subject dynamics expressions. Input must be '(x, u, t)', output must match the states vector.

    term_cost : Function (xf, tf, x0, t0)
        Terminal cost function. Input must be '(xf, tf, x0, t0)'. Output parameters will be minimized during optimization.  

    term_constr : Function (xf, tf, x0, t0)
        Terminal constraints function. Input must be '(xf, tf, x0, t0)'.

    ocp_tf0 : int
        initial guess for simulation duration
    ocp_x00 : list
        initial guess for initial states values. The lengh must match the number of states.
    ocp_xf0 : list
        initial guess for final states values. The lengh must match the number of states.
    ocp_lbx : list
        lower bounds for states values. The lengh must match the number of states.
    ocp_ubx : list
        upper bounds for states values. The lengh must match the number of states.
    ocp_lbu : list
        lower bounds for controls values. The lengh must match the number of controls.
    ocp_ubu : list
        upper bounds for controls values. The lengh must match the number of controls.
    ocp_btf : list
        lower and upper bounds for time.
    ocp_u00 : list
        initial control's values.

    Returns
    -------
    x : List of lists 
        2-dimensional array of interpolated states
    u : List of lists 
        2-dimensional array of interpolated controls
    t : List 
        interpolated time grid
    '''

    ocp = mp_l.OCP(n_states=5, n_controls=2, n_phases=1) #Setting parameters for MPOPT

    if ocp_tf0:
        ocp.tf0[0] = ocp_tf0 # guess tf
    if ocp_xf0:
        ocp.xf0[0] = ocp_xf0 # target conditions
    if ocp_x00:
        ocp.x00[0] = ocp_x00 #starting conditions
    if ocp_lbx:    
        ocp.lbx[0] = ocp_lbx
    if ocp_ubx:    
        ocp.ubx[0] = ocp_ubx
    if ocp_lbu:
        ocp.lbu[0] = ocp_lbu #u_x, u_y, u_z
    if ocp_ubu:
        ocp.ubu[0] = ocp_ubu #u_x, u_y, u_z
    if ocp_ubu:
        ocp.u00[0] = ocp_u00
    if ocp_btf:
        ocp.lbtf[0], ocp.ubtf[0] = ocp_btf[0], ocp_btf[1] # tf_min tf_max

    ocp.dynamics[0] = dyn_func 
    ocp.terminal_constraints[0] = term_constr
    ocp.terminal_costs[0] = term_cost

    ocp.validate()
    
    ###################################

    #Running
    ros_node.get_logger().info(f"Starting simulation...")

    mpo = mp_l.mpopt_adaptive(ocp, 6, 4) #6.4 
    mpo.colloc_scheme = 'LGR'
    sol = mpo.solve() 
    post= mpo.process_results(sol,plot=False)

    #Getting results
    x_output, u_output, t_output, a_output = post.get_data() #Setting results for output

    x_temp=np.empty((int(np.around(t_output[len(t_output)-1]))+1,5)) #Setting temporary array for interpolating
    u_temp=np.empty((int(np.around(t_output[len(t_output)-1]))+1,3)) #Setting temporary array for interpolating

    #Sending data for interpolation
    for i in range(5): 
        t_new, x_temp[:,i] = interpolate(t_output, x_output[:,i])

    for i in range(2):
        t_new, u_temp[:,i] = interpolate(t_output, u_output[:,i])

    t_output = t_new
    x_output = x_temp
    u_output = u_temp
        
    ros_node.get_logger().info(f"Simulation finished successfully!")

    

    return (x_output, u_output, t_output)