import rclpy
from rclpy.node import Node
import time
import numpy as np

from . import lopt_hopper as lopt

from std_msgs.msg import Float64MultiArray

class lunar_hopper(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/lunar_hopper/state', 10)
        self.control_pub = self.create_publisher(Float64MultiArray , '/lunar_hopper/control', 10)
        self.time_pub = self.create_publisher(Float64MultiArray , '/lunar_hopper/time', 10)

        self.get_logger().info(f"Starting lunar_hopper version = 0.1.0")

        #Defining states
        #Initial state vector
        self.declare_parameter('m_fuel_0', 3543000.0)  # Initial fuel mass, kg

        #Target state vector
        self.declare_parameter('m_fuel_f', 1000.0)  # target fuel mass, kg
        
        #Additional params
        self.declare_parameter('dry_mass', 85000.0 ) # Dry mass, kg
        self.declare_parameter('Fthrustmax', 20295000.0) # Maximum engines thrust, N
        self.declare_parameter('Isp', 308.0) # Impulse, sec

        self.declare_parameter('publish_freq', 1.0)

        self.m_fuel_0 = self.get_parameter('m_fuel_0').get_parameter_value().double_value

        self.m_fuel_f = self.get_parameter('m_fuel_f').get_parameter_value().double_value

        self.dry_mass = self.get_parameter('dry_mass').get_parameter_value().double_value
        self.Fthrustmax = self.get_parameter('Fthrustmax').get_parameter_value().double_value
        self.Isp = self.get_parameter('Isp').get_parameter_value().double_value

        self.get_logger().info(f"With Isp = {self.Isp}")

        # Setting Moon parameters
        self.MoonRadius = 1737.1*1000
        self.Mu_Moon =  6.67*10**(-11) * 7.342e22
        self.g0 = 9.81

        # Defining encounter for publisher
        self.i = 0       

        def dynamics0(x, u, t): #Defining dynamics for first and last phases
        
            f_thrust = self.Fthrustmax*u[1]
            r_inv = 1/(x[0]+ self.MoonRadius)
            g = self.Mu_Moon*(r_inv**2)
            f = f_thrust/(self.dry_mass + x[4])
            vh = x[2]
            vd = x[3]
            vh_dot = f*np.cos(u[0]) - g + x[3]**2*r_inv
            vd_dot = -f * np.sin(u[0]) - x[3]*x[2]*r_inv
            m_dot = -f_thrust/(self.g0 * self.Isp)

            return [vh, vd, vh_dot, vd_dot, m_dot]
        
        def terminal_constraints0(xf, tf, x0, t0):
            tc = [x0[0] - 0, 
                x0[1] - 0, 
                x0[2] - 0, 
                x0[3] - 0, 
                x0[4] - self.m_fuel_0, 
                xf[0] - 0, 
                xf[2] + 0, 
                xf[3] - 0, 
                xf[4] - 0]
            return tc
    
        def terminal_cost0(xf,tf,x0,t0):
            return -xf[1]
        
        #            [ x[0],   x[1],   x[2],   x[3],    x[4] ]
        #            [  h,       d,     Vh,     Vd,    m_fuel]
        tf0 = 100 # guess tf
        xf0 = [0, 100000, 0, 0, self.m_fuel_0] # target conditions
        x00 = [ 0, 0, 0, 0, self.m_fuel_0] #starting conditions
        lbx = [ 0, 0, -100, -100, 0]
        ubx = [50000, 1000000, 1000, 1000, self.m_fuel_0]
        lbu = [-2, 0.01]
        ubu = [2, 1]
        u00 = [np.pi/4, 0.01]
        btf = [100, 10000] # tf_min tf_max


        #################################
        # Calling simulation function using parameters and functions declared above
        self.res_x, self.res_u, self.res_t = lopt.solve(self,
                                                        dyn_func = dynamics0,
                                                        term_cost = terminal_cost0,
                                                        term_constr = terminal_constraints0,
                                                        ocp_tf0 = tf0,
                                                        ocp_btf = btf,
                                                        ocp_lbu = lbu,
                                                        ocp_lbx = lbx,
                                                        ocp_ubu = ubu,
                                                        ocp_ubx = ubx,
                                                        ocp_x00 = x00,
                                                        ocp_xf0 = xf0,
                                                        ocp_u00 = u00)
        
        self.state_msg = Float64MultiArray()
        self.control_msg = Float64MultiArray()
        self.time_msg = Float64MultiArray()
        timer_period = self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(1/timer_period, self.timer_callback)
        
    def timer_callback(self):

        self.state_msg.data = [self.res_x[self.i][0], self.res_x[self.i][1], self.res_x[self.i][2], 
                               self.res_x[self.i][3], self.res_x[self.i][4]]
        self.control_msg.data = [self.res_u[self.i][0], self.res_u[self.i][1]]
        self.time_msg.data = [self.res_t[self.i]]

        self.state_pub.publish(self.state_msg)
        self.control_pub.publish(self.control_msg)
        self.time_pub.publish(self.time_msg)
        self.get_logger().info(f"Publishing state = {self.state_msg.data}, control = {self.control_msg.data}, time = {self.time_msg.data}")

        self.i += 1
        if self.i==len(self.res_t):
            self.get_logger().info('All data published successfully')
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = lunar_hopper()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()