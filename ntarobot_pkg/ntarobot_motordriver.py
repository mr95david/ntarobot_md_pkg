#!/usr/bin/env python3
# Motor Driver 49, Node controller
# April 22, 2024. Universidade Federal do Espirito Santo.
# Elio Triana.

# Library Section
import rclpy
from rclpy.node import Node
# Parameters library
from rcl_interfaces.msg import SetParametersResult
from rclpy import Parameter
# Mensage libraries
from solver_untils.msg import RobotGeneralP
from geometry_msgs.msg import TwistStamped
# Untils libraries
from ntarobot_pkg import untils_md as md_
from ntarobot_pkg import conn_md as cnn_
# Extern libraries
import numpy as np

# General Node Class
class MotorDriverController(Node):
    # Node Inicialization
    def __init__(self) -> None:
        super().__init__("md49_controller_node")
        # First: declare parameters
        # Includes all parameters to connect and use motor driver card
        self.declare_parameters(
            namespace='', # Default value: ""
            parameters = [
                ('port_name', Parameter.Type.STRING), 
                ('serial_baud', Parameter.Type.INTEGER),
                ('mode_p', Parameter.Type.INTEGER),
                ('mode_aceleration', Parameter.Type.INTEGER),
                ('mode_regulator', Parameter.Type.BOOL),
                ('mode_timeout', Parameter.Type.BOOL),
                ('speed_value_r', Parameter.Type.INTEGER),
                ('speed_value_l', Parameter.Type.INTEGER),
                ('wheel_radius', Parameter.Type.DOUBLE),
                ('wheel_separation', Parameter.Type.DOUBLE)
            ]
        )

        # Variable declaration section
        # Instance object values
        self.mode_p = int(self.get_parameter('mode_p').value) # Value of mode for use
        self.mode_acc = int(self.get_parameter('mode_aceleration').value) # Value of mode for use
        self.mode_reg = bool(self.get_parameter('mode_regulator').value) # Value of mode for use
        self.mode_tm = bool(self.get_parameter('mode_timeout').value) # Value of mode for use
        self.value_rspeed = int(self.get_parameter('speed_value_r').value) # Value of mode for use
        self.value_lspeed = int(self.get_parameter('speed_value_l').value) # Value of mode for use
        # Memory for last speed value
        self.last_valueRspeed: int = 128
        self.last_valueLspeed: int = 128

        # Robot Dimensions Instance
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.value_w_r = 10.273
        self.value_w_i = 10.242

        # Variable for check seral connection
        self.serial_connection = True

        # Last values inicializations
        self.last_stateConn = None # Memory last-state connection from driver
        self.last_serialConnection = None # Memory of serial connection
        self.last_voltageState = None # Memory of voltage state

        # Transform matrix
        self.speed_conversion_ = np.array([
            [self.wheel_radius/2, self.wheel_radius/2],
            [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]
        ])

        # Inicialization of driver connect
        self.portConn_md = cnn_.driverMD49(
            namePort = self.get_parameter('port_name').value,
            baudRate = self.get_parameter('serial_baud').value,
            timeOut = 1.0 # Default = 1.0 seconds
        )

        # Subscribers inicialization
        self.init_subscribers_()

        # Publishers inicialization
        self.init_publishers_()

        # Timers inicialization
        self.init_timers_()
        
    # Publishers Section
    def init_publishers_(self):
        # General publisher data from robot
        self.descRobot_publisher_ = self.create_publisher(
            RobotGeneralP,
            '/ntaRobot/acState',
            10 # Default value 10
        )
        
    # Subscribers Section
    def init_subscribers_(self):
        # General subscribers variables
        # Subscriber for Cmd_vel changes
        self.cmdVel_subscriber_ = self.create_subscription(
            TwistStamped, # This can change for Twist
            '/ntaRobot/cmd_vel',
            self.callback_cmdVel,
            10 # Default value 10
        )
        self.cmdVel_subscriber_

    # Timers Section
    def init_timers_(self) -> None:
        # Timer to check actual connection from driver
        self.timer_stateConn = self.create_timer(
            1.0, # Default value 0.01
            self.callback_stateConn
        )
        # Timer to get description of actual robot
        self.timer_descRobot = self.create_timer(
            1.0, # Low frecuency for description Robot
            self.callback_robParam
        )
        # Timer for change speed motor
        self.timer_speedC = self.create_timer(
            0.01,
            self.callback_speedC
        )

    # Callback Section
    def callback_stateConn(self) -> None:
        # Function for validate state of connection from motor driver
        # Connection validation
        if not self.portConn_md.stateConn:
            # Initialice variables for posible re-connection 
            self.serial_connection = False
            self.last_serialConnection = False
            self.mode_reg = False
            # Check errors and reconection
            self.get_logger().error(self.portConn_md.errConn)
            self.serial_connection = self.portConn_md._initPort()
            
        # Msg for complete connection
        if self.portConn_md.stateConn:
            # Validation and enable Regulator
            if not self.mode_reg:
                # Try enable regulator from driver
                try:
                    _ = self.portConn_md.EnableRegulator()
                    self.mode_reg = True
                except Exception as e:
                    self.get_logger().error("Error trying to enable encoder regulation")
            # Validate State of Connection
            if self.last_serialConnection != self.serial_connection:
                # Validation of serial connection
                self.get_logger().info("Connection completed")
                self.last_serialConnection = self.serial_connection
        
        # Update Serial connection
        self.portConn_md._updateState()

    # Callback for actual parameters of robot
    def callback_robParam(self):
        # Output msg from values of robot
        # print(self.portConn_md.stateConn)
        msg: RobotGeneralP = RobotGeneralP()

        # Validate Actual Conection
        if self.portConn_md.stateConn and self.serial_connection:
            # Try get voltage state
            try:
                actual_voltage = self.valFunction(self.portConn_md.GetVolts())
            except Exception as e:
                actual_voltage = None

            # Validation for publish description actual robot
            if actual_voltage != None and self.last_voltageState != actual_voltage:
                msg.statedata = "Connected"
                msg.version = self.valFunction_int(self.portConn_md.GetVersion())
                msg.mode = self.valFunction_int(self.portConn_md.GetMode())
                msg.aceleration = self.valFunction_int(self.portConn_md.GetAcceleration())
                msg.regulatorstate = md_.DICT_STATE_PARAMS[self.mode_reg]
                msg.voltage = self.valFunction(self.portConn_md.GetVolts())

                # Publish Description robot
                self.descRobot_publisher_.publish(msg)

                # Set actualVoltage to last voltage
            self.last_voltageState = actual_voltage

    # Callback from speed TwistStamped values
    def callback_cmdVel(self, msg: TwistStamped):
        # Declare angular and linears values
        velLineal: float = float(msg.twist.linear.x)
        velAngular: float = float(msg.twist.angular.z)

        # Calculate speed robot matrx
        robot_speed = np.array([
            [velLineal],
            [velAngular]
        ])

        # Wheel speed calculate
        wheel_speed = np.matmul(np.linalg.inv(
            self.speed_conversion_), robot_speed
        ) 

        # Response 0 - 255 values for speed order from driver
        vel_rot_wd = int(round(wheel_speed[1, 0]*self.value_w_r+128))
        vel_rot_wi = int(round(wheel_speed[0, 0]*self.value_w_i+128))

        # Assing new value
        if vel_rot_wd != self.value_rspeed:
            self.value_rspeed = vel_rot_wd

        if vel_rot_wi != self.value_lspeed:
            self.value_lspeed = vel_rot_wi

        #self.get_logger().info(f"VD: {vel_rot_wd} rd/s - r_1: {int(vel_rot_wd)} - r_2: {int(round(vel_rot_wd))}")
        #self.get_logger().info(f"VI: {vel_rot_wi} rd/s - r_1: {int(vel_rot_wi)} - r_2: {int(round(vel_rot_wd))}")

    # Callback for change speed
    def callback_speedC(self):
        if self.portConn_md.stateConn and self.serial_connection:
            _ = self.portConn_md.setSpeed1(self.value_rspeed)
            _ = self.portConn_md.setSpeed2(self.value_lspeed)
            #self.get_logger().info(f"D: {self.value_rspeed}")
            #self.get_logger().info(f"I:    {self.value_lspeed}")

    # Functions section
    # Float function validation
    def valFunction(self, value):
        if value == None:
            return 0.0
        return float(value)
    # Int Function validation
    def valFunction_int(self, value):
        if value == None:
            return 0
        return int(value)

    # Close function
    def closePortFunction(self):
        self.portConn_md._closePort()
        

# Main Funciton def
def main(args = None):
    # Inicialization of rclpy
    rclpy.init(args = args)

    # Create object node
    md_driver = MotorDriverController()
    try:
        # Spin function node in ros2
        rclpy.spin(md_driver)
    except KeyboardInterrupt:
        # include close ports, and close files if u need
        
        pass # Decomment if don't have any value here
    finally:
        # destroy default node
        md_driver.closePortFunction()
        md_driver.destroy_node()
    
    # State validation  
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()