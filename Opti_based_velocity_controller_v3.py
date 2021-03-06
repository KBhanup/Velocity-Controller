#!/usr/bin/python
import threading

import rospy as rp
import math

from mavros_msgs.srv import SetMode, CommandBool
from std_srvs.srv import SetBool

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_control_msgs.msg import Setpoint             ##Needs to import this package of create msg file
from mavros_msgs.msg import PositionTarget, State


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert rpy to quaternions
    """
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

def to_rpy(qw, qx, qy, qz):
    """
    Convert quaternions to rpy
    """
    r = math.atan2(2 * (qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))

    sinP = 2 * (qw*qy - qz*qx)
    if abs(sinP) > 1:
        p = math.copysign(1.0, sinP) * math.pi / 2
    else:
        p = math.asin(sinP)

    y = math.atan2(2 * (qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    return [r, p, y]

def rotate_vector(x, y, angle):
    x_r = x * math.cos(angle) - y * math.sin(angle)
    y_r = x * math.sin(angle) + y * math.cos(angle)
    return [x_r, y_r]

class JoyOptiNode():
    def __init__(self, rate):
        rp.init_node('joy_Opti_node')

        self.rate = rate

        # Service clients
        self.set_mode_serv = rp.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_serv = rp.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.ctrl_serv = rp.ServiceProxy('/enable_controller', SetBool)

        # Subscribers
        self.state_sub = rp.Subscriber('/mavros/state', State, self.stateCallback, queue_size=1)
        self.pose_sub = rp.Subscriber('/mavros/local_position/odom', PoseStamped, self.pose_callback, queue_size=1)
        #self.joy_sub = rp.Subscriber('/joy', Joy, self.joyCallback, queue_size=1)

        # Publishers
        self.vel_cmd_pub = rp.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.setpoint_pub = rp.Publisher('/desired_setpoint', Setpoint, queue_size = 1)



        # Check that services are available
        rp.loginfo('Checking that services are available')
        rp.wait_for_service('/mavros/set_mode')
        rp.wait_for_service('/mavros/cmd/arming')
        rp.wait_for_service('/enable_controller')
        rp.loginfo('MavROS services are available')

        # Velocity cmds
        self.vel_x_cmd = 0.0
        self.vel_y_cmd = 0.0
        self.vel_z_cmd = 0.0
        self.vel_q_cmd = 0.0

        # # Control switches
        # self.arm_bttn = 0
        # self.ofb_bttn = 0        #why?
  

        # Status
        self.connected = False
        self.armed = False
        self.mode = None
    
        # Set up Controllers
        self.kp_x = 0.18
        self.kd_x = 0.15
        self.kp_y = 0.18
        self.kd_y = 0.15
        self.kp_z = 0.3
        self.kd_z = 0.2
        self.kp_w = 0.15
        
        self.Ekx = 0.0
        self.Eky = 0.0
        self.Ekz = 0.0
        self.Epx = 0.0           ## Ep = Ek-1
        self.Epy = 0.0
        self.Epz = 0.0
        self.yaw_des = 90.0  #Yaw in degrees
        self.w_error = 0.0

        # Variables
        self.lastOnline = 0
        self.arm = 0
        self.on_mission = False
        # self.mission_id = 1
        self.state_id = 0
        
        self.w_drone = 0.0
        

        #Setup the Desired Position to Hover
        setpoint_msg = Setpoint()
        self.setpoint_msg.position.x = 0.0
        self.setpoint_msg.position.y = 0.0
        self.setpoint_msg.position.z = 1.5
        setpoint_msg.velocity.x = 0.0
        setpoint_msg.velocity.y = 0.0
        setpoint_msg.velocity.z = 0.0
        setpoint_msg.orientation.x = 0.0
        setpoint_msg.orientation.y = 0.0
        setpoint_msg.orientation.z = 0.0
        self.setpoint_pub.Publish(setpoint_msg)
        

##Planned Strategy: 1. get position from OptiTrack 2.Read the Desired set point to be at 3.Compare where you are to where you have to be (error)
## 4.Using the PD loop calculate the Input command 5.Clip the input cmd between -1 & 1 6.Supply Drone  cmd_msg.velocity.x,y,z with the respective U_inpputs
# Setup Publisher

    def stateCallback(self, msg):
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

        
        t = threading.Thread(target=self.commandPublisher)
        t.start()

        rp.spin()
    #Postion CallBack
    def pose_callback(self, optidata):
        
        timeNow= optidata.header.stamp
        # optidata.header.frame_id = ''    
        self.X= optidata.pose.position.x         
        self.Y= optidata.pose.position.y
        self.Z= optidata.pose.position.z

        self.qx= optidata.pose.orientation.x          
        self.qy= optidata.pose.orientation.y
        self.qz= optidata.pose.orientation.z 
        self.qw= optidata.pose.orientation.w 

        self.rpy = to_rpy(self.qw, self.qx, self.qy, self.qz)
        self.yaw_position = self.rpy[2]  #self.yaw_position= -rpy[2]???-??
    ##**************
        self.Epx = self.Ex           ## Ep = Ek-1
        self.Epy = self.Ey
        self.Epz = self.Ez

    
        # Position conversions where the reported position is in terms of the camera frame
        
        self.Ex = self.setpoint_msg.position.x - self.X
        self.Ey = self.setpoint_msg.position.y - self.Y
        self.Ez = self.setpoint_msg.position.z - self.Z        
        yaw_diff = self.yaw_des -  self.yaw_position

        # Translate error from optitrack frame to drone body frame
        drone_error = rotate_vector(self.Ex, self.Ey,self.yaw_position)

        # Update x and y error 
        self.Ex = drone_error[0]
        self.Ey = drone_error[1]

        if yaw_diff > 180.0:
            self.yaw_error = yaw_diff - 360.0
        else:
            self.yaw_error = yaw_diff

    
    #****************

    def clip_command(self, cmd, upperBound, lowerBound):                            #what is cmd? for later
        if cmd < lowerBound:
            cmd = lowerBound
        elif cmd > upperBound:
            cmd = upperBound

        return cmd

    
    #rp.spin()
     

    def commandPublisher(self):
        r = rp.Rate(self.rate)
        cmd_msg = PositionTarget()
        cmd_msg.coordinate_frame = PositionTarget().FRAME_BODY_NED
        cmd_msg.type_mask = PositionTarget().IGNORE_PX + \
                            PositionTarget().IGNORE_PY + \
                            PositionTarget().IGNORE_PZ + \
                            PositionTarget().IGNORE_AFX + \
                            PositionTarget().IGNORE_AFY + \
                            PositionTarget().IGNORE_AFZ + \
                            PositionTarget().IGNORE_YAW
        cmd_msg.velocity.x = 0.0
        cmd_msg.velocity.y = 0.0
        cmd_msg.velocity.z = 0.0
        cmd_msg.yaw_rate = 0.0


        # Generate commands
        self.uX = self.kp_x * self.Ex + self.kd_x * (self.Ex - self.Epx)
        self.uY = self.kp_y * self.Ey + self.kd_y * (self.Ey - self.Epy)
        self.uZ = self.kp_z * self.Ez + self.kd_z * (self.Ez - self.Epz)
        self.uW = self.kp_w * self.w_error
        
        self.vel_x_cmd = self.clip_command(uX, 1, -1)              #1Max_vel  -1Min_velocity
        self.vel_y_cmd = self.clip_command(uY, 1, -1)
        self.vel_z_cmd = self.clip_command(uZ, 1, -1)
        self.vel_q_cmd = self.clip_command(uW, 1, -1)

        while not rp.is_shutdown():
         if self.connected:
            # Make sure we are in the correct flight mode
            if self.ofb_bttn == 0.0 and self.mode != 'POSCTL':
                self.set_mode_serv(0, 'POSCTL')
            elif self.ofb_bttn == 1.0 and self.mode != 'OFFBOARD':
                self.set_mode_serv(0, 'OFFBOARD')

            # Arm if requested and not armed
            if self.arm_bttn == 1.0 and not self.armed:
               self.arm_serv(True)        
        

            if self.armed:
               cmd_msg.velocity.x = self.vel_x_cmd
               cmd_msg.velocity.y = self.vel_y_cmd
               cmd_msg.velocity.z = self.vel_z_cmd
               cmd_msg.yaw_rate = self.vel_q_cmd

               cmd_msg.header.stamp = rp.Time.now()
               self.vel_cmd_pub.publish(cmd_msg)

            else:
                rp.loginfo('Vehicle not connected')

        r.sleep()     
      
      
if __name__ == '__main__':
      JoyOptiNode(30)
      rp.spin()
