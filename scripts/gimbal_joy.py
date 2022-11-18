#! /usr/bin/env python3

# General Dependancies
import rospy
from sensor_msgs.msg import Joy
from dynamixel_workbench_msgs.srv import DynamixelCommand
# from yaml import safe_load


class gimbaljoyClass():
    def __init__(self):
        dynamixel_service = "collimated_gimbal/dynamixel_command"

        self.rate = rospy.Rate(10) # 2 Hz

        self.sub_topic_name = 'joy'
        self.joy_deadman = 1 # Circle button on PS4 controller
        self.joy_x_axis = 3  # Used for Yaw
        self.joy_y_axis = 4  # Used for Pitch

        self.id_roll = 83
        self.id_pitch = 82
        self.id_yaw = 81

        self.enabled_roll = False
        self.enabled_pitch = True
        self.enabled_yaw = False

        self.limit_angle_roll = 90.0  # +/- degrees
        self.limit_angle_pitch = 45.0 # +/- degrees
        self.limit_angle_yaw = 90.0   # +/- degrees


        # NEED TO READ YAML TO GET DEFAULTS FOR IDS AND CONTROL TABLE
        self.goal_roll = 0
        self.goal_pitch = 0
        self.goal_yaw = 0

        rospy.loginfo("Waiting for Service: %s", dynamixel_service)
        rospy.wait_for_service(dynamixel_service)
        rospy.loginfo("Service acquired")
        self.dynamixel_caller = rospy.ServiceProxy(dynamixel_service, DynamixelCommand, persistent=True)

        rospy.on_shutdown(self.shutdownhook) # Custom Shutdown Hook

        rospy.loginfo("Requesting Intital Joint Positions")

        # Initialise Joints
        self.initJoints()

        rospy.sleep(5) # Sleeps for 5 sec to ensure the joints are initialised

        rospy.Subscriber(self.sub_topic_name, Joy, self.joycallback)

        # Disengage acceleration limits when using joystick
        self.safetylimits(False)


        while not rospy.is_shutdown():
            # Limit Goal calls to rate of loop
            self.jointRequester(self.goal_roll, self.goal_pitch, self.goal_yaw)
            self.rate.sleep()

        

    def joycallback(self, msg):
        roll_val = 0
        pitch_val = 0
        yaw_val = 0
        # If deadman switch is engaged
        if msg.buttons[self.joy_deadman]:
            pitch_val = msg.axes[self.joy_y_axis]
            yaw_val = msg.axes[self.joy_x_axis]

        # Save current values
        self.goal_roll = roll_val
        self.goal_pitch = pitch_val
        self.goal_yaw = yaw_val


    def initJoints(self, home=False):
        self.torqueEnable(True)
        rospy.sleep(1)
        self.safetylimits(True)
        rospy.sleep(1)

        # Roll
        if self.enabled_roll:
            self.dynamixel_caller("", self.id_roll, "Goal_Position", 2048)

        # Pitch
        if self.enabled_pitch:
            if home:
                self.dynamixel_caller("", self.id_pitch, "Goal_Position", 1024)
            else:
                self.dynamixel_caller("", self.id_pitch, "Goal_Position", 2048)
        
        # Yaw
        if self.enabled_yaw:
            self.dynamixel_caller("", self.id_yaw, "Goal_Position", 2048)


    def jointRequester(self, roll, pitch, yaw):

        goal_roll, goal_pitch, goal_yaw = self.input2encoder(roll, pitch, yaw)

        # Roll
        if self.enabled_roll:
            self.dynamixel_caller("", self.id_roll, "Goal_Position", goal_roll)

        # Pitch
        if self.enabled_pitch:
            self.dynamixel_caller("", self.id_pitch, "Goal_Position", goal_pitch)
        
        # Yaw
        if self.enabled_yaw:
            self.dynamixel_caller("", self.id_yaw, "Goal_Position", goal_yaw)

    def input2encoder(self, roll, pitch, yaw):
        # Convert -1 to 1 into 0-4095 for the Dynamixel goal positions
        # Scaled accordingly to angle limits

        # Roll
        out_roll =  2048 * (1 + (roll  * self.limit_angle_roll )/180.0)

        # Pitch
        out_pitch = 2048 * (1 + (pitch * self.limit_angle_pitch)/180.0)

        # Yaw
        out_yaw =   2048 * (1 + (yaw   * self.limit_angle_yaw  )/180.0)

        return int(out_roll), int(out_pitch), int(out_yaw)

    def shutdownhook(self):
        rospy.loginfo("Shutdown Call Received")
        rospy.loginfo("Requesting home positions")
        self.initJoints(home=True) # Return joints to initial "home" positions RPY
        rospy.sleep(5)  # Ensure the motors have got to home positions
        self.torqueEnable(False)
        rospy.sleep(1)        
        rospy.loginfo("Shutting Down")


    def safetylimits(self, enabled):
        if enabled:
            # Roll
            if self.enabled_roll:
                self.dynamixel_caller("", self.id_roll, "Profile_Acceleration", 1000)

            # Pitch
            if self.enabled_pitch:
                self.dynamixel_caller("", self.id_pitch, "Profile_Acceleration", 1000)
            
            # Yaw
            if self.enabled_yaw:
                self.dynamixel_caller("", self.id_yaw, "Profile_Acceleration", 1000)

        else:
            # Roll
            if self.enabled_roll:
                self.dynamixel_caller("", self.id_roll, "Profile_Acceleration", 4000)

            # Pitch
            if self.enabled_pitch:
                self.dynamixel_caller("", self.id_pitch, "Profile_Acceleration", 4000)
            
            # Yaw
            if self.enabled_yaw:
                self.dynamixel_caller("", self.id_yaw, "Profile_Acceleration", 4000)

    def torqueEnable(self,enabled):
        if enabled:
            # Roll
            if self.enabled_roll:
                self.dynamixel_caller("", self.id_roll,  "Torque_Enable", 1)

            # Pitch
            if self.enabled_pitch:
                self.dynamixel_caller("", self.id_pitch, "Torque_Enable", 1)
            
            # Yaw
            if self.enabled_yaw:
                self.dynamixel_caller("", self.id_yaw,   "Torque_Enable", 1)

        else:
            # Roll
            if self.enabled_roll:
                self.dynamixel_caller("", self.id_roll,  "Torque_Enable", 0)

            # Pitch
            if self.enabled_pitch:
                self.dynamixel_caller("", self.id_pitch, "Torque_Enable", 0)
            
            # Yaw
            if self.enabled_yaw:
                self.dynamixel_caller("", self.id_yaw,   "Torque_Enable", 0)





if __name__ == "__main__":
    try:
        rospy.init_node("gimbal_joy")
        rospy.loginfo("Starting Gimbal Joy Control Node")
        s = gimbaljoyClass()
        rospy.spin()
    except rospy.ROSInterruptException: pass