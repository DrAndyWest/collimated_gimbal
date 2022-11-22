#! /usr/bin/env python3

# General Dependancies
import rospy
from sensor_msgs.msg import Joy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelState, DynamixelStateList
from yaml import safe_load


class gimbaljoyClass():
    def __init__(self):
        self.dynamixel_service = "collimated_gimbal/dynamixel_command"
        self.dynamixel_state_topic = "collimated_gimbal/dynamixel_state"

        rospy.loginfo("Waiting for Service: %s", self.dynamixel_service)
        rospy.wait_for_service(self.dynamixel_service)
        rospy.loginfo("Service acquired")
        self.dynamixel_caller = rospy.ServiceProxy(self.dynamixel_service, DynamixelCommand, persistent=True)

        self.dynamixelStateSub = rospy.Subscriber(self.dynamixel_state_topic, DynamixelStateList, self.movingCallback)

        dynamixel_info_file = rospy.get_param('dynamixel_info')
        self.params = self.parseYaml(dynamixel_info_file)

        self.rate = rospy.Rate(10) # 10 Hz

        self.sub_topic_name = 'joy'
        self.joy_deadman = 1 # Circle button on PS4 controller
        self.joy_x_axis = 3  # Used for Yaw
        self.joy_y_axis = 4  # Used for Pitch


        self.enabled_roll = 'roll' in self.params
        self.enabled_pitch = 'pitch' in self.params
        self.enabled_yaw = 'yaw' in self.params

        
        self.id_roll = self.params['roll']['ID'] if self.enabled_roll else 0    # Get motor ID, else set to zero
        self.id_pitch = self.params['pitch']['ID'] if self.enabled_pitch else 0 # Get motor ID, else set to zero
        self.id_yaw = self.params['yaw']['ID'] if self.enabled_yaw else 0       # Get motor ID, else set to zero


        self.limit_angle_roll = 90.0  # +/- degrees
        self.limit_angle_pitch = 45.0 # +/- degrees
        self.limit_angle_yaw = 120.0   # +/- degrees

        self.isMoving = {}
        for key in self.params.keys():
            self.isMoving[key] = False

        self.msgReceived = False

        # NEED TO READ YAML TO GET DEFAULTS FOR IDS AND CONTROL TABLE
        self.goal_roll = 0
        self.goal_pitch = 0
        self.goal_yaw = 0

        rospy.on_shutdown(self.shutdownhook) # Custom Shutdown Hook

        rospy.loginfo("Requesting Intital Joint Positions")

        # Initialise Joints
        self.initJoints()

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
        # rospy.sleep(1)
        self.safetylimits(True)
        # rospy.sleep(1)

        for key in self.params:
            # 2048 is the center position
            goal_int = 1024 if key == 'pitch' and home else 2048
            goal_name = 'home' if home else 'ready'
            try:
                result = self.dynamixel_caller("", self.params[key]['ID'], "Goal_Position", goal_int)
                holdformsg = rospy.wait_for_message(self.dynamixel_state_topic, DynamixelStateList, timeout=1)
                rospy.sleep(0.2)
                while True:
                    if not self.isMoving[key]:
                        rospy.loginfo("%s move to %s position: completed", key, goal_name)
                        break
            except rospy.ServiceException as e:
                rospy.logwarn("Init Goal Positions call failed: %s"%e)


    def jointRequester(self, roll, pitch, yaw):

        goal_dict = self.input2encoder(roll, pitch, yaw)

        for key in self.params:
            try:
                self.dynamixel_caller("", self.params[key]['ID'], "Goal_Position", goal_dict[key])
            except rospy.ServiceException as e:
                rospy.logwarn("Goal Postion call failed: %s"%e)


    def input2encoder(self, roll, pitch, yaw):
        # Convert -1 to 1 into 0-4095 for the Dynamixel goal positions
        # Scaled accordingly to angle limits

        # Roll
        out_roll =  2048 * (1 + (roll  * self.limit_angle_roll )/180.0)

        # Pitch
        out_pitch = 2048 * (1 + (pitch * self.limit_angle_pitch)/180.0)

        # Yaw
        out_yaw =   2048 * (1 + (yaw   * self.limit_angle_yaw  )/180.0)

        output_dict = {'roll': int(out_roll), 'pitch': int(out_pitch), 'yaw': int(out_yaw)}

        return output_dict

    def shutdownhook(self):
        rospy.loginfo("Shutdown Call Received")
        rospy.loginfo("Requesting home positions")
        self.initJoints(home=True) # Return joints to initial "home" positions RPYposition
        self.torqueEnable(False)  # Disable the motors
        rospy.sleep(5)
        rospy.loginfo("Shutting Down")


    def safetylimits(self, enabled):

        for key in self.params:
            result = False
            profile_accel = 1000 if enabled else 4000
            try:
                result = self.dynamixel_caller("", self.params[key]['ID'], "Profile_Acceleration", profile_accel) # Should return True
                rospy.sleep(0.1)
                rospy.loginfo("%s joint accel limit: %d", key, profile_accel)
            except rospy.ServiceException as e:
                rospy.logwarn("Profile Acceleration call failed: %s"%e)


    def torqueEnable(self,enabled):

        for key in self.params:
            result = False
            torque_enable = 1 if enabled else 0
            try:
                result = self.dynamixel_caller("", self.params[key]['ID'], "Torque_Enable", torque_enable) # Should return True
                rospy.sleep(0.1)
                rospy.loginfo("%s torque: %d", key, enabled)
            except:
                rospy.logwarn("Torque call failed: %s"%e)


    def parseYaml(self, filename):
        f = open(filename, 'r')
        yamlfile = safe_load(f)
        f.close()
        return yamlfile

    def movingCallback(self, dynamixel_state_list):
        state_list = dynamixel_state_list.dynamixel_state
        for state in state_list:
            if state.present_velocity != 0:
                self.isMoving[state.name] = True
            else:
                self.isMoving[state.name] = False





if __name__ == "__main__":
    try:
        rospy.init_node("gimbal_joy")
        rospy.loginfo("Starting Gimbal Joy Control Node")
        s = gimbaljoyClass()
        rospy.spin()
    except rospy.ROSInterruptException: pass