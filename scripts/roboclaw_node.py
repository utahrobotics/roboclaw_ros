#!/usr/bin/env python
from __future__ import division
from math import pi, cos, sin
import diagnostic_msgs
import diagnostic_updater
from roboclaw_driver import Roboclaw
import rospy
import tf
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import threading
from serial import SerialException

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"
__maintainer__ = "mattwilsonmbw@gmail.com (Matthew Wilson)"

# TODO: how to read analog (current sensor) value

def clip(val, minval, maxval):
    """Clip commands to within bounds e.g., (-127,127)"""
    return max(min(val, maxval), minval)

# diagnostics error msg
ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"), 0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
               0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
               0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
               0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
               0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
               0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
               0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
               0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
               0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
               0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
               0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
               0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
               0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
               0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
               0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
               0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

class EncoderOdom(object):
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = rospy.Publisher('/roboclaw/odom', Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, front_left, front_right, back_left, back_right):
        """Compute vel_x and vel_theta based on new encoder measurements"""
        # TODO: could probably add something more complicated here to check that both motors are spinning
        # like if we notice that only one of the wheels is spinning, it is more likely that the robot is slipping.
        # if we could account for this, we could probably get more accurate localization. for now we just:
        # average front and back encoder values 
        enc_left = (front_left + back_left) / 2
        enc_right = (front_right + back_right) / 2
        # calculate diff from last reading
        left_ticks = -(enc_left - self.last_enc_left)
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right
        # calculate distance moved by each side
        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        # average the distances for total delta
        dist = (dist_right + dist_left) / 2.0

        # check for numerical stability (if the dists are almost the same, we are going straight)
        if abs(dist_left - dist_right) < 1e-9:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # check for numerical stability 
        if abs(d_time) < 1e-9:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        rospy.logdebug("enocder left: %d", self.last_enc_left)
        rospy.logdebug("enocder right: %d", self.last_enc_right)
        rospy.logdebug("vel_x: %d", vel_x)
        rospy.logdebug("vel_x: %d", vel_theta)
        return vel_x, vel_theta

    def update_publish(self, front1, front2, back1, back2):
        """update velocity values from encoder measurement and publish odom message"""
        vel_x, vel_theta = self.update(front1, front2, back1, back2)
        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        """Publish odometry message"""
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        # TODO: why was this commented out?
        #br = tf.TransformBroadcaster()
        #br.sendTransform((cur_x, cur_y, 0),
        #                 tf.transformations.quaternion_from_euler(0, 0, cur_theta),
        #                 current_time,
        #                 "base_link",
        #                 "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        # TODO: Why are the covariances set at these values?  Do they even make sense?
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

class Node(object):
    """ Class for running roboclaw ros node for 2 motors in a diff drive setup"""
    def __init__(self):
        """init variables and ros stuff"""
        self._have_shown_message = False # flag to not spam logging
        self._have_read_vitals = False # flag to check when vitals have been read

        # ints in [-127,127] that get sent to roboclaw to drive forward or backward 
        self.curr_drive1_cmd = 0 
        self.curr_drive2_cmd = 0

        #rospy.init_node("roboclaw_node")
        rospy.init_node("roboclaw_node", log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        self.dev = rospy.get_param("~dev")
        self.baudrate = int(rospy.get_param("~baudrate"))
        self.frontaddr = int(rospy.get_param("~frontaddr"))
        self.backaddr = int(rospy.get_param("~backaddr"))
        self.diggeraddr = int(rospy.get_param("~diggeraddr"))
        self.accel = int(rospy.get_param("~accel"))

        # open roboclaw device connection
        self.roboclaw = Roboclaw(self.dev, self.baudrate) 
        # set acceleration limits (default is 655360, we found 100000 to be decent)_
        #self.roboclaw.SetM1DefaultAccel(self.frontaddr, self.accel) 
        #self.roboclaw.SetM2DefaultAccel(self.frontaddr, self.accel)
        #self.roboclaw.SetM1DefaultAccel(self.backaddr, self.accel)
        #self.roboclaw.SetM2DefaultAccel(self.backaddr, self.accel)
        #self.roboclaw.SetM1DefaultAccel(self.diggeraddr, self.accel)
        #self.roboclaw.SetM2DefaultAccel(self.diggeraddr, self.accel)
        # diagnostics
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.pub_vitals))

        # TODO (p1): we probably just want to crash here or something if we don't have a connection
        try:
            version = self.roboclaw.ReadVersion(self.frontaddr)
            rospy.logdebug("Front Version " + str(repr(version[1])))
        except Exception as e:
            rospy.logwarn("Problem getting front roboclaw version")
            rospy.logdebug(e)
            raise SerialException("Connectivity issue. Could not read version")

        try:
            version = self.roboclaw.ReadVersion(self.backaddr)
            rospy.logdebug("Back Version "+ str(repr(version[1])))
        except Exception as e:
            rospy.logwarn("Problem getting back roboclaw version")
            rospy.logdebug(e)
            raise SerialException("Connectivity issue. Could not read version")

        try:
            version = self.roboclaw.ReadVersion(self.diggeraddr)
            rospy.logdebug("Digger Version "+ str(repr(version[1])))
            self.roboclaw.SetM1EncoderMode(self.diggeraddr, 1)
            self.roboclaw.SetM2EncoderMode(self.diggeraddr, 1)
            rospy.logdebug("Digger Encoder Mode "+ str(self.roboclaw.ReadEncoderModes(self.diggeraddr)))
        except Exception as e:
            rospy.logwarn("Problem getting digger roboclaw version")
            rospy.logdebug(e)
            raise SerialException("Connectivity issue. Could not read version")

        self.roboclaw.SpeedM1M2(self.frontaddr, 0, 0)
        self.roboclaw.ResetEncoders(self.frontaddr)
        self.roboclaw.SpeedM1M2(self.backaddr, 0, 0)
        self.roboclaw.ResetEncoders(self.backaddr)
        # TODO (p2): test resetting 
        #self.roboclaw.SpeedM1M2(self.diggeraddr, 0, 0)
        #self.roboclaw.ResetEncoders(self.diggeraddr)

        self.LINEAR_MAX_SPEED = float(rospy.get_param("~linear/x/max_velocity"))
        self.ANGULAR_MAX_SPEED = float(rospy.get_param("~angular/z/max_velocity"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width"))
        self.TIMEOUT = float(rospy.get_param("~timeout"))

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_vel_cmd_time = rospy.Time.now()
        self.last_digger_cmd_time = rospy.Time.now()
        self.last_digger_extended_time = rospy.Time.now()
        self.digger_extended = False

        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        self.digger_sub = rospy.Subscriber("/digger_spin/cmd", Float32, self.digger_spin_callback, queue_size=1)
        self.digger_extended_sub = rospy.Subscriber("/digger_extended", Bool, self.digger_extended_callback, queue_size=1)

        rospy.sleep(1) # wait for things to initialize

        rospy.logdebug("dev %s", self.dev)
        rospy.logdebug("baudrate %d", self.baudrate)
        rospy.logdebug("front address %d", self.frontaddr)
        rospy.logdebug("back address %d", self.backaddr)
        rospy.logdebug("digger address %d", self.diggeraddr)
        rospy.logdebug("max_speed %f", self.LINEAR_MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)
        rospy.logdebug("timeout %f", self.TIMEOUT)

    def run(self):
        """Run the main ros loop"""
        rospy.loginfo("Starting motor drive")

        r_time = rospy.Rate(30)
        while not rospy.is_shutdown():
            # do watchdog checks to shut down motors if we haven't heard anything in awhile
            if (rospy.Time.now() - self.last_vel_cmd_time).to_sec() > self.TIMEOUT:
                self.curr_drive1_cmd = 0
                self.curr_drive2_cmd = 0
                if (not self._have_shown_message):
                    rospy.loginfo("Did not get drive command for %d second, stopping", self.TIMEOUT)
                    self._have_shown_message = True
            else:
                self._have_shown_message = False
            if (rospy.Time.now() - self.last_digger_cmd_time).to_sec() > self.TIMEOUT:
                self.curr_digger_cmd = 0
                if (not self._have_shown_message):
                    rospy.loginfo("Did not get digger command for %d second, stopping", self.TIMEOUT)
                    self._have_shown_message = True
            else:
                self._have_shown_message = False

            # send actual commands to the devices (the values to send are generally set in sub callbacks)
            self._send_drive_cmd() 
            self._send_digger_cmd()

            # read encoder data and publish odom
            self.front_enc1, self.front_enc2 = None, None
            self.back_enc1, self.back_enc2 = None, None
            self.digger_current1, self.digger_current2 = None, None

            #print(self.roboclaw.ReadEncM1(self.frontaddr))
            #print(self.roboclaw.ReadEncM2(self.frontaddr))
            #print(self.roboclaw.ReadEncM1(self.backaddr))
            #print(self.roboclaw.ReadEncM2(self.backaddr))

            try:
                pass
                _, self.digger_current1, _ = self.roboclaw.ReadEncM1(self.diggeraddr)
                _, self.digger_current2, _ = self.roboclaw.ReadEncM2(self.diggeraddr)
                _, self.front_enc1, _ = self.roboclaw.ReadEncM1(self.frontaddr) # returns (status, ENCODER, crc) -> (_, enc, _)
                _, self.front_enc2, _ = self.roboclaw.ReadEncM2(self.frontaddr)
                _, self.back_enc1, _ = self.roboclaw.ReadEncM1(self.backaddr)
                _, self.back_enc2, _ = self.roboclaw.ReadEncM2(self.backaddr)
            except (ValueError,OSError) as e:
                rospy.logwarn("Error when trying to read encoder value: %s", e)

            if self.digger_current1 is not None:
                rospy.logdebug("Digger Current %d %d", self.digger_current1, self.digger_current2)

            if self.back_enc1 is not None:
                rospy.logdebug("Front Encoders %d %d", self.front_enc1, self.front_enc2)
                rospy.logdebug("Back Encoders %d %d", self.back_enc1, self.back_enc2)
                self.encodm.update_publish(self.front_enc1, self.front_enc2, self.back_enc1, self.back_enc2)


            # publish diagnostics
            self._read_vitals()
            self.updater.update()

            r_time.sleep()

    def digger_extended_callback(self, msg):
        self.digger_extended = msg.data
        self.last_digger_extended_time = rospy.Time.now()

    def digger_spin_callback(self, cmd):
        """Set digger command based on the float message in range (-1, 1)"""
        self.last_digger_cmd_time = rospy.Time.now()
        rospy.logdebug("Digger message: %f", cmd.data)
        # Scale to motor pwm and clip
        motor_cmd = int(cmd.data * 127)
        motor_cmd = clip(motor_cmd, -127, 127)
        self.curr_digger_cmd = motor_cmd

    def _send_digger_cmd(self):
        """Sends the current digger command to the Roboclaw devices over Serial"""

        # TODO: test this code more thoroughly 

        # If the linear actuator is not extended, or we haven't heard from it in a while,
        # set the digger speed to 0
        #if not self.digger_extended or ( (rospy.Time.now() - self.last_digger_extended_time).to_sec > self.timeout):
        #    self.curr_digger_cmd = 0
        #else:
        #    self.curr_digger_cmd = self.curr_digger_cmd

        # TODO: need to check the directionality of these when we hook up everything
        try:
            if self.curr_digger_cmd >= 0:
                self.roboclaw.ForwardM1(self.diggeraddr, self.curr_digger_cmd)
                self.roboclaw.BackwardM2(self.diggeraddr, self.curr_digger_cmd)
            else:
                self.roboclaw.BackwardM1(self.diggeraddr, -self.curr_digger_cmd)
                self.roboclaw.ForwardM2(self.diggeraddr, -self.curr_digger_cmd)
        except OSError as e:
            rospy.logwarn("Roboclaw OSError: %d", e.errno)
            rospy.logdebug(e)


    def cmd_vel_callback(self, twist):
        """Set motor command based on the incoming twist message"""
        self.last_vel_cmd_time = rospy.Time.now()

        rospy.logdebug("Twist: -Linear X: %d    -Angular Z: %d", twist.linear.x, twist.angular.z)
        linear_x = -twist.linear.x
        angular_z = twist.angular.z

        if linear_x > self.LINEAR_MAX_SPEED:
            linear_x = self.LINEAR_MAX_SPEED
        elif linear_x < -self.LINEAR_MAX_SPEED:
            linear_x = -self.LINEAR_MAX_SPEED

        # Take linear x and angular z values and compute command
        drive1_cmd = linear_x/self.LINEAR_MAX_SPEED + angular_z/self.ANGULAR_MAX_SPEED
        drive2_cmd = linear_x/self.LINEAR_MAX_SPEED - angular_z/self.ANGULAR_MAX_SPEED

        # Scale to motor pwm
        drive1_cmd = int(drive1_cmd * 127)
        drive2_cmd = int(drive2_cmd * 127)
        # Clip to command bounds
        drive1_cmd = clip(drive1_cmd, -127, 127)
        drive2_cmd = clip(drive2_cmd, -127, 127)
        # update the current commands
        self.curr_drive1_cmd = drive1_cmd
        self.curr_drive2_cmd = drive2_cmd
        rospy.logdebug("drive1 command = %d", self.curr_drive1_cmd)
        rospy.logdebug("drive2 command = %d", self.curr_drive2_cmd)

    def _send_drive_cmd(self):
        """Sends the current motor commands to the Roboclaw devices over Serial"""
        try:
            if self.curr_drive1_cmd >= 0:
                self.roboclaw.ForwardM1(self.frontaddr, self.curr_drive1_cmd)
                self.roboclaw.ForwardM1(self.backaddr, self.curr_drive1_cmd)
            else:
                self.roboclaw.BackwardM1(self.frontaddr, -self.curr_drive1_cmd)
                self.roboclaw.BackwardM1(self.backaddr, -self.curr_drive1_cmd)

            if self.curr_drive2_cmd >= 0:
                self.roboclaw.BackwardM2(self.frontaddr, self.curr_drive2_cmd)
                self.roboclaw.BackwardM2(self.backaddr, self.curr_drive2_cmd)
            else:
                self.roboclaw.ForwardM2(self.frontaddr, -self.curr_drive2_cmd)
                self.roboclaw.ForwardM2(self.backaddr, -self.curr_drive2_cmd)
        except OSError as e:
            rospy.logwarn("Roboclaw OSError: %d", e.errno)
            rospy.logdebug(e)


    def _read_vitals(self):
        """Check battery voltage and temperatures from roboclaw"""
        try:
            statusfront = self.roboclaw.ReadError(self.frontaddr)[1]
            statusback = self.roboclaw.ReadError(self.backaddr)[1]
            statusdigger = self.roboclaw.ReadError(self.diggeraddr)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return

        self.statefront,  self.messagefront = ERRORS[statusfront]
        self.stateback,   self.messageback = ERRORS[statusback]
        self.statedigger, self.messagedigger = ERRORS[statusdigger]
        try:
            # read main and logic voltages
            self.front_voltage = float(self.roboclaw.ReadMainBatteryVoltage(self.frontaddr)[1] / 10)
            self.front_logic = float(self.roboclaw.ReadLogicBatteryVoltage(self.frontaddr)[1] / 10)
            self.back_voltage = float(self.roboclaw.ReadMainBatteryVoltage(self.backaddr)[1] / 10)
            self.back_logic = float(self.roboclaw.ReadLogicBatteryVoltage(self.frontaddr)[1] / 10)
            self.digger_voltage = float(self.roboclaw.ReadMainBatteryVoltage(self.diggeraddr)[1] / 10)
            self.digger_logic = float(self.roboclaw.ReadLogicBatteryVoltage(self.diggeraddr)[1] / 10)
            # read currents
            self.front_currents = self.roboclaw.ReadCurrents(self.frontaddr)
            self.back_currents = self.roboclaw.ReadCurrents(self.backaddr)
            self.digger_currents = self.roboclaw.ReadCurrents(self.diggeraddr)
            self._have_read_vitals = True

            #rospy.logdebug("Front V %d", self.front_voltage)
            #rospy.logdebug("Back V %d", self.back_voltage)
            #rospy.logdebug("Digger V %d", self.digger_voltage)

        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)

    def pub_vitals(self, stat):
        """Publish vitals (called by diagnostics updater)"""
        if self._have_read_vitals:
            stat.summary(self.statefront,  self.messagefront)
            stat.summary(self.stateback,   self.messageback)
            stat.summary(self.statedigger, self.messagedigger)
            stat.add("Front Main Batt V:", self.front_voltage)
            stat.add("Front Logic Batt V:", self.front_logic)
            stat.add("Front Left Current:", float(self.front_currents[1] / 100))
            stat.add("Front Right Current:", float(self.front_currents[2] / 100))
            stat.add("Back Left Current:", float(self.back_currents[1] / 100))
            stat.add("Back Right Current:", float(self.back_currents[2] / 100))
            stat.add("Digger Left Current:", float(self.digger_currents[1] / 100))
            stat.add("Digger Right Current:", float(self.digger_currents[2] / 100))
        return stat

    def shutdown(self):
	"""Handle shutting down the node"""
        rospy.loginfo("Shutting down")
        # so they don't get called after we're dead
        if hasattr(self, "cmd_vel_sub"):
            self.cmd_vel_sub.unregister() 
        if hasattr(self, "digger_sub"):
            self.digger_sub.unregister()

        # stop motors
        try:
            self.roboclaw.ForwardM1(self.frontaddr, 0)
            self.roboclaw.ForwardM2(self.frontaddr, 0)
            self.roboclaw.ForwardM1(self.backaddr, 0)
            self.roboclaw.ForwardM2(self.backaddr, 0)
            self.roboclaw.ForwardM1(self.diggeraddr, 0)
            self.roboclaw.ForwardM2(self.diggeraddr, 0)
            rospy.loginfo("Closed Roboclaw serial connection")
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                self.roboclaw.ForwardM1(self.frontaddr, 0)
                self.roboclaw.ForwardM2(self.frontaddr, 0)
                self.roboclaw.ForwardM1(self.backaddr, 0)
                self.roboclaw.ForwardM2(self.backaddr, 0)
                self.roboclaw.ForwardM1(self.diggeraddr, 0)
                self.roboclaw.ForwardM2(self.diggeraddr, 0)
            except OSError as e:
                rospy.logfatal("Could not shutdown motors!!!!")
                rospy.logfatal(e)
        #quit()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
