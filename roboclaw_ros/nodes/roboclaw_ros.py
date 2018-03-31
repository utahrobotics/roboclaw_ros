#!/usr/bin/env python
from __future__ import division
from math import pi, cos, sin
import diagnostic_msgs
import diagnostic_updater
from roboclaw_driver.roboclaw_driver import Roboclaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import threading
from serial import SerialException

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"
__maintainer__ = "mattwilsonmbw@gmail.com (Matthew Wilson)"

class EncoderOdom:
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

    def update(self, enc_left, enc_right):
        left_ticks = -(enc_left - self.last_enc_left)
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        #rospy.loginfo(self.TICKS_PER_METER)
        dist = (dist_right + dist_left) / 2.0
        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        # TODO find better way to determine if going straight, this means slight deviation is accounted for
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time
        rospy.logdebug("enocder left: %d", self.last_enc_left)
        rospy.logdebug("enocder right: %d", self.last_enc_right)
        #rospy.loginfo(vel_x)
        #rospy.loginfo(vel_theta)
        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        vel_x, vel_theta = self.update(enc_left, enc_right)
        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        # TODO: why was this commented out? Do we not need it?
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


class Node:
    """ Class for running roboclaw ros node for 2 motors in a diff drive setup"""
    def __init__(self):
        self.lock = threading.Lock()
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
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

        #rospy.init_node("roboclaw_ros")
        rospy.init_node("roboclaw_ros", log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        self.dev = rospy.get_param("~dev")
        self.baudrate = int(rospy.get_param("~baudrate"))
        self.frontaddr = int(rospy.get_param("~frontaddr"))
        self.backaddr = int(rospy.get_param("~backaddr"))
        self.accel = int(rospy.get_param("~accel"))

        # open roboclaw device connection
        self.roboclaw = Roboclaw(self.dev, self.baudrate) 
        # set acceleration limits (default is 655360, we found 100000 to be decent)_
        self.roboclaw.SetM1DefaultAccel(self.frontaddr, self.accel) 
        self.roboclaw.SetM2DefaultAccel(self.frontaddr, self.accel)
        self.roboclaw.SetM1DefaultAccel(self.backaddr, self.accel)
        self.roboclaw.SetM2DefaultAccel(self.backaddr, self.accel)
        # diagnostics
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals))

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
            rospy.logwarn("Problem getting rear roboclaw version")
            rospy.logdebug(e)
            raise SerialException("Connectivity issue. Could not read version")

        self.roboclaw.SpeedM1M2(self.frontaddr, 0, 0)
        self.roboclaw.ResetEncoders(self.frontaddr)
        self.roboclaw.SpeedM1M2(self.backaddr, 0, 0)
        self.roboclaw.ResetEncoders(self.backaddr)

        self.LINEAR_MAX_SPEED = float(rospy.get_param("~linear/x/max_velocity"))
        self.ANGULAR_MAX_SPEED = float(rospy.get_param("~angular/z/max_velocity"))
        self.TICKS_PER_METER = float(rospy.get_param("~ticks_per_meter"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width"))

        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time = rospy.get_rostime()

        self.sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=5)
        self.TIMEOUT = 2

        rospy.sleep(1) # wait for things to initialize

        rospy.logdebug("dev %s", self.dev)
        rospy.logdebug("baudrate %d", self.baudrate)
        rospy.logdebug("front address %d", self.frontaddr)
        rospy.logdebug("back address %d", self.backaddr)
        rospy.logdebug("max_speed %f", self.LINEAR_MAX_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)

    def run(self):
        """Run the main ros loop"""
        self._has_showed_message = False # flag to not spam logging
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(30)
        while not rospy.is_shutdown():
            with self.lock:
                if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > self.TIMEOUT:
                    try:
                        self.roboclaw.ForwardM1(self.frontaddr, 0)
                        self.roboclaw.ForwardM2(self.frontaddr, 0)
                        self.roboclaw.ForwardM1(self.backaddr, 0)
                        self.roboclaw.ForwardM2(self.backaddr, 0)
                    except OSError as e:
                        rospy.logfatal("COULD NOT STOP MOTORS!")
                        rospy.logdebug(e)
                    if (not self._has_showed_message):
                        rospy.loginfo("Did not get command for 1 second, stopping")
                        self._has_showed_message = True
                else:
                    self._has_showed_message = False

                # TODO need find solution to the OSError11 looks like sync problem with serial status1, enc1, crc1 = None, None, None
                status2, enc1, enc2, crc2 = None, None, None, None

                # TODO: incorporate both front and back encoders

                try:
                    status1, enc1, crc1 = self.roboclaw.ReadEncM1(self.backaddr)
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                    rospy.logdebug(e)

                try:
                    status2, enc2, crc2 = self.roboclaw.ReadEncM2(self.backaddr)
                except ValueError:
                    pass
                except OSError as e:
                    rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
                    rospy.logdebug(e)

                if ((enc1 is not None) and (enc2 is not None)):
                    rospy.logdebug(" Encoders %d %d" % (enc1, enc2))
                    self.encodm.update_publish(enc1, enc2)
                self.updater.update()
            r_time.sleep()

    def cmd_vel_callback(self, twist):
        with self.lock:
            """Command the motors based on the incoming twist message"""
            now_time = rospy.get_rostime()
            dt = (now_time - self.last_set_speed_time).to_sec()
            self.last_set_speed_time = now_time

            rospy.logdebug("Twist: -Linear X: %d    -Angular Z: %d", twist.linear.x, twist.angular.z)
            linear_x = -twist.linear.x
            angular_z = twist.angular.z

            if linear_x > self.LINEAR_MAX_SPEED:
                linear_x = self.LINEAR_MAX_SPEED
            elif linear_x < -self.LINEAR_MAX_SPEED:
                linear_x = -self.LINEAR_MAX_SPEED

            # Take linear x and angular z values and compute command
            motor1_command = linear_x/self.LINEAR_MAX_SPEED + angular_z/self.ANGULAR_MAX_SPEED
            motor2_command = linear_x/self.LINEAR_MAX_SPEED - angular_z/self.ANGULAR_MAX_SPEED

            # Scale to motor pwm
            motor1_command = int(motor1_command * 127)
            motor2_command = int(motor2_command * 127)

            # Clip commands to within bounds (-127,127)
            motor1_command =  int(max(-127, min(127, motor1_command)))
            motor2_command =  int(max(-127, min(127, motor2_command)))

            rospy.logdebug("motor1 command = %d",int(motor1_command))
            rospy.logdebug("motor2 command = %d",int(motor2_command))

            try:
                if motor1_command >= 0:
                    self.roboclaw.ForwardM1(self.frontaddr, motor1_command)
                    self.roboclaw.ForwardM1(self.backaddr, motor1_command)
                else:
                    self.roboclaw.BackwardM1(self.frontaddr, -motor1_command)
                    self.roboclaw.BackwardM1(self.backaddr, -motor1_command)

                if motor2_command >= 0:
                    self.roboclaw.ForwardM2(self.frontaddr, motor2_command)
                    self.roboclaw.ForwardM2(self.backaddr, motor2_command)
                else:
                    self.roboclaw.BackwardM2(self.frontaddr, -motor2_command)
                    self.roboclaw.BackwardM2(self.backaddr, -motor2_command)

            except OSError as e:
                rospy.logwarn("Roboclaw OSError: %d", e.errno)
                rospy.logdebug(e)



    def check_vitals(self, stat):
        """Check battery voltage and temperatures from roboclaw"""
        try:
            statusfront = self.roboclaw.ReadError(self.frontaddr)[1]
            statusrear = self.roboclaw.ReadError(self.backaddr)[1]
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
            return
        statefront, messagefront = self.ERRORS[statusfront]
        staterear, messagerear = self.ERRORS[statusfront]
        stat.summary(statefront, messagefront)
        stat.summary(staterear, messagerear)
        try:
            stat.add("Front Main Batt V:", float(self.roboclaw.ReadMainBatteryVoltage(self.frontaddr)[1] / 10))
            stat.add("Front Logic Batt V:", float(self.roboclaw.ReadLogicBatteryVoltage(self.frontaddr)[1] / 10))
            stat.add("Front Temp1 C:", float(self.roboclaw.ReadTemp(self.frontaddr)[1] / 10))
            stat.add("Front Temp2 C:", float(self.roboclaw.ReadTemp2(self.frontaddr)[1] / 10))

            front_currents = self.roboclaw.ReadCurrents(self.frontaddr)
            stat.add("Front Left Current:", float(front_currents[1] / 100))
            stat.add("Front Right Current:", float(front_currents[2] / 100))

            back_currents = self.roboclaw.ReadCurrents(self.backaddr)
            stat.add("Back Left Current:", float(back_currents[1] / 100))
            stat.add("Back Right Current:", float(back_currents[2] / 100))

            stat.add("Back Main Batt V:", float(self.roboclaw.ReadMainBatteryVoltage(self.backaddr)[1] / 10))
            stat.add("Back Logic Batt V:", float(self.roboclaw.ReadLogicBatteryVoltage(self.backaddr)[1] / 10))
            stat.add("Back Temp1 C:", float(self.roboclaw.ReadTemp(self.backaddr)[1] / 10))
            stat.add("Back Temp2 C:", float(self.roboclaw.ReadTemp2(self.backaddr)[1] / 10))
        except OSError as e:
            rospy.logwarn("Diagnostics OSError: %d", e.errno)
            rospy.logdebug(e)
        return stat

    def shutdown(self):
	"""Handle shutting down the node"""
        rospy.loginfo("Shutting down")
        if hasattr(self, "sub"):
            self.sub.unregister() # so it doesn't get called after we're dead
        try:
            self.roboclaw.ForwardM1(self.frontaddr, 0)
            self.roboclaw.ForwardM2(self.frontaddr, 0)
            self.roboclaw.ForwardM1(self.backaddr, 0)
            self.roboclaw.ForwardM2(self.backaddr, 0)
            rospy.loginfo("Closed Roboclaw serial connection")
        except OSError:
            rospy.logerr("Shutdown did not work trying again")
            try:
                self.roboclaw.ForwardM1(self.frontaddr, 0)
                self.roboclaw.ForwardM2(self.frontaddr, 0)
                self.roboclaw.ForwardM1(self.backaddr, 0)
                self.roboclaw.ForwardM2(self.backaddr, 0)
            except OSError as e:
                rospy.logerr("Could not shutdown motors!!!!")
                rospy.logdebug(e)
        #quit()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
