#!/usr/bin/env python
import roslib

roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    t0 = 0
    count = 0  
    freq = 10.0
    old_distance = 0
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        try:
            if count == 0:
                (t0, r0) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
                old_distance = math.sqrt(t0[0]**2 + t0[1]**2)
                count = 1
                continue
            (t1, r1) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        new_distance = math.sqrt(t1[0] ** 2 + t1[1] ** 2)
        diff = new_distance - old_distance
        
        cmd = geometry_msgs.msg.Twist()
    
        euler1 = tf.transformations.euler_from_quaternion(r1)
        theta = euler1[2]
        
        if abs(diff) < 0.02*old_distance or theta > 0.1:
            linear = 0
        else:
            linear = 0.8 * abs(diff) * freq

        if t1[0] > 0:
            if new_distance < old_distance:
                linear = -linear
        

        if t1[0] < 0:
            if new_distance > old_distance:
                linear = -linear

        cmd.linear.x = linear
        cmd.angular.z = theta*freq
        turtle_vel.publish(cmd)

        
        rate.sleep()
