#!/usr/bin/env python


import rospy, math
import numpy as np
from random import *
import sys, termios, tty, select, os

from custom_messages.msg import DMCTS_Travel_Goal
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


class State(object):

  def __init__(self, xi, yi):
    self.x = xi # locs
    self.y = yi # locs

class Quad(object):
  index = 0
  cLoc = State(0.0,0.0) # x,y
  gLoc = State(0.0,0.0) # x,y
  goal_initialized = False
  cruising_speed = 0.0
  
  def init(self, ai, aa, sx, sy, cs):
    # Initial values
    self.goal_altitude = aa
    self.gLoc.z = aa
    self.cLoc.x = sx
    self.cLoc.y = sy
    self.index = ai
    self.cruising_speed = cs
    # Setup publisher
    self.pub_odom = rospy.Publisher('/uav' + str(self.index) + '/ground_truth/state', Odometry, queue_size=10)
    self.pub_clock = rospy.Publisher('/clock', Clock, queue_size=10)
    # Setup Subs
    self.goal_sub = rospy.Subscriber('/dmcts_' + str(self.index) + '/travel_goal', DMCTS_Travel_Goal, self.goal_callback )
    # Setup Timer
    self.odom_timer = rospy.Timer(rospy.Duration(0.01), self.odom_callback)

  def odom_callback(self, event):
    self.update_loc()
    msg = Odometry()
    msg.pose.pose.position.x = self.cLoc.x
    msg.pose.pose.position.y = self.cLoc.y
    msg.pose.pose.position.z = self.goal_altitude
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = self.cruising_speed
    msg.twist.twist.linear.y = 0.0
    msg.twist.twist.linear.z = 0.0
    msg.twist.twist.angular.x = 0.0
    msg.twist.twist.angular.y = 0.0
    msg.twist.twist.angular.z = 0.0
    self.pub_odom.publish(msg)

    clk = Clock()
    clk.clock = rospy.get_rostime()
    self.pub_clock.publish(clk)
    
  def update_loc(self):
    if self.goal_initialized:
      [dx, dy] = self.position_from_a_to_b(self.cLoc, self.gLoc)
      dist = math.sqrt( dx**2 + dy**2)
      if dist < 0.01*self.cruising_speed:
        self.cLoc.x = self.gLoc.x
        self.cLoc.y = self.gLoc.y
      else:
        self.cLoc.x += 0.01*self.cruising_speed * dx / dist
        self.cLoc.y += 0.01*self.cruising_speed * dy / dist
      
  def goal_callback(self, msg):
    #rospy.loginfo("goal_callback: goal_in: %0.2f, %0.2f", msg.x,msg.y)
    self.path = []
    self.gLoc.x = msg.x
    self.gLoc.y = msg.y
    self.goal_initialized = True
    
  def position_from_a_to_b( self, a, b ):
    x = b.x - a.x
    y = b.y - a.y

    return [x,y]

if __name__ == '__main__':
  rospy.init_node('fake_dmcts_quad')
  ai = rospy.get_param('agent_index')
  aa = rospy.get_param('agent_altitude')
  sx = rospy.get_param('start_x')
  sy = rospy.get_param('start_y')
  cs = rospy.get_param('cruising_speed')

  rospy.loginfo("Fake_DMCTS_Quad::initializing")
  rospy.loginfo(" Fake_DMCTS_Quad::agent index: %i", ai)
  rospy.loginfo(" Fake_DMCTS_Quad::agent altitude: %.1f", aa)
  rospy.loginfo(" Fake_DMCTS_Quad::start_x: %0.1f", sx)
  rospy.loginfo(" Fake_DMCTS_Quad::start_y: %0.1f", sy)
  rospy.loginfo(" Fake_DMCTS_Quad::cruising_speed: %0.1f", cs)

  quad = Quad()
  quad.init(ai, aa, sx, sy, cs)
  rospy.loginfo("Fake_DMCTS_Quad::initialized")
  rospy.spin()
