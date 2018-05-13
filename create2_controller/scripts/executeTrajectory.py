#!/usr/bin/env python

import rospy
from create2_controller.msg import TrajectoryState2D
import ugv_trajectory

if __name__ == '__main__':
  rospy.init_node('executeTrajectory')
  # frame = rospy.get_param("~frame")
  traj_file_name = rospy.get_param("~trajectory", "figure8.csv")
  stretchtime = rospy.get_param("~stretchtime", 4.0)
  shiftx = rospy.get_param("~shiftx", 0.0)
  shifty = rospy.get_param("~shifty", 0.0)

  trajectory = ugv_trajectory.Trajectory()
  trajectory.loadcsv(traj_file_name)
  trajectory.stretchtime(stretchtime)
  trajectory.shift(shiftx, shifty)

  r = rospy.Rate(10) # hz
  pub_desired_state = rospy.Publisher("desired_state", TrajectoryState2D, queue_size=1)

  start = rospy.Time.now()
  while not rospy.is_shutdown():
    now = rospy.Time.now()
    t = (now - start).to_sec()
    if t > trajectory.duration:
      break

    e = trajectory.eval(t)

    msg = TrajectoryState2D()
    msg.position.x = e.pos[0]
    msg.position.y = e.pos[1]
    msg.velocity.x = e.vel[0]
    msg.velocity.y = e.vel[1]
    msg.acceleration.x = e.acc[0]
    msg.acceleration.y = e.acc[1]
    pub_desired_state.publish(msg)

    print("t: ", t, msg.position.x, msg.position.y)

    r.sleep()
