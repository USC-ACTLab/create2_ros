#!/usr/bin/env python

import rospy
from create2_controller import TrajectoryState2D
import ugv_trajectory

if __name__ == '__main__':
  rospy.init_node('executeTrajectory')
  # frame = rospy.get_param("~frame")
  trajectory = rospy.get_param("~trajectory", "figure8.csv")
  stretchtime = rospy.get_param("~stretchtime", 1.0)
  shiftx = rospy.get_param("~shiftx", 0.0)
  shifty = rospy.get_param("~shifty", 0.0)

  trajectory = ugv_trajectory.Trajectory()
  trajectory.loadcsv(trajectory)
  trajectory.stretchtime(stretchtime)
  trajectory.shift(shiftx, shifty)

  r = rospy.Rate(50) # 50hz
  pub_desired_state = rospy.Publisher("desired_state", TrajectoryState2D, queue_size=1)

  start = rospy.Time.now()
  while not rospy.is_shutdown():
    now = rospy.Time.now()
    t = (now - start).to_sec()
    print("t: ", t)
    if t > self.trajectory.duration:
      break

    e = self.trajectory.eval(t)

    msg = TrajectoryState2D()
    msg.position.x = e.pos[0]
    msg.position.y = e.pos[1]
    msg.velocity.x = e.vel[0]
    msg.velocity.y = e.vel[1]
    msg.acceleration.x = e.acc[0]
    msg.acceleration.y = e.acc[1]
    pub_desired_state.publish(msg)

    r.sleep()
