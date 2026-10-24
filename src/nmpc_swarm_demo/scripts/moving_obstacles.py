#!/usr/bin/env python
"""
moving_obstacles.py

Spawns 3 simple orange cylinder obstacles in Gazebo Classic and drives them
back and forth along straight lines. Their current 2D positions are published
as geometry_msgs/PoseArray on /moving_obstacles so the central controller can
use them for NMPC obstacle avoidance.

Robustness notes:
  * Waits for Gazebo to be ready and for the /gazebo/spawn_sdf_model and
    /gazebo/set_model_state services before spawning.
  * If Gazebo services are unavailable the node keeps publishing the last known
    (static) poses so the rest of the stack does not hard-fail.
"""

import math
import rospy
import gazebo_msgs.srv
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
import gazebo_msgs.srv


class MovingObstacles:
    def __init__(self):
        rospy.init_node("moving_obstacles", anonymous=True)

        # ---- Parameters ----
        self.rate_hz = rospy.get_param("~rate_hz", 20.0)
        self.obstacle_radius = rospy.get_param("~obstacle_radius", 0.25)
        self.obstacle_height = rospy.get_param("~obstacle_height", 0.5)
        self.obstacle_color = rospy.get_param("~obstacle_color", "1.0 0.5 0.0 1.0")  # orange

        # Each obstacle: name, start (x,y), end (x,y), speed (m/s)
        # Moves back and forth: pos = start + (end-start) * triangle(t)
        self.obstacles = [
            dict(name="obstacle_0", start=(1.0, 0.0), end=(1.0, 2.0), speed=0.15),
            dict(name="obstacle_1", start=(-1.0, 1.0), end=(1.0, 1.0), speed=0.12),
            dict(name="obstacle_2", start=(0.0, -1.0), end=(2.0, -1.0), speed=0.10),
        ]

        # Per-obstacle progress in [0, 1] along start->end->start
        self.progress = {o["name"]: 0.0 for o in self.obstacles}
        self.direction = {o["name"]: 1.0 for o in self.obstacles}

        # Cached last-known poses (fallback if Gazebo is unreachable)
        self.poses = {o["name"]: None for o in self.obstacles}

        # ---- Publishers ----
        self.pose_array_pub = rospy.Publisher("/moving_obstacles", PoseArray, queue_size=10)

        # ---- Gazebo service clients ----
        self.spawn_client = None
        self.set_state_client = None
        self._wait_for_gazebo_services()

        # ---- Spawn obstacles once ----
        self._spawn_obstacles()

        rospy.loginfo("moving_obstacles: node ready with %d obstacles", len(self.obstacles))

    # ------------------------------------------------------------------ #
    # Gazebo service handling
    # ------------------------------------------------------------------ #
    def _wait_for_gazebo_services(self):
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=15.0)
            self.spawn_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", gazebo_msgs.srv.SpawnModel)
        except rospy.ROSException:
            rospy.logwarn("moving_obstacles: /gazebo/spawn_sdf_model not available")
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
            self.set_state_client = rospy.ServiceProxy("/gazebo/set_model_state", gazebo_msgs.srv.SetModelState)
        except rospy.ROSException:
            rospy.logwarn("moving_obstacles: /gazebo/set_model_state not available")

    def _sdf_cylinder(self, name, radius, height, color):
        """Build an inline SDF for an orange cylinder obstacle."""
        rgba = color.replace(" ", ",")
        r, g, b, a = [float(v) for v in rgba.split(",")]
        return (
            '<?xml version="1.0" ?>\n'
            '<sdf version="1.6">\n'
            '  <model name="%s">\n'
            '    <static>false</static>\n'
            '    <link name="link">\n'
            '      <visual name="visual">\n'
            '        <geometry><cylinder><radius>%f</radius><length>%f</length></cylinder></geometry>\n'
            '        <material><ambient>%f %f %f %f</ambient><diffuse>%f %f %f %f</diffuse></material>\n'
            '      </visual>\n'
            '      <collision name="collision">\n'
            '        <geometry><cylinder><radius>%f</radius><length>%f</length></cylinder></geometry>\n'
            '      </collision>\n'
            '    </link>\n'
            '  </model>\n'
            '</sdf>\n'
        ) % (name, radius, height, r, g, b, a, r, g, b, a, radius, height)

    def _spawn_obstacles(self):
        if self.spawn_client is None:
            rospy.logwarn("moving_obstacles: spawn client unavailable, skipping spawn")
            return
        for obs in self.obstacles:
            sdf = self._sdf_cylinder(obs["name"], self.obstacle_radius, self.obstacle_height, self.obstacle_color)
            start_x, start_y = obs["start"]
            pose = Pose()
            pose.position.x = start_x
            pose.position.y = start_y
            pose.position.z = self.obstacle_height / 2.0
            pose.orientation.w = 1.0
            try:
                self.spawn_client(
                    model_name=obs["name"],
                    model_xml=sdf,
                    robot_namespace="",
                    initial_pose=pose,
                    reference_frame="world",
                )
                rospy.loginfo("moving_obstacles: spawned %s at (%.2f, %.2f)", obs["name"], start_x, start_y)
            except rospy.ServiceException as exc:
                rospy.logerr("moving_obstacles: failed to spawn %s: %s", obs["name"], exc)

    # ------------------------------------------------------------------ #
    # Motion update
    # ------------------------------------------------------------------ #
    def _triangle_progress(self, progress):
        """Map progress in [0, 1] to a triangle wave in [0, 1] (forward then back)."""
        if progress <= 0.5:
            return 2.0 * progress
        return 2.0 * (1.0 - progress)

    def update(self, dt):
        pose_array = PoseArray()
        pose_array.header = Header(stamp=rospy.Time.now(), frame_id="odom")

        for obs in self.obstacles:
            name = obs["name"]
            sx, sy = obs["start"]
            ex, ey = obs["end"]
            speed = obs["speed"]

            # Advance progress along the segment; length normalizes speed
            seg_len = math.hypot(ex - sx, ey - sy)
            if seg_len > 1e-6:
                self.progress[name] += self.direction[name] * (speed * dt) / seg_len
            # Clamp and bounce at ends
            if self.progress[name] >= 1.0:
                self.progress[name] = 1.0
                self.direction[name] = -1.0
            elif self.progress[name] <= 0.0:
                self.progress[name] = 0.0
                self.direction[name] = 1.0

            t = self._triangle_progress(self.progress[name])
            x = sx + (ex - sx) * t
            y = sy + (ey - sy) * t

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = self.obstacle_height / 2.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            # Cache for fallback publishing
            self.poses[name] = pose

        return pose_array

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_time = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now

            pose_array = self.update(dt)

            # Also nudge the Gazebo models so the visuals match the published poses
            self._sync_gazebo_models()

            self.pose_array_pub.publish(pose_array)
            rate.sleep()

    def _sync_gazebo_models(self):
        if self.set_state_client is None:
            return
        from gazebo_msgs.msg import ModelState
        for obs in self.obstacles:
            name = obs["name"]
            pose = self.poses[name]
            if pose is None:
                continue
            state = ModelState()
            state.model_name = name
            state.pose = pose
            state.reference_frame = "world"
            try:
                self.set_state_client(state)
            except rospy.ServiceException:
                pass  # best-effort; published PoseArray is the source of truth


if __name__ == "__main__":
    try:
        MovingObstacles().run()
    except rospy.ROSInterruptException:
        pass
