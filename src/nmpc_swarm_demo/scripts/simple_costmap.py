#!/usr/bin/env python
"""
simple_costmap.py

Simplified costmap for the NMPC swarm demo.
- Global costmap: fixed frame, includes static obstacles + moving obstacles
- Local costmap: rolling window around each robot
- Publishes OccupancyGrid for RViz visualization
- Provides cost query service for NMPC

Designed for class project: minimal, readable, handles moving obstacles.
"""

import rospy
import numpy as np
import ast
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs


class SimpleCostmap:
    def __init__(self, name, global_frame="odom", robot_frame="base_link"):
        self.name = name
        self.global_frame = global_frame
        self.robot_frame = robot_frame

        # Parameters
        self.resolution = rospy.get_param("~resolution", 0.05)
        self.global_size_x = rospy.get_param("~global_size_x", 20.0)  # meters
        self.global_size_y = rospy.get_param("~global_size_y", 20.0)
        self.local_size_x = rospy.get_param("~local_size_x", 6.0)
        self.local_size_y = rospy.get_param("~local_size_y", 6.0)
        self.inflation_radius = rospy.get_param("~inflation_radius", 0.4)
        self.lethal_cost = rospy.get_param("~lethal_cost", 100)  # int8 max for OccupancyGrid
        self.inscribed_radius = rospy.get_param("~inscribed_radius", 0.2)

        # Static obstacles (from central_controller params)
        static_obstacles_param = rospy.get_param("~static_obstacles", [[1.5, 0.5], [-1.0, -1.0]])
        if isinstance(static_obstacles_param, str):
            self.static_obstacles = ast.literal_eval(static_obstacles_param)
        else:
            self.static_obstacles = static_obstacles_param

        # Grid dimensions
        self.global_nx = int(self.global_size_x / self.resolution)
        self.global_ny = int(self.global_size_y / self.resolution)
        self.local_nx = int(self.local_size_x / self.resolution)
        self.local_ny = int(self.local_size_y / self.resolution)

        # Origin (centered at 0,0 for global)
        self.global_origin_x = -self.global_size_x / 2.0
        self.global_origin_y = -self.global_size_y / 2.0

        # Costmaps (0=free, 254=lethal, 255=unknown)
        # Use int8 range: -128 to 127. Map: 0=free, 100=lethal, -1=unknown
        self.global_costmap = np.full((self.global_ny, self.global_nx), 0, dtype=np.int8)
        self.local_costmap = np.full((self.local_ny, self.local_nx), 0, dtype=np.int8)

        # Moving obstacles
        self.moving_obstacles = []  # list of (x, y)

        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers
        self.global_pub = rospy.Publisher("/%s/global_costmap" % name, OccupancyGrid, queue_size=1, latch=True)
        self.local_pub = rospy.Publisher("/%s/local_costmap" % name, OccupancyGrid, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber("/moving_obstacles", PoseArray, self._obstacles_cb, queue_size=10)
        rospy.Subscriber("/%s/odom" % name, Odometry, self._odom_cb, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(0.1), self._update_timer)  # 10 Hz

        # Build static layer once
        self._build_static_layer()

        rospy.loginfo("simple_costmap [%s]: initialized (global=%dx%d, local=%dx%d, res=%.2f)",
                      name, self.global_nx, self.global_ny, self.local_nx, self.local_ny, self.resolution)

    def _build_static_layer(self):
        """Mark static obstacles in global costmap with inflation."""
        for ox, oy in self.static_obstacles:
            self._mark_obstacle_global(ox, oy, self.inflation_radius)

    def _mark_obstacle_global(self, x, y, inflation_r):
        """Mark obstacle at (x,y) with inflation in global costmap."""
        gx = int((x - self.global_origin_x) / self.resolution)
        gy = int((y - self.global_origin_y) / self.resolution)
        inf_cells = int(inflation_r / self.resolution)

        for dy in range(-inf_cells, inf_cells + 1):
            for dx in range(-inf_cells, inf_cells + 1):
                mx, my = gx + dx, gy + dy
                if 0 <= mx < self.global_nx and 0 <= my < self.global_ny:
                    dist = np.hypot(dx, dy) * self.resolution
                    if dist <= inflation_r:
                        # Quadratic falloff from lethal to free
                        cost = int(self.lethal_cost * (1.0 - dist / inflation_r) ** 2)
                        self.global_costmap[my, mx] = max(self.global_costmap[my, mx], cost)

    def _obstacles_cb(self, msg):
        self.moving_obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _update_timer(self, event):
        self._update_local_costmap()
        self._publish_costmaps()

    def _update_local_costmap(self):
        """Update local costmap: rolling window centered on robot + moving obstacles."""
        # Clear local costmap
        self.local_costmap.fill(0)

        # Origin of local map in global frame
        local_origin_x = self.robot_x - self.local_size_x / 2.0
        local_origin_y = self.robot_y - self.local_size_y / 2.0

        # Copy relevant portion of global costmap (static obstacles)
        gx0 = int((local_origin_x - self.global_origin_x) / self.resolution)
        gy0 = int((local_origin_y - self.global_origin_y) / self.resolution)

        for ly in range(self.local_ny):
            gy = gy0 + ly
            if 0 <= gy < self.global_ny:
                for lx in range(self.local_nx):
                    gx = gx0 + lx
                    if 0 <= gx < self.global_nx:
                        self.local_costmap[ly, lx] = self.global_costmap[gy, gx]

        # Add moving obstacles (with inflation)
        for ox, oy in self.moving_obstacles:
            lx = int((ox - local_origin_x) / self.resolution)
            ly = int((oy - local_origin_y) / self.resolution)
            inf_cells = int(self.inflation_radius / self.resolution)

            for dy in range(-inf_cells, inf_cells + 1):
                for dx in range(-inf_cells, inf_cells + 1):
                    mx, my = lx + dx, ly + dy
                    if 0 <= mx < self.local_nx and 0 <= my < self.local_ny:
                        dist = np.hypot(dx, dy) * self.resolution
                        if dist <= self.inflation_radius:
                            cost = int(self.lethal_cost * (1.0 - dist / self.inflation_radius) ** 2)
                            self.local_costmap[my, mx] = max(self.local_costmap[my, mx], cost)

    def _publish_costmaps(self):
        now = rospy.Time.now()

        # Global costmap
        global_msg = OccupancyGrid()
        global_msg.header = Header(stamp=now, frame_id=self.global_frame)
        global_msg.info = MapMetaData(
            resolution=self.resolution,
            width=self.global_nx,
            height=self.global_ny,
            origin=Pose(Point(self.global_origin_x, self.global_origin_y, 0), Quaternion(0, 0, 0, 1))
        )
        global_msg.data = self.global_costmap.flatten().tolist()
        self.global_pub.publish(global_msg)

        # Local costmap
        local_origin_x = self.robot_x - self.local_size_x / 2.0
        local_origin_y = self.robot_y - self.local_size_y / 2.0

        local_msg = OccupancyGrid()
        local_msg.header = Header(stamp=now, frame_id=self.global_frame)
        local_msg.info = MapMetaData(
            resolution=self.resolution,
            width=self.local_nx,
            height=self.local_ny,
            origin=Pose(Point(local_origin_x, local_origin_y, 0), Quaternion(0, 0, 0, 1))
        )
        local_msg.data = self.local_costmap.flatten().tolist()
        self.local_pub.publish(local_msg)

    def get_cost_at(self, x, y, use_local=True):
        """Query cost at world position (x,y). Returns 0-254."""
        if use_local:
            local_origin_x = self.robot_x - self.local_size_x / 2.0
            local_origin_y = self.robot_y - self.local_size_y / 2.0
            mx = int((x - local_origin_x) / self.resolution)
            my = int((y - local_origin_y) / self.resolution)
            if 0 <= mx < self.local_nx and 0 <= my < self.local_ny:
                return int(self.local_costmap[my, mx])
        else:
            mx = int((x - self.global_origin_x) / self.resolution)
            my = int((y - self.global_origin_y) / self.resolution)
            if 0 <= mx < self.global_nx and 0 <= my < self.global_ny:
                return int(self.global_costmap[my, mx])
        return 0


def main():
    rospy.init_node("simple_costmap", anonymous=True)
    robot_name = rospy.get_param("~robot_name", "robot_0")
    costmap = SimpleCostmap(robot_name)
    rospy.spin()


if __name__ == "__main__":
    main()