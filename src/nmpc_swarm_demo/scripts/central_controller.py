#!/usr/bin/env python
"""
central_controller.py

Single-node swarm controller for 2 differential-drive robots (robot_0, robot_1).

Architecture (kept minimal for a college PBL demo):
  * ONE node, ONE CasADi NMPC solver shared by all robots (run in a clear `for` loop).
  * Reads /move_base_simple/goal (RViz 2D Nav Goal) as the swarm target.
  * Reads /robot_N/odom for each robot's current state.
  * Reads /moving_obstacles (PoseArray) for obstacle positions.
  * Computes a per-robot lateral formation offset so both robots reach the goal
    side-by-side instead of stacking on top of each other.
  * Solves a small-horizon NMPC (unicycle dynamics: x_dot=v cosθ, y_dot=v sinθ,
    theta_dot=ω) with CasADi + IPOPT per robot per control cycle.
  * Publishes /robot_N/cmd_vel and /robot_N/path (predicted NMPC trajectory).

NMPC formulation:
  State  z = [x, y, theta]
  Control u = [v, omega]
  Dynamics (Euler): z_{k+1} = z_k + dt * f(z_k, u_k)
  Cost:  sum_k  w_pos * ||(x,y) - target||^2
         + w_theta * (1 - cos(theta - theta_target))   [heading, wrap-safe]
         + R_v   * v_k^2                                [control effort]
         + R_dv  * (v_k - v_{k-1})^2                   [acceleration smoothness]
  Constraints: v_min <= v <= v_max,  w_min <= omega <= w_max
  Soft penalty (barrier-ish, quadratic slack):
    * inter-robot distance  >= d_min   (penalty when too close)
    * robot-to-obstacle distance >= d_obs_min (penalty when too close)

Fallback: if IPOPT fails or odom is stale, publish v=0, omega=0.
"""

import math
import rospy
import casadi as ca
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header


class RobotState:
    """Latest odometry for one robot plus a staleness flag."""
    def __init__(self, name):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.last_time = None
        self.ready = False


class CentralController:
    def __init__(self):
        rospy.init_node("central_controller", anonymous=True)

        # -------------------- parameters --------------------
        self.robots = ["robot_0", "robot_1"]
        self.dt = rospy.get_param("~dt", 0.1)            # control period [s]
        self.horizon = rospy.get_param("~horizon", 10)   # NMPC steps
        self.ctrl_rate = rospy.get_param("~ctrl_rate", 10.0)  # Hz

        # kinematic limits
        self.v_min = rospy.get_param("~v_min", -0.4)
        self.v_max = rospy.get_param("~v_max", 0.5)
        self.w_min = rospy.get_param("~w_min", -1.5)
        self.w_max = rospy.get_param("~w_max", 1.5)

        # formation: lateral offset applied to left/right of goal heading
        self.formation_offset = rospy.get_param("~formation_offset", 0.35)

        # safety distances
        self.d_min = rospy.get_param("~d_min", 0.5)      # inter-robot min distance
        self.d_obs_min = rospy.get_param("~d_obs_min", 0.45)  # robot-obstacle min distance

        # cost weights
        self.w_pos = rospy.get_param("~w_pos", 10.0)
        self.w_theta = rospy.get_param("~w_theta", 1.0)
        self.R_v = rospy.get_param("~R_v", 0.5)
        self.R_dv = rospy.get_param("~R_dv", 1.0)
        self.w_avoid = rospy.get_param("~w_avoid", 50.0)  # soft-avoidance penalty weight

        # staleness: odom older than this is ignored
        self.odom_max_age = rospy.get_param("~odom_max_age", 1.0)

        # -------------------- solver build (once) --------------------
        self._build_nmpc_solver()

        # -------------------- state --------------------
        self.states = {name: RobotState(name) for name in self.robots}
        self.prev_u = {name: np.array([0.0, 0.0]) for name in self.robots}

        self.goal = None          # PoseStamped from RViz
        self.goal_time = None
        self.obstacles = []       # list of (x, y)

        # -------------------- pubs / subs --------------------
        self.cmd_pubs = {}
        self.path_pubs = {}
        self.paths = {name: Path() for name in self.robots}
        for name in self.robots:
            self.cmd_pubs[name] = rospy.Publisher("/%s/cmd_vel" % name, Twist, queue_size=10)
            self.path_pubs[name] = rospy.Publisher("/%s/path" % name, Path, queue_size=10)
            self.paths[name].header.frame_id = "odom"

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._goal_cb, queue_size=10)
        rospy.Subscriber("/moving_obstacles", PoseArray, self._obstacles_cb, queue_size=10)
        for name in self.robots:
            rospy.Subscriber("/%s/odom" % name, Odometry,
                             self._odom_cb, callback_args=name, queue_size=10)

        rospy.loginfo("central_controller: NMPC solver built for robots %s", self.robots)

    # ------------------------------------------------------------------ #
    # CasADi NMPC solver (built once, reused every cycle)
    # ------------------------------------------------------------------ #
    def _build_nmpc_solver(self):
        dt = self.dt
        N = self.horizon

        # Decision variables: states (N+1) x 3, controls N x 2
        z = ca.SX.sym("z", 3, N + 1)   # [x, y, theta] per step
        u = ca.SX.sym("u", 2, N)       # [v, omega] per step
        n_obs_max = self._n_obs_max()
        p = ca.SX.sym("p", 8 + 2 * n_obs_max)  # parameter vector

        # Parameter layout (fixed size):
        #   p[0:3]   = current state [x0, y0, theta0]
        #   p[3:5]   = target [tx, ty]
        #   p[5]     = target heading
        #   p[6:8]   = other robot current [ox, oy] (for inter-robot avoidance)
        #   p[8:]    = obstacle positions, 2 each (up to n_obs_max)

        cost = 0
        g = []          # constraints
        lbg = []        # lower bounds
        ubg = []        # upper bounds

        # Initial condition constraint: z[:,0] == current state
        g.append(z[:, 0] - p[0:3])
        lbg += [0.0, 0.0, 0.0]
        ubg += [0.0, 0.0, 0.0]

        # Dynamics + cost over horizon
        for k in range(N):
            xk = z[:, k]
            uk = u[:, k]
            x_next = ca.vertcat(
                xk[0] + dt * uk[0] * ca.cos(xk[2]),
                xk[1] + dt * uk[0] * ca.sin(xk[2]),
                xk[2] + dt * uk[1],
            )
            g.append(z[:, k + 1] - x_next)
            lbg += [0.0, 0.0, 0.0]
            ubg += [0.0, 0.0, 0.0]

            # Position tracking cost
            cost += self.w_pos * ((xk[0] - p[3]) ** 2 + (xk[1] - p[4]) ** 2)
            # Heading cost (wrap-safe: 1 - cos(delta))
            cost += self.w_theta * (1.0 - ca.cos(xk[2] - p[5]))
            # Control effort
            cost += self.R_v * uk[0] ** 2

            # Acceleration smoothness: penalize change in v from previous step
            if k >= 1:
                cost += self.R_dv * (uk[0] - u[0, k - 1]) ** 2

            # Inter-robot soft avoidance (only meaningful for k>=0 positions)
            # distance^2 to other robot >= d_min^2  --> penalty when violated
            dx_o = xk[0] - p[6]   # other robot x (current, static over horizon for simplicity)
            dy_o = xk[1] - p[7]
            dist2_o = dx_o * dx_o + dy_o * dy_o + 1e-3
            # soft penalty: if dist2 < d_min^2, add quadratic penalty
            viol_o = fmax(0.0, self.d_min ** 2 - dist2_o)
            cost += self.w_avoid * viol_o ** 2

            # Obstacle soft avoidance
            for i in range(self._n_obs_max()):
                ox = p[8 + 2 * i]
                oy = p[9 + 2 * i]
                dx = xk[0] - ox
                dy = xk[1] - oy
                dist2 = dx * dx + dy * dy + 1e-3
                viol = fmax(0.0, self.d_obs_min ** 2 - dist2)
                cost += self.w_avoid * viol ** 2

        # Terminal cost (same as stage at last step)
        xN = z[:, N]
        cost += self.w_pos * ((xN[0] - p[3]) ** 2 + (xN[1] - p[4]) ** 2)
        cost += self.w_theta * (1.0 - ca.cos(xN[2] - p[5]))

        # Pack NLP
        nlp = {"x": ca.vertcat(z.reshape((-1, 1)), u.reshape((-1, 1))),
               "f": cost,
               "g": ca.vertcat(*g),
               "p": p}

        opts = {"ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "print_time": False}
        self.solver = ca.nlpsol("solver", "ipopt", nlp, opts)

        # Decision-variable bounds
        n_z = 3 * (N + 1)
        n_u = 2 * N
        lbx = []
        ubx = []
        # states unbounded
        lbx += [-ca.inf] * n_z
        ubx += [ca.inf] * n_z
        # controls bounded
        for _ in range(N):
            lbx += [self.v_min, self.w_min]
            ubx += [self.v_max, self.w_max]
        self.lbx = lbx
        self.ubx = ubx

        self.n_z = n_z
        self.n_u = n_u
        self.n_g = len(lbg)
        self.lbg = lbg
        self.ubg = ubg

        rospy.loginfo("central_controller: CasADi/IPOPT NMPC ready (horizon=%d, dt=%.2f)", N, dt)

    def _n_obs_max(self):
        # Fixed max obstacles to keep parameter vector size constant
        return 3

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _goal_cb(self, msg):
        self.goal = msg
        self.goal_time = rospy.Time.now()
        rospy.loginfo("central_controller: new goal (%.2f, %.2f)",
                      msg.pose.position.x, msg.pose.position.y)

    def _obstacles_cb(self, msg):
        self.obstacles = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def _odom_cb(self, msg, name):
        st = self.states[name]
        st.x = msg.pose.pose.position.x
        st.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        st.theta = self._yaw_from_quaternion(q)
        st.v = msg.twist.twist.linear.x
        st.omega = msg.twist.twist.angular.z
        st.last_time = rospy.Time.now()
        st.ready = True

    @staticmethod
    def _yaw_from_quaternion(q):
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # ------------------------------------------------------------------ #
    # Formation target computation
    # ------------------------------------------------------------------ #
    def _formation_targets(self):
        """Return {robot_name: (tx, ty, theta_target)} with lateral offsets."""
        targets = {}
        if self.goal is None:
            return targets

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        q = self.goal.pose.orientation
        gtheta = self._yaw_from_quaternion(q)

        # lateral offset direction (perpendicular to goal heading)
        off_x = -math.sin(gtheta) * self.formation_offset
        off_y = math.cos(gtheta) * self.formation_offset

        for i, name in enumerate(self.robots):
            sign = 1.0 if i == 0 else -1.0
            targets[name] = (gx + sign * off_x, gy + sign * off_y, gtheta)
        return targets

    # ------------------------------------------------------------------ #
    # Solve NMPC for one robot
    # ------------------------------------------------------------------ #
    def _solve_robot(self, name, target):
        st = self.states[name]
        if not st.ready:
            return None
        if st.last_time is None or (rospy.Time.now() - st.last_time).to_sec() > self.odom_max_age:
            return None  # stale odom

        tx, ty, ttheta = target

        # Build parameter vector
        n_obs_max = self._n_obs_max()
        p_len = 8 + 2 * n_obs_max  # state(3)+target(2)+theta(1)+other(2)+obs(2*n_obs_max)
        p = np.zeros(p_len)
        p[0:3] = [st.x, st.y, st.theta]
        p[3:5] = [tx, ty]
        p[5] = ttheta

        # Other robot's current position (static over horizon)
        other = [n for n in self.robots if n != name][0]
        ost = self.states[other]
        p[6] = ost.x if ost.ready else st.x
        p[7] = ost.y if ost.ready else st.y

        # Obstacles (pad with far-away values if fewer than max)
        for i in range(n_obs_max):
            if i < len(self.obstacles):
                p[8 + 2 * i] = self.obstacles[i][0]
                p[9 + 2 * i] = self.obstacles[i][1]
            else:
                p[8 + 2 * i] = 1e3
                p[9 + 2 * i] = 1e3

        # Initial guess: hold previous control, propagate state
        x0 = np.zeros(self.n_z)
        u0 = np.zeros(self.n_u)
        for k in range(self.horizon + 1):
            x0[3 * k + 0] = st.x
            x0[3 * k + 1] = st.y
            x0[3 * k + 2] = st.theta
        for k in range(self.horizon):
            u0[2 * k + 0] = self.prev_u[name][0]
            u0[2 * k + 1] = self.prev_u[name][1]

        try:
            sol = self.solver(x0=np.concatenate([x0, u0]),
                              lbx=self.lbx, ubx=self.ubx,
                              lbg=self.lbg, ubg=self.ubg,
                              p=p)
        except Exception as exc:
            rospy.logerr("central_controller: IPOPT exception for %s: %s", name, exc)
            return None

        if sol is None:
            rospy.logwarn("central_controller: IPOPT returned None for %s", name)
            return None

        try:
            status = self.solver.stats()["return_status"]
        except Exception:
            status = None
        ok = (status == 0) or (isinstance(status, str) and "succeed" in status.lower())
        if not ok:
            rospy.logwarn("central_controller: IPOPT failed for %s (%s)", name, status)
            return None

        sol_x = np.array(sol["x"]).flatten()
        u_sol = sol_x[self.n_z:]  # controls after states
        v = float(u_sol[0])
        omega = float(u_sol[1])

        # Clamp to safety bounds (solver already bounds, but be defensive)
        v = max(self.v_min, min(self.v_max, v))
        omega = max(self.w_min, min(self.w_max, omega))

        self.prev_u[name] = np.array([v, omega])
        return v, omega, sol_x

    # ------------------------------------------------------------------ #
    # Path message from NMPC solution
    # ------------------------------------------------------------------ #
    def _publish_path(self, name, sol_x):
        path = Path()
        path.header = Header(stamp=rospy.Time.now(), frame_id="odom")
        N = self.horizon
        for k in range(N + 1):
            pose = Pose()
            pose.position.x = float(sol_x[3 * k + 0])
            pose.position.y = float(sol_x[3 * k + 1])
            pose.position.z = 0.0
            # yaw as quaternion (z-only)
            yaw = float(sol_x[3 * k + 2])
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self.path_pubs[name].publish(path)

    # ------------------------------------------------------------------ #
    # Main control loop
    # ------------------------------------------------------------------ #
    def run(self):
        rate = rospy.Rate(self.ctrl_rate)
        while not rospy.is_shutdown():
            targets = self._formation_targets()

            for name in self.robots:
                twist = Twist()  # default: zeros (safe fallback)
                sol_x = None

                if self.goal is not None and name in targets:
                    res = self._solve_robot(name, targets[name])
                    if res is not None:
                        v, omega, sol_x = res
                        twist.linear.x = v
                        twist.angular.z = omega
                        self._publish_path(name, sol_x)
                    else:
                        rospy.logdebug("central_controller: fallback zero cmd for %s", name)
                else:
                    rospy.logdebug("central_controller: no goal/target for %s", name)

                self.cmd_pubs[name].publish(twist)

            rate.sleep()


if __name__ == "__main__":
    try:
        CentralController().run()
    except rospy.ROSInterruptException:
        pass
