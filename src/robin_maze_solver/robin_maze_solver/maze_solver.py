import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray



LEFT_TRACK = [
    (0.60, 2.38), (2.42, 2.38), (2.42, 3.04), (3.66, 3.04), (3.66, 0.79),               # Sector 1 # Inlet & zig-zag awal
    (7.91, 0.79), (7.91, 5.86),                                                         # Sector 2 # Straight Corridor
    (7.68, 6.62), (7.11, 7.16), (6.36, 7.35), (5.61, 7.15), (5.06, 6.61), (4.84, 5.86), # Sector 3 # U-Turn
    (4.84, 4.27), (3.94, 4.27), (3.94, 6.78), (3.00, 6.78), (1.50, 7.54),               # Sector 4 # Transition & Open Chamber
    (0.28, 7.54), (0.28, 4.80)                                                          # Sector 5 # Final turn & finish
]


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(value, low, high):
    return max(low, min(high, value))


class SlewLimiter:
    def __init__(self, rate):
        self.rate = rate
        self.value = 0.0

    def step(self, target, dt):
        d = self.rate * dt
        self.value = clamp(target, self.value - d, self.value + d)
        return self.value

    def reset(self):
        self.value = 0.0


class Odometry:
    def __init__(self, r, s):
        self.r = r
        self.s = s
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_rotation = 0.0
        self.v = 0.0   
        self.w = 0.0  
        self.last_time = None
        self._clear()

    def _clear(self):
        self.sum_l = 0.0
        self.sum_r = 0.0
        self.n = 0

    def add_sample(self, phi_l, phi_r):
        self.sum_l += phi_l
        self.sum_r += phi_r
        self.n += 1

    def update(self, now):
        if self.last_time is None:
            self.last_time = now
            self._clear()
            return
        dt = now - self.last_time
        if dt <= 0.0 or self.n == 0:
            return
        phi_l = self.sum_l / self.n
        phi_r = self.sum_r / self.n
        self._clear()
        self.last_time = now
        if dt > 0.5:
            return

        v = self.r / 2.0 * (phi_l + phi_r)
        w = self.r / self.s * (-phi_l + phi_r)
        self.v, self.w = v, w

        th = self.theta
        if abs(w) > 1e-6:
            self.x += v / w * (math.sin(th + w * dt) - math.sin(th))
            self.y -= v / w * (math.cos(th + w * dt) - math.cos(th))
        else:
            self.x += v * math.cos(th) * dt
            self.y += v * math.sin(th) * dt
        self.theta = normalize_angle(th + w * dt)
        self.total_rotation += w * dt


class Path:
    def __init__(self, pts, waypoint_tolerance, vel_max, decel):
        self.pts = pts
        self.n = len(pts) - 1                       # jumlah segmen
        self.waypoint_tolerance = waypoint_tolerance
        self.vel_max = vel_max
        self.decel = decel
        self.seg_len = []
        self.seg_dir = []
        for i in range(self.n):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            L = math.hypot(dx, dy)
            self.seg_len.append(L)
            self.seg_dir.append((dx / L, dy / L) if L > 1e-9 else (1.0, 0.0))
        self.turn = [0.0] * (self.n + 1)
        for k in range(1, self.n):
            h1 = math.atan2(self.seg_dir[k - 1][1], self.seg_dir[k - 1][0])
            h2 = math.atan2(self.seg_dir[k][1], self.seg_dir[k][0])
            self.turn[k] = abs(normalize_angle(h2 - h1))
        self.idx = 0

    def _project(self, i, x, y):
        px, py = self.pts[i]
        ux, uy = self.seg_dir[i]
        t = clamp((x - px) * ux + (y - py) * uy, 0.0, self.seg_len[i])
        return t, math.hypot(x - (px + ux * t), y - (py + uy * t))

    def advance(self, x, y):
        passed = []
        while self.idx < self.n - 1:
            t, d0 = self._project(self.idx, x, y)
            _, d1 = self._project(self.idx + 1, x, y)
            near = (self.seg_len[self.idx] - t) <= self.waypoint_tolerance
            if d1 <= d0 or near:
                self.idx += 1
                passed.append(self.idx)
            else:
                break
        return passed

    def lookahead(self, x, y, dist):
        t, _ = self._project(self.idx, x, y)
        s = t + dist
        i = self.idx
        while s > self.seg_len[i] and i < self.n - 1:
            s -= self.seg_len[i]
            i += 1
        s = min(s, self.seg_len[i])
        return (self.pts[i][0] + self.seg_dir[i][0] * s,
                self.pts[i][1] + self.seg_dir[i][1] * s)

    def corner_speed(self, k):
        if k >= self.n:
            return 0.0 
        frac = 1.0 - self.turn[k] / (math.pi / 2.0)
        return clamp(self.vel_max * frac, 0.4, self.vel_max)

    def speed_limit(self, x, y):
        t, _ = self._project(self.idx, x, y)
        s = self.seg_len[self.idx] - t
        v_lim = self.vel_max
        brake_dist = self.vel_max ** 2 / (2.0 * self.decel)
        k = self.idx + 1
        while k <= self.n:
            v_c = self.corner_speed(k)
            v_lim = min(v_lim, math.sqrt(v_c ** 2 + 2.0 * self.decel * s))
            if s > brake_dist:
                break
            if k < self.n:
                s += self.seg_len[k]
            k += 1
        return v_lim

    def goal_distance(self, x, y):
        gx, gy = self.pts[-1]
        return math.hypot(gx - x, gy - y)


class maze_solver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        self.declare_parameter('mode', 'track')
        self.declare_parameter('track', 'left')
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.17)
        self.declare_parameter('vel_max', 10.0) # 4.0, 8.0
        self.declare_parameter('angular_max', 6.0) # 2.0, 4.0 
        self.declare_parameter('accel_linear', 4.0) # 1.0, 3.0
        self.declare_parameter('accel_angular', 5.0) # 3.0, 4.0
        self.declare_parameter('K_Vel', 3.0)
        self.declare_parameter('K_Theta', 2.0)
        self.declare_parameter('turn_in_place', 0.6)
        self.declare_parameter('waypoint_tolerance', 0.08)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('calib_distance', 6.0)
        self.declare_parameter('calib_turns', 4.0)

        gp = lambda n: self.get_parameter(n).value
        self.mode = str(gp('mode')).lower()
        track = str(gp('track')).lower()
        r = float(gp('wheel_radius'))
        s = float(gp('wheel_separation'))
        self.vel_max = float(gp('vel_max'))
        self.angular_max = float(gp('angular_max'))
        accel_linear = float(gp('accel_linear'))
        accel_angular = float(gp('accel_angular'))
        self.K_Vel = float(gp('K_Vel'))
        self.K_Theta = float(gp('K_Theta'))
        self.turn_in_place = float(gp('turn_in_place'))
        waypoint_tolerance = float(gp('waypoint_tolerance'))
        self.goal_tol = float(gp('goal_tolerance'))
        self.calib_distance = float(gp('calib_distance'))
        self.calib_turns = float(gp('calib_turns'))
        self.r, self.s = r, s
        self.accel_angular = accel_angular

        if self.mode == 'lurus':
            pts = [(0.0, 0.0), (self.calib_distance, 0.0)]
            self.goal_tol = 0.01
        elif self.mode == 'rotate':
            pts = None
        else:
            self.mode = 'track'
            if track != 'right':
                track = 'left'
            pts = [(0.0, 0.0)] + (RIGHT_TRACK if track == 'right' else LEFT_TRACK)
        self.track = track

        self.path = Path(pts, waypoint_tolerance, self.vel_max, 0.8 * accel_linear) if pts else None

        self.odom = Odometry(r, s)
        self.lim_v = SlewLimiter(accel_linear)
        self.lim_w = SlewLimiter(accel_angular)

        self.got_wheel_data = False
        self.started = False
        self.finished = False
        self.start_time = None
        self.last_ctrl_time = None
        self.last_log_time = None
        self.dt_ctrl = 0.02                         # kontrol 50 Hz

        self.create_subscription(
            Float64MultiArray, '/wheel/speed', self.wheel_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt_ctrl, self.control_loop)

        self.get_logger().info(
            f'Mode {self.mode.upper()} | track {self.track.upper()} | '
            f'r={r} m, s={s} m | use_sim_time={gp("use_sim_time")} | '
            f'waiting data /wheel/speed ...')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wheel_callback(self, msg):
        if len(msg.data) < 2:
            return
        self.odom.add_sample(float(msg.data[0]), float(msg.data[1]))
        self.got_wheel_data = True
        self.odom.update(self.now_sec())

    def control_loop(self):
        if self.finished:
            self.stop()
            return

        now = self.now_sec()
        if not self.started:
            if not self.got_wheel_data or now == 0.0:
                return
            self.started = True
            self.start_time = now
            self.last_ctrl_time = now
            self.get_logger().info('START! Robot start moving.')
            return

        dt = now - self.last_ctrl_time
        if dt <= 0.0:
            return                                  # jam simulasi belum maju
        self.last_ctrl_time = now
        dt = min(dt, 0.2)

        if self.mode == 'rotate':
            v_cmd, w_cmd, info = self.rotate_command()
        else:
            v_cmd, w_cmd, info = self.track_command()
        if self.finished:
            return

        self.send(self.lim_v.step(v_cmd, dt), self.lim_w.step(w_cmd, dt))
        self.log_status(info)

    def track_command(self):
        o, p = self.odom, self.path

        for k in p.advance(o.x, o.y):
            self.get_logger().info(
                f'Waypoint {k}/{p.n} completed (x={o.x:.2f}, y={o.y:.2f})')

        d_goal = p.goal_distance(o.x, o.y)
        if d_goal <= self.goal_tol:
            self.finish()
            return 0.0, 0.0, 0.0

        look = clamp(0.2 + 0.2 * abs(self.lim_v.value), 0.25, 0.6)
        cx, cy = p.lookahead(o.x, o.y, look)
        e_theta = normalize_angle(math.atan2(cy - o.y, cx - o.x) - o.theta)

        w_cmd = clamp(self.K_Theta * e_theta, -self.angular_max, self.angular_max)

        v_lim = min(p.speed_limit(o.x, o.y), self.K_Vel * d_goal, self.vel_max)
        v_lim = max(v_lim, 0.05)                    # cegah macet di dekat finish

        ramp = clamp((self.turn_in_place - abs(e_theta)) /
                     (0.3 * self.turn_in_place), 0.0, 1.0)
        v_cmd = v_lim * max(0.0, math.cos(e_theta)) * ramp
        return v_cmd, w_cmd, d_goal

    def rotate_command(self):
        remaining = 2.0 * math.pi * self.calib_turns - self.odom.total_rotation
        if remaining <= 0.002:
            self.finish()
            return 0.0, 0.0, 0.0
        w_cmd = min(self.K_Theta * remaining,
                    math.sqrt(2.0 * 0.8 * self.acc_ang * remaining),
                    self.angular_max)
        return 0.0, max(w_cmd, 0.05), remaining

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.lim_v.reset()
        self.lim_w.reset()
        self.send(0.0, 0.0)

    def finish(self):
        self.finished = True
        self.stop()
        o = self.odom
        elapsed = self.now_sec() - self.start_time
        log = self.get_logger()
        log.info('=' * 55)
        if self.mode == 'straight':
            log.info(f'Straight calibration done: distance estimation = {o.x:.3f} m')
            log.info(f'  r_new = {self.r} x (x_gazebo / {o.x:.3f})')
        elif self.mode == 'rotate':
            turns = o.total_rotation / (2.0 * math.pi)
            log.info(f'Rotate calibration done: estimation = {turns:.3f} rotation')
            log.info('  Check the robots orientation at the Gazebo (initially facing +x):')
            log.info('  above target (excess rotation) -> s is reduced')
            log.info('  below target (under-rotation)   -> s is increased')
            log.info(f'  s_new = {self.s} x {360 * turns:.0f} / real_angle_degress')
        else:
            log.info(f'FINISH! Track {self.track.upper()}')
        log.info(f'Travel Time : {elapsed:.2f} seconds')
        log.info(f'Final position (estimate): x ={o.x:.2f} m, y ={o.y:.2f} m, '
                 f'theta = {math.degrees(o.theta):.1f} deg')
        log.info('=' * 55)

    def log_status(self, remaining):
        now = self.now_sec()
        if self.last_log_time is not None and now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        o = self.odom
        wp = (f'WP {self.path.idx + 1}/{self.path.n}'
              if self.path else 'rotate')
        self.get_logger().info(
            f'time = {now - self.start_time:6.1f} s | v = {self.lim_v.value:4.2f} \n'
            f'| x ={o.x:5.2f} y = {o.y:5.2f} | theta = {math.degrees(o.theta):6.1f} deg \n'
            f'| remaining = {remaining:.2f} |  {wp} ')


def main(args=None):
    rclpy.init(args=args)
    node = maze_solver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()