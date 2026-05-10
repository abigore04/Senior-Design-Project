#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
coverage_metal_detector_v20.py — FINAL
=============================================================================
Yahboom Transbot — tank tracks / skid steer
ROS Melodic / Python 2.7

DESIGNED FOR TANK TRACKS:
  - Every turn verifies final heading and corrects if needed
  - After every avoidance: explicit perpendicular return to lane line
  - Cross-track error measured and corrected before resuming row
  - All turns use PID with stall burst for track slippage
  - Odometry-based distance with DIST_SCALE calibration

OBSTACLE AVOIDANCE (from v3, hardened):
  8-step sequence with 3-phase edge tracking.
  After avoidance: perpendicular correction back to exact lane centre.

ARM: FIXED J7=90 J8=105 — set once, never moves.

ROBOT: 0.70m long (with arm), 0.26m wide, 0.26m tall (LiDAR height)
       Arm centered on hood. LiDAR faces rear (offset 180 deg).
"""

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from transbot_msgs.msg import Arm, Joint

# =============================================================================
# CONSTANTS
# =============================================================================
DIST_SCALE          = 1.0433   # real/odom ratio from calibration
LIDAR_ANGLE_OFFSET  = 180.0   # LiDAR faces rear
FOV_HALF_DEG        = 10.0    # front sector for obstacle detection

# Arm — set once, never changes
ARM_J7      = 90.0
ARM_J8      = 105.0
ARM_TIME_MS = 600

# Metal
METAL_TOPIC = "/metal_detections"
METAL_PAUSE = 3.0

# Motion — conservative for tank tracks
V_FWD        = 0.18   # scan speed (slightly slower for accuracy)
V_AVOID      = 0.12   # speed during avoidance manoeuvres
V_RETURN     = 0.10   # speed for perpendicular lane return
MAX_WZ_ROT   = 0.35   # max turn speed
MAX_WZ_DRIVE = 0.50   # max heading correction during drive

# PID rotation — tuned for tank track slippage
KP_ROT  = 2.2    # slightly higher P for tracks
KI_ROT  = 0.08   # more I to overcome static friction
KD_ROT  = 0.25

# PID heading
KP_HEAD = 4.5
KI_HEAD = 0.10
KD_HEAD = 0.50

# Composite controller
K_GYRO                = 0.30   # higher gyro damping for track wobble
K_WALL                = 0.20
WALL_BALANCE_MAX_DIST = 1.80
K_CROSS_TRACK         = 1.50   # stronger cross-track for tracks (was 1.20)
MAX_CROSS_TRACK_CORR  = 0.40

# Coverage frame
RELOCALIZE_FWD_DIST   = 0.30   # settle drive after recovery
OBSTACLE_MEMORY_DIST  = 0.45
MAX_OBSTACLE_REPEATS  = 2

# Lane return tolerance
LANE_RETURN_TOL       = 0.05   # 5 cm tolerance for lane centre
LANE_RETURN_MAX_DIST  = 2.0    # max perpendicular correction distance

# Rotation tolerances — tighter for tracks
SLOW_DEG     = 45.0
SLOW_WZ_MIN  = 0.12   # higher minimum to overcome track friction
FINE_TOL_DEG = 1.5     # slightly looser — tracks can't hold <1 deg
YAW_TOL_DEG  = 2.0

# Stall detection — more aggressive for tracks
STALL_WINDOW   = 1.5   # check more often (tracks stall faster)
STALL_ANG_MIN  = 0.04
STALL_BURST_WZ = 0.50  # stronger burst to break track grip
STALL_BURST_T  = 0.5

# Robot geometry
ROBOT_W         = 0.26
ROBOT_L         = 0.70   # with arm
LIDAR_TO_FRONT  = 0.60
LIDAR_TO_REAR   = 0.10
SPOOL_WIDTH     = 0.22
SPOOL_TURN_RADIUS = 0.70

# Obstacle distances
BUMPER_DANGER    = 0.18   # hard stop: 0.60 + 0.18 = 0.78 m
BUMPER_SLOW      = 0.50   # slow zone: 0.60 + 0.50 = 1.10 m

# v3 obstacle avoidance
SIDE_CLEAR_THRESH = 0.85
SIDE_CLEAR_COUNT  = 5
OBST_DROP_THRESH  = 0.80
OBST_RISE_DELTA   = 0.35
OBST_RISE_THRESH  = 0.85
OBST_RISE_COUNT   = 5
EXTRA_CLEARANCE   = 0.80   # generous clearance for 0.70m long robot
ZONE_HARD_BLOCK   = 0.30

# Misc
SCAN_PCT   = 15
DIST_TOL   = 0.04
RATE_HZ    = 50
MAX_STALLS = 5

# =============================================================================
# HELPERS
# =============================================================================
def clamp(v, lo, hi): return max(lo, min(hi, v))

def norm_angle(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a < -math.pi: a += 2.0*math.pi
    return a

def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

def yaw_from_quat_list(q):
    return math.atan2(2.0*(q[3]*q[2]+q[0]*q[1]), 1.0-2.0*(q[1]*q[1]+q[2]*q[2]))

def scan_sector_pct(scan, deg_lo, deg_hi, pct=15):
    if scan is None: return float("inf")
    a_min=scan.angle_min; a_inc=scan.angle_increment; n=len(scan.ranges)
    if n==0 or a_inc==0.0: return float("inf")
    offset=math.radians(LIDAR_ANGLE_OFFSET)
    r_lo=norm_angle(math.radians(min(deg_lo,deg_hi))+offset)
    r_hi=norm_angle(math.radians(max(deg_lo,deg_hi))+offset)
    sectors=[(r_lo,math.pi),(-math.pi,r_hi)] if r_lo>r_hi else [(r_lo,r_hi)]
    valid=[]
    for (s_lo,s_hi) in sectors:
        i0=clamp(int(math.floor((s_lo-a_min)/a_inc)),0,n-1)
        i1=clamp(int(math.ceil((s_hi-a_min)/a_inc)),0,n-1)
        for i in xrange(i0,i1+1):
            r=scan.ranges[i]
            if not(math.isnan(r) or math.isinf(r)) and 0.05<r<20.0:
                valid.append(r)
    if not valid: return float("inf")
    valid.sort()
    return valid[clamp(int(len(valid)*pct/100.0)-1,0,len(valid)-1)]

# =============================================================================
# PID
# =============================================================================
class PIDController(object):
    def __init__(self,kp,ki,kd,max_out,min_out=0.0):
        self.kp=kp;self.ki=ki;self.kd=kd;self.max_out=max_out;self.min_out=min_out
        self.integral=0.0;self.prev_error=0.0
    def reset(self): self.integral=0.0;self.prev_error=0.0
    def compute(self,error,dt):
        if dt<=0: return 0.0
        self.integral+=error*dt
        ki_s=max(self.ki,1e-9)
        self.integral=clamp(self.integral,-self.max_out/ki_s,self.max_out/ki_s)
        d=(error-self.prev_error)/dt;self.prev_error=error
        out=self.kp*error+self.ki*self.integral+self.kd*d
        s=1.0 if out>=0 else -1.0;m=abs(out)
        if m<1e-9: return 0.0
        return s*clamp(m,self.min_out,self.max_out)

# =============================================================================
# NODE
# =============================================================================
class CoverageNode(object):

    def __init__(self):
        rospy.init_node("metal_detector_coverage",anonymous=False)
        self.pub_vel=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self.pub_arm=rospy.Publisher("/TargetAngle",Arm,queue_size=1)
        rospy.Subscriber("/odom",Odometry,self._cb_odom,queue_size=1)
        rospy.Subscriber("/imu/data",Imu,self._cb_imu,queue_size=1)
        rospy.Subscriber("/scan",LaserScan,self._cb_scan,queue_size=1)
        rospy.Subscriber(METAL_TOPIC,MarkerArray,self._cb_metal,queue_size=1)
        self.odom=None;self.imu=None;self.scan=None;self.gyro_bias=0.0
        self._known_mc=0;self._mflag=False;self._mpos=[]
        self.rate=rospy.Rate(RATE_HZ)
        self.tf_listener=tf.TransformListener()
        self.zone_danger=LIDAR_TO_FRONT+BUMPER_DANGER
        self.zone_slow=LIDAR_TO_FRONT+BUMPER_SLOW
        self.pid_rot=PIDController(KP_ROT,KI_ROT,KD_ROT,MAX_WZ_ROT,SLOW_WZ_MIN)
        self.pid_head=PIDController(KP_HEAD,KI_HEAD,KD_HEAD,MAX_WZ_DRIVE,0.0)
        self.length_m=None;self.width_m=None;self.lane_spacing=None
        self.mo_x=None;self.mo_y=None;self.mo_yaw=None
        self.last_obs_xy=None;self.obs_hits=0
        rospy.on_shutdown(self._shutdown)

    def _set_arm(self):
        msg=Arm()
        for jid,a in ((7,ARM_J7),(8,ARM_J8)):
            j=Joint();j.id=jid;j.run_time=ARM_TIME_MS;j.angle=float(a);msg.joint.append(j)
        self.pub_arm.publish(msg)
        rospy.loginfo("ARM FIXED: J7=%.0f J8=%.0f",ARM_J7,ARM_J8)
        rospy.sleep(ARM_TIME_MS/1000.0+0.2)

    def _cb_odom(self,m): self.odom=m
    def _cb_imu(self,m): self.imu=m
    def _cb_scan(self,m): self.scan=m
    def _cb_metal(self,m):
        c=len(m.markers)
        if c>self._known_mc:
            n=m.markers[-1]
            mx=round(n.pose.position.x,3);my=round(n.pose.position.y,3)
            self._mpos.append((mx,my));self._known_mc=c;self._mflag=True
            rospy.logwarn("*** METAL #%d map=(%.3f,%.3f) ***",c,mx,my)

    def _shutdown(self):
        rospy.loginfo("Shutdown.")
        for _ in xrange(10): self.pub_vel.publish(Twist());rospy.sleep(0.05)
        rospy.loginfo("Metal: %d",len(self._mpos))
        for i,(x,y) in enumerate(self._mpos): rospy.loginfo("  #%d x=%.3f y=%.3f",i+1,x,y)

    def _check_metal(self):
        if not self._mflag: return
        self.stop();rospy.logwarn("Metal dwell %.1f s",METAL_PAUSE)
        rospy.sleep(METAL_PAUSE);self._mflag=False

    # ── Pose ──────────────────────────────────────────────────────────────────
    def odom_pose(self):
        if self.odom:
            p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
            return p.x,p.y,yaw_from_quat(q)
        return 0.,0.,0.

    def map_pose(self):
        try:
            self.tf_listener.waitForTransform("/map","/base_footprint",rospy.Time(0),rospy.Duration(0.1))
            t,r=self.tf_listener.lookupTransform("/map","/base_footprint",rospy.Time(0))
            return t[0],t[1],yaw_from_quat_list(r)
        except: return self.odom_pose()

    def stop(self): self.pub_vel.publish(Twist())

    def _gz(self):
        return (self.imu.angular_velocity.z-self.gyro_bias) if self.imu else 0.0

    # ── Coverage frame ────────────────────────────────────────────────────────
    def _set_origin(self):
        x,y,yaw=self.map_pose()
        self.mo_x=x;self.mo_y=y;self.mo_yaw=yaw
        rospy.loginfo("Origin: x=%.3f y=%.3f yaw=%.1f",x,y,math.degrees(yaw))

    def _cov(self,x=None,y=None):
        """Returns (along, cross) in mission frame."""
        if self.mo_x is None: return 0.,0.
        if x is None: x,y,_=self.map_pose()
        dx=x-self.mo_x;dy=y-self.mo_y
        c=math.cos(self.mo_yaw);s=math.sin(self.mo_yaw)
        return dx*c+dy*s, -dx*s+dy*c

    def _row_heading(self,li):
        if self.mo_yaw is None: return self.map_pose()[2]
        return norm_angle(self.mo_yaw) if li%2==0 else norm_angle(self.mo_yaw+math.pi)

    def _row_cross(self,li): return li*self.lane_spacing

    def _row_prog(self,li):
        a,_=self._cov()
        return a if li%2==0 else self.length_m-a

    def _cross_error(self,li):
        """Signed cross-track error: positive = robot is too far left."""
        _,cross=self._cov()
        return self._row_cross(li)-cross

    # ── LiDAR ─────────────────────────────────────────────────────────────────
    def front(self): return scan_sector_pct(self.scan,-FOV_HALF_DEG,FOV_HALF_DEG,SCAN_PCT)
    def right(self): return scan_sector_pct(self.scan,-120.,-60.,SCAN_PCT)
    def left(self):  return scan_sector_pct(self.scan,60.,120.,SCAN_PCT)
    def _obs_side(self,s): return self.right() if s>0 else self.left()

    def _turn_ok(self,side):
        d=self.left() if side>0 else self.right()
        if d<SPOOL_TURN_RADIUS:
            rospy.logwarn("Turn blocked (%.2f<%.2f)",d,SPOOL_TURN_RADIUS);return False
        return True

    def _obs_repeated(self):
        x,y,_=self.map_pose()
        if self.last_obs_xy is None:
            self.last_obs_xy=(x,y);self.obs_hits=1;return False
        d=math.hypot(x-self.last_obs_xy[0],y-self.last_obs_xy[1])
        if d<OBSTACLE_MEMORY_DIST: self.obs_hits+=1
        else: self.last_obs_xy=(x,y);self.obs_hits=1
        return self.obs_hits>MAX_OBSTACLE_REPEATS

    def _obs_clear(self): self.last_obs_xy=None;self.obs_hits=0

    # ═══════════════════════════════════════════════════════════════════════════
    # ROTATE — PID with stall burst, verify final heading
    # ═══════════════════════════════════════════════════════════════════════════
    def rotate(self, angle_rad, timeout=30.0):
        """
        Rotate by angle_rad using PID.
        Tank track specific:
          - Higher I gain to overcome static friction
          - Stall burst if tracks grip and don't move
          - After rotation: verify heading, correct if >3 deg off
        """
        if abs(angle_rad)<math.radians(YAW_TOL_DEG): return

        side=1 if angle_rad>0 else -1
        wc=0
        while not self._turn_ok(side):
            if wc>10: rospy.logwarn("No room — turning anyway");break
            rospy.sleep(0.5);wc+=1

        _,_,y0=self.odom_pose()
        yt=norm_angle(y0+angle_rad)
        rospy.loginfo("ROTATE %+.1f deg",math.degrees(angle_rad))

        t0=rospy.Time.now().to_sec();lt=t0;st=t0;sy=y0
        self.pid_rot.reset()

        while not rospy.is_shutdown():
            now=rospy.Time.now().to_sec();dt=max(now-lt,0.001)
            if now-t0>timeout: rospy.logwarn("Rotate timeout");break
            _,_,yn=self.odom_pose()
            err=norm_angle(yt-yn)
            if abs(err)<math.radians(FINE_TOL_DEG): break

            # Stall detection
            if now-st>=STALL_WINDOW:
                if abs(norm_angle(yn-sy))<STALL_ANG_MIN:
                    rospy.logwarn("STALL — burst %.2f rad/s for %.1f s",STALL_BURST_WZ,STALL_BURST_T)
                    bs=1.0 if angle_rad>0 else -1.0;tb=now
                    while rospy.Time.now().to_sec()-tb<STALL_BURST_T:
                        tw=Twist();tw.angular.z=bs*STALL_BURST_WZ
                        self.pub_vel.publish(tw);self.rate.sleep()
                st=now;sy=yn

            wz=self.pid_rot.compute(err,dt)
            if abs(err)<math.radians(SLOW_DEG):
                f=abs(err)/math.radians(SLOW_DEG)
                wz=clamp(wz,-max(MAX_WZ_ROT*f,SLOW_WZ_MIN),max(MAX_WZ_ROT*f,SLOW_WZ_MIN))

            tw=Twist();tw.angular.z=wz;self.pub_vel.publish(tw)
            lt=now;self.rate.sleep()

        self.stop();rospy.sleep(0.2)

        # TANK TRACK VERIFY: if final error > 3 deg, do a correction pass
        _,_,yf=self.odom_pose()
        ferr=norm_angle(yt-yf)
        if abs(ferr)>math.radians(3.0):
            rospy.logwarn("Track slip: %.1f deg off — correcting",math.degrees(ferr))
            self.rotate(ferr, timeout=10.0)
        else:
            rospy.loginfo("Rotate OK: err=%.1f deg",math.degrees(ferr))

    def snap_heading(self, target_yaw, timeout=20.0):
        err=norm_angle(target_yaw-self.map_pose()[2])
        if abs(err)<math.radians(YAW_TOL_DEG): return
        self.rotate(err,timeout=timeout)

    # ═══════════════════════════════════════════════════════════════════════════
    # DRIVE STRAIGHT — composite controller with metal check
    # ═══════════════════════════════════════════════════════════════════════════
    def drive(self, distance, obstacles=True, timeout=60.0, scanning=True,
              th=None, tc=None, map_prog=False, speed=None):
        """
        Drive forward with composite heading controller.
        th = target heading (None = use current)
        tc = target cross-track (None = no cross correction)
        """
        if distance<=0: return True
        if speed is None: speed=V_FWD
        dadj=distance*DIST_SCALE
        x0,y0,_=self.odom_pose();a0,_=self._cov()
        t0=rospy.Time.now().to_sec();lt=t0;self.pid_head.reset()

        while not rospy.is_shutdown():
            now=rospy.Time.now().to_sec();dt=max(now-lt,0.001)
            if now-t0>timeout: rospy.logwarn("Drive timeout");self.stop();return False
            if scanning: self._check_metal()
            xo,yo,_=self.odom_pose();xm,ym,ym_yaw=self.map_pose()

            if map_prog:
                an,_=self._cov(xm,ym);trav=abs(an-a0)
            else:
                trav=math.hypot(xo-x0,yo-y0)
            if trav>=dadj-DIST_TOL: break

            yr=ym_yaw if th is None else th
            wz_pid=self.pid_head.compute(norm_angle(yr-ym_yaw),dt)
            wz_ff=-K_GYRO*self._gz()

            wz_wall=0.0
            dl=self.left();dr=self.right()
            if not math.isinf(dl) and dl<WALL_BALANCE_MAX_DIST and \
               not math.isinf(dr) and dr<WALL_BALANCE_MAX_DIST:
                wz_wall=K_WALL*(dl-dr)

            wz_ct=0.0
            if tc is not None:
                _,cn=self._cov(xm,ym)
                wz_ct=clamp(K_CROSS_TRACK*(tc-cn),-MAX_CROSS_TRACK_CORR,MAX_CROSS_TRACK_CORR)

            wz=clamp(wz_pid+wz_ff+wz_wall+wz_ct,-MAX_WZ_DRIVE,MAX_WZ_DRIVE)
            spd=speed

            if obstacles:
                df=self.front()
                if df<=self.zone_danger:
                    rospy.logwarn("STOP: obstacle %.2f m",df);self.stop();return False
                elif df<self.zone_slow:
                    r=(df-self.zone_danger)/(self.zone_slow-self.zone_danger)
                    spd=speed*(0.30+0.70*clamp(r,0.,1.))

            tw=Twist();tw.linear.x=spd;tw.angular.z=wz
            self.pub_vel.publish(tw);lt=now;self.rate.sleep()

        self.stop();rospy.sleep(0.15);return True

    def reverse(self, distance=0.30, timeout=10.0):
        rospy.loginfo("Reverse %.2f m",distance)
        x0,y0,yr=self.odom_pose();t0=rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec()-t0>timeout: break
            x,y,yaw=self.odom_pose()
            if math.hypot(x-x0,y-y0)>=distance-0.02: break
            wz=clamp(KP_HEAD*norm_angle(yr-yaw),-MAX_WZ_DRIVE,MAX_WZ_DRIVE)
            tw=Twist();tw.linear.x=-V_AVOID;tw.angular.z=wz
            self.pub_vel.publish(tw);self.rate.sleep()
        self.stop();rospy.sleep(0.2)

    # ═══════════════════════════════════════════════════════════════════════════
    # RETURN TO LANE — perpendicular correction for tank tracks
    # ═══════════════════════════════════════════════════════════════════════════
    def _return_to_lane(self, lane_index):
        """
        After obstacle avoidance, the robot may be offset from its lane.
        This function:
          1. Measures cross-track error
          2. If error > tolerance: turn perpendicular, drive to correct, turn back
          3. Verify heading matches row target

        This is essential for tank tracks which accumulate lateral error
        during the multiple turns of obstacle avoidance.
        """
        ce = self._cross_error(lane_index)
        th = self._row_heading(lane_index)

        if abs(ce) <= LANE_RETURN_TOL:
            rospy.loginfo("Lane OK: cross err=%.3f m (tol=%.3f)",ce,LANE_RETURN_TOL)
            self.snap_heading(th)
            return

        rospy.loginfo("LANE CORRECTION: cross err=%.3f m -> driving %.3f m perpendicular",
                      ce, abs(ce))

        # Turn perpendicular toward the lane centre
        perp_angle = math.pi/2.0 if ce > 0 else -math.pi/2.0
        perp_heading = norm_angle(th + perp_angle)

        self.snap_heading(perp_heading)
        corr_dist = min(abs(ce), LANE_RETURN_MAX_DIST)
        self.drive(corr_dist, obstacles=False, scanning=False, speed=V_RETURN)
        self.snap_heading(th)

        # Verify correction
        ce2 = self._cross_error(lane_index)
        rospy.loginfo("Lane correction done: err was %.3f m, now %.3f m", ce, ce2)

    # ═══════════════════════════════════════════════════════════════════════════
    # OBSTACLE AVOIDANCE — v3 8-step with 3-phase edge tracking
    # ═══════════════════════════════════════════════════════════════════════════
    def _choose_side(self, allowed):
        dl=self.left();dr=self.right()
        dl=dl if not math.isinf(dl) else 99.
        dr=dr if not math.isinf(dr) else 99.
        d_a=dl if allowed>0 else dr
        best=+1 if dl>=dr else -1
        rospy.loginfo("Side choice: L=%.2f R=%.2f allowed=%s",
                      dl,dr,"LEFT" if allowed>0 else "RIGHT")
        if best==allowed:
            rospy.loginfo("-> %s (more room + in zone)","LEFT" if allowed>0 else "RIGHT")
            return allowed
        if d_a>=ZONE_HARD_BLOCK:
            rospy.logwarn("-> %s (less room but in zone)","LEFT" if allowed>0 else "RIGHT")
            return allowed
        rospy.logwarn("!! Allowed blocked (%.2f m) -> using opposite",d_a)
        return -allowed

    def _sidestep_until_clear(self, side, timeout=20.0):
        """Drive forward until obstacle clears from watching side."""
        x0,y0,yr=self.odom_pose();t0=rospy.Time.now().to_sec();cc=0
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec()-t0>timeout:
                rospy.logwarn("Sidestep timeout");break
            ds=self._obs_side(side)
            if not math.isinf(ds) and ds>SIDE_CLEAR_THRESH:
                cc+=1
                if cc>=SIDE_CLEAR_COUNT:
                    rospy.loginfo("Side clear: %.2f m",ds);break
            else: cc=0
            if self.front()<=self.zone_danger:
                rospy.logwarn("Front blocked during sidestep");break
            x,y,yaw=self.odom_pose()
            wz=clamp(KP_HEAD*norm_angle(yr-yaw),-MAX_WZ_DRIVE,MAX_WZ_DRIVE)
            tw=Twist();tw.linear.x=V_AVOID;tw.angular.z=wz
            self.pub_vel.publish(tw);self.rate.sleep()
        self.stop();rospy.sleep(0.15)
        x1,y1,_=self.odom_pose()
        return math.hypot(x1-x0,y1-y0)

    def _drive_past(self, side, timeout=40.0):
        """3-phase edge tracking: detect → track min → detect edge → clearance."""
        x0,y0,yr=self.odom_pose();t0=rospy.Time.now().to_sec()
        phase=0;d_min=float("inf");cnt=0;ex,ey=x0,y0

        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec()-t0>timeout:
                rospy.logwarn("Drive-past timeout phase %d",phase);break
            x,y,yaw=self.odom_pose()
            ds=self._obs_side(side)

            if phase==0:
                if not math.isinf(ds) and ds<OBST_DROP_THRESH:
                    cnt+=1
                    if cnt>=OBST_RISE_COUNT:
                        phase=1;d_min=ds;cnt=0
                        rospy.loginfo("Phase 1: obstacle at %.2f m",ds)
                else: cnt=0

            elif phase==1:
                if not math.isinf(ds) and ds<d_min: d_min=ds
                if not math.isinf(ds) and ds>d_min+OBST_RISE_DELTA and ds>OBST_RISE_THRESH:
                    cnt+=1
                    if cnt>=OBST_RISE_COUNT:
                        phase=2;ex=x;ey=y;cnt=0
                        rospy.loginfo("Phase 2: edge! d=%.2f min=%.2f",ds,d_min)
                else: cnt=0

            elif phase==2:
                if math.hypot(x-ex,y-ey)>=EXTRA_CLEARANCE:
                    rospy.loginfo("Past obstacle (%.2f m clearance)",EXTRA_CLEARANCE);break

            if self.front()<=self.zone_danger:
                rospy.logwarn("Front blocked during drive-past");break

            wz=clamp(KP_HEAD*norm_angle(yr-yaw),-MAX_WZ_DRIVE,MAX_WZ_DRIVE)
            tw=Twist();tw.linear.x=V_AVOID;tw.angular.z=wz
            self.pub_vel.publish(tw);self.rate.sleep()

        self.stop();rospy.sleep(0.15)
        x1,y1,_=self.odom_pose()
        return math.hypot(x1-x0,y1-y0)

    def avoid(self, lane_index=0, allowed_side=+1):
        """
        8-step avoidance + perpendicular lane return.
        After completion, robot is back on its exact lane line.
        """
        rospy.logwarn("=== OBSTACLE (row %d, front=%.2f m) ===",lane_index,self.front())

        if self._obs_repeated():
            rospy.logwarn("Repeated obstacle — skip");self._obs_clear();return

        side=self._choose_side(allowed_side)
        half=math.pi/2.0

        # [1] Turn perpendicular
        rospy.loginfo("[1/8] Turn %s","LEFT" if side>0 else "RIGHT")
        self.rotate(side*half)

        # [2] Drive until side clears
        rospy.loginfo("[2/8] Sidestep until clear")
        sd=self._sidestep_until_clear(side)
        rospy.loginfo("[2/8] Sidestepped %.3f m",sd)

        # [3] Turn back to row heading
        rospy.loginfo("[3/8] Turn back")
        self.rotate(-side*half)

        # [4] Drive past with 3-phase tracking
        rospy.loginfo("[4/8] Drive past obstacle")
        self._drive_past(side)

        # [5] Turn toward lane
        rospy.loginfo("[5/8] Turn toward lane")
        self.rotate(-side*half)

        # [6] Return to lane (drive back the sidestep distance)
        rospy.loginfo("[6/8] Return %.3f m",sd)
        self.drive(sd, obstacles=False, scanning=False, speed=V_RETURN)

        # [7] Turn to row heading
        rospy.loginfo("[7/8] Turn to heading")
        self.rotate(side*half)

        # [8] TANK TRACK SPECIAL: perpendicular correction to exact lane centre
        rospy.loginfo("[8/8] Lane correction")
        self._return_to_lane(lane_index)

        self._obs_clear()
        rospy.logwarn("=== AVOIDANCE COMPLETE ===")

    # ═══════════════════════════════════════════════════════════════════════════
    # U-TURN
    # ═══════════════════════════════════════════════════════════════════════════
    def _uturn(self, sign, next_li):
        a=sign*(math.pi/2.0)
        self.rotate(a)
        self.drive(self.lane_spacing,obstacles=False,scanning=False,speed=V_AVOID)
        self.rotate(a)
        nh=self._row_heading(next_li);nc=self._row_cross(next_li)
        self.snap_heading(nh)
        self.drive(RELOCALIZE_FWD_DIST,obstacles=False,scanning=False,th=nh,tc=nc)
        # Verify lane after U-turn
        self._return_to_lane(next_li)

    # ═══════════════════════════════════════════════════════════════════════════
    # ROW DRIVER
    # ═══════════════════════════════════════════════════════════════════════════
    def drive_row(self, length, li=0):
        th=self._row_heading(li);tc=self._row_cross(li)
        stalls=0;margin=max(0.40,LIDAR_TO_FRONT+LIDAR_TO_REAR)
        allowed=+1 if li%2==0 else -1
        rx,ry,rh=self.map_pose()

        rospy.loginfo("Row %d: heading=%.0f deg cross=%.3f m side=%s",
                      li,math.degrees(th),tc,"LEFT" if allowed>0 else "RIGHT")
        self.snap_heading(th)

        if self.front()<=self.zone_danger:
            rospy.logwarn("Row %d blocked — reverse",li)
            self.reverse(0.30);rospy.sleep(0.3)
            if self.front()<=self.zone_danger:
                rospy.logwarn("Row %d still blocked — skip",li);return

        covered=0.0
        while not rospy.is_shutdown() and covered<length-DIST_TOL:
            remaining=length-covered
            ok=self.drive(remaining,obstacles=True,scanning=True,th=th,tc=tc,map_prog=True)

            x1,y1,_=self.map_pose()
            dx=x1-rx;dy=y1-ry
            along=dx*math.cos(rh)+dy*math.sin(rh)
            covered=max(covered,clamp(along,0.,length))

            if ok: break
            if length-covered<margin:
                rospy.logwarn("Row %d near end — accept",li);break
            stalls+=1
            if stalls>MAX_STALLS:
                rospy.logwarn("Row %d too many stalls",li);break

            self.avoid(lane_index=li,allowed_side=allowed)
            self.snap_heading(th)

            # After avoidance: longer settle drive with cross-track correction
            self.drive(RELOCALIZE_FWD_DIST*3,obstacles=True,scanning=True,
                       th=th,tc=tc,speed=V_AVOID)

            x2,y2,_=self.map_pose()
            dx2=x2-rx;dy2=y2-ry
            along2=dx2*math.cos(rh)+dy2*math.sin(rh)
            covered=max(covered,clamp(along2,0.,length))
            rospy.loginfo("Row %d: after recovery %.2f / %.2f m",li,covered,length)

        rospy.loginfo("Row %d done (%.2f / %.2f m)",li,covered,length)

    # ═══════════════════════════════════════════════════════════════════════════
    # STARTUP
    # ═══════════════════════════════════════════════════════════════════════════
    def wait_topics(self,t=30.):
        rospy.loginfo("Waiting for /odom /imu/data /scan ...")
        t0=rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.odom and self.imu and self.scan:
                rospy.loginfo("Topics OK.");return
            if rospy.Time.now().to_sec()-t0>t: raise RuntimeError("No sensors!")
            rospy.sleep(0.1)

    def wait_map(self,t=15.):
        rospy.loginfo("Waiting for /map TF ...")
        t0=rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform("/map","/base_footprint",rospy.Time(0),rospy.Duration(1.))
                rospy.loginfo("Map OK.");return
            except: pass
            if rospy.Time.now().to_sec()-t0>t:
                rospy.logwarn("No map — odom fallback.");return
            rospy.sleep(0.5)

    def cal_gyro(self,d=2.):
        rospy.loginfo("Gyro cal %.0f s — DO NOT MOVE",d)
        s=[];t0=rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec()-t0<d:
            if self.imu: s.append(self.imu.angular_velocity.z)
            rospy.sleep(0.02)
        if s: self.gyro_bias=sum(s)/float(len(s));rospy.loginfo("Bias=%.6f (%d)",self.gyro_bias,len(s))

    def prompt(self):
        print("\n"+"="*60)
        print("  Transbot Coverage v20 — TANK TRACK EDITION")
        print("  Obstacle: v3 8-step + 3-phase + lane return")
        print("  Arm: FIXED J7=%.0f J8=%.0f" % (ARM_J7,ARM_J8))
        print("  Stop=%.2f m  Slow=%.2f m" % (self.zone_danger,self.zone_slow))
        print("="*60)
        print("BOTTOM-LEFT corner. FORWARD=Row 0. LEFT=lane advance.\n")
        self.length_m=float(raw_input("LENGTH (m): "))
        self.width_m=float(raw_input("WIDTH  (m): "))
        s=raw_input("Lane [%.3f m]: " % SPOOL_WIDTH).strip()
        self.lane_spacing=float(s) if s else SPOOL_WIDTH
        n=int(math.ceil(self.width_m/self.lane_spacing))
        print("\n  %d rows x %.2f m" % (n,self.length_m))
        raw_input("\nENTER to START ...")

    # ═══════════════════════════════════════════════════════════════════════════
    # MAIN
    # ═══════════════════════════════════════════════════════════════════════════
    def run(self):
        self.wait_topics()
        self.wait_map()
        self.prompt()
        self.cal_gyro(2.0)
        self._set_arm()
        rospy.loginfo("Starting in 3 s ...")
        rospy.sleep(3.0)
        self._set_origin()

        n=int(math.ceil(self.width_m/self.lane_spacing))
        ts=+1
        rospy.loginfo("Mission: %d rows x %.2f m (lane=%.3f)",n,self.length_m,self.lane_spacing)

        for i in xrange(n):
            if rospy.is_shutdown(): break
            rospy.loginfo("=== Row %d / %d ===",i+1,n)
            self.drive_row(self.length_m,li=i)
            if i==n-1: rospy.loginfo("Last row.");break
            self._uturn(ts,next_li=i+1)
            ts*=-1

        self.stop()
        rospy.loginfo("DONE. Metal: %d",len(self._mpos))
        for i,(x,y) in enumerate(self._mpos): rospy.loginfo("  #%d x=%.3f y=%.3f",i+1,x,y)

if __name__=="__main__":
    try: CoverageNode().run()
    except rospy.ROSInterruptException: pass
    except RuntimeError as e: rospy.logerr("Fatal: %s",e)
