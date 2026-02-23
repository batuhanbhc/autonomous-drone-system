from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from mavros_msgs.msg import StatusText
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import GPSRAW

from pymavlink import mavutil
from tf_transformations import euler_from_quaternion
from collections import deque
from dataclasses import dataclass
import math

from mavros_gcs.panel_utils.helpers import (
    _radians_to_degree,
    _wrap_degrees
)


@dataclass
class BatteryView:
    voltage     : float | None = None
    percentage  : float | None = None 
    current     : float | None = None

    def update_from_msg(self, msg: BatteryState):
        self.voltage = float(msg.voltage)
        self.percentage = float(msg.percentage)
        self.current = float(msg.current)



@dataclass
class StateView:
    connected   : bool | None = None
    armed       : bool | None = None
    guided      : bool | None = None
    mode        : str | None = None
    sys_status  : str | None = None

    def update_from_msg(self, msg: State):
        self.connected = bool(msg.connected)
        self.armed = bool(msg.armed)
        self.guided = bool(msg.guided)
        self.mode = str(msg.mode)
        self.sys_status = mavutil.mavlink.enums['MAV_STATE'][int(msg.system_status)].name



@dataclass
class ExtendedStateView:
    vtol_state      : int | None = None
    landed_state    : int | None = None

    def __post_init__(self):
        self.vtol_map = {
            ExtendedState.VTOL_STATE_UNDEFINED: "UNDEFINED",
            ExtendedState.VTOL_STATE_TRANSITION_TO_FW: "MULTICOPTER_TO_FIXED-WING",
            ExtendedState.VTOL_STATE_TRANSITION_TO_MC: "FIXED-WING_TO_MULTICOPTER",
            ExtendedState.VTOL_STATE_MC: "MULTICOPTER",
            ExtendedState.VTOL_STATE_FW: "FIXED-WING",
        }
        self.landed_map = {
            ExtendedState.LANDED_STATE_UNDEFINED: "UNDEFINED",
            ExtendedState.LANDED_STATE_ON_GROUND: "ON_GROUND",
            ExtendedState.LANDED_STATE_IN_AIR: "IN_AIR",
            ExtendedState.LANDED_STATE_TAKEOFF: "TAKEOFF",
            ExtendedState.LANDED_STATE_LANDING: "LANDING",
        }

    def update_from_msg(self, msg: ExtendedState):
        self.vtol_state = int(msg.vtol_state)
        self.landed_state = int(msg.landed_state)

    def vtol_state_name(self) -> str | None:
        if self.vtol_state is None:
            return None
        return self.vtol_map.get(self.vtol_state, f"UNKNOWN({self.vtol_state})")

    def landed_state_name(self) -> str | None:
        if self.landed_state is None:
            return None
        return self.landed_map.get(self.landed_state, f"UNKNOWN({self.landed_state})")
    


@dataclass
class VelocityView:
    vx          : float | None = None
    vy          : float | None = None
    vz          : float | None = None
    frame_id    : str | None = None

    def update_from_msg(self, msg: TwistStamped):
        v = msg.twist.linear
        self.vx = float(v.x)
        self.vy = float(v.y)
        self.vz = float(v.z)
        self.frame_id = msg.header.frame_id



@dataclass
class OdometryView:
    # position
    x: float | None = None
    y: float | None = None
    z: float | None = None

    # x-y distance to local origin
    dist_lo : float | None = None

    # orientation
    yaw     : float | None = None
    roll    : float | None = None
    pitch   : float | None = None

    frame_id        : str | None = None
    child_frame_id  : str | None = None

    def update_from_msg(self, msg: Odometry):
        p = msg.pose.pose.position
        self.x = float(p.x)
        self.y = float(p.y)
        self.z = float(p.z)

        # use euclidean distance for x-y distance to origin
        self.dist_lo = math.sqrt(self.x**2 + self.y**2)

        # orientation
        q = msg.pose.pose.orientation
        roll, pitch, yaw_enu = euler_from_quaternion([q.x, q.y, q.z, q.w])

        heading_from_north = math.pi/2 - yaw_enu

        self.roll = _radians_to_degree(roll)
        self.pitch = _radians_to_degree(pitch)
        self.yaw = _wrap_degrees(_radians_to_degree(heading_from_north))

        # references
        self.frame_id = msg.header.frame_id
        self.child_frame_id = msg.child_frame_id



@dataclass
class StatusTextView:
    queue_size: int | None = None

    def __post_init__(self):
        # last N messages; when full, oldest is dropped automatically
        self.history = deque(maxlen=self.queue_size)

        self.severity_map = {
            StatusText.EMERGENCY: "[bold red]EMERGENCY[/bold red]",
            StatusText.ALERT: "[bold red]ALERT[/bold red]",
            StatusText.CRITICAL: "[bold red]CRITICAL[/bold red]",
            StatusText.ERROR: "[red]ERROR[/red]",
            StatusText.WARNING: "[yellow]WARNING[/yellow]",
            StatusText.NOTICE: "[cyan]NOTICE[/cyan]",
            StatusText.INFO: "[green]INFO[/green]",
            StatusText.DEBUG: "[dim]DEBUG[/dim]",
        }

    def update_from_msg(self, m: StatusText):
        sec = int(m.header.stamp.sec)
        nanosec = int(m.header.stamp.nanosec)
        sev_int = int(m.severity)
        sev_str = self.severity_map.get(sev_int, f"UNKNOWN({sev_int})")
        text = str(m.text)

        self.history.append({
            "sec": sec,
            "nanosec": nanosec,
            "severity_int": sev_int,
            "severity_str": sev_str,
            "text": text,
        })


from dataclasses import dataclass

@dataclass
class GPSView:
    fix_type: int | None = None
    lat_deg: float | None = None
    lon_deg: float | None = None

    satellites_visible: int | None = None

    # quality
    eph: int | None = None   # HDOP (raw uint16)
    epv: int | None = None   # VDOP (raw uint16)

    # motion
    vel_mps: float | None = None
    cog_deg: float | None = None

    # v2.0 fields (may be 0 / unset depending on source)
    h_acc_m: float | None = None
    v_acc_m: float | None = None

    def __post_init__(self):
        self.fix_map = {
            GPSRAW.GPS_FIX_TYPE_NO_GPS:    "NO_GPS",
            GPSRAW.GPS_FIX_TYPE_NO_FIX:    "NO_FIX",
            GPSRAW.GPS_FIX_TYPE_2D_FIX:    "2D",
            GPSRAW.GPS_FIX_TYPE_3D_FIX:    "3D",
            GPSRAW.GPS_FIX_TYPE_DGPS:      "DGPS",
            GPSRAW.GPS_FIX_TYPE_RTK_FLOAT: "RTK_FLOAT",
            GPSRAW.GPS_FIX_TYPE_RTK_FIXED: "RTK_FIXED",
            GPSRAW.GPS_FIX_TYPE_STATIC:    "STATIC",
            GPSRAW.GPS_FIX_TYPE_PPP:       "PPP",
        }

    def fix_type_name(self) -> str | None:
        if self.fix_type is None:
            return None
        return self.fix_map.get(self.fix_type, f"UNKNOWN({self.fix_type})")

    def update_from_msg(self, msg: GPSRAW):
        self.fix_type = int(msg.fix_type)

        # degE7 → degrees
        self.lat_deg = float(msg.lat) / 1e7 if msg.lat != 0 else None
        self.lon_deg = float(msg.lon) / 1e7 if msg.lon != 0 else None

        self.satellites_visible = int(msg.satellites_visible) if msg.satellites_visible != 255 else None

        # Unknown sentinels: UINT16_MAX
        self.eph = None if msg.eph == 65535 else int(msg.eph)
        self.epv = None if msg.epv == 65535 else int(msg.epv)

        # vel: cm/s → m/s, unknown sentinel UINT16_MAX
        self.vel_mps = None if msg.vel == 65535 else float(msg.vel) / 100.0

        # cog: cdeg → deg, unknown sentinel UINT16_MAX
        self.cog_deg = None if msg.cog == 65535 else float(msg.cog) / 100.0

        # v2 accuracy fields (mm → m). Some stacks may publish 0 if unavailable.
        self.h_acc_m = float(msg.h_acc) / 1000.0 if getattr(msg, "h_acc", 0) not in (0, None) else None
        self.v_acc_m = float(msg.v_acc) / 1000.0 if getattr(msg, "v_acc", 0) not in (0, None) else None