#!/usr/bin/env python3
from __future__ import annotations
import threading 
import time
import os
import yaml

# rclpy imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

# message type imports
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from mavros_msgs.msg import StatusText
from nav_msgs.msg import Odometry
from mavros_msgs.msg import GPSRAW
from drone_msgs.msg import DroneState
from drone_msgs.msg import DroneInfo

from ament_index_python.packages import get_package_share_directory

# console ui imports
from rich import box
from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.columns import Columns
from rich.text import Text

from mavros_gcs.panel_utils.views import (
    StateView,
    BatteryView,
    ExtendedStateView,
    StatusTextView,
    OdometryView,
    GPSView,
    DroneStateView,
    DroneInfoView,
)

from mavros_gcs.panel_utils.helpers import (
    _format_bool_to_str,
    _format_float_to_str,
    _format_str_to_str,
    _stamp_to_clock_str
)


def load_yaml_from_pkg(pkg: str, rel: str):
    """
    Loads a YAML file from a ROS2 package share directory and returns the FULL root dict.
    """
    yaml_path = os.path.join(get_package_share_directory(pkg), rel)
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a dict: {yaml_path}")

    return data


class InfoPanelNode(Node):
    def __init__(self):
        super().__init__("info_panel")
        
        # --------------------------------------------------
        # node parameters
        self.declare_parameter("drone_id", 0)
        drone_id = int(self.get_parameter("drone_id").value)

        # ... after reading YAML topic_command/topic_action ...
        base = f"/drone_{drone_id}"

        # ------------------------
        # read yaml config file
        self.declare_parameter("config_pkg", "mavros_config")
        self.declare_parameter("config_rel", "config/control_params.yaml")
        config_pkg = self.get_parameter("config_pkg").value
        config_rel = self.get_parameter("config_rel").value
        root_cfg = load_yaml_from_pkg(config_pkg, config_rel)

        panel_cfg = root_cfg.get("panel", {})
        mavros_topics = root_cfg.get("mavros_topics", {})

        if not isinstance(panel_cfg, dict):
            raise RuntimeError("YAML key 'panel' must be a dict")
        if not isinstance(mavros_topics, dict):
            raise RuntimeError("YAML key 'mavros_topics' must be a dict")
        
        # ------------------------
        # panel setting
        self.queue_size = int(panel_cfg.get("queue_size", 10))
        self.stale_threshold_s = float(panel_cfg.get("stale_threshold_s", 5.0))
        self.refresh_hz = float(panel_cfg.get("refresh_hz", 4.0))

        # MAVROS topics
        t_state = base + mavros_topics["state"]
        t_battery = base + mavros_topics["battery"]
        t_extended_state = base + mavros_topics["extended_state"]
        t_statustext = base + mavros_topics["statustext"]
        t_odom = base + mavros_topics["odom"]
        t_gps1_raw = base + mavros_topics["gps1_raw"]
        
        # control_gate topics
        t_control_state = f"/drone_{drone_id}/cmd_gate/state"
        t_drone_info = f"/drone_{drone_id}/cmd_gate/info"

        # --------------------------------------------------
        self._ui_lock = threading.Lock()

        # cached views
        self.battery = BatteryView()
        self.state = StateView()
        self.ext_state = ExtendedStateView()
        self.statustext = StatusTextView(queue_size=self.queue_size)
        self.odom = OdometryView()
        self.gps1 = GPSView()
        self.control_state = DroneStateView()
        self.drone_info = DroneInfoView(queue_size=self.queue_size)

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # subscriptions
        self.create_subscription(ExtendedState, t_extended_state, self._on_extended_state, reliable_qos)
        self.create_subscription(StatusText, t_statustext, self._on_statustext, qos_profile_sensor_data)
        self.create_subscription(BatteryState, t_battery, self._on_battery, qos_profile_sensor_data)
        self.create_subscription(State, t_state, self._on_state, reliable_qos)
        self.create_subscription(Odometry, t_odom, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(GPSRAW, t_gps1_raw, self._on_gps1_raw, qos_profile_sensor_data)
        self.create_subscription(DroneState, t_control_state, self._on_control_state, qos_profile_sensor_data)
        self.create_subscription(DroneInfo, t_drone_info, self._on_drone_info, qos_latched)

        # UI refresh rate
        period = 1.0 / float(self.refresh_hz)
        self.create_timer(period, self._on_refresh_tick)

        # rich live UI
        self._console = Console()
        self._live = None

        # registry of section render functions
        self._sections = {
            "State": self._render_state_panel,
            "Extended State": self._render_extended_state_panel,
            "Battery": self._render_battery_panel,
            "Odometry": self._render_odom_panel,
            "StatusText": self._render_statustext_panel,
            "GPS1": self._render_gps1_panel,
            "Control Gate": self._render_control_state_panel,
            "Drone Info": self._render_drone_info_panel,
        }

        # boolean dict to mark sections as dirty when they get an update
        self._dirty = {name: True for name in self._sections.keys()}

        # panel object dict for storing panels in cache
        self._cached_panels = {}

        # dict to keep when the panel last got an update
        self._last_update = {name: None for name in self._sections.keys()}

        # boolean dict to mark a panel as stale
        self._is_stale = {name: True for name in self._sections.keys()}

    def _mark_dirty(self, section):
        self._dirty[section] = True
        self._last_update[section] = time.monotonic()

    # ------------------- callbacks -------------------

    def _on_battery(self, msg: BatteryState):
        with self._ui_lock:
            self.battery.update_from_msg(msg)
            self._mark_dirty("Battery")

    def _on_state(self, msg: State):
        with self._ui_lock:
            self.state.update_from_msg(msg)
            self._mark_dirty("State")

    def _on_extended_state(self, msg: ExtendedState):
        with self._ui_lock:
            self.ext_state.update_from_msg(msg)
            self._mark_dirty("Extended State")

    def _on_odom(self, msg: Odometry):
        with self._ui_lock:
            self.odom.update_from_msg(msg)
            self._mark_dirty("Odometry")

    def _on_statustext(self, msg: StatusText):
        with self._ui_lock:
            self.statustext.update_from_msg(msg)
            self._mark_dirty("StatusText")

    def _on_gps1_raw(self, msg: GPSRAW):
        with self._ui_lock:
            self.gps1.update_from_msg(msg)
            self._mark_dirty("GPS1")
    
    def _on_control_state(self, msg: DroneState):
        with self._ui_lock:
            self.control_state.update_from_msg(msg)
            self._mark_dirty("Control Gate")
    
    def _on_drone_info(self, msg: DroneInfo):
        with self._ui_lock:
            self.drone_info.update_from_msg(msg)
            self._mark_dirty("Drone Info")


    # ---- Snapshot Functions (for copying) ----
    def _snapshot_battery(self):
        return {
            "voltage": self.battery.voltage,
            "current": self.battery.current,
            "percentage": self.battery.percentage,
        }

    def _snapshot_state(self):
        return {
            "connected": self.state.connected,
            "armed": self.state.armed,
            "guided": self.state.guided,
            "mode": self.state.mode,
            "sys_status": self.state.sys_status,
        }

    def _snapshot_ext_state(self):
        return {
            "vtol": self.ext_state.vtol_state_name(),
            "landed": self.ext_state.landed_state_name(),
        }

    def _snapshot_odom(self):
        return {
            "x": self.odom.x, "y": self.odom.y, "z": self.odom.z,
            "vx": self.odom.vx, "vy": self.odom.vy, "vz": self.odom.vz,
            "dist_lo": self.odom.dist_lo,
            "yaw": self.odom.yaw, "roll": self.odom.roll, "pitch": self.odom.pitch,
        }

    def _snapshot_statustext(self):
        return [dict(item) for item in self.statustext.history]

    def _snapshot_gps1(self):
        return {
            "fix": self.gps1.fix_type_name(),
            "lat": self.gps1.lat_deg,
            "lon": self.gps1.lon_deg,
            "sats": self.gps1.satellites_visible,
            "eph": self.gps1.eph,
            "epv": self.gps1.epv,
            "vel": self.gps1.vel_mps,
            "cog": self.gps1.cog_deg,
            "hacc": self.gps1.h_acc_m,
            "vacc": self.gps1.v_acc_m,
        }
    
    def _snapshot_control_state(self):
        return {
            "control_mode": self.control_state.control_mode,
            "velocity_h": self.control_state.velocity_h,
            "velocity_v": self.control_state.velocity_v,
            "keyboard_on": self.control_state.keyboard_on,
            "safety_switch_on": self.control_state.safety_switch_on,
            "system_killed": self.control_state.system_killed,
            "time_since_action_s": self.control_state.time_since_action_s,
        }

    def _snapshot_drone_info(self):
        return [dict(item) for item in self.drone_info.history]
    
    # ------------------- rendering -------------------
    def _render_state_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
            expand=True,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        t.add_row("Connected", _format_bool_to_str(data["connected"]))
        t.add_row("Armed", _format_bool_to_str(data["armed"]))
        t.add_row("Guided", _format_bool_to_str(data["guided"]))
        t.add_row("Mode", _format_str_to_str(data["mode"]))
        t.add_row("System Status", _format_str_to_str(data["sys_status"]))

        return Panel(t, title="State", border_style="magenta")
    
    def _render_extended_state_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
            expand=True,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        t.add_row("VTOL State", _format_str_to_str(data["vtol"]))
        t.add_row("Landed State", _format_str_to_str(data["landed"]))

        return Panel(t, title="Extended State", border_style="blue")
    
    def _render_battery_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
            expand=True,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        t.add_row("Voltage (V)", _format_float_to_str(data["voltage"], 2))
        t.add_row("Current (A)", _format_float_to_str(data["current"], 2))
        t.add_row("Percentage (%)", _format_float_to_str(data["percentage"], 1, multiplier=100))

        return Panel(t, title="Battery", border_style="green")

    def _render_odom_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        t.add_row("Position")
        t.add_row("X (m)", _format_float_to_str(data["x"], 3))
        t.add_row("Y (m)", _format_float_to_str(data["y"], 3))
        t.add_row("Z (m)", _format_float_to_str(data["z"], 3), end_section=True)

        t.add_row("Velocity (Body)")
        t.add_row("X (m/s)", _format_float_to_str(data["vx"], 3))
        t.add_row("Y (m/s)", _format_float_to_str(data["vy"], 3))
        t.add_row("Z (m/s)", _format_float_to_str(data["vz"], 3), end_section=True)

        t.add_row("Local Origin Distance (m)", _format_float_to_str(data["dist_lo"]), end_section=True)

        t.add_row("Orientation (earth-fixed)")
        t.add_row("Yaw (deg)", _format_float_to_str(data["yaw"], 3))
        t.add_row("Roll (deg)", _format_float_to_str(data["roll"], 3))
        t.add_row("Pitch (deg)", _format_float_to_str(data["pitch"], 3), end_section=True)

        return Panel(t, title="Odometry", border_style="yellow")
        
    def _render_statustext_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=True,
            show_edge=False,
            pad_edge=False,
            expand=True,
        )
        t.add_column("Time", no_wrap=True)
        t.add_column("Severity", no_wrap=True)
        t.add_column("Text", overflow="fold")

        if not data:
            t.add_row("[dim]—[/dim]", "[dim]—[/dim]", "[dim]No message yet[/dim]")
        else:
            for item in reversed(data):
                ts = _stamp_to_clock_str(item["sec"], item["nanosec"])
                t.add_row(ts, item["severity_str"], item["text"])
        return Panel(t, title=f"Status Text (last {self.queue_size})", border_style="blue", expand=True)
        
    def _render_gps1_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
            expand=True,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        t.add_row("Fix", _format_str_to_str(data["fix"]))
        t.add_row("Sats", _format_str_to_str(str(data["sats"]) if data["sats"] is not None else None))

        # lat/lon with nice formatting
        lat = None if data["lat"] is None else f"{data['lat']:.7f}"
        lon = None if data["lon"] is None else f"{data['lon']:.7f}"
        t.add_row("Lat", _format_str_to_str(lat))
        t.add_row("Lon", _format_str_to_str(lon))

        t.add_row("Speed (m/s)", _format_float_to_str(data["vel"], 2))
        t.add_row("COG (deg)", _format_float_to_str(data["cog"], 1))

        # DOPs / accuracies
        t.add_row("HDOP", _format_float_to_str(data["eph"], 2, multiplier=0.01))
        t.add_row("VDOP", _format_float_to_str(data["epv"], 2, multiplier=0.01)) 
        t.add_row("hAcc (m)", _format_float_to_str(data["hacc"], 2))
        t.add_row("vAcc (m)", _format_float_to_str(data["vacc"], 2))

        return Panel(t, title="GPS1", border_style="cyan")

    def _render_control_state_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
            expand=True,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        # map enum -> text
        mode = None
        if data["control_mode"] is not None:
            if data["control_mode"] == 0:
                mode = "AUTO"
            elif data["control_mode"] == 1:
                mode = "MANUAL"
            else:
                mode = f"UNKNOWN({data['control_mode']})"

        t.add_row("Control Mode", _format_str_to_str(mode))
        t.add_row("Vel H (m/s)", _format_float_to_str(data["velocity_h"], 2))
        t.add_row("Vel V (m/s)", _format_float_to_str(data["velocity_v"], 2))
        t.add_row("Keyboard", _format_bool_to_str(data["keyboard_on"]))
        t.add_row("Safe", _format_bool_to_str(data["safety_switch_on"]))
        t.add_row("System Killed", _format_bool_to_str(data["system_killed"]))

        tsa = data["time_since_action_s"]
        tsa_str = None if tsa is None else f"{tsa:.2f}"
        t.add_row("Since Action (s)", _format_str_to_str(tsa_str))

        return Panel(t, title="Control Gate", border_style="red")
    
    def _render_drone_info_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=True,
            show_edge=False,
            pad_edge=False,
            expand=True,
        )
        t.add_column("Time", no_wrap=True)
        t.add_column("Level", no_wrap=True)
        t.add_column("Text", overflow="fold")

        if not data:
            t.add_row("[dim]—[/dim]", "[dim]—[/dim]", "[dim]No message yet[/dim]")
        else:
            for item in reversed(data):
                ts = _stamp_to_clock_str(item["sec"], item["nanosec"])
                t.add_row(ts, item["level_str"], item["text"])

        return Panel(t, title=f"Drone Info (last {self.queue_size})", border_style="red", expand=True)
    
    # ---- build console ui layout ----

    def _build_layout_from_snapshot(self, snap, dirty):
        header = Text("mavros_info — terminal information panel (Ctrl+C to exit)", style="bold")
        panels = []

        def get(name, build_fn):
            if dirty.get(name, True) or name not in self._cached_panels:
                self._cached_panels[name] = build_fn()

            p = self._cached_panels[name]

            # stale overlay
            if snap.get("_stale", {}).get(name, False):
                return Panel(
                    p.renderable,
                    title=f"{p.title} [STALE]",
                    border_style="bright_red",
                    expand=p.expand,
                )

            return p

        panels.append(get("Odometry", lambda: self._render_odom_panel(snap["Odometry"])))
        panels.append(get("State", lambda: self._render_state_panel(snap["State"])))
        panels.append(get("Extended State", lambda: self._render_extended_state_panel(snap["Extended State"])))
        panels.append(get("Control Gate", lambda: self._render_control_state_panel(snap["Control Gate"])))
        panels.append(get("Battery", lambda: self._render_battery_panel(snap["Battery"])))
        panels.append(get("StatusText", lambda: self._render_statustext_panel(snap["StatusText"])))
        panels.append(get("Drone Info", lambda: self._render_drone_info_panel(snap["Drone Info"])))
        panels.append(get("GPS1", lambda: self._render_gps1_panel(snap["GPS1"])))
        

        body = Columns(panels, equal=False, expand=True, padding=(0, 1))
        return Panel(body, title=header, border_style="green", expand=True)


    # ---- ui timer callback ----
    def _on_refresh_tick(self):
        # take snapshot under lock
        with self._ui_lock:
            # one time flag at the start
            if self._live is None:
                start_live = True
            else:
                start_live = False

            now = time.monotonic()
            new_is_stale = {}
            for name, ts in self._last_update.items():
                new_is_stale[name] = (ts is None) or ((now - ts) > self.stale_threshold_s)

            stale_changed = any(new_is_stale[k] != self._is_stale[k] for k in new_is_stale.keys())
            self._is_stale = new_is_stale

            if not start_live and not any(self._dirty.values()) and not stale_changed:
                return

            dirty = dict(self._dirty)  # which sections changed
            for k in self._dirty.keys():
                self._dirty[k] = False  # clear all dirty flags

            snap = {}
            snap["_stale"] = dict(self._is_stale)
            snap["Battery"] = self._snapshot_battery()
            snap["State"] = self._snapshot_state()
            snap["Extended State"] = self._snapshot_ext_state()
            snap["Odometry"] = self._snapshot_odom()
            snap["StatusText"] = self._snapshot_statustext()
            snap["GPS1"] = self._snapshot_gps1()
            snap["Control Gate"] = self._snapshot_control_state()
            snap["Drone Info"] = self._snapshot_drone_info()

        # render outside lock using 'snap'
        layout = self._build_layout_from_snapshot(snap, dirty)

        # update live
        if start_live:
            self._live = Live(layout, console=self._console, auto_refresh=False)
            self._live.start()
            self._live.refresh()
        else:
            self._live.update(layout, refresh=True)


    def shutdown_ui(self):
        if self._live is not None:
            self._live.stop()
            self._live = None



def main(args=None):
    rclpy.init(args=args)
    node = InfoPanelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_ui()
        node.destroy_node()
        rclpy.try_shutdown()