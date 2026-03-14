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
from drone_msgs.msg import Toggle

from ament_index_python.packages import get_package_share_directory

# console ui imports
from rich import box
from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.layout import Layout
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
    RecordActiveView,
    StreamActiveView,
)

from mavros_gcs.panel_utils.helpers import (
    _format_bool_to_str,
    _format_float_to_str,
    _format_str_to_str,
    _stamp_to_clock_str
)


def load_yaml_from_pkg(pkg: str, rel: str):
    yaml_path = os.path.join(get_package_share_directory(pkg), rel)
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a dict: {yaml_path}")
    return data


class InfoPanelNode(Node):
    def __init__(self):
        super().__init__("info_panel")

        self.declare_parameter("drone_id", 0)
        drone_id = int(self.get_parameter("drone_id").value)
        base = f"/drone_{drone_id}"

        self.declare_parameter("config_pkg", "mavros_config")
        self.declare_parameter("config_rel", "config/control_params.yaml")
        config_pkg = self.get_parameter("config_pkg").value
        config_rel = self.get_parameter("config_rel").value
        root_cfg = load_yaml_from_pkg(config_pkg, config_rel)

        panel_cfg = root_cfg.get("panel", {})
        mavros_topics = root_cfg.get("mavros_topics", {})

        self.queue_size = int(panel_cfg.get("queue_size", 10))
        self.stale_threshold_s = float(panel_cfg.get("stale_threshold_s", 5.0))
        self.refresh_hz = float(panel_cfg.get("refresh_hz", 4.0))

        t_state          = base + mavros_topics["state"]
        t_battery        = base + mavros_topics["battery"]
        t_extended_state = base + mavros_topics["extended_state"]
        t_statustext     = base + mavros_topics["statustext"]
        t_odom           = base + mavros_topics["odom"]
        t_gps1_raw       = base + mavros_topics["gps1_raw"]
        t_control_state  = f"/drone_{drone_id}/cmd_gate/state"
        t_drone_info     = f"/drone_{drone_id}/cmd_gate/info"

        self._ui_lock = threading.Lock()

        self.battery       = BatteryView()
        self.state         = StateView()
        self.ext_state     = ExtendedStateView()
        self.statustext    = StatusTextView(queue_size=self.queue_size)
        self.odom          = OdometryView()
        self.gps1          = GPSView()
        self.control_state = DroneStateView()
        self.drone_info    = DroneInfoView(queue_size=self.queue_size)
        self.record_active = RecordActiveView()
        self.stream_active = StreamActiveView()

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(ExtendedState, t_extended_state, self._on_extended_state, reliable_qos)
        self.create_subscription(StatusText,     t_statustext,     self._on_statustext,     qos_profile_sensor_data)
        self.create_subscription(BatteryState,   t_battery,        self._on_battery,        qos_profile_sensor_data)
        self.create_subscription(State,          t_state,          self._on_state,           reliable_qos)
        self.create_subscription(Odometry,       t_odom,           self._on_odom,            qos_profile_sensor_data)
        self.create_subscription(GPSRAW,         t_gps1_raw,       self._on_gps1_raw,        qos_profile_sensor_data)
        self.create_subscription(DroneState,     t_control_state,  self._on_control_state,   qos_profile_sensor_data)
        self.create_subscription(DroneInfo,      t_drone_info,     self._on_drone_info,       qos_latched)
        self.create_subscription(
            Toggle,
            f"/drone_{drone_id}/camera/record/active",
            self._on_record_active,
            reliable_qos,
        )
        self.create_subscription(
            Toggle,
            f"/drone_{drone_id}/camera/stream/active",
            self._on_stream_active,
            reliable_qos,
        )
        
        period = 1.0 / float(self.refresh_hz)
        self.create_timer(period, self._on_refresh_tick)

        self._console = Console()
        self._live    = None

        self._section_names = [
            "State", "Extended State", "Battery", "Odometry",
            "StatusText", "GPS1", "Control Gate", "Drone Info",
            "Drone Pipeline",
        ]
        self._dirty       = {n: True  for n in self._section_names}
        self._last_update = {n: None  for n in self._section_names}
        self._is_stale    = {n: True  for n in self._section_names}

        # Build a fixed Layout tree once; we only swap renderable content each tick.
        # Grid structure:
        #
        #  ┌─────────────────────────────────────────────────────┐
        #  │                     header                          │
        #  ├──────────┬──────────┬──────────┬────────────────────┤
        #  │  col_0   │  col_1   │  col_2   │      col_3         │
        #  │  Odom    │  State   │  GPS1    │  StatusText (tall) │
        #  │          │  ExtSt   │  Battery │                    │
        #  │          │  CtrlGt  │  DrInfo  │                    │
        #  └──────────┴──────────┴──────────┴────────────────────┘
        #
        # col_3 is wider (ratio=2) because log panels need more text space.

        self._layout = Layout()
        self._layout.split_column(
            Layout(name="header", size=3),
            Layout(name="body"),
        )
        self._layout["body"].split_row(
            Layout(name="col_0", ratio=3),   # Odometry — tall, needs room
            Layout(name="col_1", ratio=2),   # State / ExtState / CtrlGate
            Layout(name="col_2", ratio=2),   # GPS1 / Battery / DroneInfo
            Layout(name="col_3", ratio=3),   # StatusText + DroneInfo log
        )

        # col_0: just Odometry
        # col_1: State + ExtState + CtrlGate stacked
        self._layout["col_1"].split_column(
            Layout(name="state"),
            Layout(name="ext_state"),
            Layout(name="ctrl_gate"),
        )
        # col_2: GPS1 + Battery + Drone Pipeline stacked
        self._layout["col_2"].split_column(
            Layout(name="gps1"),
            Layout(name="battery"),
            Layout(name="drone_pipeline"),
        )
        # col_3: StatusText + DroneInfo stacked
        self._layout["col_3"].split_column(
            Layout(name="statustext"),
            Layout(name="drone_info"),
        )

    # ------------------------------------------------------------------ #
    #  dirty helpers
    # ------------------------------------------------------------------ #
    def _mark_dirty(self, section):
        self._dirty[section] = True
        self._last_update[section] = time.monotonic()

    # ------------------------------------------------------------------ #
    #  ROS callbacks
    # ------------------------------------------------------------------ #
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

    def _on_record_active(self, msg: Toggle):
        with self._ui_lock:
            self.record_active.update_from_msg(msg)
            self._mark_dirty("Drone Pipeline")

    def _on_stream_active(self, msg: Toggle):
        with self._ui_lock:
            self.stream_active.update_from_msg(msg)
            self._mark_dirty("Drone Pipeline")
            
    # ------------------------------------------------------------------ #
    #  Snapshots
    # ------------------------------------------------------------------ #
    def _snapshot_battery(self):
        return {"voltage": self.battery.voltage, "current": self.battery.current,
                "percentage": self.battery.percentage}

    def _snapshot_state(self):
        return {"connected": self.state.connected, "armed": self.state.armed,
                "guided": self.state.guided, "mode": self.state.mode,
                "sys_status": self.state.sys_status}

    def _snapshot_ext_state(self):
        return {"vtol": self.ext_state.vtol_state_name(),
                "landed": self.ext_state.landed_state_name()}

    def _snapshot_odom(self):
        return {"x": self.odom.x, "y": self.odom.y, "z": self.odom.z,
                "vx": self.odom.vx, "vy": self.odom.vy, "vz": self.odom.vz,
                "dist_lo": self.odom.dist_lo,
                "yaw": self.odom.yaw, "roll": self.odom.roll, "pitch": self.odom.pitch}

    def _snapshot_statustext(self):
        return [dict(item) for item in self.statustext.history]

    def _snapshot_gps1(self):
        return {"fix": self.gps1.fix_type_name(), "lat": self.gps1.lat_deg,
                "lon": self.gps1.lon_deg, "sats": self.gps1.satellites_visible,
                "eph": self.gps1.eph, "epv": self.gps1.epv,
                "vel": self.gps1.vel_mps, "cog": self.gps1.cog_deg,
                "hacc": self.gps1.h_acc_m, "vacc": self.gps1.v_acc_m}

    def _snapshot_control_state(self):
        return {"control_mode": self.control_state.control_mode,
                "velocity_h": self.control_state.velocity_h,
                "velocity_v": self.control_state.velocity_v,
                "keyboard_on": self.control_state.keyboard_on,
                "safety_switch_on": self.control_state.safety_switch_on,
                "system_killed": self.control_state.system_killed,
                "time_since_action_s": self.control_state.time_since_action_s}

    def _snapshot_drone_info(self):
        return [dict(item) for item in self.drone_info.history]

    def _snapshot_drone_pipeline(self):
        return {
            "recording": self.record_active.recording,
            "streaming": self.stream_active.streaming,
        }
    
    # ------------------------------------------------------------------ #
    #  Panel renderers  (unchanged logic, same as before)
    # ------------------------------------------------------------------ #
    @staticmethod
    def _make_kv_table() -> Table:
        t = Table(box=box.SIMPLE, show_header=False, show_edge=False,
                  pad_edge=False, expand=True)
        t.add_column(justify="left")
        t.add_column(justify="right")
        return t

    def _render_state_panel(self, data, stale=False) -> Panel:
        t = self._make_kv_table()
        t.add_row("Connected",     _format_bool_to_str(data["connected"]))
        t.add_row("Armed",         _format_bool_to_str(data["armed"]))
        t.add_row("Guided",        _format_bool_to_str(data["guided"]))
        t.add_row("Mode",          _format_str_to_str(data["mode"]))
        t.add_row("System Status", _format_str_to_str(data["sys_status"]))
        title  = "State [STALE]" if stale else "State"
        border = "bright_red"   if stale else "magenta"
        return Panel(t, title=title, border_style=border)

    def _render_extended_state_panel(self, data, stale=False) -> Panel:
        t = self._make_kv_table()
        t.add_row("VTOL State",   _format_str_to_str(data["vtol"]))
        t.add_row("Landed State", _format_str_to_str(data["landed"]))
        title  = "Extended State [STALE]" if stale else "Extended State"
        border = "bright_red" if stale else "blue"
        return Panel(t, title=title, border_style=border)

    def _render_battery_panel(self, data, stale=False) -> Panel:
        t = self._make_kv_table()
        t.add_row("Voltage (V)",     _format_float_to_str(data["voltage"],    2))
        t.add_row("Current (A)",     _format_float_to_str(data["current"],    2))
        t.add_row("Percentage (%)",  _format_float_to_str(data["percentage"], 1, multiplier=100))
        title  = "Battery [STALE]" if stale else "Battery"
        border = "bright_red" if stale else "green"
        return Panel(t, title=title, border_style=border)

    def _render_odom_panel(self, data, stale=False) -> Panel:
        t = Table(box=box.SIMPLE, show_header=False, show_edge=False,
                  pad_edge=False, expand=True)
        t.add_column(justify="left")
        t.add_column(justify="right")
        t.add_row("[bold]Position[/bold]")
        t.add_row("X (m)", _format_float_to_str(data["x"], 3))
        t.add_row("Y (m)", _format_float_to_str(data["y"], 3))
        t.add_row("Z (m)", _format_float_to_str(data["z"], 3), end_section=True)
        t.add_row("[bold]Velocity (Body)[/bold]")
        t.add_row("X (m/s)", _format_float_to_str(data["vx"], 3))
        t.add_row("Y (m/s)", _format_float_to_str(data["vy"], 3))
        t.add_row("Z (m/s)", _format_float_to_str(data["vz"], 3), end_section=True)
        t.add_row("Local Origin Dist (m)", _format_float_to_str(data["dist_lo"]), end_section=True)
        t.add_row("[bold]Orientation (earth-fixed)[/bold]")
        t.add_row("Yaw (deg)",   _format_float_to_str(data["yaw"],   3))
        t.add_row("Roll (deg)",  _format_float_to_str(data["roll"],  3))
        t.add_row("Pitch (deg)", _format_float_to_str(data["pitch"], 3))
        title  = "Odometry [STALE]" if stale else "Odometry"
        border = "bright_red" if stale else "yellow"
        return Panel(t, title=title, border_style=border)

    def _render_statustext_panel(self, data, stale=False) -> Panel:
        t = Table(box=box.SIMPLE, show_header=True, show_edge=False,
                  pad_edge=False, expand=True)
        t.add_column("Time",     no_wrap=True)
        t.add_column("Severity", no_wrap=True)
        t.add_column("Text",     overflow="fold")
        if not data:
            t.add_row("[dim]—[/dim]", "[dim]—[/dim]", "[dim]No message yet[/dim]")
        else:
            for item in reversed(data):
                ts = _stamp_to_clock_str(item["sec"], item["nanosec"])
                t.add_row(ts, item["severity_str"], item["text"])
        title  = f"Status Text (last {self.queue_size}) [STALE]" if stale else f"Status Text (last {self.queue_size})"
        border = "bright_red" if stale else "blue"
        return Panel(t, title=title, border_style=border, expand=True)

    def _render_gps1_panel(self, data, stale=False) -> Panel:
        t = self._make_kv_table()
        t.add_row("Fix",       _format_str_to_str(data["fix"]))
        t.add_row("Sats",      _format_str_to_str(str(data["sats"]) if data["sats"] is not None else None))
        lat = None if data["lat"] is None else f"{data['lat']:.7f}"
        lon = None if data["lon"] is None else f"{data['lon']:.7f}"
        t.add_row("Lat",        _format_str_to_str(lat))
        t.add_row("Lon",        _format_str_to_str(lon))
        t.add_row("Speed (m/s)", _format_float_to_str(data["vel"],  2))
        t.add_row("COG (deg)",   _format_float_to_str(data["cog"],  1))
        t.add_row("HDOP",        _format_float_to_str(data["eph"],  2, multiplier=0.01))
        t.add_row("VDOP",        _format_float_to_str(data["epv"],  2, multiplier=0.01))
        t.add_row("hAcc (m)",    _format_float_to_str(data["hacc"], 2))
        t.add_row("vAcc (m)",    _format_float_to_str(data["vacc"], 2))
        title  = "GPS1 [STALE]" if stale else "GPS1"
        border = "bright_red"   if stale else "cyan"
        return Panel(t, title=title, border_style=border)

    def _render_control_state_panel(self, data, stale=False) -> Panel:
        t = self._make_kv_table()
        mode = None
        if data["control_mode"] is not None:
            mode = {0: "AUTO", 1: "MANUAL"}.get(data["control_mode"],
                                                  f"UNKNOWN({data['control_mode']})")
        t.add_row("Control Mode",    _format_str_to_str(mode))
        t.add_row("Vel H (m/s)",     _format_float_to_str(data["velocity_h"], 2))
        t.add_row("Vel V (m/s)",     _format_float_to_str(data["velocity_v"], 2))
        t.add_row("Keyboard",        _format_bool_to_str(data["keyboard_on"]))
        t.add_row("Safe",            _format_bool_to_str(data["safety_switch_on"]))
        t.add_row("System Killed",   _format_bool_to_str(data["system_killed"]))
        tsa = data["time_since_action_s"]
        t.add_row("Since Action (s)", _format_str_to_str(None if tsa is None else f"{tsa:.2f}"))
        title  = "Control Gate [STALE]" if stale else "Control Gate"
        border = "bright_red"           if stale else "red"
        return Panel(t, title=title, border_style=border)

    def _render_drone_info_panel(self, data, stale=False) -> Panel:
        t = Table(box=box.SIMPLE, show_header=True, show_edge=False,
                  pad_edge=False, expand=True)
        t.add_column("Time",  no_wrap=True)
        t.add_column("Level", no_wrap=True)
        t.add_column("Text",  overflow="fold")
        if not data:
            t.add_row("[dim]—[/dim]", "[dim]—[/dim]", "[dim]No message yet[/dim]")
        else:
            for item in reversed(data):
                ts = _stamp_to_clock_str(item["sec"], item["nanosec"])
                t.add_row(ts, item["level_str"], item["text"])
        title  = f"Drone Info (last {self.queue_size}) [STALE]" if stale else f"Drone Info (last {self.queue_size})"
        border = "bright_red" if stale else "red"
        return Panel(t, title=title, border_style=border, expand=True)

    def _render_drone_pipeline_panel(self, data, stale=False) -> Panel:
        t = self._make_kv_table()

        rec = data["recording"]
        if rec is None:
            rec_val = "[dim]Unknown[/dim]"
        elif rec:
            rec_val = "[bold green]● RECORDING[/bold green]"
        else:
            rec_val = "[dim red]○ OFF[/dim red]"
        t.add_row("Save Video", rec_val)

        stream = data["streaming"]
        if stream is None:
            stream_val = "[dim]Unknown[/dim]"
        elif stream:
            stream_val = "[bold green]● STREAMING[/bold green]"
        else:
            stream_val = "[dim red]○ OFF[/dim red]"
        t.add_row("Stream", stream_val)

        title  = "Drone Pipeline [STALE]" if stale else "Drone Pipeline"
        border = "bright_red" if stale else "magenta"
        return Panel(t, title=title, border_style=border)

    # ------------------------------------------------------------------ #
    #  Layout update  — mutate the fixed Layout tree in-place
    # ------------------------------------------------------------------ #
    def _update_layout(self, snap: dict) -> None:
        is_stale = snap["_stale"]

        header_text = Text(
            "mavros_info — terminal information panel   (Ctrl+C to exit)",
            style="bold white on dark_green",
            justify="center",
        )
        self._layout["header"].update(Panel(header_text, border_style="green"))

        # Assign each panel to its fixed slot
        self._layout["col_0"].update(
            self._render_odom_panel(snap["Odometry"], stale=is_stale["Odometry"])
        )
        self._layout["state"].update(
            self._render_state_panel(snap["State"], stale=is_stale["State"])
        )
        self._layout["ext_state"].update(
            self._render_extended_state_panel(snap["Extended State"], stale=is_stale["Extended State"])
        )
        self._layout["ctrl_gate"].update(
            self._render_control_state_panel(snap["Control Gate"], stale=is_stale["Control Gate"])
        )
        self._layout["gps1"].update(
            self._render_gps1_panel(snap["GPS1"], stale=is_stale["GPS1"])
        )
        self._layout["battery"].update(
            self._render_battery_panel(snap["Battery"], stale=is_stale["Battery"])
        )
        self._layout["statustext"].update(
            self._render_statustext_panel(snap["StatusText"], stale=is_stale["StatusText"])
        )
        self._layout["drone_info"].update(
            self._render_drone_info_panel(snap["Drone Info"], stale=is_stale["Drone Info"])
        )
        self._layout["drone_pipeline"].update(
            self._render_drone_pipeline_panel(
                snap["Drone Pipeline"], stale=is_stale["Drone Pipeline"]
            )
        )

    # ------------------------------------------------------------------ #
    #  Timer callback
    # ------------------------------------------------------------------ #
    def _on_refresh_tick(self):
        with self._ui_lock:
            start_live = self._live is None

            now = time.monotonic()
            new_is_stale = {
                name: (ts is None) or ((now - ts) > self.stale_threshold_s)
                for name, ts in self._last_update.items()
            }
            stale_changed = any(new_is_stale[k] != self._is_stale[k] for k in new_is_stale)
            self._is_stale = new_is_stale

            if not start_live and not any(self._dirty.values()) and not stale_changed:
                return

            # reset dirty flags
            for k in self._dirty:
                self._dirty[k] = False

            snap = {
                "_stale":         dict(self._is_stale),
                "Battery":        self._snapshot_battery(),
                "State":          self._snapshot_state(),
                "Extended State": self._snapshot_ext_state(),
                "Odometry":       self._snapshot_odom(),
                "StatusText":     self._snapshot_statustext(),
                "GPS1":           self._snapshot_gps1(),
                "Control Gate":   self._snapshot_control_state(),
                "Drone Info":     self._snapshot_drone_info(),
                "Drone Pipeline": self._snapshot_drone_pipeline(),
            }

        # Render outside the lock
        self._update_layout(snap)

        if start_live:
            self._live = Live(
                self._layout,
                console=self._console,
                auto_refresh=False,
                screen=True,          # <-- full-screen mode; no scrollback bleed
            )
            self._live.start()
            self._live.refresh()
        else:
            self._live.update(self._layout, refresh=True)

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