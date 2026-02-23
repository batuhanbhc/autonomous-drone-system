#!/usr/bin/env python3
from __future__ import annotations
import threading 
import time

# rclpy imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import (
    QoSProfile,
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
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import GPSRAW

# console ui imports
from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.columns import Columns
from rich.text import Text
from rich import box

from mavros_gcs.panel_utils.views import (
    StateView,
    BatteryView,
    ExtendedStateView,
    StatusTextView,
    OdometryView,
    VelocityView,
    GPSView
)

from mavros_gcs.panel_utils.helpers import (
    _format_bool_to_str,
    _format_float_to_str,
    _format_str_to_str,
    _stamp_to_clock_str
)

class InfoPanelNode(Node):
    """
    - Each topic has a small "view model" (dataclass) holding latest values.
    - Callbacks only update the cached values.
    - A timer refreshes the terminal UI at a fixed rate independent of topic rates.
    """

    def __init__(self):
        super().__init__("mavros_info_panel")
        
        # --------------------------------------------------
        # node parameters
        # determines the size of history for StatusText topic panel
        self.queue_size = int(self.declare_parameter("queue_size", 5).value)

        # determines the thresholds in seconds for marking panels as stale since last update
        self.stale_threshold_s = float(self.declare_parameter("stale_threshold_s", 2.0).value)

        # determines how often to refresh terminal ui
        self.refresh_hz = self.declare_parameter("refresh_hz", 2.0).value
        # --------------------------------------------------

        self._ui_lock = threading.Lock()

        # cached views
        self.battery = BatteryView()
        self.state = StateView()
        self.ext_state = ExtendedStateView()
        self.statustext = StatusTextView(queue_size=self.queue_size)
        self.odom = OdometryView()
        self.vel_body = VelocityView()
        self.vel_local = VelocityView()
        self.gps1 = GPSView()

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # subscriptions
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_extended_state, reliable_qos)
        self.create_subscription(StatusText, "/mavros/statustext/recv", self._on_statustext, qos_profile_sensor_data)
        self.create_subscription(BatteryState, "/mavros/battery", self._on_battery, qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self._on_state, reliable_qos)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_body", self._on_vel_body, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._on_vel_local, qos_profile_sensor_data)
        self.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", self._on_gps1_raw, qos_profile_sensor_data)
        
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
            "Velocity (Body)": self._render_vel_body_panel,
            "Velocity (Origin)": self._render_vel_origin_panel,
            "StatusText": self._render_statustext_panel,
            "GPS1": self._render_gps1_panel,
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

    def _on_vel_body(self, msg: TwistStamped):
        with self._ui_lock:
            self.vel_body.update_from_msg(msg)
            self._mark_dirty("Velocity (Body)")

    def _on_vel_local(self, msg: TwistStamped):
        with self._ui_lock:
            self.vel_local.update_from_msg(msg)
            self._mark_dirty("Velocity (Origin)")

    def _on_statustext(self, msg: StatusText):
        with self._ui_lock:
            self.statustext.update_from_msg(msg)
            self._mark_dirty("StatusText")

    def _on_gps1_raw(self, msg: GPSRAW):
        with self._ui_lock:
            self.gps1.update_from_msg(msg)
            self._mark_dirty("GPS1")


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
            "frame_id": self.odom.frame_id,
            "child_frame_id": self.odom.child_frame_id,
            "x": self.odom.x, "y": self.odom.y, "z": self.odom.z,
            "dist_lo": self.odom.dist_lo,
            "yaw": self.odom.yaw, "roll": self.odom.roll, "pitch": self.odom.pitch,
        }

    def _snapshot_vel(self, vel: VelocityView):
        return {"vx": vel.vx, "vy": vel.vy, "vz": vel.vz}

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

        formatted_frame_id = _format_str_to_str(data["frame_id"])
        formatted_child_frame_id = _format_str_to_str(data["child_frame_id"])

        t.add_row("Frame ID", formatted_frame_id)
        t.add_row("Child Frame ID", formatted_child_frame_id, end_section=True)

        t.add_row(f"Position (frame: {formatted_frame_id})")
        t.add_row("X (m)", _format_float_to_str(data["x"], 3))
        t.add_row("Y (m)", _format_float_to_str(data["y"], 3))
        t.add_row("Z (m)", _format_float_to_str(data["z"], 3), end_section=True)

        t.add_row("Local Origin Distance (m)", _format_float_to_str(data["dist_lo"]), end_section=True)

        t.add_row("Orientation (earth-fixed)")
        t.add_row("Yaw (deg)", _format_float_to_str(data["yaw"], 3))
        t.add_row("Roll (deg)", _format_float_to_str(data["roll"], 3))
        t.add_row("Pitch (deg)", _format_float_to_str(data["pitch"], 3), end_section=True)

        return Panel(t, title="Odometry", border_style="yellow")
    
    def _render_vel_body_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        t.add_row("X (m/s)", _format_float_to_str(data["vx"], 3))
        t.add_row("Y (m/s)", _format_float_to_str(data["vy"], 3))
        t.add_row("Z (m/s)", _format_float_to_str(data["vz"], 3))

        return Panel(t, title="Velocity (Body)", border_style="red")
    
    def _render_vel_origin_panel(self, data) -> Panel:
        t = Table(
            box=box.SIMPLE,
            show_header=False,
            show_edge=False,
            pad_edge=False,
        )
        t.add_column(justify="left")
        t.add_column(justify="right")

        t.add_row("X (m/s)", _format_float_to_str(data["vx"], 3))
        t.add_row("Y (m/s)", _format_float_to_str(data["vy"], 3))
        t.add_row("Z (m/s)", _format_float_to_str(data["vz"], 3))

        return Panel(t, title="Velocity (Origin)", border_style="white")
        
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

        panels.append(get("State", lambda: self._render_state_panel(snap["State"])))
        panels.append(get("Extended State", lambda: self._render_extended_state_panel(snap["Extended State"])))
        panels.append(get("Battery", lambda: self._render_battery_panel(snap["Battery"])))
        panels.append(get("Odometry", lambda: self._render_odom_panel(snap["Odometry"])))
        panels.append(get("Velocity (Body)", lambda: self._render_vel_body_panel(snap["Velocity (Body)"])))
        panels.append(get("Velocity (Origin)", lambda: self._render_vel_origin_panel(snap["Velocity (Origin)"])))
        panels.append(get("StatusText", lambda: self._render_statustext_panel(snap["StatusText"])))
        panels.append(get("GPS1", lambda: self._render_gps1_panel(snap["GPS1"])))

        body = Columns(panels, equal=False, expand=False)
        return Panel.fit(body, title=header, border_style="green")


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
            snap["Velocity (Body)"] = self._snapshot_vel(self.vel_body)
            snap["Velocity (Origin)"] = self._snapshot_vel(self.vel_local)
            snap["StatusText"] = self._snapshot_statustext()
            snap["GPS1"] = self._snapshot_gps1()

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