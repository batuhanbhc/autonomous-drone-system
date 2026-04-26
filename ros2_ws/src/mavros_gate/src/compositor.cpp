/*
 * compositor.cpp
 *
 * Runs all three mavros_gate nodes in a single OS process using a
 * MultiThreadedExecutor.  Co-location eliminates inter-process IPC overhead
 * for the high-rate topics shared between nodes; when intra-process
 * communication is available the DDS middleware is bypassed entirely for
 * those message paths.
 *
 * Node topology (intra-process paths marked ★):
 *
 *   mcu_bridge ──★──► altitude_controller   (McuVerticalEstimate, ~20 Hz, sensor QoS)
 *   mcu_bridge ──★──► control_gate          (McuVerticalEstimate, ~20 Hz, best-effort)
 *   control_gate ──★─► altitude_controller  (AltitudeControllerInput, reliable)
 *   altitude_controller ──★──► control_gate (Float32, sensor QoS)
 *
 * All nodes receive the drone_id and serial_port ROS parameters that are
 * forwarded from the launch file.
 *
 * Thread model
 * ────────────
 * MultiThreadedExecutor is used so that timer callbacks (state publisher,
 * watchdogs) and subscription callbacks can execute concurrently across the
 * three nodes.  Each node continues to protect its own mutable state with the
 * same mutexes it already uses (state_mtx_, last_act_mtx_, vert_est_mtx_,
 * gains_mtx_), so no new data-races are introduced.
 *
 * The number of executor threads is left at the default (one per CPU core).
 * You can tune it by passing a count to MultiThreadedExecutor's constructor.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "mavros_gate/control_gate.hpp"
#include "mavros_gate/mcu_bridge.hpp"
#include "mavros_gate/altitude_controller.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ── Node options: enable intra-process communication ─────────────────────
  // All three nodes share the same context so messages published on a topic
  // where the publisher and subscriber are in the same process are handed off
  // via a shared_ptr rather than serialised through DDS.
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);

  // ── Instantiate nodes ────────────────────────────────────────────────────
  auto mcu_node  = std::make_shared<McuBridgeNode>(opts);
  auto gate_node = std::make_shared<ControlGateNode>(opts);
  auto alt_node  = std::make_shared<AltitudeControllerNode>(opts);

  // ── Multi-threaded executor ───────────────────────────────────────────────
  // Default thread count = hardware_concurrency().
  // Pass an explicit count if you need deterministic thread budgeting, e.g.:
  //   rclcpp::executors::MultiThreadedExecutor exec(
  //     rclcpp::ExecutorOptions{}, /*num_threads=*/4);
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(mcu_node);
  exec.add_node(gate_node);
  exec.add_node(alt_node);

  RCLCPP_INFO(rclcpp::get_logger("compositor"),
    "mavros_gate compositor running: mcu_bridge + control_gate + altitude_controller "
    "(intra-process comms enabled)");

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
