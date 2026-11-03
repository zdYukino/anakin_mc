"""Exercise the actual MC node with ROS fixtures, without SDK channels or a robot.

Source ROS and the installed SDK environment before running. The test owns its MC
process and uses the caller's isolated ROS_DOMAIN_ID.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from unique_identifier_msgs.msg import UUID
from anakin_sdk.msg import ControlTarget, InferenceStatus, RobotStateFrame
from anakin_sdk.srv import SwitchInferenceMode


JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint",
    "right_ankle_roll_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
POSE = [-.1, 0., 0., .3, -.2, 0., -.1, 0., 0., .3, -.2, 0.,
        .07, .06, -.05, .15, .25, -.12, .7, .05, -.03, .02,
        -.1, -.22, .08, .8, -.05, .03, -.02]
STAND = [-.1, 0., 0., .3, -.2, 0., -.1, 0., 0., .3, -.2, 0.,
         0., 0., 0., 0., .3, 0., 1., 0., 0., 0.,
         0., -.3, 0., 1., 0., 0., 0.]


def new_uuid():
    result = UUID()
    result.uuid = list(uuid.uuid4().bytes)
    return result


class Fixture(Node):
    def __init__(self):
        super().__init__("anakin_fast_switch_fixture")
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(RobotStateFrame, "anakin/state", best_effort)
        self.create_subscription(ControlTarget, "anakin/control_target", self.target, best_effort)
        self.create_subscription(InferenceStatus, "anakin/inference_status", self.status, 128)
        self.client = self.create_client(SwitchInferenceMode, "anakin/switch_inference_mode")
        self.session = new_uuid()
        self.instance = None
        self.request = None
        self.epoch = 0
        self.state_seq = 0
        self.mode = RobotStateFrame.MODE_PREPARING
        self.pose = list(POSE)
        self.targets = []
        self.statuses = []
        self.create_timer(.01, self.publish_frame)

    def publish_frame(self):
        frame = RobotStateFrame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.schema_version = 3
        frame.schema_id = "anakin.g1.canonical29.v3"
        frame.session_id = self.session
        frame.epoch = self.epoch
        self.state_seq += 1
        frame.state_seq = self.state_seq
        frame.validity = 7
        frame.gain_profile = "g1_asap_merged_sim_v1"
        frame.execution_profile = "g1_sdk_pd_v1"
        frame.command_semantics = 1
        frame.execution_authorized = self.request is not None
        frame.execution_mode = self.mode
        frame.joints.name = JOINTS
        frame.joints.position = self.pose
        frame.joints.velocity = [0.] * 29
        frame.imu.orientation.w = 1.
        frame.command_base_height = .78
        if self.instance:
            frame.mc_instance_seen = self.instance
        if self.request:
            frame.request_id_seen = self.request.request_id
        self.publisher.publish(frame)

    def target(self, target):
        self.targets.append((time.monotonic(), target))
        if self.request and target.epoch == self.epoch and target.phase == ControlTarget.TARGET_ACTIVE:
            self.mode = RobotStateFrame.MODE_ACTIVE

    def status(self, status):
        if any(status.mc_instance_id.uuid):
            self.instance = status.mc_instance_id
        self.statuses.append((time.monotonic(), status))

    def wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=.01)
        raise AssertionError("Timed out waiting for " + description)

    def begin(self, model, profile="g1_direct_v1"):
        self.epoch += 1
        request = SwitchInferenceMode.Request()
        request.session_id = self.session
        request.expected_mc_instance_id = self.instance
        request.request_id = new_uuid()
        request.next_epoch = self.epoch
        request.source_state_seq = self.state_seq + 1
        request.schema_id = "anakin.g1.canonical29.v3"
        request.target_model = model
        request.gain_profile = "g1_asap_merged_sim_v1"
        request.execution_profile = "g1_sdk_pd_v1"
        request.command_semantics = 1
        request.transition_profile = profile
        request.allow_source_continuation = True
        request.timeout.sec = 5
        self.request = request
        self.mode = RobotStateFrame.MODE_PREPARING
        started = time.monotonic()
        future = self.client.call_async(request)
        self.wait(future.done, 2., "service response")
        if not future.result().accepted:
            raise AssertionError("MC rejected request: " + future.result().detail)
        return started

    def current_targets(self):
        return [(t, target) for t, target in self.targets if target.epoch == self.epoch]

    def active(self):
        return any(target.phase == ControlTarget.TARGET_ACTIVE for _, target in self.current_targets())

def max_error(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mc", type=Path, required=True)
    parser.add_argument("--loco", type=Path, required=True)
    parser.add_argument("--mimic", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    fixture = Fixture()
    with tempfile.TemporaryDirectory(prefix="anakin-fast-switch-") as tmp:
        log_path = Path(tmp) / "mc.log"
        trace_path = Path(tmp) / "trace.jsonl"
        with log_path.open("w+") as log:
            process = subprocess.Popen([
                str(args.mc), "--ros-args", "-p", f"loco_model:={args.loco}",
                "-p", f"mimic_model:={args.mimic}", "-p", f"trace_jsonl:={trace_path}"
            ], stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
            try:
                fixture.wait(lambda: fixture.instance and fixture.client.service_is_ready(), 5., "MC discovery")
                fixture.begin("asap_loco")
                fixture.wait(fixture.active, 2., "initial loco")
                until = time.monotonic() + .16
                fixture.wait(lambda: time.monotonic() >= until, 1., "background mimic history")
                entry = fixture.begin("asap_cr7")
                fixture.wait(lambda: bool(fixture.current_targets()), 1., "first mimic target")
                entry_time, entry_target = fixture.current_targets()[0]
                assert entry_target.phase == ControlTarget.TARGET_ACTIVE and entry_target.model_id == "asap_cr7"
                assert entry_time - entry < .2, f"Mimic entry was delayed: {entry_time-entry:.3f}s"
                assert all(abs(entry_target.target.points[0].positions[i] - POSE[i]) < 1e-9
                           for i in (19, 20, 21, 26, 27, 28)), "Mimic unowned-joint behavior changed"
                until = time.monotonic() + .16
                fixture.wait(lambda: time.monotonic() >= until, 1., "background loco history")
                # The legacy profile name is still accepted, but no longer delays a model switch.
                returned = fixture.begin("asap_loco", "g1_quick_return_v1")
                fixture.wait(lambda: bool(fixture.current_targets()), 1., "first loco return target")
                first_time, first = fixture.current_targets()[0]
                assert first.phase == ControlTarget.TARGET_ACTIVE and first.model_id == "asap_loco"
                assert first_time - returned < .2, f"Loco return was delayed: {first_time-returned:.3f}s"
                upper_error = max_error(first.target.points[0].positions[12:], STAND[12:])
                assert upper_error < 1e-9, f"Loco standing target error {upper_error} rad"
                fixture.pose[16] += .2
                fixture.pose[25] -= .1
                fixture.wait(lambda: len(fixture.current_targets()) >= 4, 1., "continued loco targets")
                assert all(max_error(target.target.points[0].positions[12:], STAND[12:]) < 1e-9
                           for _, target in fixture.current_targets()), \
                    "Loco failed to retain the standing reference while feedback changed"
                assert all(math.isfinite(q) for _, m in fixture.current_targets()
                           for q in m.target.points[0].positions), "Nonfinite output"
            except Exception:
                log.flush()
                print(log_path.read_text())
                raise
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.)
            records = {(row["epoch"], row["target_seq"]): row
                       for row in map(json.loads, trace_path.read_text().splitlines())}
            for message in (entry_target, first):
                row = records[(message.epoch, message.seq)]
                controlled = range(12) if message.model_id == "asap_loco" else [
                    i for i in range(29) if i not in (19, 20, 21, 26, 27, 28)
                ]
                assert all(abs(row["published_q_ref"][i] - row["policy_q_ref"][i]) < 1e-9
                           for i in controlled), "First target was blended or modified"
                action_count = 12 if message.model_id == "asap_loco" else 23
                assert all(value == 0. for value in row["observation"][:action_count]), \
                    "Prewarm inserted the previous model's action"
                if message.model_id == "asap_loco":
                    assert max_error(row["observation"][-18:-1], STAND[12:]) < 1e-6, \
                        "Loco observation does not use its actual standing reference"
            print(f"PASS direct ROS entry={entry_time-entry:.3f}s, return={first_time-returned:.3f}s; "
                  "loco unowned targets and observations use stand, histories remain model-owned")
    fixture.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
