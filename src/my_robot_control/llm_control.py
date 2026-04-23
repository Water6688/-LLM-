import json
import math
import os
import re
import ssl
import urllib.error
import urllib.request

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_API_URL = ""  #输入你所调用的API
API_URL = os.environ.get("LLM_ROBOT_API_URL", DEFAULT_API_URL)

SERVO_CENTER = 2048
SERVO_MIN = 0
SERVO_MAX = 4095
SERVO_COUNT = 6
STEP_DURATION_SEC = 1.5

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
JOINT_DIRECTIONS = [-1, -1, -1, -1, -1, 1]
ARM_CONTROLLER_NAME = "arm_controller"
GRIPPER_CONTROLLER_NAME = "gripper_controller"
ARM_ACTION_NAME = f"/{ARM_CONTROLLER_NAME}/follow_joint_trajectory"
GRIPPER_ACTION_NAME = f"/{GRIPPER_CONTROLLER_NAME}/follow_joint_trajectory"

_NUMERIC_PATTERN = re.compile(r"-?\d+(?:\.\d+)?")


def _http_post_json(url, payload, timeout_sec=60):
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    context = ssl._create_unverified_context()
    with urllib.request.urlopen(request, context=context, timeout=timeout_sec) as response:
        response_text = response.read().decode("utf-8")
    return json.loads(response_text)


def _extract_candidate_payload(data):
    if isinstance(data, dict):
        for key in ("robot_command", "robot_commands", "commands", "actions", "trajectory", "data", "result"):
            if key in data and data[key] is not None:
                return data[key]
    return data


def _normalize_numeric_sequence(values):
    numbers = []
    for value in values:
        if isinstance(value, bool):
            raise ValueError("布尔值不能作为舵机位置")
        if isinstance(value, (int, float)):
            numbers.append(int(round(value)))
            continue
        if isinstance(value, str):
            matches = _NUMERIC_PATTERN.findall(value)
            if not matches:
                raise ValueError(f"无法解析数值：{value}")
            numbers.extend(int(round(float(match))) for match in matches)
            continue
        raise ValueError(f"不支持的数值类型：{type(value)!r}")
    return numbers


def _extract_groups_from_dict(data):
    ordered_keys = [f"joint{i}" for i in range(1, 7)]
    if all(key in data for key in ordered_keys):
        return [_normalize_numeric_sequence(data[key] for key in ordered_keys)]

    for key in ("groups", "group", "robot_command", "robot_commands", "commands", "actions", "trajectory", "data", "result"):
        if key in data and data[key] is not None:
            return parse_command_groups(data[key])

    numeric_values = [value for value in data.values() if isinstance(value, (int, float, str))]
    if len(numeric_values) == len(data):
        numbers = _normalize_numeric_sequence(numeric_values)
        if len(numbers) == SERVO_COUNT:
            return [numbers]
        if len(numbers) % SERVO_COUNT == 0:
            return [numbers[index:index + SERVO_COUNT] for index in range(0, len(numbers), SERVO_COUNT)]

    raise ValueError("无法从字典中解析机械臂动作")


def parse_command_groups(payload):
    if payload is None:
        raise ValueError("模型没有返回动作数据")

    if isinstance(payload, str):
        text = payload.strip()
        if not text:
            raise ValueError("模型返回了空字符串")
        try:
            return parse_command_groups(json.loads(text))
        except json.JSONDecodeError:
            numbers = _normalize_numeric_sequence(_NUMERIC_PATTERN.findall(text))
            if len(numbers) == SERVO_COUNT:
                return [numbers]
            if len(numbers) % SERVO_COUNT == 0:
                return [numbers[index:index + SERVO_COUNT] for index in range(0, len(numbers), SERVO_COUNT)]
            raise ValueError("字符串里提取到的数值数量不是 6 的倍数")

    if isinstance(payload, dict):
        return _extract_groups_from_dict(payload)

    if isinstance(payload, (list, tuple)):
        if not payload:
            raise ValueError("模型返回了空动作列表")

        if all(not isinstance(item, (list, tuple, dict)) for item in payload):
            numbers = _normalize_numeric_sequence(payload)
            if len(numbers) == SERVO_COUNT:
                return [numbers]
            if len(numbers) % SERVO_COUNT == 0:
                return [numbers[index:index + SERVO_COUNT] for index in range(0, len(numbers), SERVO_COUNT)]
            raise ValueError("动作数量不是 6 的倍数")

        groups = []
        for item in payload:
            groups.extend(parse_command_groups(item))
        return groups

    raise ValueError(f"不支持的返回类型：{type(payload)!r}")


def get_ai_command(prompt):
    response = _http_post_json(API_URL, {"prompt": prompt})
    payload = _extract_candidate_payload(response)
    return parse_command_groups(payload)


def clamp_servo_position(position):
    return max(SERVO_MIN, min(SERVO_MAX, int(round(position))))


def servo_position_to_joint_radians(raw_position, joint_direction):
    clamped_position = clamp_servo_position(raw_position)
    return joint_direction * (clamped_position - SERVO_CENTER) * (2.0 * math.pi / 4096.0)


def build_joint_trajectory(joint_names, points, duration_per_step=STEP_DURATION_SEC):
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)

    for index, joint_positions in enumerate(points):
        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in joint_positions]
        total_seconds = (index + 1) * duration_per_step
        point.time_from_start = Duration(
            sec=int(total_seconds),
            nanosec=int((total_seconds % 1.0) * 1e9),
        )
        trajectory.points.append(point)

    return trajectory


class LlmMoveItBridge:
    def __init__(self):
        self.node = rclpy.create_node("llm_moveit_bridge")
        self.arm_client = ActionClient(self.node, FollowJointTrajectory, ARM_ACTION_NAME)
        self.gripper_client = ActionClient(self.node, FollowJointTrajectory, GRIPPER_ACTION_NAME)

    def wait_for_servers(self):
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"无法连接动作服务器：{ARM_ACTION_NAME}")
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"无法连接动作服务器：{GRIPPER_ACTION_NAME}")

    def send_trajectory_async(self, client, joint_names, points):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = build_joint_trajectory(joint_names, points)
        return client.send_goal_async(goal)

    def execute_groups(self, command_groups):
        arm_points = []
        gripper_points = []

        for group_index, group in enumerate(command_groups, start=1):
            if len(group) != SERVO_COUNT:
                raise ValueError(f"第 {group_index} 组动作不是 6 个舵机值")
            joint_positions = [
                servo_position_to_joint_radians(raw_position, joint_direction)
                for raw_position, joint_direction in zip(group, JOINT_DIRECTIONS)
            ]
            arm_points.append(joint_positions[:5])
            gripper_points.append([joint_positions[5]])

        arm_send_future = self.send_trajectory_async(self.arm_client, JOINT_NAMES[:5], arm_points)
        gripper_send_future = self.send_trajectory_async(self.gripper_client, JOINT_NAMES[5:], gripper_points)

        rclpy.spin_until_future_complete(self.node, arm_send_future)
        rclpy.spin_until_future_complete(self.node, gripper_send_future)

        arm_goal_handle = arm_send_future.result()
        gripper_goal_handle = gripper_send_future.result()
        if arm_goal_handle is None or not arm_goal_handle.accepted:
            raise RuntimeError("控制器拒绝了 arm 轨迹目标")
        if gripper_goal_handle is None or not gripper_goal_handle.accepted:
            raise RuntimeError("控制器拒绝了 gripper 轨迹目标")

        arm_result_future = arm_goal_handle.get_result_async()
        gripper_result_future = gripper_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, arm_result_future)
        rclpy.spin_until_future_complete(self.node, gripper_result_future)

        arm_result = arm_result_future.result()
        gripper_result = gripper_result_future.result()
        if arm_result is None:
            raise RuntimeError("arm 控制器返回空结果")
        if gripper_result is None:
            raise RuntimeError("gripper 控制器返回空结果")

        return arm_result.result, gripper_result.result


def main():

    # 完美对齐的粗边框版本
    print("╔" + "═" * 70 + "╗")
    print("║" + " " * 20 + "🤖  LEROBOT 智能机械臂控制系统" + " " * 20 + "║")
    print("╠" + "═" * 70 + "╣")
    print("║" + " " * 18 + "🎯 自然语言交互 | 输入 exit 安全退出" + " " * 16 + "║")
    print(f"║ 🚀 动作服务器: /arm_controller/follow_joint_trajectory".ljust(65) + "║")
    print(f"║ 🧩 夹爪服务器: /gripper_controller/follow_joint_trajectory".ljust(65) + "║")
    print("╚" + "═" * 70 + "╝")
    rclpy.init()
    bridge = LlmMoveItBridge()

    try:
        bridge.wait_for_servers()
        while rclpy.ok():
            try:
                prompt = input("输入指令：").strip()
            except EOFError:
                break

            if not prompt:
                continue
            if prompt.lower() == "exit":
                break

            try:
                command_groups = get_ai_command(prompt)
                print(f"→ 解析到 {len(command_groups)} 组动作")
                for index, group in enumerate(command_groups, start=1):
                    print(f"  组 {index}: {group}")
                bridge.execute_groups(command_groups)
                print("→ 动作已下发")
            except Exception as exc:
                print(f"错误：{exc}")
    except KeyboardInterrupt:
        print("\n输入 exit 退出")
    finally:
        bridge.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()