from pykos import KOS
import time

# Connect to KOS running on localhost at port 50051
client = KOS(ip='192.168.42.1', port=50051)

ACTUATOR_NAME_TO_ID = {
    "left_shoulder_yaw": 11,
    "left_shoulder_pitch": 12,
    "left_elbow_yaw": 13,
    "left_gripper": 14,
    "right_shoulder_yaw": 21,
    "right_shoulder_pitch": 22,
    "right_elbow_yaw": 23,
    "right_gripper": 24,
    "left_hip_yaw": 31,
    "left_hip_roll": 32,
    "left_hip_pitch": 33,
    "left_knee_pitch": 34,
    "left_ankle_pitch": 35,
    "right_hip_yaw": 41,
    "right_hip_roll": 42,
    "right_hip_pitch": 43,
    "right_knee_pitch": 44,
    "right_ankle_pitch": 45
}

ACTUATOR_ID_TO_NAME = {v: k for k, v in ACTUATOR_NAME_TO_ID.items()}

for key in ACTUATOR_NAME_TO_ID:
    id = ACTUATOR_NAME_TO_ID[key]

    response = client.actuator.configure_actuator(
        actuator_id=id,
        kp=32,
        kd=32,
        ki=32,
        max_torque=100.0,
        torque_enabled=True,
        zero_position=False
    )
    if response.success:
        print(f"{key}: {id} Actuator configured successfully.")
                
    else:
        print(f"Failed to configure actuator: {response.error}")

commands = [ ]
# commands.append({"actuator_id": 35, "position": 0})
# commands.append({"actuator_id": 45, "position": 0})
# commands.append({"actuator_id": 43, "position": 0})
# commands.append({"actuator_id": 44, "position": 0})
# commands.append({"actuator_id": 34, "position": 0})
commands.append({"actuator_id": 21, "position": 0})
commands.append({"actuator_id": 11, "position": 0})
# commands.append({"actuator_id": 24, "position": 0})
# commands.append({"actuator_id": 14, "position": 0})
# commands.append({"actuator_id": 22, "position": 0})
# commands.append({"actuator_id": 12, "position": 0})
response = client.actuator.command_actuators(commands)

time.sleep(0.5)

commands = [ ]
# commands.append({"actuator_id": 35, "position": 30})
# commands.append({"actuator_id": 45, "position": -30})
# commands.append({"actuator_id": 43, "position": 30})
# commands.append({"actuator_id": 44, "position": 30})
# commands.append({"actuator_id": 34, "position": -30})
commands.append({"actuator_id": 21, "position": 60})
commands.append({"actuator_id": 11, "position": -60})
# commands.append({"actuator_id": 24, "position": 10})
# commands.append({"actuator_id": 14, "position": 10})
# commands.append({"actuator_id": 22, "position": -60})
# commands.append({"actuator_id": 12, "position": 60})
response = client.actuator.command_actuators(commands)

time.sleep(0.5)

commands = [ ]
# commands.append({"actuator_id": 35, "position": 0})
# commands.append({"actuator_id": 45, "position": 0})
# commands.append({"actuator_id": 43, "position": 0})
# commands.append({"actuator_id": 44, "position": 0})
# commands.append({"actuator_id": 34, "position": 0})
commands.append({"actuator_id": 21, "position": 0})
commands.append({"actuator_id": 11, "position": 0})
# commands.append({"actuator_id": 24, "position": 0})
# commands.append({"actuator_id": 14, "position": 0})
# commands.append({"actuator_id": 22, "position": 0})
# commands.append({"actuator_id": 12, "position": 0})
response = client.actuator.command_actuators(commands)