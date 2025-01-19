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

commands = [ ]

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

# Generate actuator commands for smooth transition
def generate_smooth_transition(current_states, future_states, num_frames):
    frame_commands = []
    for frame in range(1, num_frames + 1):
        progress = frame / num_frames
        commands = []
        for actuator_id, current_value in current_states.items():
            future_value = future_states.get(actuator_id, current_value)
            interpolated_value = current_value + progress * (future_value - current_value)
            commands.append({"actuator_id": actuator_id, "position": interpolated_value})
        frame_commands.append(commands)
    return frame_commands

def fingers_unhold(length, sleep_duration=0.1):
    commands = []
    if length == 'wide':
        position = 60
        sleep_duration = 0.3
    elif length == 'medium':
        position = 40
        sleep_duration = 0.3
    elif length == 'narrow':
        position = 20
    print(position)
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_gripper'], "position": position})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_gripper'], "position": -position})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    
    fingers_hold()

    return response

def fingers_hold(sleep_duration=0.1):
    commands = []
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_gripper'], "position": 0})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_gripper'], "position": 0})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    return response

def hands_wide_open(length, sleep_duration=0.1):
    if length == 'wide':
        position = 60
        default_sleep_duration = 0.3
    elif length == 'medium':
        position = 40
        default_sleep_duration = 0.3
    elif length == 'narrow':
        position = 20

    if sleep_duration == 0.1:
        sleep_duration = default_sleep_duration

    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_yaw'], "position": position})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_yaw'], "position": -position})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    hands_wide_rest()
    return response

def hands_wide_rest(sleep_duration=0.1):
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_yaw'], "position": 0})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_yaw'], "position": 0})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    return response

def hand_forward_backward(length, sleep_duration=0.1, which_hand='right'):
    if length == 'wide':
        position = 60
        sleep_duration = 0.3
    elif length == 'medium':
        position = 40
        sleep_duration = 0.3
    elif length == 'narrow':
        position = 20

    if which_hand == 'left':
        position = -position

    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_pitch'], "position": -position})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_pitch'], "position": -position})
    client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)

    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_pitch'], "position": position})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_pitch'], "position": position})
    response = client.actuator.command_actuators(commands)

    hands_forward_backward_rest()

    return response

def hands_forward_backward_rest(sleep_duration=0.1):
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_pitch'], "position": 0})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_pitch'], "position": 0})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    return response

def hip_rotate(sleep_duration=0.1):
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 60})
    # commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_hip_roll'], "position": 10})

    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    return response

def hip_reset(sleep_duration=0.1):
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 0})
    # commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_hip_roll'], "position": 0})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    return response

def sit(sleep_duration=0.1):
    # commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_hip_pitch'], "position": -30})
    # commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_hip_pitch'], "position": 30})

    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": -60})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 60})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    return response

def stand(sleep_duration=0.1):
    # commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_hip_pitch'], "position": 0})
    # commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_hip_pitch'], "position": 0})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": 0})
    commands.append({"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 0})
    response = client.actuator.command_actuators(commands)
    time.sleep(sleep_duration)
    return response


# sit(0.3)
# stand()

# hip_rotate(0.2)
# hip_reset()

# time.sleep(1)

# fingers_unhold('medium')
# fingers_hold()

# hand_forward_backward('narrow',sleep_duration=0.2, which_hand='left')
# hands_forward_backward_rest(sleep_duration=0.4)
# hand_forward_backward('narrow',sleep_duration=0.2, which_hand='right')
# hands_forward_backward_rest(sleep_duration=0.4)

# hands_wide_open('narrow',sleep_duration=0.2)
# hands_wide_rest()