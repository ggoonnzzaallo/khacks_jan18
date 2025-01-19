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
    "right_ankle_pitch": 45,
}

# Reverse map for logging/debugging
ACTUATOR_ID_TO_NAME = {v: k for k, v in ACTUATOR_NAME_TO_ID.items()}


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

def send_commands(commands, sleep_duration=0.1, log=False, FPS=1):
    """
    Send actuator commands and handle responses.
    """
    if log:
        print(f"Sending commands: {commands}")
    
    print(commands)

    transition_duration = 1.0  # seconds
    num_frames = int(FPS * transition_duration)

    future_states = {}
    current_states = {}

    for command in commands:
        actuator_id = command['actuator_id']
        position = command['position']
        future_states[actuator_id] = position

    actuator_ids = future_states.keys()

    states = client.actuator.get_actuators_state(actuator_ids)

    for state in states.states:
        actuator_id = int(state.actuator_id)
        position = float(state.position)
        current_states[actuator_id] = position


    print(current_states, future_states)

    frame_commands = generate_smooth_transition(current_states, future_states, num_frames)

    for frame, commands in enumerate(frame_commands):
        print(f"Frame {frame + 1}/{num_frames}: Sending commands: {commands}")
        response = client.actuator.command_actuators(commands)
        time.sleep(1 / FPS)  # Maintain 10 FPS

    # response = client.actuator.command_actuators(commands)
    # time.sleep(sleep_duration)
    # return response


def configure_actuators():
    for key, id in ACTUATOR_NAME_TO_ID.items():
        response = client.actuator.configure_actuator(
            actuator_id=id,
            kp=32,
            kd=32,
            ki=32,
            max_torque=100.0,
            torque_enabled=True,
            zero_position=False,
        )
        if response.success:
            print(f"{key}: {id} Actuator configured successfully.")
        else:
            print(f"Failed to configure actuator {key}: {response.error}")


def fingers_unhold(length='medium', sleep_duration=0.1):
    position_map = {'wide': 60, 'medium': 40, 'narrow': 20}
    position = position_map.get(length, 40)
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_gripper'], "position": position},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_gripper'], "position": -position},
    ]
    return send_commands(commands, sleep_duration)


def fingers_hold(sleep_duration=0.1):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_gripper'], "position": 0},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_gripper'], "position": 0},
    ]
    return send_commands(commands, sleep_duration)


def hands_wide_open(length='medium', sleep_duration=0.1):
    position_map = {'wide': 60, 'medium': 40, 'narrow': 20}
    position = position_map.get(length, 40)
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_yaw'], "position": position},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_yaw'], "position": -position},
    ]
    return send_commands(commands, sleep_duration)


def hands_wide_rest(sleep_duration=0.1):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_yaw'], "position": 0},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_yaw'], "position": 0},
    ]
    return send_commands(commands, sleep_duration)


def hands_forward_backward(length='medium', sleep_duration=0.1, which_hand='right'):
    position_map = {'wide': 60, 'medium': 40, 'narrow': 20}
    position = position_map.get(length, 40)

    if which_hand == 'left':
        position = -position

    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_pitch'], "position": -position},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_pitch'], "position": -position},
    ]
    
    send_commands(commands, sleep_duration)

def hands_forward_backward_rest(sleep_duration=0.1):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_shoulder_pitch'], "position": 0},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_shoulder_pitch'], "position": 0},
    ]
    return send_commands(commands, sleep_duration)


def hip_rotate(sleep_duration=0.1):
    commands = [{"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 60}]
    return send_commands(commands, sleep_duration)


def hip_reset(sleep_duration=0.1):
    commands = [{"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 0}]
    return send_commands(commands, sleep_duration)

def bend_down(sleep_duration=0.3):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": -10},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 10},
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_hip_pitch'], "position": 40},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_hip_pitch'], "position": -40},
    ]
    send_commands(commands, sleep_duration, FPS=15)

def stand_up(sleep_duration=0.3):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": 0},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 0},
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_hip_pitch'], "position": 0},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_hip_pitch'], "position": 0},
    ]
    send_commands(commands, sleep_duration, FPS=15)


def sit(sleep_duration=0.3):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": -60},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 60},
    ]
    return send_commands(commands, sleep_duration)


def stand(sleep_duration=0.3):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": 0},
        {"actuator_id": ACTUATOR_NAME_TO_ID['left_knee_pitch'], "position": 0},
    ]
    return send_commands(commands, sleep_duration)


def right_leg_forward(sleep_duration=0.3):
    commands = [
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_ankle_pitch'], "position": 20},
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_ankle_pitch'], "position": 20},
        


        # {"actuator_id": ACTUATOR_NAME_TO_ID['right_ankle_pitch'], "position": -10},
        # {"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": -10},
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_hip_pitch'], "position": 10},
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_ankle_pitch'], "position": -20},
        {"actuator_id": ACTUATOR_NAME_TO_ID['right_ankle_pitch'], "position": 20},
        # {"actuator_id": ACTUATOR_NAME_TO_ID['right_ankle_pitch'], "position": 0},
        # {"actuator_id": ACTUATOR_NAME_TO_ID['right_knee_pitch'], "position": -20},
    ]
    return send_commands(commands, sleep_duration)

# Example usage
configure_actuators()

# hands_wide_open('medium', sleep_duration=0.5)
# hands_wide_rest()

# hands_forward_backward('medium',sleep_duration=0.5, which_hand='left')
# hands_forward_backward_rest()

# hands_forward_backward('medium',sleep_duration=0.5, which_hand='right')
# hands_forward_backward_rest()

# fingers_unhold('narrow', sleep_duration=0.3)
# fingers_hold()

# bend_down(sleep_duration=1)
# time.sleep(1)
# stand_up(sleep_duration=0.3)

# right_leg_forward()
