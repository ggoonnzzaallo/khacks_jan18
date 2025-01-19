import librosa
import numpy as np
from pydub import AudioSegment
from pydub.playback import play
import threading
import time
import movementsv2  # Ensure this module is implemented or mock it for testing


def segment_audio(file_path):
    """
    Segments audio based on sharp changes in music (onsets).
    """
    y, sr = librosa.load(file_path)
    onset_frames = librosa.onset.onset_detect(y=y, sr=sr, backtrack=True)
    onset_times = librosa.frames_to_time(onset_frames, sr=sr)
    segments = []
    for i in range(len(onset_times) - 1):
        segments.append({
            "start_time": round(onset_times[i], 2),
            "end_time": round(onset_times[i+1], 2)
        })
    return segments


def analyze_audio_portions(file_path):
    """
    Analyzes segmented audio and calculates tempo for each segment.
    """
    segments = segment_audio(file_path)
    y, sr = librosa.load(file_path)
    tempos = []
    for segment in segments:
        start_sample = int(segment['start_time'] * sr)
        end_sample = int(segment['end_time'] * sr)
        chunk = y[start_sample:end_sample]
        tempo_result = librosa.beat.beat_track(y=chunk, sr=sr)
        tempo = tempo_result[0]
        tempos.append({
            "start_time": segment['start_time'],
            "end_time": segment['end_time'],
            "tempo": round(float(tempo), 2)
        })
    return tempos


def determine_actions(tempos):
    """
    Maps tempo values to actions.
    """
    actions = []
    for chunk in tempos:
        tempo = chunk['tempo']
        if tempo < 80:
            action = "wave"
        elif 80 <= tempo <= 120:
            action = "rotate"
        else:
            action = "step"
        actions.append({
            "start_time": chunk['start_time'],
            "end_time": chunk['end_time'],
            "tempo": chunk['tempo'],
            "action": action,
            "duration": chunk['end_time'] - chunk['start_time']
        })
    return actions


def consolidate_actions(actions):
    """
    Consolidates consecutive actions of the same type by merging their time ranges.
    """
    if not actions:
        return []
    consolidated = []
    current_action = actions[0]
    for i in range(1, len(actions)):
        if actions[i]['action'] == current_action['action']:
            current_action['end_time'] = actions[i]['end_time']
            current_action['duration'] += actions[i]['duration']
        else:
            consolidated.append(current_action)
            current_action = actions[i]
    consolidated.append(current_action)
    return consolidated


def play_audio(file_path, start_event):
    """
    Plays the audio file using pydub. Waits for the start signal to begin playback.
    """
    start_event.wait()  # Wait for signal to start
    audio = AudioSegment.from_file(file_path)
    play(audio)


def execute_movements(consolidated_actions, start_event):
    """
    Executes movements synchronized with the action durations. Waits for the start signal.
    """
    start_event.wait()  # Wait for signal to start
    start_time = time.time()  # Record the starting time
    for idx, action_info in enumerate(consolidated_actions):
        elapsed_time = time.time() - start_time
        delay = max(0, action_info['start_time'] - elapsed_time)  # Ensure no negative delay
        time.sleep(delay)  # Wait until the start of the action
        hand = 'left' if idx % 2 == 0 else 'right'
        print(f"Performing {action_info['action']} with {hand} hand for {action_info['duration']} seconds.")

        if action_info['action'] == 'wave':
            movementsv2.hands_forward_backward('narrow', sleep_duration=action_info['duration']/2, which_hand=hand)
            movementsv2.hands_forward_backward_rest()
        elif action_info['action'] == 'rotate':
            movementsv2.hands_wide_open('narrow', sleep_duration=action_info['duration']/2)
            movementsv2.hands_wide_rest()
        elif action_info['action'] == 'step':
            movementsv2.fingers_unhold('narrow', sleep_duration=action_info['duration']/2)
            movementsv2.fingers_hold()

# Main logic
audio_file = "iron_man_trim.m4a"  # Replace with your audio file path

# Preprocess audio and prepare actions
tempo_results = analyze_audio_portions(audio_file)
actions = determine_actions(tempo_results)
consolidated_actions = consolidate_actions(actions)

# Event to synchronize start
start_event = threading.Event()

# Use threading to play audio and execute movements simultaneously
audio_thread = threading.Thread(target=play_audio, args=(audio_file, start_event))
movement_thread = threading.Thread(target=execute_movements, args=(consolidated_actions, start_event))

# Start both threads
audio_thread.start()
movement_thread.start()

# Signal threads to start simultaneously
start_event.set()

# Wait for both threads to complete
audio_thread.join()
movement_thread.join()
