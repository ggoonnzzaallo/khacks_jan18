#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    # Load the model from the scene XML file instead of the robot file directly
    model = mujoco.MjModel.from_xml_path("scene.xml")
    data = mujoco.MjData(model)

    # Create a visualization window
    viewer = mujoco.viewer.launch(model, data)

    # Set initial pose to standing position
    mujoco.mj_resetDataKeyframe(model, data, 0)  

    # Print information about the model
    print(f"Number of actuators: {model.nu}")
    # Get actuator names from model.names, which stores all names as a single string
    actuator_adr = model.name_actuatoradr
    actuator_names = [model.names[adr:].decode().split('\x00')[0] for adr in actuator_adr]
    print(f"Actuator names: {actuator_names}")
    print(f"Control range: {model.actuator_ctrlrange}")

    try:
        t_start = time.time()
        while viewer.is_running():
            # Example motion: Simple sinusoidal movement for all joints
            t = time.time() - t_start  # Use relative time for smoother motion
            
            positions = np.array([
                1 * np.sin(t),      # L_Hip_Roll
                1 * np.sin(t),      # L_Hip_Yaw
                1 * np.sin(t + 1),  # L_Hip_Pitch
                1 * np.sin(t),      # L_Knee_Pitch
                1 * np.sin(t - 1),  # L_Ankle_Pitch
                1 * np.sin(t),      # R_Hip_Roll
                1 * np.sin(t),      # R_Hip_Yaw
                1 * np.sin(t + 1),  # R_Hip_Pitch
                1 * np.sin(t),      # R_Knee_Pitch
                1 * np.sin(t - 1),  # R_Ankle_Pitch
            ])

            # Set the position targets for all actuators
            data.ctrl[:] = positions

            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Print current positions every 5 seconds
            if int(t) % 5 == 0:
                print(f"Time: {t:.2f}, Control signals: {data.ctrl}")
            
            # Update the viewer
            viewer.sync()
            
            # Add a small delay to control simulation speed
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    finally:
        viewer.close()

if __name__ == "__main__":
    main()