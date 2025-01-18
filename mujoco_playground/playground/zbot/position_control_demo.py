import mujoco
import numpy as np
import time

def main():
    # Load the model from the scene XML file instead of the robot file directly
    model = mujoco.MjModel.from_xml_path("mujoco_playground/playground/resources/zbot/scene.xml")
    data = mujoco.MjData(model)

    # Create a visualization window
    viewer = mujoco.Viewer(model, data)

    # Set initial pose to standing position
    mujoco.mj_resetDataKeyframe(model, data, 1)  # 1 corresponds to the "standing" keyframe

    try:
        while True:
            # Example motion: Simple sinusoidal movement for all joints
            t = time.time()
            
            # Define positions for all actuators
            # The order matches the actuator definitions in the XML:
            # [L_Hip_Roll, L_Hip_Yaw, L_Hip_Pitch, L_Knee_Pitch, L_Ankle_Pitch,
            #  R_Hip_Roll, R_Hip_Yaw, R_Hip_Pitch, R_Knee_Pitch, R_Ankle_Pitch]
            
            positions = np.array([
                0.3 * np.sin(t),      # L_Hip_Roll
                0.1 * np.sin(t),      # L_Hip_Yaw
                0.3 * np.sin(t + 1),  # L_Hip_Pitch
                0.5 * np.sin(t),      # L_Knee_Pitch
                0.3 * np.sin(t - 1),  # L_Ankle_Pitch
                0.3 * np.sin(t),      # R_Hip_Roll
                0.1 * np.sin(t),      # R_Hip_Yaw
                0.3 * np.sin(t + 1),  # R_Hip_Pitch
                0.5 * np.sin(t),      # R_Knee_Pitch
                0.3 * np.sin(t - 1),  # R_Ankle_Pitch
            ])

            # Set the position targets for all actuators
            data.ctrl[:] = positions

            # Step the simulation
            mujoco.mj_step(model, data)
            
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