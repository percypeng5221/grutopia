import os
import requests
import cv2
import json
import re
import numpy as np
from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from utils import generate_first_payload_w_pos, generate_follow_up_payload_w_pos, encode_image_from_array

# Configuration and Setup
_CUR_DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_FILE_PATH = '/home/percy/princeton/GRUtopia/demo/configs/h1_house.yaml'
SIM_CONFIG = SimulatorConfig(CONFIG_FILE_PATH)

# OpenAI API Key - Use environment variable for security
API_KEY = os.getenv("OPENAI_API_KEY")
if not API_KEY:
    raise ValueError("Please set the OPENAI_API_KEY environment variable.")

HEADERS = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {API_KEY}"
}

# Initialize the simulation environment
env = BaseEnv(SIM_CONFIG, headless=False)
task_name = env.config.tasks[0].name
robot_name = env.config.tasks[0].robots[0].name
from omni.isaac.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat

# Define actions
actions = {'h1': {'move_along_path': [[SIM_CONFIG.config.tasks[0].robots[0].position]]}}

def process_gpt_response(rgb_image, robot_state, previous_outputs, conversation_history, first_iteration=False):
    """
    Process the GPT response with the given RGB image, robot state, and conversation history.
    """
    robot_position, robot_rotation = robot_state

    # Generate payload
    if first_iteration:
        tmp_payload = generate_first_payload_w_pos(rgb_image, robot_position, robot_rotation)
    else:
        tmp_payload = generate_follow_up_payload_w_pos(
            rgb_image, robot_position, robot_rotation, previous_outputs, conversation_history
        )

    # Send request to GPT API
    response = requests.post(
        "https://api.openai.com/v1/chat/completions",
        headers=HEADERS,
        json=tmp_payload
    )

    try:
        response.raise_for_status()
        response_json = response.json()
    except requests.RequestException as e:
        print(f"API request failed: {e}")
        if e.response is not None:
            print(f"Response status code: {e.response.status_code}")
            print(f"Response content: {e.response.text}")
        return None, previous_outputs, conversation_history

    # Extract content from response
    content = response_json.get("choices", [{}])[0].get("message", {}).get("content", "")
    # Extract the JSON enclosed between '<<' and '>>'
    json_match = re.search(r'<<(.*?)>>', content, re.DOTALL)
    if json_match:
        json_string = json_match.group(1).strip()
        try:
            parsed_output = json.loads(json_string)
        except json.JSONDecodeError as e:
            print(f"JSONDecodeError: {e}")
            print("Invalid JSON string received:")
            print(json_string)
            return None, previous_outputs, conversation_history

        if isinstance(parsed_output, dict):
            # Save the combined output into a single JSON file
            json_file_path = os.path.join(_CUR_DIR, "gpt_output.json")
            with open(json_file_path, "w") as json_file:
                json.dump(parsed_output, json_file, indent=4)
            print(f"Output saved as '{json_file_path}'.")

            # Extract next move point and reason to return
            next_move_point = parsed_output.get('next_move_point', {})
            x = next_move_point.get("x", robot_position[0])
            y = next_move_point.get("y", robot_position[1])
            z = next_move_point.get("z", robot_position[2])
            yaw = next_move_point.get("yaw", robot_rotation[2])
            reason = next_move_point.get("reason", "")

            # Update previous outputs
            if first_iteration:
                previous_outputs['objects'] = parsed_output.get('objects', [])
                previous_outputs['next_move_points'] = [next_move_point]
            else:
                # Update objects
                new_objects = parsed_output.get('objects', [])
                previous_outputs['objects'].extend(new_objects)
                # Remove duplicates based on 'name' and 'position'
                unique_objects = {
                    (obj['name'], tuple(obj['position'].values())): obj
                    for obj in previous_outputs['objects']
                }
                previous_outputs['objects'] = list(unique_objects.values())

                # Update next move points
                previous_outputs['next_move_points'].append(next_move_point)

            # Update conversation history
            conversation_history.append({
                "role": "assistant",
                "content": content
            })

            # Print the reason for the next move point
            if reason:
                print(f"Reason for next move: {reason}")

            return {'x': x, 'y': y, 'z': z, "yaw": yaw}, previous_outputs, conversation_history
        else:
            print("Invalid JSON format received.")
            return None, previous_outputs, conversation_history
    else:
        print("No JSON content found in the response.")
        return None, previous_outputs, conversation_history

def main():
    i = 0
    yaw = 0.0
    path_finished = False
    actions['h1'] = {'move_along_path': [[SIM_CONFIG.config.tasks[0].robots[0].position]]}
    conversation_history = []  # Initialize conversation history
    previous_outputs = {'objects': [], 'next_move_points': []}  # Initialize previous outputs

    # Create a named window for displaying the camera input
    cv2.namedWindow('Robot Camera Feed', cv2.WINDOW_NORMAL)

    while env.simulation_app.is_running():
        i += 1
        env_actions = [actions]
        obs = env.step(actions=env_actions)

        # Retrieve the RGB image and the robot's state
        rgb = cv2.cvtColor(obs[task_name][robot_name]['camera']['rgba'], cv2.COLOR_RGBA2BGR)
        robot_position = tuple(float(x) for x in obs[task_name][robot_name]['position'])
        robot_orientation = obs[task_name][robot_name]['orientation']
        robot_rotation = tuple(float(x) for x in quat_to_euler_angles(robot_orientation))
        path_finished = obs[task_name][robot_name]['move_along_path'].get('finished', False)
        rotate_finished = obs[task_name][robot_name]['rotate'].get('finished', False)
        # Display the RGB image in a separate window
        cv2.imshow('Robot Camera Feed', rgb)

        # Check if 'q' is pressed to exit the simulation
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if path_finished:
            rotate_count = 0
            while not rotate_finished and rotate_count < 500:
                rotate_count += 1
                rotate_finished = obs[task_name][robot_name]['rotate'].get('finished', False)
                actions["h1"] = {'rotate': [euler_angles_to_quat(np.array([0, 0, yaw]))]}
                env_actions = [actions]
                obs = env.step(actions=env_actions)
                rgb = cv2.cvtColor(obs[task_name][robot_name]['camera']['rgba'], cv2.COLOR_RGBA2BGR)
                robot_position = tuple(float(x) for x in obs[task_name][robot_name]['position'])
                robot_orientation = obs[task_name][robot_name]['orientation']
                robot_rotation = tuple(float(x) for x in quat_to_euler_angles(robot_orientation))
                # Display the RGB image in a separate window
                cv2.imshow('Robot Camera Feed', rgb)

                # Check if 'q' is pressed to exit the simulation
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            print("Finished rotating")
            print(f"Iteration: {i}")

            # Process the GPT response
            first_iteration = (i == 1)
            response_data, previous_outputs, conversation_history = process_gpt_response(
                rgb, (robot_position, robot_rotation), previous_outputs, conversation_history, first_iteration
            )

            if response_data:
                x = response_data.get("x", robot_position[0])
                y = response_data.get("y", robot_position[1])
                z = response_data.get("z", robot_position[2])
                yaw = response_data.get("yaw", robot_rotation[2])
                reason = response_data.get("reason", "")
                print(f"Next move point updated - x: {x}, y: {y}, z: {z}, yaw: {yaw}")
                if reason:
                    print(f"Reason for next move: {reason}")

                # Update the actions with new move point
                actions['h1'] = {'move_along_path': [[(x, y, z)]]}

    # Release the OpenCV window resources and close the window
    cv2.destroyAllWindows()
    env.simulation_app.close()
    print("Simulation terminated gracefully.")


if __name__ == "__main__":
    main()