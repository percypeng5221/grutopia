import os
import requests
import threading
import cv2
import queue
from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from utils import generate_first_payload
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

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

# Initialize the simulation environment in headless mode
env = BaseEnv(SIM_CONFIG, headless=False)
task_name = env.config.tasks[0].name
robot_name = env.config.tasks[0].robots[0].name

# Define actions (customize as needed)
actions = {'h1': {'move_with_keyboard': []}}

# GPT Worker Thread Definition
def gpt_worker(task_queue, new_task_event, stop_event):
    """
    Worker thread to handle GPT API requests sequentially.
    """
    while not stop_event.is_set():
        # Wait until there is a new task or stop_event is set
        new_task_event.wait()

        if stop_event.is_set():
            break

        try:
            # Get the new task (RGB image)
            rgb_image = task_queue.get(timeout=1)
        except queue.Empty:
            new_task_event.clear()
            continue  # No task available

        # Generate payload and make API request
        tmp_payload = generate_first_payload(rgb_image)
        try:
            response = requests.post(
                "https://api.openai.com/v1/chat/completions",
                headers=HEADERS,
                json=tmp_payload
            )
            response.raise_for_status()  # Raise an error for bad status codes
            response_json = response.json()
        except requests.RequestException as e:
            print(f"API request failed: {e}")
            task_queue.task_done()
            new_task_event.clear()
            continue

        # Process the response to extract URDF content
        content = response_json.get("choices", [{}])[0].get("message", {}).get("content", "")
        urdf_start = content.find("<robot")
        urdf_end = content.rfind("</robot>") + len("</robot>")

        if urdf_start != -1 and urdf_end != -1:
            urdf_content = content[urdf_start:urdf_end]
            urdf_file_path = os.path.join(_CUR_DIR, "output.urdf")
            try:
                with open(urdf_file_path, "w") as urdf_file:
                    urdf_file.write(urdf_content)
                print(f"URDF file saved as '{urdf_file_path}'.")
            except IOError as e:
                print(f"Failed to write URDF file: {e}")
        else:
            print("URDF content not found in the response.")

        print("Processed a GPT request.")
        task_queue.task_done()
        new_task_event.clear()  # Indicate that the worker is ready for a new task

# Main Simulation Loop
def main():
    # Initialize a queue for tasks using the correct queue module
    task_queue = queue.Queue()

    # Events to signal new tasks and stop
    new_task_event = threading.Event()
    stop_event = threading.Event()

    # Start the GPT worker thread
    worker_thread = threading.Thread(target=gpt_worker, args=(task_queue, new_task_event, stop_event))
    worker_thread.start()

    i = 0

    try:
        while env.simulation_app.is_running():
            i += 1
            env_actions = [actions]  # Simplified list append
            obs = env.step(actions=env_actions)

            # Retrieve the RGB image
            rgb = cv2.cvtColor(obs[task_name][robot_name]['camera']['rgba'], cv2.COLOR_RGBA2BGR)
            print("position and rotation: ", obs[task_name][robot_name]['position'], quat_to_euler_angles(obs[task_name][robot_name]['orientation']))
            # Check if the worker is ready for a new task
            if not new_task_event.is_set() and task_queue.empty():
                # Enqueue the current RGB image for GPT processing
                task_queue.put(rgb)
                new_task_event.set()
                print(f"Enqueued iteration {i} for GPT processing.")

            # Optional: Add a small sleep to prevent tight looping if necessary
            # time.sleep(0.01)

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")

    finally:
        # Signal the worker thread to stop and wait for it
        stop_event.set()
        new_task_event.set()  # Wake the worker if it's waiting
        worker_thread.join()
        env.simulation_app.close()
        print("Simulation terminated gracefully.")

if __name__ == "__main__":
        main()