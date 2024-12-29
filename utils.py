import base64
import cv2
import json

# OpenAI API Key
api_key = " "
headers = {
  "Content-Type": "application/json",
  "Authorization": f"Bearer {api_key}"
}


def generate_first_payload(image_array):
    base64_image = encode_image_from_array(image_array)
    payload = {
        "model": "gpt-4o",
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "You are required to navigate through a specific environment. Can you write an urdf file where all the links are connected to each other to represent the scene? "
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_image}",
                            "detail": "high"
                        }
                    }
                ]
            }
        ],
        "max_tokens": 4096
    }
    
    return payload

def generate_second_payload(first_response, base_info, object_info):
    """
    Generates the payload for the second request to the LLM.

    Args:
    - first_response (str): The response from the first request to the LLM.
    - base_info (list[dict]): The list of dictionaries containing base position, quaternion, and Euler angles.
    - object_info (list[dict]): The list of dictionaries containing the objects' information.

    Returns:
    - dict: The payload for the second request.
    """
    combined_content = f"{first_response}\n\nThe specific object type, dimension and height and the pose and rotation of all the objects are given to you. Remember, you need to go through this environment and you are at (3, 0, 3) position. Select the necessary objects and adjust the urdf file accordingly. Don't forget to add a very very small base link to connect all the objects! Remember to add collision! Please add explanation before adding an object describing why you think this object is neccessary to be included in the urdf file!\n\n"

    # Combine base_info with object_info
    for i, (base, obj) in enumerate(zip(base_info, object_info)):
        combined_content += (
            f"Object {i+1}:\n"
            f"Type: {obj['type']}\n"
            f"Object Info:\n"
            f"  - Dimensions: {obj.get('size', obj.get('radius', ''))}\n"
            f"  - Height: {obj.get('height', '')}\n"
            f"Object Position: {base['position']}\n"
            f"Object Euler Angles: {base['euler_angles']}\n\n"
        )

    second_payload = {
        "model": "gpt-4o",  # or your desired model
        "messages": [
            {"role": "system", "content": "You are an assistant that continues conversations."},
            {"role": "user", "content": combined_content}
        ]
    }

    return second_payload

def encode_image_from_array(image_array):

    _, buffer = cv2.imencode('.jpg', image_array)
    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
    return jpg_as_text

def generate_first_payload(image_array):
    base64_image = encode_image_from_array(image_array)
    payload = {
        "model": "gpt-4o",
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": "You are required to navigate through a specific environment. Can you write an urdf file where all the links are connected to each other to represent the scene? "
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_image}",
                            "detail": "high"
                        }
                    }
                ]
            }
        ],
        "max_tokens": 4096
    }
    
    return payload

def generate_first_payload_w_pos(image_array, robot_position, robot_rotation):
    base64_image = encode_image_from_array(image_array)
    
    position_str = f"Position: (x: {robot_position[0]}, y: {robot_position[1]}, z: {robot_position[2]})"
    rotation_str = f"Rotation (Euler angles): (roll: {robot_rotation[0]}, pitch: {robot_rotation[1]}, yaw: {robot_rotation[2]})"
    
    # Update messages to include the 'reason' for 'next_move_point' in the instructions and example
    messages = [
        {
            "role": "user",
            "content": [
                {
                    "type": "text",
                    "text": (
                        "You are assisting in robot navigation. Based on the robot's current position and orientation, analyze the environment and provide a single valid JSON output containing:\n\n"
                        "1. An 'objects' key with a **cumulative** list of significant objects the robot has encountered during navigation. Each object should be a dictionary with the keys:\n"
                        "   - 'name' (string): the name of the object.\n"
                        "   - 'position' (dictionary): with numeric keys 'x', 'y', and 'z' representing the object's position.\n"
                        "   - 'reason' (string): why the object is significant.\n\n"
                        "2. A 'next_move_point' key with the following keys:\n"
                        "   - 'x' (number): the x-coordinate of the desired next position.\n"
                        "   - 'y' (number): the y-coordinate.\n"
                        "   - 'z' (number): the z-coordinate (height).\n"
                        "   - 'yaw' (number): the desired orientation angle in radians.\n"
                        "   - 'reason' (string): why this move point is chosen.\n\n"
                        f"**Example format:**\n"
                        "{{\n"
                        "    \"objects\": [\n"
                        "        {{\n"
                        "            \"name\": \"table\",\n"
                        "            \"position\": {{\n"
                        "                \"x\": 5.5,\n"
                        "                \"y\": 1.2,\n"
                        "                \"z\": 0.8\n"
                        "            }},\n"
                        "            \"reason\": \"potential obstacle\"\n"
                        "        }}\n"
                        "    ],\n"
                        "    \"next_move_point\": {{\n"
                        "        \"x\": 4.9,\n"
                        "        \"y\": -0.4,\n"
                        "        \"z\": 1.0,\n"
                        "        \"yaw\": -1.7,\n"
                        "        \"reason\": \"to avoid the table and proceed towards goal\"\n"
                        "    }}\n"
                        "}}\n\n"
                        "**Important Instructions:**\n"
                        "- Provide the output as **valid JSON**.\n"
                        "- **Do not include any additional text** outside the JSON output.\n"
                        "- **Ensure numeric values are not in quotes or parentheses**.\n"
                        "- Use proper commas and colons.\n"
                        "- Enclose the JSON output between '<<' and '>>'.\n"
                        f"{position_str}\n{rotation_str}\n"
                    )
                },
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}"
                    }
                }
            ]
        }
    ]
    
    payload = {
        "model": "gpt-4o",  # Replace with your desired model
        "messages": messages,
        "max_tokens": 4096
    }
    
    return payload

def generate_follow_up_payload_w_pos(image_array, robot_position, robot_rotation, previous_outputs, conversation_history):
    base64_image = encode_image_from_array(image_array)
    
    position_str = f"Position: (x: {robot_position[0]}, y: {robot_position[1]}, z: {robot_position[2]})"
    rotation_str = f"Rotation (Euler angles): (roll: {robot_rotation[0]}, pitch: {robot_rotation[1]}, yaw: {robot_rotation[2]})"
    
    # Include previous outputs in the prompt
    previous_objects_str = json.dumps(previous_outputs.get('objects', []), indent=4)
    previous_move_points_str = json.dumps(previous_outputs.get('next_move_points', []), indent=4)
    
    # Current user message with example
    user_message_content = (
        "You are assisting in robot navigation. Based on the robot's current position and orientation, and considering the previous objects and move points, analyze the environment and provide a single valid JSON output containing:\n\n"
        "1. An 'objects' key with a **cumulative** list of significant objects the robot has encountered during navigation **including previous objects**. Each object should be a dictionary with the keys:\n"
        "   - 'name' (string): the name of the object.\n"
        "   - 'position' (dictionary): with numeric keys 'x', 'y', and 'z' representing the object's position.\n"
        "   - 'reason' (string): why the object is significant.\n\n"
        "2. A 'next_move_point' key with the following keys:\n"
        "   - 'x' (number): the x-coordinate of the desired next position.\n"
        "   - 'y' (number): the y-coordinate.\n"
        "   - 'z' (number): the z-coordinate (height).\n"
        "   - 'yaw' (number): the desired orientation angle in radians.\n"
        "   - 'reason' (string): why this move point is chosen.\n\n"
        f"**Example format:**\n"
        "{{\n"
        "    \"objects\": [\n"
        "        {{\n"
        "            \"name\": \"table\",\n"
        "            \"position\": {{\n"
        "                \"x\": 5.5,\n"
        "                \"y\": 1.2,\n"
        "                \"z\": 0.8\n"
        "            }},\n"
        "            \"reason\": \"potential obstacle\"\n"
        "        }}\n"
        "    ],\n"
        "    \"next_move_point\": {{\n"
        "        \"x\": 4.985,\n"
        "        \"y\": -0.401,\n"
        "        \"z\": 1.0,\n"
        "        \"yaw\": -1.71405583285261,\n"
        "        \"reason\": \"to avoid the table and proceed towards goal\"\n"
        "    }}\n"
        "}}\n\n"
        "**Important Instructions:**\n"
        "- Provide the output as **valid JSON**.\n"
        "- **Include all previously detected objects in the 'objects' list, adding any new ones you've identified.**\n"
        "- **Do not include any additional text** outside the JSON output.\n"
        "- **Ensure numeric values are not in quotes or parentheses**.\n"
        "- Use proper commas and colons.\n"
        "- Enclose the JSON output between '<<' and '>>'.\n"
        f"Previous Objects:\n{previous_objects_str}\n\n"
        f"Previous Next Move Points:\n{previous_move_points_str}\n\n"
        f"{position_str}\n{rotation_str}\n"
    )
    
    # Create the message history with previous conversation
    messages = conversation_history.copy()
    
    # Append the current user message with text and image
    messages.append({
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": user_message_content
            },
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{base64_image}"
                }
            }
        ]
    })
    
    payload = {
        "model": "gpt-4o",  # Replace with your desired model
        "messages": messages,
        "max_tokens": 4096
    }
    
    return payload