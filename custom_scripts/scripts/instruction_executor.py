#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import subprocess
import inspect

action_finished_publisher = None

class ExtraInputError(Exception):
    def __init__(self, message="There was more input than expected."):
        self.message = message
        super(ExtraInputError, self).__init__(self.message)

def format_actions(actions_info):
    """
    Received actions line from extract_information() and returns formatted list
    of actions to take.
    """
    actions_list = actions_info.split()
    formatted_actions = []
    
    for action in actions_list:
        # Separate the number inside parentheses if present
        if "(" in action and ")" in action:
            action_name, number = action.split("(")
            number = number.replace(")", "")
            formatted_actions.append("{} {}".format(action_name.strip(), number.strip()))
        else:
            formatted_actions.append(action)
    
    return formatted_actions

def extract_information(input_string):
    """
    Received GPT response with summary, speak, and actions. The isolates 
    actions. Passes to format_actions to return correct formatting.
    """
    actions_info = ""

    # Split the input string into lines
    lines = input_string.split("\n")
    
    for line in lines:
        # Check if the line contains 'actions:'
        if "actions:" in line:
            # Extract the part after 'actions:'
            actions_info = line.split("actions:", 1)[1].strip()

    return format_actions(actions_info)

def handle_gpt_response(msg):
    """
    When receiving publish on /gpt_response, uses extract_information() to get
    formatted and isolated list of actions that can be parsed and executed.
    """
    global action_finished_publisher
    rospy.loginfo("instructions_executor.py received GPT response")
    pub = rospy.Publisher('/instructions_errors', String, queue_size=10)
    file_name = inspect.getfile(inspect.currentframe())
    
    try:
        actions_list = extract_information(msg.data)  
        rospy.loginfo("instructions_executor.py actions list:\n{}".format(actions_list))
        is_finished = False
        for command in actions_list:
            if "grab" in command:
                if is_finished:
                    raise ExtraInputError()
                if len(command) == 6:
                    location = command[5]
                else:
                    location = command[5:7]                                                             # THIS IS BAD
                rospy.loginfo("instructions_executor.py Grabbing at {}".format(location))
                subprocess.call(['rosrun', 'custom_scripts', 'grab.py', location])
            elif "place" in command:
                if is_finished:
                    raise ExtraInputError()
                if len(command) == 7:
                    location = command[6]
                else:
                    location = command[6:8]
                rospy.loginfo("instructions_executor.py Placing at {}".format(location))
                subprocess.call(['rosrun', 'custom_scripts', 'place.py', location])
            elif "done" in command:
                if is_finished:
                    raise ExtraInputError()
                rospy.loginfo("instructions_executor.py Finished with step.")
                is_finished = True
                subprocess.call(['rosrun', 'custom_scripts', 'move_arm_for_photo.py'])
                subprocess.call(['rosrun', 'custom_scripts', 'open_gripper.py'])
            else:
                rospy.loginfo("instructions_executor.py Command not recognized.")
                raise ExtraInputError("Unrecognized command")
    except ExtraInputError as e:
        error_message = "ExtraInputError in {}: {}".format(file_name, e)
        rospy.logerr(error_message)
        pub.publish(error_message)
    except Exception as e:
        error_message = "An unexpected error occurred in {}: {}".format(file_name, e)
        rospy.logerr(error_message)
        pub.publish(error_message)

    action_finished_publisher.publish("action_finished")

def main():
    global action_finished_publisher
    rospy.init_node('instruction_executor', anonymous=True)
    rospy.loginfo("instruction_executor.py entered")
    rospy.Subscriber('/gpt_response', String, handle_gpt_response)
    pub = rospy.Publisher('/instructions_errors', String, queue_size=10)
    action_finished_publisher = rospy.Publisher('/action_finished', String, queue_size=10)
    rospy.loginfo("Robot now listening to /chatgpt_response for instructions...")
    rospy.spin()
    rospy.loginfo("instruction_executor.py exiting")

if __name__ == '__main__':
    main()
