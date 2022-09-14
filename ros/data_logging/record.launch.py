from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess

# Target FILENAME containing topics
FILENAME = "select_topics.txt"
# Output directory tag
BAGNAME = "captured"
# Timestamp
TIMESTAMP = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
DIR_NAME = BAGNAME + "_" + TIMESTAMP

def generate_launch_description():
    """Record predefined topics from file
    Args:
        GLOBAL topics (str): Preprocessed topic names in standard ROS2 format
        GLOBAL TIMESTAMP (str): Time format - yyyy_mm_dd-H_M_S
    """
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', DIR_NAME, import_topics_from(FILENAME)],
            shell=True, output='screen'
        )
    ])


def import_topics_from(filename: str):
    '''Read topics from file and return correct ROS2 string format
    Args:
        FILENAME (str): FILENAME containing topic names separated by rows
    Returns:
        str: "{'/topic1','/topic2', ..."}
    '''
    lines = []
    topics = []
    try:
        # Read topics from list, ignore blank
        with open(filename) as f:
            lines = list(line for line in (l.strip() for l in f) if line)
        # Remove '\n'
        is_zero_idx_slash = 0
        for x in lines:
            # Check if line starts with '/'
            if x[is_zero_idx_slash] == '/':
                topics.append(x.replace("\n", ""))
            else:
                raise TypeError("Wrong format: Topics start with '/'")
        topics = ' '.join(topics)
        return topics
            
    except FileNotFoundError:
        print(f"{FILENAME}: File Not Found, correct FILENAME/path?")