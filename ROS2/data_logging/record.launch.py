import launch
from datetime import datetime

# Target filename containing topics
filename = "select_topics.txt"
bagname = "recorded"
timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")

def generate_launch_description():
    """Record predefined topics from file
    Args:
        topics (str): Preprocessed topic names in standard ROS2 format
        timestamp (str): Time format - yyyy_mm_dd-H_M_S
    """
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
                 bagname + '_' + timestamp, import_topics_from(filename)],
            output='screen'
        )
    ])


def import_topics_from(filename):
    '''Read topics from file and return correct ROS2 string format
    Args:
        filename (str): filename containing topic names separated by rows
    Returns:
        str: Format "/topic_1 /topic_2 /topic_3"
    '''
    lines = []
    topics = []
    try:
        # Read topics from list, ignore blank
        with open(filename) as f:
            lines = list(line for line in (l.strip() for l in f) if line)
        # Remove '\n'
        for x in lines:
            # Check if line starts with '/'
            if x[0] == '/':
                topics.append(x.replace("\n", ""))
            else:
                raise TypeError("Wrong format: Topics start with '/'")
        # Format into bag format
        topics = ' '.join(topics)
        return topics
    except FileNotFoundError:
        print(f"{filename}: File Not Found, correct filename/path?")
