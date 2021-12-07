#  import launch
from datetime import datetime

# Target filename containing topics
filename = "select_topics.txt"
bagname = "captured"
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
                 import_topics_from(filename), bagname + '_' + timestamp],
            output='screen'
        )
    ])

def format_topics(topics: list) -> str:
        # Format into bag format for multiple topics
        is_num_topics = len(topics)
        topics = ' '.join(topics)
        # Check if number of topics is greater than 1
        if is_num_topics > 1:  
            # Format: "/topic1 /topic2 ..." -> "{'/topic1','/topic2', ..."}
            topics = topics.replace("/", "'/")
            topics = topics.replace(" ", "',")
            topics = "{" + topics + "'}"
            return topics
        else: # Single topics format
            return topics


def import_topics_from(filename: str):
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
        return format_topics(topics)
            
    except FileNotFoundError:
        print(f"{filename}: File Not Found, correct filename/path?")

print(import_topics_from(filename))
