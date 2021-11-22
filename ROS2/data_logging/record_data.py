import launch
from datetime import datetime
filename = "select_critical_topics.txt"

def select_topics(filename):
    read_topics = []
    topics = []
    # Read critical topics from list
    with open(filename) as f:
        read_topics = f.readlines()
    # Remove '\n'
    for x in read_topics:
        topics.append(x.replace("\n", ""))
    # Format into bag format
    topics = ' '.join(topics)
    return topics

def generate_launch_description():
    now = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'critical_data_' + now, select_topics(filename)],
            output='screen'
        )
    ])


