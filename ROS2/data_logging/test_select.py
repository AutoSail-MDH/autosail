import launch
from datetime import datetime

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

def generate_launch_description(topics):
    now = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', 'critical_data_' + now, topics],
            output='screen'
        )
    ])


def main():
    filename = "select_critical_topics.txt"
    topics = select_topics(filename)
    generate_launch_description(topics)

if __name__ == '__main__':
    main()