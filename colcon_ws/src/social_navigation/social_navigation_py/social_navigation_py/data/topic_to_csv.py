# Pulled from github.com/ros2/rosbag2/issues/473
# Message by nayak2358
import csv
import rosbag_api as bag
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

path_to_workspace = '/home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/data'
filename = '6_27_24'
bag_path = f'{path_to_workspace}/bag_data/{filename}.db3'
topic_name = '/robot1/robot_location'
csv_path = f'{path_to_workspace}/deserialized_data/{filename}.csv'

### connect to the database
conn, c = bag.connect(bag_path)

### get all topics names and types
topic_names = bag.getAllTopicsNames(c, print_out=False)
topic_types = bag.getAllMsgsTypes(c, print_out=False)

### Create a map for quicker lookup
type_map = {topic_names[i]:topic_types[i] for i in range(len(topic_types))}

### get all timestamps and all messages
t, msgs = bag.getAllMessagesInTopic(c, topic_name, print_out=False)

### To extract the attributes
def extract_fields(message):
    extracted_fields = []

    def recursive_extract(msg, parent_field=''):
        slots_without_underscores = [field[1:] if field.startswith('_') else field for field in msg.__slots__]
        for attr_name in slots_without_underscores:
            attr_value = getattr(msg, attr_name)
            if hasattr(attr_value, '__slots__'):
                recursive_extract(attr_value, f"{parent_field}.{attr_name}" if parent_field else attr_name)
            else:
                extracted_fields.append(f"{parent_field}.{attr_name}" if parent_field else attr_name)

    recursive_extract(message)
    
    return extracted_fields

### Open CSV file for writing
with open(csv_path, mode='w', newline='') as file:
    writer = csv.writer(file)

    ### Write headers
    ### Extract message structure to dynamically determine headers
    msg_type = get_message(type_map[topic_name])
    sample_msg = deserialize_message(msgs[0], msg_type)
    timestamp = sample_msg.header.stamp.sec + sample_msg.header.stamp.nanosec / 1e9
    
    headers = extract_fields(sample_msg)
    writer.writerow(['Time'] + headers)
    
    ### Debugging
    print(sample_msg)
    print(sample_msg.header.stamp.sec)
    print(eval(f"sample_msg.{headers[0]}"))
    
    ### Deserialize messages and write to CSV
    for msg in msgs:
        deserialized_msg = deserialize_message(msg, msg_type)
        timestamp = deserialized_msg.header.stamp.sec + deserialized_msg.header.stamp.nanosec / 1e9
        row = [timestamp] + [eval(f"sample_msg.{field}") for field in headers]
        writer.writerow(row)

### Close connection to the database
bag.close(conn)

print("Deserialized messages saved to", csv_path)