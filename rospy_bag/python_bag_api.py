# example bag read

import rosbag
print('initializing bag')
bag = rosbag.Bag('/home/drew/bagfiles/final_testing_bags/D01_final.bag.active')
for topic, msg, t in bag.read_messages(topics=['D01/task']):
    print(topic, msg, t)
bag.close()