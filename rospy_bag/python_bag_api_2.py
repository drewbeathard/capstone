import rosbag


if __name__ == '__main__':
    bag = rosbag.Bag('/home/drew/bagfiles/final_testing_bags/turtle_test.bag')

    for (topic, msg, t) in bag.read_messages():
        print(topic, msg, t)