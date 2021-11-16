import rosbag


if __name__ == '__main__':
    bag = rosbag.Bag('/mnt/1tb/combined_bag/combine_lidar_and_odom.bag')

    for (topic, msg, t) in bag.read_messages(topics = ['/H01/odometry', '/H01/horiz/os_cloud_node/points']):
        #print(topic, msg, t)

        print('Topic: ' + str(topic))
        #print('Message:' + str(msg))
        print('t: ' +str(t) + ' \n\n\n\n\n\n\n')