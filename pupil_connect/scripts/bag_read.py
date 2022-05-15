import rosbag

bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print(topic)
    print(msg)
    print(t)
bag.close()
