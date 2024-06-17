
import rospy
import rosnode


class LogThisTopic():
    def __init__(self, topic, msgtype):
        self.topic = topic
        self.msgtype = msgtype




def main():

    rospy.init_node("test")

    topic = "/imu"
    msgtype = "Imu"
    test1 = LogThisTopic(topic, msgtype)

    rospy.spin()

if __name__=="__main__":
    main()