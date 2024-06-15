
import rospy

class TopicLogger():
    def __init__(self, topic_string):
        self.topic = topic_string
        try:
            self.msgtype = rospy.get_published_topics(namespace=self.topic)[0][1]
        except:
            pass
        # def function -> create empty arrays for topic data
        rospy.Subscriber(self.topic, self.msgtype, self.callback_logger)

    #
    def callback_logger(msg):
        pass


    







def main():

    # Initialize node:
    


    test1 = TopicLogger("/vicon/burger2/burger2") # msg type -> TransformStamped
    # ...
    # ...



    # eventually write csv file (can figure out path later)
    # name e.g., "/vicon/burger2/burger2" -> "vicon_burger2_burger2.csv"



if __name__=="__main__":
    main()