class Move_to_Bag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.bag_follow = BagFollow()
        self.bag_status = rospy.Subsciber("/bag_status", String, self.bag_status_callback)
        self.move_success = False
        
    def bag_status_callback(self, msg):
        self.bag_status = msg.data
        if self.bag_status == "Success":
            self.move_success = True
        else:
            self.move_success = False
        
    def Move_to_Bag(self):
        while self.move_success is False:
            rospy.sleep(5)
            continue

    def execute(self, userdata):
        rospy.loginfo('Executing state MovetoBag')

        if self.Move_to_Bag(): 
            rospy.loginfo('Robot succeeded to MovetoBag')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to MovetoBag')
            return 'aborted'