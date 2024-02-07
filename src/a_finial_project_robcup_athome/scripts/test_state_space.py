class FindHuman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.find_human_sub = rospy.Subscriber('/goal_centeroid', String, self.find_human_callback)
        self.human_goal = None
        self.attempt = 0
        
    def find_human_callback(self, msg):
        self.human_goal = msg.data
        
    def FindHuman(self):
        while self.human_goal is None:
            if self.attempt > 3:
                return False
            else:
                # say something
                self.attempt += 1
                rospy.sleep(1)
                continue
        return True