class Find_Bag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.bag_goal = None
        self.bag_direction = None
        self.find_bag_sub = rospy.Subscriber("/pick_centroid", String, self.detect_bag_callback)
        self.bag_direction_sub = rospy.Subscriber("/arm_status", String, self.detect_direction_callback)
        self.attempt = 0
        self.bag_look_direction = None
        self.move_pose = [0.7,-0.98]
        self.LookToPoint = FollowPoint()
        
    def detect_bag_callback(self, msg):
        self.bag_goal = msg.data
        if self.bag_goal is not None:
            self.bag_found = True
        else:
            self.bag_found = False
        
    def detect_direction_callback(self, msg):
        self.bag_direction = msg.data
        if self.bag_direction == "left":
            self.bag_look_direction = -1.0
        elif self.bag_direction == "right":
            self.bag_look_direction = 1.0
            
    def continuous_move_head(self, look_direction, stop_event, interval=1.0):
        while not stop_event.is_set():
            move_head(look_direction)
            rospy.sleep(interval)
            
    def move_head(self):
        stop_event = threading.Event()
        look_direction = [self.move_pose[0] * self.bag_look_direction, self.move_pose[1]]
        head_thread = threading.Thread(target=self.continuous_move_head, args=(look_direction, stop_event))
        head_thread.start()
    
    def FindBag(self):
        if self.bag_direction is not None:
            response_something("I am looking for the bag on the %s side" % self.bag_direction)      
        self.move_head()
        self.LookToPoint
        while not self.bag_found:
            if self.attempt > 3:
                return False
            else:
                self.attempt += 1
                rospy.sleep(5)
                continue
        return True

    def execute(self, userdata):
        rospy.loginfo('Executing state InitPosition')
        if self.FindBag(): 
            rospy.loginfo('Robot succeeded to Find Bag')
            return 'succeeded'
        else:
            rospy.loginfo('Robot not succeeded to Find Bag')
            return 'aborted'