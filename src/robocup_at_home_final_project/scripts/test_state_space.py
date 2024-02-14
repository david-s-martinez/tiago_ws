class Move_to_Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Move to Human')
        try:
            subprocess.call(['rosrun', 'robocup_at_home_final_project', 'tiago_human_follow.py'])
            rospy.loginfo('Robot succeeded to Move to Bag')
            return 'succeeded'
        except:
            rospy.loginfo("FAILED TO LAUNCH")
            return 'aborted'