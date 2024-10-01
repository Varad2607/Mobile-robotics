import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

# BEGIN QUESTION 2.3
"*** REPLACE THIS LINE ***"
# END QUESTION 2.3


def norm_python(data):
    """Compute the norm for each row of a numpy array using Python for loops.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_python(data)
    array([ 5., 13.])
    """
    n, d = data.shape
    norm = np.zeros(n)
    # BEGIN QUESTION 2.1
    for i in range(0,n,1):
        sum=0.0
        for j in range (0,d,1):
            sum+=data[i][j]**2
        norm[i]=sum**0.5
        

    
    # END QUESTION 2.1
    return norm


def norm_numpy(data):
    """Compute the norm for each row of a numpy array using numpy functions.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_numpy(data)
    array([ 5., 13.])
    """
    # You can call np.sqrt, np.sum, np.square, etc.
    # Hint: you may find the `axis` parameter useful.
    # BEGIN QUESTION 2.2
    data=np.sqrt(np.sum(np.power(data,2),axis=1))
    
    return data

class PoseListener:
    """Collect car poses."""

    def __init__(self, size=100):
        self.size = size
        self.done = False
        self.storage = []  # a list of (x, y) tuple
        self.hdr_plt=[]
        # Create a subscriber for the car pose.
        # Hint: once you've figured out the right message type, don't forget to
        # import it at the top! If the message type from `rostopic info` is
        # "X_msgs/Y", the Python import would be "from X_msgs.msg import Y".
        # BEGIN QUESTION 2.3
        "*** REPLACE THIS LINE ***"
        self.subscriber = rospy.Subscriber("/car/car_pose", PoseStamped, self.callback)
        # END QUESTION 2.3

    def callback(self, msg):
        """Store the x and y coordinates of the car."""
        hdr = msg.header
        pos=msg.pose
        rospy.loginfo(
            "Received a new message with timestamp " + str(hdr.stamp.secs) + "(s)"
        )
        # Extract and store the x and y position from the message data
        # BEGIN QUESTION 2.4
        self.storage.append((pos.position.x,pos.position.y))
        self.hdr_plt.append(hdr.stamp.secs)
        # END QUESTION 2.4
        if len(self.storage) == self.size:
            self.done = True
            rospy.loginfo("Received enough samples, trying to unsubscribe")
            self.subscriber.unregister()
