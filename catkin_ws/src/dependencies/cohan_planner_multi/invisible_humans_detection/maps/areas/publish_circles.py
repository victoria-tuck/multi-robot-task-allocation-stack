import yaml
import rospy
from visualization_msgs.msg import Marker, MarkerArray

with open('maze.yaml', 'r') as file:
  document = yaml.load(file)

rospy.init_node('publish_centers')
pub_ = rospy.Publisher('centers', MarkerArray, queue_size = 1)
marker_array = MarkerArray()

while not rospy.is_shutdown():
  m_id = 0
  for i in range(0, len(document['centers'])):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = m_id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = document['centers'][i][0]
    marker.pose.position.y = document['centers'][i][1]
    marker.pose.position.z = 0.0
    # t = rospy.Duration(0.2)
    # marker.lifetime = t
    marker.scale.x = document['radii'][i]
    marker.scale.y = document['radii'][i]
    marker.scale.z = 0.0
    marker.color.a = 0.4
    marker.color.b = 1.0
    marker_array.markers.append(marker)

    marker = Marker()
    marker.text = str(i) 
    m_id = m_id+1
    marker.header.frame_id = "map"
    marker.id = m_id
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = document['centers'][i][0]
    marker.pose.position.y = document['centers'][i][1]
    marker.pose.position.z = 0.0
    # t = rospy.Duration(0.2)
    # marker.lifetime = t
    # marker.scale.x = 0
    # marker.scale.y = 0
    marker.scale.z = 2.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker_array.markers.append(marker)
    m_id = m_id+1

  pub_.publish(marker_array)
  rospy.sleep(0.1)
# rospy.spin()
