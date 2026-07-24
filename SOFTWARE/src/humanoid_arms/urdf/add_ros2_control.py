import xml.etree.ElementTree as ET

ET.register_namespace('', '')

tree = ET.parse('humanoid_arms_collision.urdf')
root = tree.getroot()

joints = [
    "leftarmyaw", "leftelbowpitch", "leftforearmyaw", "leftgripper",
    "leftshoulderpitch", "leftshoulderroll", "leftwristpitch", "leftwristroll",
    "rightarmyaw", "rightelbowpitch", "rightforearmyaw", "rightgripper",
    "rightshoulderpitch", "rightshoulderroll", "rightwristpitch", "rightwristroll",
]

ros2_control = ET.SubElement(root, 'ros2_control', name="humanoid_arms", type="system")

hardware = ET.SubElement(ros2_control, 'hardware')
plugin = ET.SubElement(hardware, 'plugin')
plugin.text = 'gazebo_ros2_control/GazeboSystem'

for joint_name in joints:
    j = ET.SubElement(ros2_control, 'joint', name=joint_name)
    ET.SubElement(j, 'command_interface', name='position')
    ET.SubElement(j, 'state_interface', name='position')
    ET.SubElement(j, 'state_interface', name='velocity')

ET.indent(root, space='  ')
tree.write('humanoid_arms_ros2_control.urdf', xml_declaration=True, encoding='unicode')
print("Done — ros2_control block added with 16 joints")
