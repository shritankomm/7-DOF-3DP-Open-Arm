import xml.etree.ElementTree as ET

tree = ET.parse('humanoid_arms_final.urdf')
root = tree.getroot()

gazebo = ET.SubElement(root, 'gazebo')
plugin = ET.SubElement(gazebo, 'plugin',
    filename='gz_ros2_control-system',
    name='gz_ros2_control::GazeboSimROS2ControlPlugin')
params = ET.SubElement(plugin, 'parameters')
params.text = 'CONTROLLERS_PATH'

ET.indent(root, space='  ')
tree.write('humanoid_arms_final.urdf', xml_declaration=True, encoding='unicode')
print("Done — Gazebo plugin block added")
