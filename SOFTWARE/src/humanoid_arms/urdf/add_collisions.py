import xml.etree.ElementTree as ET
import copy

ET.register_namespace('', 'http://www.w3.org/2001/XMLSchema')

tree = ET.parse('humanoid_arms_fixed.urdf')
root = tree.getroot()

added = 0
skipped = 0

for link in root.findall('link'):
    visual = link.find('visual')
    if visual is None:
        skipped += 1
        continue
    if link.find('collision') is not None:
        skipped += 1
        continue

    collision = ET.SubElement(link, 'collision')
    origin = visual.find('origin')
    if origin is not None:
        collision.append(copy.deepcopy(origin))
    geometry = visual.find('geometry')
    if geometry is not None:
        collision.append(copy.deepcopy(geometry))
    added += 1

ET.indent(root, space='  ')
tree.write('humanoid_arms_collision.urdf', xml_declaration=True, encoding='unicode')
print(f"Done — added collision to {added} links, skipped {skipped}")
