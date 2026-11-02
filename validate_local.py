"""Static checks only; not a Gazebo simulator or SDF schema validator."""
from pathlib import Path
import ast
import math
import xml.etree.ElementTree as E

p = Path(__file__).parent / 'src' / 'beni_sim'
for f in p.rglob('*.py'):
    ast.parse(f.read_text(), filename=str(f))
for f in [p/'package.xml', *p.rglob('*.sdf')]:
    E.parse(f)

def normalize(node):
    node.text = (node.text or '').strip()
    node.tail = ''
    for child in node:
        normalize(child)
    return node

a = normalize(E.parse(p/'models/beni.sdf').getroot().find('model'))
b = normalize(E.parse(p/'worlds/room.sdf').getroot().find("world/model[@name='beni']"))
assert E.tostring(a) == E.tostring(b), 'Robot model and world differ'
links = {x.attrib['name'] for x in a.findall('link')}
for joint in a.findall('joint'):
    assert joint.findtext('parent') in links and joint.findtext('child') in links
for inert in a.findall('link/inertial'):
    assert float(inert.findtext('mass')) > 0
    for key in ['ixx', 'iyy', 'izz']:
        assert float(inert.findtext('inertia/'+key)) > 0
for side in ['left', 'right']:
    # Local wheel axis +Z rotated by Rx(-pi/2) must be global +Y.
    roll = float(a.findtext(f"link[@name='{side}_wheel']/pose").split()[3])
    assert abs(-math.sin(roll)-1.0) < 1e-8, 'Wheel axis is reversed'
plugin = a.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
assert float(plugin.findtext('wheel_separation')) == .29
assert float(plugin.findtext('wheel_radius')) == .1
print('PASS: Python syntax, XML parsing, model consistency, joints, inertias, wheel axes')
