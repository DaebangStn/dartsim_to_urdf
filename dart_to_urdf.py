import os
from xml.etree import ElementTree as ET
from xml.dom import minidom
import transformations as tf
import numpy as np

input_dart = "data/skeleton.xml"
output_urdf = "data/skeleton.urdf"
relative_path_of_obj_to_urdf = "scaled"


class Node:
    def __init__(self, parent, _name, transformation_joint):
        self.parent = parent
        self.name = _name
        self.trJnt, self.trGlobal = self._compute_tr(transformation_joint)

    def _compute_tr(self, transformation_xml):
        if self.parent is None:
            trParentGlobal = tf.identity_matrix()
        else:
            trParentGlobal = self.parent.trGlobal
        if trParentGlobal is None:
            raise ValueError(f"Node[{self.name}]'s Parent[{self.parent.name}] has no transformation matrix")

        trGlobal = Node.trans_to_mtx(transformation_xml)
        trJnt = trGlobal @ np.linalg.inv(trParentGlobal)  # Ji * Ji-1 ... = Global

        return trJnt, trGlobal

    @staticmethod
    def trans_to_mtx(transformation_node):
        xyz = transformation_node.get("translation").strip().split(' ')
        xyz = [float(i) for i in xyz]
        linear = transformation_node.get("linear").strip().split(' ')
        linear = [float(i) for i in linear]
        if len(xyz) != 3:
            raise ValueError(f"translation attribute has wrong format. "
                             f"it must have 3 elements. but, got {len(xyz)}")
        if len(linear) != 9:
            raise ValueError(f"linear attribute has wrong format. "
                             f"it must have 9 elements. but, got {len(linear)}")

        return np.array([[linear[0], linear[1], linear[2], xyz[0]],
                         [linear[3], linear[4], linear[5], xyz[1]],
                         [linear[6], linear[7], linear[8], xyz[2]],
                         [0.0, 0.0, 0.0, 1.0]])


def find_node(_list, _name):
    for j in _list:
        if j.name == _name:
            return j
    return None


def compute_box_moments(mass: float, x: float, y: float, z: float):
    return "1.0", "1.0", "1.0", "1.0", "1.0", "1.0"  # TODO


def get_rpy_xyz(mtx: np.ndarray):
    euler = tf.euler_from_matrix(mtx, 'rxyz')
    rpy = ' '.join([str(theta) for theta in euler])
    xyz = mtx[0:3, 3]
    xyz = ' '.join([str(x) for x in xyz])

    return rpy, xyz
    # return "0.0 0.0 0.0", "0.0 0.0 0.0"


def convert_joint_axis_from_dart_to_urdf(axis_string):
    axes = axis_string.split(" ")
    axes = [round(abs(float(ax))) for ax in axes]
    return ' '.join(map(str, axes))


def get_link(_body, _node):
    link_xml = ET.Element("link", attrib={"name": _node.name})

    link_visual = ET.SubElement(link_xml, "visual")
    link_collision = ET.SubElement(link_xml, "collision")
    link_inertial = ET.SubElement(link_xml, "inertial")

    body_transform_mtx = Node.trans_to_mtx(_body.find("Transformation"))
    rpy, xyz = get_rpy_xyz(body_transform_mtx @ np.linalg.inv(_node.trGlobal))
    vis_rpy, vis_xyz = get_rpy_xyz(np.linalg.inv(_node.trGlobal))

    # construct the visual element
    scale = "1.0 1.0 1.0"
    filename = os.path.join(relative_path_of_obj_to_urdf, _body.get("obj"))
    visual_origin = ET.SubElement(link_visual, "origin", attrib={"rpy": vis_rpy, "xyz": vis_xyz})
    visual_geometry = ET.SubElement(link_visual, "geometry")
    geometry_mesh = ET.SubElement(visual_geometry, "mesh", attrib={"filename": filename, "scale": scale})

    # construct the collision element
    body_type = _body.get("type")
    if body_type != "Box":
        raise Exception(f"{body_type} is not supported")
    size = _body.get("size")
    collision_origin = ET.SubElement(link_collision, "origin", attrib={"rpy": rpy, "xyz": xyz})
    collision_geometry = ET.SubElement(link_collision, "geometry")
    geometry_box = ET.SubElement(collision_geometry, "box", attrib={"size": size})

    # construct the inertial element
    mass = _body.get("mass")
    ixx, ixy, ixz, iyy, iyz, izz = compute_box_moments(float(mass), *[float(s) for s in xyz.split(" ")])
    inertial_origin = ET.SubElement(link_inertial, "origin", attrib={"rpy": rpy, "xyz": xyz})
    inertial_mass = ET.SubElement(link_inertial, "mass", attrib={"value": mass})
    inertial_inertia = ET.SubElement(link_inertial, "inertia",
                                     attrib={"ixx": ixx, "ixy": ixy, "ixz": ixz, "iyy": iyy, "iyz": iyz, "izz": izz})

    return link_xml


def get_joint(_joint, _node: Node):
    if _node.parent is None:
        return None

    joint_name = _joint.get("bvh")
    joint_type = _joint.get("type")
    if joint_type == "Revolute":
        joint_type = "revolute"
    elif joint_type == "Ball":
        joint_type = "spherical"
    else:
        raise ValueError(f"Joint type for {joint_type} is not supported")
    if joint_name is None:
        joint_name = node_name + joint_type
    joint_xml = ET.Element('joint', attrib={"name": joint_name, "type": joint_type})

    joint_parent = ET.SubElement(joint_xml, "parent", attrib={"link": parent_name})
    joint_child = ET.SubElement(joint_xml, "child", attrib={"link": node_name})
    if joint_type == "revolute":
        joint_axis = ET.SubElement(joint_xml, "axis",
                                   attrib={"xyz": convert_joint_axis_from_dart_to_urdf(_joint.get("axis"))})
        joint_dynamics = ET.SubElement(joint_xml, "dynamics", attrib={"friction": "0.5"})

    # construct the origin element
    rpy, xyz = get_rpy_xyz(_node.trJnt)
    joint_origin = ET.SubElement(joint_xml, "origin", attrib={"rpy": rpy, "xyz": xyz})

    return joint_xml


with open(input_dart) as fi:
    dart = ET.parse(fi).getroot()
    name = dart.get('name')

    urdf_root = ET.Element('robot')
    urdf_root.set('name', name)

    visited_nodes = []
    xml_nodes = dart.findall("Node")
    for xml_node in xml_nodes:  # compute transformation matrix in advance
        node_name = xml_node.get("name")
        parent_name = xml_node.get("parent")
        parent_node = find_node(visited_nodes, parent_name)
        trans = xml_node.find("Joint").find("Transformation")
        n = Node(parent_node, node_name, trans)
        visited_nodes.append(n)

    for xml_node in xml_nodes:
        node_name = xml_node.get("name")
        node = find_node(visited_nodes, node_name)
        parent_name = xml_node.get("parent")
        dart_body = xml_node.find("Body")
        dart_joint = xml_node.find("Joint")

        link = get_link(dart_body, node)
        joint = get_joint(dart_joint, node)

        urdf_root.append(link)

        if joint is not None:
            urdf_root.append(joint)

    plain_urdf = ET.tostring(urdf_root, 'utf-8')
    parsed = minidom.parseString(plain_urdf)
    pretty_xml = parsed.toprettyxml(indent="  ")

    with open(output_urdf, 'w') as fo:
        fo.write(pretty_xml)
