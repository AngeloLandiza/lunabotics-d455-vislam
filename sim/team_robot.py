"""The team's rover (EDT Lunabotics uic_bot, 2027 layout) as an Isaac Sim USD, for sim/isaac_lunar_sim.py --robot team.

Source: sim/robot/uic_bot_2027.xacro (the 2026 robot of educationmoment/EDT-Lunabotics-2026 with the 2027 sensor
layout). Its meshes live in that repository's src/description_pkg/meshes, so a checkout is needed: set EDT_ROBOT_DIR,
or clone it to ~/lunabotics/edt-lunabotics-2026 (the same lookup as sim/bp1_sandbox/robot.py).

The xacro uses only $(find ...), one xacro:arg and the use_sim xacro:if / xacro:unless blocks, so it is expanded here
without the xacro tool: Gazebo plugin blocks are dropped (Isaac does not read them) and mesh paths are made absolute.
Isaac's URDF importer then writes a USD (cached under ~/.cache/lunabotics_sim, rebuilt when the xacro changes).
Fixed sensor links are merged into base_link (the sim mounts its own sensor assets at the xacro poses below); the four
continuous wheel joints get velocity drives; the body keeps its 65 kg and the wheels 2.27 kg each.
"""
import hashlib
import os
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.join(HERE, 'robot', 'uic_bot_2027.xacro')
CACHE = os.path.expanduser('~/.cache/lunabotics_sim/team_robot')
EDT_CANDIDATES = [os.path.expanduser('~/lunabotics/edt-lunabotics-2026'),
                  os.path.normpath(os.path.join(HERE, '..', '..', 'edt-lunabotics-2026'))]
XACRO_NS = '{http://www.ros.org/wiki/xacro}'

# Geometry the sim's drive and spawn need (all from the xacro: Verified against the team URDF)
WHEEL_RADIUS = 0.1651              # collision cylinder radius
TRACK = 0.54                       # wheel centres at y = +-0.27
BASE_HEIGHT = 0.1 + WHEEL_RADIUS   # base_link above the ground: axles 0.1 m below base_link
D455_MOUNT = (0.535, 0.0, 0.37)    # base_d455_joint xyz
D455_PITCH_DEG = 40.0              # base_d455_joint pitch 0.6981 rad
LIDAR_MOUNT = ((0.45, 0.0, 0.4849), (3.14159, -0.785398, 0.0))   # base_lidar3d_joint (front, 45 deg down)
REAR_CAM_MOUNT = ((-0.42, 0.0, 0.37), (0.0, -0.2618, 3.14159))   # base_rear_cam_joint (facing back, 15 deg up)


def edt_dir():
    cands = ([os.environ['EDT_ROBOT_DIR']] if os.environ.get('EDT_ROBOT_DIR') else []) + EDT_CANDIDATES
    for c in cands:
        if os.path.isdir(os.path.join(c, 'src', 'description_pkg', 'meshes')):
            return c
    raise FileNotFoundError('EDT-Lunabotics-2026 checkout not found (team robot meshes); set EDT_ROBOT_DIR or clone '
                            'it to ~/lunabotics/edt-lunabotics-2026. Looked in: ' + ', '.join(cands))


def visual_from_stl(link):
    """Visual meshes from the STLs instead of the .dae files. The .dae files are Blender exports that also carry
    Blender's default camera and lamp and node transforms Isaac's importer applies differently from RViz (body turned,
    wheels lying flat, stray cubes). The STLs sit in the URDF link frame (body.stl is the collision mesh too);
    wheel.stl is modelled spinning about x, so it is turned 90 deg about z onto the y axle."""
    for vis in link.findall('visual'):
        mesh = vis.find('geometry/mesh')
        if mesh is None or not mesh.get('filename', '').endswith('.dae'):
            continue
        mesh.set('filename', mesh.get('filename')[:-4] + '.stl')
        if link.get('name', '').endswith('wheel_link'):
            origin = vis.find('origin')
            if origin is None:
                origin = ET.SubElement(vis, 'origin', {'xyz': '0 0 0'})
            origin.set('rpy', '0 0 1.5708')


def urdf_text():
    """The xacro expanded to plain URDF (use_sim true), meshes as absolute paths."""
    pkg = os.path.join(edt_dir(), 'src', 'description_pkg')
    root = ET.parse(XACRO).getroot()
    out = ET.Element('robot', {'name': 'uic_bot'})
    for el in root:
        tag = el.tag
        if tag.startswith(XACRO_NS) or tag == 'gazebo' or not isinstance(tag, str):
            continue                    # xacro:arg / property / if (Gazebo plugins and materials only)
        if tag == 'link':
            visual_from_stl(el)
            if el.get('name') != 'base_link' and not el.get('name', '').endswith('wheel_link'):
                for child in list(el):       # sensor boxes: the importer misplaces them when it merges fixed
                    if child.tag in ('visual', 'collision'):   # joints (they floated above the rover), and the
                        el.remove(child)     # sim mounts its own sensor assets at the xacro poses
        out.append(el)
    text = ET.tostring(out, encoding='unicode')
    text = text.replace('file:///$(find description_pkg)', pkg).replace('$(find description_pkg)', pkg)
    if '$(' in text or '${' in text:
        raise ValueError('unexpanded xacro in uic_bot_2027.xacro: ' + re.findall(r'\$[({][^)}]*[)}]', text)[0])
    return '<?xml version="1.0"?>\n' + text


def usd_path():
    """Path of the imported USD (built on first use; needs Isaac Sim's URDF importer, i.e. inside SimulationApp)."""
    text = urdf_text()
    tag = hashlib.sha1(text.encode()).hexdigest()[:10]
    out_dir = os.path.join(CACHE, tag)
    done = os.path.join(out_dir, 'usd_path.txt')
    if os.path.exists(done):
        with open(done) as f:
            path = f.read().strip()
        if os.path.exists(path):
            return path
    os.makedirs(out_dir, exist_ok=True)
    urdf = os.path.join(out_dir, 'uic_bot.urdf')
    with open(urdf, 'w') as f:
        f.write(text)
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    cfg = URDFImporterConfig(urdf_path=urdf, usd_path=out_dir, fix_base=False, merge_fixed_joints=True,
                             collision_type='Convex Hull',
                             joint_drive_type={'.*wheel.*': 'force'}, joint_target_type={'.*wheel.*': 'velocity'},
                             override_joint_stiffness={'.*wheel.*': 0.0},
                             override_joint_damping={'.*wheel.*': 1.0e4})
    path = URDFImporter(cfg).import_urdf()
    with open(done, 'w') as f:
        f.write(path)
    print(f'[team_robot] imported {urdf} -> {path}', flush=True)
    return path


def fix_physics(stage, root):
    """Corrections after referencing the imported USD under root (call before the first physics step):
    - PhysX articulation self-collision off: body.stl's convex hull spans the full width and reaches below the axles,
      so it encloses all four wheels, and PhysX's default (self-collision on; the importer no longer authors
      PhysxArticulationAPI) pushes wheels and body apart: the frame racks and the wheels splay;
    - wheel inertia about the spin axis: the URDF gives izz = 0.051 (a cylinder along z), but the wheels spin about
      their link's y axis, so iyy and izz are swapped (thin-disc values of a 2.27 kg, 0.1651 m wheel are 0.031 / 0.015).
    """
    from pxr import Gf, Sdf, UsdPhysics
    n_root = n_wheel = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(root):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.AddAppliedSchema('PhysxArticulationAPI')
            prim.CreateAttribute('physxArticulation:enabledSelfCollisions', Sdf.ValueTypeNames.Bool).Set(False)
            n_root += 1
        if prim.GetName().endswith('wheel_link') and prim.HasAPI(UsdPhysics.MassAPI):
            UsdPhysics.MassAPI(prim).GetDiagonalInertiaAttr().Set(Gf.Vec3f(0.0255, 0.051, 0.0255))
            n_wheel += 1
    print(f'[team_robot] self-collision off on {n_root} articulation root(s), wheel inertia fixed on {n_wheel} wheels',
          flush=True)


if __name__ == '__main__':
    print(urdf_text()[:2000])
