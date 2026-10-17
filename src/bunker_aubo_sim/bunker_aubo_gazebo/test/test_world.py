import os
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(os.path.dirname(HERE), 'worlds')
WORLD = os.path.join(WORLDS, 'indoor_office.sdf')


def test_world_parses_and_has_required_systems():
    root = ET.parse(WORLD).getroot()
    world = root.find('world')
    plugins = {p.get('filename') for p in world.findall('plugin')}
    for required in ('gz-sim-physics-system', 'gz-sim-sensors-system',
                     'gz-sim-imu-system', 'gz-sim-scene-broadcaster-system',
                     'gz-sim-user-commands-system'):
        assert required in plugins


def test_all_textures_exist():
    root = ET.parse(WORLD).getroot()
    maps = [e.text for e in root.iter('albedo_map')]
    assert len(maps) >= 20
    for m in maps:
        assert os.path.isfile(os.path.join(WORLDS, m)), m


def test_world_is_furnished():
    root = ET.parse(WORLD).getroot()
    models = root.find('world').findall('model')
    assert len(models) > 30
    collisions = sum(1 for _ in root.iter('collision'))
    assert collisions > 150


def test_world_file_is_up_to_date():
    gen = os.path.join(os.path.dirname(HERE), 'scripts', 'generate_world.py')
    out = subprocess.run([sys.executable, gen], capture_output=True, text=True, check=True).stdout
    with open(WORLD) as f:
        assert f.read().strip() == out.strip()


def test_simple_room_is_up_to_date():
    gen = os.path.join(os.path.dirname(HERE), 'scripts', 'generate_simple_world.py')
    out = subprocess.run([sys.executable, gen], capture_output=True, text=True, check=True).stdout
    with open(os.path.join(WORLDS, 'simple_room.sdf')) as f:
        assert f.read().strip() == out.strip()


def test_simple_room_has_required_systems():
    world = ET.parse(os.path.join(WORLDS, 'simple_room.sdf')).getroot().find('world')
    plugins = {p.get('filename') for p in world.findall('plugin')}
    assert {'gz-sim-physics-system', 'gz-sim-sensors-system', 'gz-sim-imu-system'} <= plugins
