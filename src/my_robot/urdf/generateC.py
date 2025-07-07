import random
import math
from lxml import etree

# Создаем корневой элемент SDF
sdf = etree.Element('sdf', version='1.10', nsmap={'ros2_control': "http://playerstage.sourceforge.net/gazebo/xmlschema/#ros2_control"})

# Создаем мир
world = etree.SubElement(sdf, 'world', name='curved_bushes_world')

# Добавляем физику
physics = etree.SubElement(world, 'physics', name='1ms', type='ode')
etree.SubElement(physics, 'max_step_size').text = '0.005'
etree.SubElement(physics, 'real_time_factor').text = '1'
etree.SubElement(physics, 'real_time_update_rate').text = '1000'

# Добавляем плагины
plugins = [
    {'filename': 'gz-sim-physics-system', 'name': 'gz::sim::systems::Physics'},
    {'filename': 'gz-sim-user-commands-system', 'name': 'gz::sim::systems::UserCommands'},
    {'filename': 'gz-sim-scene-broadcaster-system', 'name': 'gz::sim::systems::SceneBroadcaster'},
    {'filename': 'gz-sim-sensors-system', 'name': 'gz::sim::systems::Sensors', 'render_engine': 'ogre2'},
    {'filename': 'gz-sim-imu-system', 'name': 'gz::sim::systems::Imu'}
]

for plugin in plugins:
    pl = etree.SubElement(world, 'plugin', attrib={k: v for k, v in plugin.items() if v is not None})

# Добавляем гравитацию и другие параметры
etree.SubElement(world, 'gravity').text = '0 0 -9.8'
etree.SubElement(world, 'magnetic_field').text = '5.5645e-06 2.28758e-05 -4.23884e-05'
etree.SubElement(world, 'atmosphere', type='adiabatic')

# Сцена
scene = etree.SubElement(world, 'scene')
etree.SubElement(scene, 'ambient').text = '0.4 0.4 0.4 1'
etree.SubElement(scene, 'background').text = '0.7 0.7 0.7 1'
etree.SubElement(scene, 'shadows').text = 'true'

# Параметры для расстановки кустов
bushes_per_row = 50  # Кустов в ряду
row_spacing = 0.5    # Расстояние между рядами (ширина дорожки для робота)
path_length = 20.0    # Длина пути
random_offset = 0.0   # Максимальное случайное смещение

# Функция для создания изогнутой линии кустов
def create_curved_row(world, row_num, base_y, amplitude, frequency):
    for i in range(bushes_per_row):
        # Позиция вдоль пути
        t = i / bushes_per_row * path_length - path_length/2
        
        # Изгиб синусоидой
        curve_offset = amplitude * math.sin(frequency * t)
        
        # Координаты куста
        x = t
        y = base_y + curve_offset
        
        # Добавляем случайное смещение
        x += random.uniform(-random_offset, random_offset)
        y += random.uniform(-random_offset, random_offset)
        
        # Создаем модель куста
        bush_model = etree.SubElement(world, 'model', name=f'bush_{row_num}_{i}')
        etree.SubElement(bush_model, 'pose').text = f'{x} {y} 0 0 0 0'
        
        link = etree.SubElement(bush_model, 'link', name='link')
        visual = etree.SubElement(link, 'visual', name='bush_visual')
        etree.SubElement(visual, 'pose').text = '0 0 0 0 0 0'
        
        geometry = etree.SubElement(visual, 'geometry')
        mesh = etree.SubElement(geometry, 'mesh')
        etree.SubElement(mesh, 'uri').text = 'model://src/my_robot/meshes/bushes.STL'
        etree.SubElement(mesh, 'scale').text = '0.0002 0.0002 0.0002'
        
        material = etree.SubElement(visual, 'material')
        etree.SubElement(material, 'ambient').text = '0.1 0.5 0.1 1'
        etree.SubElement(material, 'diffuse').text = '0.3 0.7 0.3 1'
        etree.SubElement(material, 'specular').text = '0.1 0.1 0.1 1'
        
        etree.SubElement(link, 'pose').text = '0 0 0 0 0 0'
        etree.SubElement(link, 'enable_wind').text = 'false'
        
        etree.SubElement(bush_model, 'static').text = 'true'
        etree.SubElement(bush_model, 'self_collide').text = 'false'

# Создаем два изогнутых ряда
create_curved_row(world, 0, -row_spacing/2, 0.5, 0.5)  # Левый ряд
create_curved_row(world, 1, row_spacing/2, 0.5, 0.5)   # Правый ряд

# Добавляем ground plane
ground_plane = etree.SubElement(world, 'model', name="ground_plane")
etree.SubElement(ground_plane, 'static').text = 'true'
link = etree.SubElement(ground_plane, 'link', name="link")
collision = etree.SubElement(link, 'collision', name="collision")
geometry = etree.SubElement(collision, 'geometry')
etree.SubElement(geometry, 'plane').text = '\n            <normal>0 0 1</normal>\n            <size>100 100</size>\n          '
surface = etree.SubElement(collision, 'surface')
friction = etree.SubElement(surface, 'friction')
ode = etree.SubElement(friction, 'ode')
etree.SubElement(ode, 'mu').text = '100'
etree.SubElement(ode, 'mu2').text = '50'

visual = etree.SubElement(link, 'visual', name="visual")
geometry = etree.SubElement(visual, 'geometry')
etree.SubElement(geometry, 'plane').text = '\n            <normal>0 0 1</normal>\n            <size>100 100</size>\n          '
material = etree.SubElement(visual, 'material')
etree.SubElement(material, 'ambient').text = '0.4 0.3 0.1 1'
etree.SubElement(material, 'diffuse').text = '0.5 0.4 0.2 1'
etree.SubElement(material, 'specular').text = '0.1 0.1 0.1 1'

# Добавляем свет
light = etree.SubElement(world, 'light', name='sun', type='directional')
etree.SubElement(light, 'pose').text = '0 0 10 0 0 0'
etree.SubElement(light, 'cast_shadows').text = 'true'
etree.SubElement(light, 'intensity').text = '1'
etree.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
etree.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
etree.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'

attenuation = etree.SubElement(light, 'attenuation')
etree.SubElement(attenuation, 'range').text = '1000'
etree.SubElement(attenuation, 'linear').text = '0.01'
etree.SubElement(attenuation, 'constant').text = '0.9'
etree.SubElement(attenuation, 'quadratic').text = '0.001'

spot = etree.SubElement(light, 'spot')
etree.SubElement(spot, 'inner_angle').text = '0'
etree.SubElement(spot, 'outer_angle').text = '0'
etree.SubElement(spot, 'falloff').text = '0'

# Записываем XML в файл
tree = etree.ElementTree(sdf)
tree.write('curved_bushes_world.sdf', pretty_print=True, encoding='utf-8', xml_declaration=True)

print("SDF файл успешно создан: curved_bushes_world.sdf")