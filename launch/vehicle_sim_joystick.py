from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # choose vehicle
    # vehicle_name = 'toyota_86'
    vehicle_name = 'tt02'

    urdf_file_name = vehicle_name + '.urdf'
    
    # urdf is path of urdf_file_name file. we use define it as the follow due to the CMakeLists.txt file.
    urdf = os.path.join(
        get_package_share_directory('vehicle_sim'),
        '../../../../urdf',
        urdf_file_name
    )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        return LaunchDescription([
            Node(
                package='vehicle_sim',
                executable='vehicle_sim_node',
                name='vehicle_sim_node',
                parameters=[{'vehicle_name': vehicle_name}]
            ),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node'
            ),
            Node(
                package='vehicle_sim',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                arguments=['odom', '0', '0', '0', '0', '0', '0']
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc,
                             'publish_frequency': 100.0}],
                arguments=[urdf]),
            Node(
                package='rviz2', # NOTE: doesnt work when launched using VSCode
                executable='rviz2',
                name='rviz2',
                arguments=['-d', 'config/vehicle_sim_urdf.rviz']
            )
        ])
