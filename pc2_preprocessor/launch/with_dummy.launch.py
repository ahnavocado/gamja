from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='pc2_preprocessor', executable='dummy_cloud_pub',
             name='dummy_cloud_pub', output='screen'),
        Node(package='pc2_preprocessor', executable='pc2_preprocessor_node',
             name='pc2_preprocessor', output='screen',
             parameters=[{
               'input_topic': '/pointcloud',
               'filtered_cloud_topic': '/pointcloud/filtered',
               'grid_topic': '/map',
               'map_frame': 'map',
               'voxel_leaf': 0.15,
               'z_min': -1.0,
               'z_max': 3.0,
               'ground_dist_thresh': 0.15,
               'grid_res': 0.10,
               'x_lim': 10.0,
               'y_lim': 10.0,
               'inflate_radius': 0.3,
             }])
    ])
