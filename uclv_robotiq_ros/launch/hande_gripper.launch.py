from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'modbus_type', default_value="rtu_over_tcp"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper_ip', default_value="192.168.1.110"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'frequency', default_value="200.0"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'port', default_value="54321"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port', default_value="/dev/ttyUSB0"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'baudrate', default_value="115200"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'slave_id', default_value="9"
        )    
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'reset_and_activate', default_value="true"
        )    
    )
  
    gripper_node = Node(
        package="uclv_robotiq_ros",
        executable="robotiq_2f_gripper_node",
        parameters=[{
            'modbus_type': LaunchConfiguration('modbus_type'),
            'gripper_ip': LaunchConfiguration('gripper_ip'),
            'frequency': LaunchConfiguration('frequency'),
            'port': LaunchConfiguration('port'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'slave_id': LaunchConfiguration('slave_id'),
            'reset_and_activate': LaunchConfiguration('reset_and_activate'),
        }],
    )
    
    nodes_to_start = [
        gripper_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)