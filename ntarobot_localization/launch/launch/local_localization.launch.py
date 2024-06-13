from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

# Este archivo corresponde al launch del filtro de kalman desarrollado para la union
# de los valores obtenidos de la imu y la odometria
def generate_launch_description():
    # Se declaran los argumentos (En este caso no tenemos un programa de python por lo que no es necesario)
    # Se inicia el nodo de transformacion, teniendo en cuenta las posiciones iniciales del efk position
    static_transform_publisher = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = [
            "--x", "0", "--y", "0", "--z", "0.103",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "base_footprint_ekf", "--child-frame-id", "imu_link_ekf"
        ]
    )   

    # Ahora se crea el nodo de ejecucon de localizacion
    robot_localization = Node(
        package = "robot_localization",
        executable = "ekf_node",
        name = "ekf_filter_node",
        output = "screen",
        parameters = [
            os.path.join(
                get_package_share_directory("ntarobot_localization"),
                "config",
                "ekf.yaml"
                )
        ]
    ) 

    # node de publicacion de ekf
    imu_republisher_cpp = Node(
        package = "ntarobot_localization",
        executable = "imu_republish"
    )

    # Seccion de return
    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        imu_republisher_cpp
    ])