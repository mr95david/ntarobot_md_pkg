#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Se definen las funciones del modulo de chrono
using namespace std::chrono_literals;

// Creamos un publicador rapido
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

// Funcion de callback para el subscriptor creado
void imuCallback(const sensor_msgs::msg::Imu & imu)
{
    // Se crea un nuevo valor de imu
    sensor_msgs::msg::Imu newimu;
    // Se iguala con el valor actual
    newimu = imu;
    // Se cambia el valor de header para identificarlo de la imu actualmente publicada.
    newimu.header.frame_id = "base_footprint_ekf";
    // Finalmente se publica el valor de la imu
    imu_pub->publish(newimu);
}

// Se crea la funcion main del nodo
int main(int argc, char* argv[]){
    // Primero se inicializa el nodo
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_replublisher_node");
    // Se crea un step para ejecutar 
    rclcpp::sleep_for(1s);
    // Se publica el valor de imu
    imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_ekf", 10);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/bno055/imu",
        10,
        imuCallback
    );

    // Se Suscribe al valor de imu obtenido en tiempo real
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}