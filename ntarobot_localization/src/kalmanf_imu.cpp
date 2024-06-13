// Include de librerias necesarias
// #include "ntarobot_localization/kalmanf_imu.hpp"
#include "ntarobot_localization/kalmanf_imu.hpp"

// Asignacion de valor de placeholder
using std::placeholders::_1;

// Creamos la clase, que funciona como medio de ejecucion del nodo
KalmanF_imu::KalmanF_imu(const std::string & name)
    : Node(name)
    , mean_(0.0) // Esta corresponde a una inicializaicon rapida de variables de instancia necesarias
    , variance_(1000.0)
    , imu_angular_z_(0.0)
    , is_first_odom_(true)
    , last_angular_z_(0.0)
    , motion_(0.0)
    , motion_vairance_(4.0) // Es necesario agregar ambien la inicializacion basica de las demas variables de instancia
    , measurement_variance_(0.5) // 
{
    // Primero inicializamos los subscriptores
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ntaRobot/odom_msg", 
        10,
        std::bind(&KalmanF_imu::odomCallback, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/bno055/imu",
        10,
        std::bind(&KalmanF_imu::imuCallback, this, _1));

    // Inicializacion de publicadores
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/ntaRobot/odom_kalman",
        10);
};

void KalmanF_imu::odomCallback(const nav_msgs::msg::Odometry & odom){
    // Tambien se actualiza el valor de odometria por kalman
    kalman_odom_ = odom; 

    // Se valida si es la primera medicion realizada de la odometria
    if(is_first_odom_)
    {
        // Se asigna el valor de z a la media
        mean_ = odom.twist.twist.angular.z;
        last_angular_z_ = odom.twist.twist.angular.z;
        // Finalmente se actualiza el estado de lectura de la primera odometria
        is_first_odom_ = false;
        return;
    }
    // Se define el valor obtenido en caso no corresponde a la primer odometria
    motion_ = odom.twist.twist.angular.z - last_angular_z_;


    // Steps of kalman filters
    statePrediction();
    measurementUpdate();

    // 
    last_angular_z_ = odom.twist.twist.angular.z;

    // Aca se actualiza el valor de la odometria obtenida del calculo de kalman
    kalman_odom_.twist.twist.angular.z = mean_;
    // Finalmente se publica este valor obtenido
    odom_pub_->publish(kalman_odom_);
};

void KalmanF_imu::imuCallback(const sensor_msgs::msg::Imu & imu){
    // Se va a actualizar el valor de angular z
    imu_angular_z_ = imu.angular_velocity.z;
};

// La siguiente funcion actualiza el valor de la posicion medida por la odometria y la imu
void KalmanF_imu::measurementUpdate(){
    // calculate nueva media y varianza para asignar los valores
    mean_  = (measurement_variance_ * mean_ + variance_ * imu_angular_z_) / (variance_ + measurement_variance_);
    // Tambien se calcula la nueva varianza
    variance_ = (variance_ * measurement_variance_) / (variance_ + measurement_variance_);

    // Esto valida los valores obtenidos del sistema y la distribucion gaussiana del sistema
}

// Creamos la funcion para la prediccion de paso
void KalmanF_imu::statePrediction(){
    // Calculate best value from mean of gaussian distribution
    mean_ = mean_ + motion_;
    // Tambien se calcula el valor de varianza
    variance_ = variance_ + motion_vairance_;
}

// Se crea la funcion main de ejecucion del nodo
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanF_imu>("kalmanf_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}