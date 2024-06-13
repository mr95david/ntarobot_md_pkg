#ifndef KALMANF_IMU_HPP
#define KALMANF_IMU_HPP

// Primero se incluyen las librerias necesarias para el procesamiento
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanF_imu : public rclcpp::Node
{
    // Seccion publica del nodo
    public:
        // Se inicia el constructor de la clase
        KalmanF_imu(const std::string & name);

    private:
        // Se crean las variables y seccion de uso del nodo
        // Seccion de subscriptores
        // Se agrega la subscripcion de la odometria
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        // Se crea la subscripcion a los valores publicados por la imu
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

        // Seccion de creacion de publicadores
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        // Valores de variables de instancia, medias y valores de covarianza necesarios
        // Segun se obtenga la respuesta de estas variables se define si es necesario,
        // mantener o cambiar los siguientes 
        double mean_; // Variable for media valor
        double variance_; // Valor de varianza
        double imu_angular_z_; // Valor angular desde la imu
        bool is_first_odom_; // validacion de primer valor de odometria obtenido
        double last_angular_z_; // El ultimo valor obtenido de la velocidad y posicion angular z
        double motion_; // 

        // Variable de instancia para la publicacion de la nueva odometria
        nav_msgs::msg::Odometry kalman_odom_;

        // Seguido a esto 
        // Ademas se agregan las variables de instancia para la varianza de movimiento y medidas obtenidas
        double motion_vairance_;
        // Ademas la varianza del movimiento dado
        double measurement_variance_;

        // Seccion de funciones de callback
        // Callback de valores de odometria
        void odomCallback(const nav_msgs::msg::Odometry & odom);
        // Callback de valores de imu
        void imuCallback(const sensor_msgs::msg::Imu & imu);

        // Seccion de funciones propias de clase
        // Seccion de funciones propias de la clase
        void measurementUpdate();
        // Se declara la funcion para la prediccion de estado
        void statePrediction();
};

#endif 