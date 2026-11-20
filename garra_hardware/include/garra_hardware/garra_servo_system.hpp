#ifndef GARRA_HARDWARE__GARRA_SERVO_SYSTEM_HPP_
#define GARRA_HARDWARE__GARRA_SERVO_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

namespace garra_hardware
{

// O DS3218MG nao informa a posicao real. O estado publicado e o ultimo comando aceito, depois
// de limitado a faixa calibrada. Se um comando sai da faixa, o estado diverge do comando e o
// controlador de trajetoria aborta o movimento.
class GarraServoSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;
  using return_type = hardware_interface::return_type;

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Junta
  {
    double zero_us{1500.0};
    double us_por_rad{0.0};
    int min_us{0};
    int max_us{0};
    int pulso{0};
    double comando{0.0};
    double posicao{0.0};
    double velocidade{0.0};
  };

  bool abrir_serial();
  void fechar_serial();
  bool handshake();
  bool enviar(const std::string & texto);
  void ler_linhas(std::vector<std::string> & linhas);
  int pulso_de(const Junta & junta, double angulo) const;
  void tratar_sensor(const std::string & linha);
  void publicar_sensor(bool forcar);

  std::vector<Junta> juntas_;
  std::string porta_;
  int fd_{-1};
  std::string buffer_;
  double tempo_desde_envio_{0.0};
  double tempo_sem_resposta_{0.0};

  // Chave fim de curso dos dedos (comando "S" do firmware), publicada em /garra/cubo_preso.
  rclcpp::Node::SharedPtr no_sensor_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sensor_;
  bool sensor_ativo_{true};        // vira false se o firmware nao conhece o "S"
  bool sensor_respondeu_{false};
  bool aguardando_sensor_{false};
  int cubo_preso_{-1};             // -1 = ainda sem leitura
  double tempo_desde_sensor_{0.0};
  double tempo_desde_publicacao_{0.0};

  // Plataforma do eixo Y (comando "Y" do firmware): /garra/plataforma_y, std_msgs/Int8
  // -1 = esquerda, 0 = para, +1 = direita; -2/+2 = esquerda/direita DEVAGAR ("Y L E/D").
  // Para sozinha se o topico ficar quieto.
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_plataforma_;
  int plataforma_pedido_{0};       // ultimo valor recebido no topico
  int plataforma_enviado_{0};      // ultimo valor mandado ao firmware
  double tempo_desde_pedido_y_{0.0};
  double tempo_desde_envio_y_{0.0};
  // Homing da plataforma: /garra/plataforma_y/home (String "E" ou "D") -> "Y H E/D" no firmware;
  // o fim ("Y HOME E OK", "... ERRO", "... CANCELADO") sai em /garra/plataforma_y/home/resultado.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_home_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_home_;
  char home_pedido_{0};            // 'E' ou 'D' esperando para ser enviado (0 = nada)
};

}  // namespace garra_hardware

#endif  // GARRA_HARDWARE__GARRA_SERVO_SYSTEM_HPP_
