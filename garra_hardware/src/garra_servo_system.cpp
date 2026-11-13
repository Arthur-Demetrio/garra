#include "garra_hardware/garra_servo_system.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace garra_hardware
{

namespace
{
const auto LOGGER = rclcpp::get_logger("GarraServoSystem");

constexpr int PULSO_MIN = 500;
constexpr int PULSO_MAX = 2500;
constexpr double PERIODO_ENVIO_S = 0.02;
constexpr double TEMPO_MAX_SEM_RESPOSTA_S = 1.0;
// Abrir a porta reinicia o Mega. Enviar dados durante o boot pode prende-lo no bootloader,
// entao o primeiro comando so sai depois dessa espera.
constexpr auto ESPERA_BOOT = std::chrono::milliseconds(2500);
constexpr auto TEMPO_MAX_HANDSHAKE = std::chrono::seconds(6);
constexpr double PERIODO_SENSOR_S = 0.05;      // pergunta a chave dos dedos a 20 Hz
constexpr double REPUBLICAR_SENSOR_S = 1.0;    // publica mesmo sem mudanca, como sinal de vida
const char * TOPICO_SENSOR = "/garra/cubo_preso";
}  // namespace

hardware_interface::CallbackReturn GarraServoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    porta_ = info_.hardware_parameters.at("porta");
    juntas_.resize(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto & info_junta = info_.joints[i];
      Junta & junta = juntas_[i];
      junta.zero_us = std::stod(info_junta.parameters.at("zero_us"));
      junta.us_por_rad = std::stod(info_junta.parameters.at("us_por_rad"));
      junta.min_us = std::stoi(info_junta.parameters.at("min_us"));
      junta.max_us = std::stoi(info_junta.parameters.at("max_us"));

      if (
        junta.min_us < PULSO_MIN || junta.max_us > PULSO_MAX || junta.min_us >= junta.max_us ||
        junta.us_por_rad == 0.0)
      {
        RCLCPP_ERROR(LOGGER, "Calibracao invalida na junta %s", info_junta.name.c_str());
        return CallbackReturn::ERROR;
      }

      double inicial = 0.0;
      for (const auto & estado : info_junta.state_interfaces) {
        if (estado.name == hardware_interface::HW_IF_POSITION && !estado.initial_value.empty()) {
          inicial = std::stod(estado.initial_value);
        }
      }
      junta.pulso = pulso_de(junta, inicial);
      junta.posicao = (junta.pulso - junta.zero_us) / junta.us_por_rad;
      junta.comando = junta.posicao;
    }
  } catch (const std::exception & erro) {
    RCLCPP_ERROR(LOGGER, "Parametro ausente ou invalido: %s", erro.what());
    return CallbackReturn::ERROR;
  }

  // J1-J3 do braco; o J4 (dedos) e opcional no firmware.
  if (juntas_.size() != 3 && juntas_.size() != 4) {
    RCLCPP_ERROR(LOGGER, "O firmware aceita 3 ou 4 juntas, recebidas %zu", juntas_.size());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

int GarraServoSystem::pulso_de(const Junta & junta, double angulo) const
{
  const double pulso = junta.zero_us + angulo * junta.us_por_rad;
  return static_cast<int>(std::lround(
    std::clamp(pulso, static_cast<double>(junta.min_us), static_cast<double>(junta.max_us))));
}

bool GarraServoSystem::abrir_serial()
{
  fd_ = ::open(porta_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(LOGGER, "Nao foi possivel abrir %s: %s", porta_.c_str(), std::strerror(errno));
    return false;
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(LOGGER, "tcgetattr em %s: %s", porta_.c_str(), std::strerror(errno));
    fechar_serial();
    return false;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(LOGGER, "tcsetattr em %s: %s", porta_.c_str(), std::strerror(errno));
    fechar_serial();
    return false;
  }
  return true;
}

void GarraServoSystem::fechar_serial()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  buffer_.clear();
}

bool GarraServoSystem::enviar(const std::string & texto)
{
  const ssize_t escrito = ::write(fd_, texto.data(), texto.size());
  if (escrito == static_cast<ssize_t>(texto.size())) {
    return true;
  }
  if (escrito < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return true;  // Buffer cheio: o proximo ciclo envia um comando mais recente.
  }
  RCLCPP_ERROR(LOGGER, "Falha ao escrever na serial: %s", std::strerror(errno));
  return false;
}

void GarraServoSystem::ler_linhas(std::vector<std::string> & linhas)
{
  char dados[128];
  ssize_t lidos = 0;
  while ((lidos = ::read(fd_, dados, sizeof(dados))) > 0) {
    buffer_.append(dados, static_cast<size_t>(lidos));
  }
  size_t fim = 0;
  while ((fim = buffer_.find('\n')) != std::string::npos) {
    std::string linha = buffer_.substr(0, fim);
    buffer_.erase(0, fim + 1);
    if (!linha.empty() && linha.back() == '\r') {
      linha.pop_back();
    }
    linhas.push_back(linha);
  }
}

bool GarraServoSystem::handshake()
{
  std::this_thread::sleep_for(ESPERA_BOOT);
  tcflush(fd_, TCIOFLUSH);
  const auto limite = std::chrono::steady_clock::now() + TEMPO_MAX_HANDSHAKE;
  while (std::chrono::steady_clock::now() < limite) {
    if (!enviar("V\n")) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::vector<std::string> linhas;
    ler_linhas(linhas);
    if (std::find(linhas.begin(), linhas.end(), "GARRA 1") != linhas.end()) {
      return true;
    }
  }
  RCLCPP_ERROR(LOGGER, "O firmware nao respondeu em %s", porta_.c_str());
  return false;
}

hardware_interface::CallbackReturn GarraServoSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!abrir_serial()) {
    return CallbackReturn::ERROR;
  }
  if (!handshake()) {
    fechar_serial();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOGGER, "Firmware conectado em %s", porta_.c_str());

  if (!no_sensor_) {
    no_sensor_ = rclcpp::Node::make_shared("garra_sensor_cubo");
    // transient_local: quem assina depois recebe o ultimo estado na hora
    pub_sensor_ = no_sensor_->create_publisher<std_msgs::msg::Bool>(
      TOPICO_SENSOR, rclcpp::QoS(1).reliable().transient_local());
  }
  sensor_ativo_ = true;
  sensor_respondeu_ = false;
  aguardando_sensor_ = false;
  cubo_preso_ = -1;
  return CallbackReturn::SUCCESS;
}

void GarraServoSystem::tratar_sensor(const std::string & linha)
{
  if (linha == "S 0" || linha == "S 1") {
    aguardando_sensor_ = false;
    const int valor = linha[2] == '1' ? 1 : 0;
    if (!sensor_respondeu_) {
      RCLCPP_INFO(LOGGER, "Sensor de cubo dos dedos ativo em %s", TOPICO_SENSOR);
      sensor_respondeu_ = true;
    }
    if (valor != cubo_preso_) {
      cubo_preso_ = valor;
      publicar_sensor(true);
    }
  }
}

void GarraServoSystem::publicar_sensor(bool forcar)
{
  if (!pub_sensor_ || cubo_preso_ < 0) {
    return;
  }
  if (!forcar && tempo_desde_publicacao_ < REPUBLICAR_SENSOR_S) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = cubo_preso_ == 1;
  pub_sensor_->publish(msg);
  tempo_desde_publicacao_ = 0.0;
}

hardware_interface::CallbackReturn GarraServoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  fechar_serial();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GarraServoSystem::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto & junta : juntas_) {
    junta.comando = junta.posicao;
    junta.velocidade = 0.0;
  }
  tempo_desde_envio_ = PERIODO_ENVIO_S;
  tempo_sem_resposta_ = 0.0;
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GarraServoSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Nao ha comando de soltar: os servos seguram a ultima posicao para o braco nao cair.
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GarraServoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < juntas_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &juntas_[i].posicao);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &juntas_[i].velocidade);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> GarraServoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < juntas_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &juntas_[i].comando);
  }
  return interfaces;
}

hardware_interface::return_type GarraServoSystem::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  std::vector<std::string> linhas;
  ler_linhas(linhas);
  tempo_sem_resposta_ = linhas.empty() ? tempo_sem_resposta_ + dt : 0.0;
  for (const auto & linha : linhas) {
    if (!linha.empty() && linha[0] == 'S') {
      tratar_sensor(linha);
    } else if (linha == "E comando" && aguardando_sensor_ && !sensor_respondeu_) {
      // Firmware antigo, sem o comando "S": desliga o sensor e segue sem ele.
      sensor_ativo_ = false;
      aguardando_sensor_ = false;
      RCLCPP_WARN(
        LOGGER, "Firmware sem sensor de cubo (comando S desconhecido); %s nao sera publicado",
        TOPICO_SENSOR);
    } else if (!linha.empty() && linha[0] == 'E') {
      RCLCPP_WARN(LOGGER, "Firmware recusou um comando: %s", linha.c_str());
    }
  }
  tempo_desde_publicacao_ += dt;
  publicar_sensor(false);
  if (tempo_sem_resposta_ > TEMPO_MAX_SEM_RESPOSTA_S) {
    RCLCPP_ERROR(LOGGER, "Firmware sem responder ha mais de %.1f s", TEMPO_MAX_SEM_RESPOSTA_S);
    return return_type::ERROR;
  }

  for (auto & junta : juntas_) {
    if (std::isnan(junta.comando)) {
      continue;
    }
    junta.pulso = pulso_de(junta, junta.comando);
    const double posicao = (junta.pulso - junta.zero_us) / junta.us_por_rad;
    junta.velocidade = dt > 0.0 ? (posicao - junta.posicao) / dt : 0.0;
    junta.posicao = posicao;
  }
  return return_type::OK;
}

hardware_interface::return_type GarraServoSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  tempo_desde_envio_ += period.seconds();
  tempo_desde_sensor_ += period.seconds();
  if (sensor_ativo_ && tempo_desde_sensor_ >= PERIODO_SENSOR_S) {
    tempo_desde_sensor_ = 0.0;
    aguardando_sensor_ = true;
    if (!enviar("S\n")) {
      return return_type::ERROR;
    }
  }
  if (tempo_desde_envio_ < PERIODO_ENVIO_S) {
    return return_type::OK;
  }
  tempo_desde_envio_ = 0.0;

  for (const auto & junta : juntas_) {
    if (std::isnan(junta.comando)) {
      return return_type::OK;
    }
  }

  std::string corpo = "M";
  for (const auto & junta : juntas_) {
    corpo += " " + std::to_string(pulso_de(junta, junta.comando));
  }
  unsigned soma = 0;
  for (const char c : corpo) {
    soma ^= static_cast<unsigned char>(c);
  }
  char checksum[8];
  std::snprintf(checksum, sizeof(checksum), "*%02X\n", soma);
  return enviar(corpo + checksum) ? return_type::OK : return_type::ERROR;
}

}  // namespace garra_hardware

PLUGINLIB_EXPORT_CLASS(garra_hardware::GarraServoSystem, hardware_interface::SystemInterface)
