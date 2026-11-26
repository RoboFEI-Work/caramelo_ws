// Espera de "braco assentou" pela hora de CHEGADA do joint_states (2026-11-26).
// O caso real: relogio da Pi 0,43 s atrasado em relacao ao PC (log de 17/11);
// com o metodo antigo cada leitura esperava esse atraso (~885 ms por movimento).
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "manip_task_execution/arm_settle.hpp"

namespace mte = manip_task_execution;
using namespace std::chrono_literals;

class ArmSettleCache : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_arm_settle_cache");
    cache_ = std::make_unique<mte::JointStateCache>(*node_, "/test_arm_settle/joint_states");
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/test_arm_settle/joint_states", 10);
    exec_.add_node(node_);
    spin_ = std::thread([this]() {exec_.spin();});
  }

  void TearDown() override
  {
    parar_pub();
    exec_.cancel();
    spin_.join();
  }

  /// Publica a 100 Hz com o carimbo `atraso` no passado; `mover` = j1 andando.
  void publicar(std::chrono::milliseconds atraso, bool mover)
  {
    parar_pub();
    rodando_ = true;
    pub_thread_ = std::thread([this, atraso, mover]() {
      double j1 = 0.0;
      while (rodando_) {
        sensor_msgs::msg::JointState m;
        m.header.stamp = node_->now() - rclcpp::Duration(atraso);
        m.name = {"j1", "j2"};
        if (mover) {
          j1 += 0.01;    // 1 rad/s
        }
        m.position = {j1, 0.5};
        pub_->publish(m);
        std::this_thread::sleep_for(10ms);
      }
    });
    std::this_thread::sleep_for(150ms);
  }

  void parar_pub()
  {
    rodando_ = false;
    if (pub_thread_.joinable()) {
      pub_thread_.join();
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<mte::JointStateCache> cache_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread spin_, pub_thread_;
  std::atomic<bool> rodando_{false};
  const std::vector<std::string> names_{"j1", "j2"};
};

TEST_F(ArmSettleCache, ParadoComRelogioAtrasadoAssentaRapido)
{
  publicar(430ms, false);
  mte::ArmSettleParams p;   // janela 0,12 s, teto 1,5 s
  mte::ArmSettleResult r;
  ASSERT_TRUE(mte::waitForGroupToSettleCached(*cache_, names_, p, r));
  EXPECT_TRUE(r.settled);
  EXPECT_LT(r.waited_s, 0.30);   // antes: ~0,885 s
  EXPECT_GE(r.waited_s, p.window_s);
}

TEST_F(ArmSettleCache, SeMexendoNaoAssenta)
{
  publicar(0ms, true);
  mte::ArmSettleParams p;
  p.timeout_s = 0.5;
  mte::ArmSettleResult r;
  ASSERT_TRUE(mte::waitForGroupToSettleCached(*cache_, names_, p, r));
  EXPECT_FALSE(r.settled);
  EXPECT_EQ(r.moving_joint, "j1");
}

TEST_F(ArmSettleCache, SemAmostraNovaVoltaAoMetodoAntigo)
{
  publicar(0ms, false);
  parar_pub();
  std::this_thread::sleep_for(200ms);   // ultima amostra com mais de 0,1 s
  mte::ArmSettleParams p;
  mte::ArmSettleResult r;
  EXPECT_FALSE(mte::waitForGroupToSettleCached(*cache_, names_, p, r));
  std::vector<double> v;
  EXPECT_FALSE(cache_->read(names_, 0.1, v));
}

TEST_F(ArmSettleCache, JuntaFaltandoVoltaAoMetodoAntigo)
{
  publicar(0ms, false);
  mte::ArmSettleParams p;
  mte::ArmSettleResult r;
  EXPECT_FALSE(mte::waitForGroupToSettleCached(*cache_, {"j1", "j9"}, p, r));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
