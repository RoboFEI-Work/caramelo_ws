// Testes da memoria de slots do ContainerStateStore (sem ROS, 2026-10-19):
// add/get com TTL, removeTableSlotsForTag (todas as WS, preserva as outras
// tags e a secao 'containers:') e arquivo inexistente/vazio. Cada teste usa
// um yaml temporario proprio em temp_directory_path().
#include <gtest/gtest.h>

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "manip_task_execution/container_state_store.hpp"

using manip_task_execution::ContainerStateStore;
using manip_task_execution::TableSlotRecord;

namespace
{

class ContainerStateStoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    dir_ = std::filesystem::temp_directory_path() /
      ("caramelo_css_" + std::to_string(::getpid()) + "_" + info->name());
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
    path_ = (dir_ / "container_states.yaml").string();
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  static TableSlotRecord makeRecord(const std::string & tag, const std::string & slot, double stamp)
  {
    TableSlotRecord rec;
    rec.slot = slot;
    rec.table_pose = "Mesa15";
    rec.tag_frame = tag;
    rec.frame = "manip_base_link";
    rec.x = 0.10;
    rec.y = 0.30;
    rec.z = -0.05;
    rec.stamp_sec = stamp;
    return rec;
  }

  /// Grava um registro; o erro (se houver) fica em last_err_.
  bool addOk(
    const std::string & ws, const std::string & tag, const std::string & slot,
    double stamp)
  {
    ContainerStateStore store(path_);
    return store.addTableSlotUsed(ws, makeRecord(tag, slot, stamp), &last_err_);
  }

  std::vector<TableSlotRecord> slotsOf(const std::string & ws, double ttl, double now)
  {
    ContainerStateStore store(path_);
    std::vector<TableSlotRecord> out;
    std::string err;
    EXPECT_TRUE(store.getTableSlotsUsed(ws, ttl, now, &out, &err)) << err;
    return out;
  }

  std::string fileText() const
  {
    std::ifstream ifs(path_);
    std::stringstream ss;
    ss << ifs.rdbuf();
    return ss.str();
  }

  std::filesystem::path dir_;
  std::string path_;
  std::string last_err_;
};

TEST_F(ContainerStateStoreTest, AddAndGetRespectTtl)
{
  ContainerStateStore store(path_);
  std::string err;
  ASSERT_TRUE(addOk("WS_4", "tag_3", "Mesa15.1", 100.0)) << last_err_;
  ASSERT_TRUE(addOk("WS_4", "tag_5", "Mesa15.2", 150.0)) << last_err_;

  // TTL 0 = sem filtro.
  auto all = slotsOf("WS_4", 0.0, 1000.0);
  ASSERT_EQ(all.size(), 2u);
  EXPECT_EQ(all[0].tag_frame, "tag_3");
  EXPECT_EQ(all[0].slot, "Mesa15.1");
  EXPECT_DOUBLE_EQ(all[0].x, 0.10);
  EXPECT_EQ(all[1].tag_frame, "tag_5");

  // now=220, ttl=100: o de stamp 100 (idade 120) expira, o de 150 fica.
  auto fresh = slotsOf("WS_4", 100.0, 220.0);
  ASSERT_EQ(fresh.size(), 1u);
  EXPECT_EQ(fresh[0].tag_frame, "tag_5");

  // Outra WS sem registro.
  EXPECT_TRUE(slotsOf("WS_6", 0.0, 1000.0).empty());
}

TEST_F(ContainerStateStoreTest, RemoveTagInAllWorkstationsKeepsOthersAndContainers)
{
  ContainerStateStore store(path_);
  std::string err;
  ASSERT_TRUE(store.setOccupied("container1", "tag_9", &err)) << err;
  ASSERT_TRUE(addOk("WS_4", "tag_3", "Mesa15.1", 100.0)) << last_err_;
  ASSERT_TRUE(addOk("WS_4", "tag_5", "Mesa15.2", 101.0)) << last_err_;
  ASSERT_TRUE(addOk("WS_6", "tag_3", "superficie", 102.0)) << last_err_;
  ASSERT_TRUE(addOk("WS_6", "tag_3", "stack:tag_5", 103.0)) << last_err_;

  std::size_t removed = 0;
  ASSERT_TRUE(store.removeTableSlotsForTag("tag_3", &removed, &err)) << err;
  EXPECT_EQ(removed, 3u);

  auto ws4 = slotsOf("WS_4", 0.0, 1000.0);
  ASSERT_EQ(ws4.size(), 1u);
  EXPECT_EQ(ws4[0].tag_frame, "tag_5");
  EXPECT_EQ(ws4[0].slot, "Mesa15.2");
  EXPECT_TRUE(slotsOf("WS_6", 0.0, 1000.0).empty());

  // 'containers:' intacto.
  std::string name;
  ASSERT_TRUE(store.findContainerByTag("tag_9", &name, &err)) << err;
  EXPECT_EQ(name, "container1");
  EXPECT_NE(fileText().find("containers:"), std::string::npos);

  // Segunda remocao da mesma tag: nada a fazer, sucesso com 0.
  removed = 99;
  ASSERT_TRUE(store.removeTableSlotsForTag("tag_3", &removed, &err)) << err;
  EXPECT_EQ(removed, 0u);
}

TEST_F(ContainerStateStoreTest, RemoveUnknownTagDoesNotRewriteFile)
{
  ContainerStateStore store(path_);
  std::string err;
  ASSERT_TRUE(addOk("WS_4", "tag_3", "Mesa15.1", 100.0)) << last_err_;
  const std::string before = fileText();

  std::size_t removed = 99;
  ASSERT_TRUE(store.removeTableSlotsForTag("tag_7", &removed, &err)) << err;
  EXPECT_EQ(removed, 0u);
  EXPECT_EQ(fileText(), before);

  // Tag vazia e' erro de uso.
  EXPECT_FALSE(store.removeTableSlotsForTag("", &removed, &err));
  EXPECT_FALSE(err.empty());
}

TEST_F(ContainerStateStoreTest, MissingFileIsEmptyMemory)
{
  ContainerStateStore store(path_);
  std::string err;
  EXPECT_TRUE(slotsOf("WS_4", 0.0, 1000.0).empty());

  std::size_t removed = 99;
  EXPECT_TRUE(store.removeTableSlotsForTag("tag_3", &removed, &err)) << err;
  EXPECT_EQ(removed, 0u);
  EXPECT_FALSE(std::filesystem::exists(path_));  // nao cria arquivo a toa
}

TEST_F(ContainerStateStoreTest, EmptyFileIsEmptyMemory)
{
  {
    std::ofstream ofs(path_, std::ios::trunc);
  }
  ASSERT_TRUE(std::filesystem::exists(path_));

  ContainerStateStore store(path_);
  std::string err;
  EXPECT_TRUE(slotsOf("WS_4", 0.0, 1000.0).empty());

  std::size_t removed = 99;
  EXPECT_TRUE(store.removeTableSlotsForTag("tag_3", &removed, &err)) << err;
  EXPECT_EQ(removed, 0u);

  // Gravar por cima do arquivo vazio funciona e a leitura volta o registro.
  ASSERT_TRUE(addOk("WS_4", "tag_3", "Mesa15", 100.0)) << last_err_;
  EXPECT_EQ(slotsOf("WS_4", 0.0, 1000.0).size(), 1u);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
