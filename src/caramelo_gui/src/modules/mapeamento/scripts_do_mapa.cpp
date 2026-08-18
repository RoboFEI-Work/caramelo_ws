#include "modules/mapeamento/scripts_do_mapa.hpp"

#include <QProcess>

#include "modules/mapas/mapas_module.hpp"

namespace
{

const char * kPacote = "caramelo_navigation";

// Numero para linha de comando. 'f' com 6 casas: em milimetros isso e' mais
// resolucao do que qualquer mapa de 5 cm por pixel guarda, e evita a notacao
// cientifica que o argparse leria como texto.
QString numero(double v)
{
  return QString::number(v, 'f', 6);
}

// Traduz a saida do script para UMA frase de operador.
//
// Os scripts ja' escrevem em portugues, mas nem tudo que sai deles e' para
// leigo: "Nao consegui obter TF de map para ['base_footprint']" e' a forma
// tecnica de dizer que o robo nao esta localizado. E' esse caso que aparece
// toda vez que alguem tenta gravar a pose atual com a localizacao desligada.
QString fraseDeErro(const QString & saida)
{
  const QStringList linhas = saida.split('\n', Qt::SkipEmptyParts);

  for (const QString & bruta : linhas) {
    if (bruta.contains("TF") && bruta.contains("Nao consegui obter")) {
      return QString(
        "O robo ainda nao sabe onde esta no mapa. Ligue o mapeamento (ou a "
        "localizacao), espere o robo aparecer no mapa e tente de novo — ou "
        "marque o ponto clicando no mapa, sem sair do lugar.");
    }
    if (bruta.contains("nomenclatura do rulebook")) {
      return QString(
        "Nome de ponto fora do padrao. Use START, FINISH, ou a sigla com o "
        "numero: WS1, SH1, PP1, RT1.");
    }
    if (bruta.contains("Nao encontrei a pasta do mapa")) {
      return QString(
        "Nao encontrei os arquivos desta arena no robo. Salve o mapa antes de "
        "marcar pontos nele.");
    }
  }

  for (int i = linhas.size() - 1; i >= 0; --i) {
    const QString linha = linhas.at(i).trimmed();
    if (linha.startsWith("Erro")) {
      return linha;
    }
  }
  for (int i = linhas.size() - 1; i >= 0; --i) {
    const QString linha = linhas.at(i).trimmed();
    if (!linha.isEmpty()) {
      return linha;
    }
  }
  return QString("O comando terminou com erro e nao explicou o motivo.");
}

}  // namespace

ScriptsDoMapa::ScriptsDoMapa(QObject * parent)
: QObject(parent)
{
  processo_ = new QProcess(this);
  // Um canal so': o script escreve o resultado no stdout e o erro no stderr,
  // e a tela quer a ultima frase, venha de onde vier.
  processo_->setProcessChannelMode(QProcess::MergedChannels);

  connect(
    processo_, &QProcess::readyReadStandardOutput, this, [this]() {
      saida_ += QString::fromUtf8(processo_->readAllStandardOutput());
    });

  connect(
    processo_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
    [this](int codigo, QProcess::ExitStatus estado) {
      saida_ += QString::fromUtf8(processo_->readAllStandardOutput());
      const bool ok = (estado == QProcess::NormalExit && codigo == 0);
      encerrar(ok, ok ? atual_.sucesso : fraseDeErro(saida_));
    });

  connect(
    processo_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError erro) {
      if (erro != QProcess::FailedToStart) {
        return;   // os outros erros terminam em finished, que ja' reporta
      }
      encerrar(
        false,
        "Nao consegui rodar o programa que grava os pontos. O software do robo "
        "parece incompleto nesta maquina — avise quem cuida da instalacao.");
    });
}

void ScriptsDoMapa::enfileirar(const Tarefa & tarefa)
{
  fila_.enqueue(tarefa);
  if (!rodando_) {
    proxima();
  }
}

void ScriptsDoMapa::proxima()
{
  if (fila_.isEmpty()) {
    rodando_ = false;
    return;
  }
  atual_ = fila_.dequeue();
  saida_.clear();
  rodando_ = true;
  ja_reportou_ = false;
  emit comecou(atual_.rotulo);

  QStringList completo{"run", kPacote, atual_.programa};
  completo += atual_.args;
  processo_->start("ros2", completo);
}

void ScriptsDoMapa::encerrar(bool ok, const QString & mensagem)
{
  if (ja_reportou_) {
    return;
  }
  ja_reportou_ = true;
  rodando_ = false;
  emit terminou(atual_.rotulo, ok, mensagem);
  proxima();
}

void ScriptsDoMapa::prepararArena(const QString & arena)
{
  // START e FINISH so'. O modelo cheio do rulebook (WS1..WS4, SH1, PP1, RT1)
  // enche a arena de pontos em [0,0,0] que ninguem vai marcar: viram aviso
  // permanente na conferencia e, pior, um dock que existe no arquivo e nao
  // existe na arena real. O que existe de verdade entra no passo de marcar
  // pontos, um a um, ja' com pose. START e FINISH ficam porque toda prova
  // comeca e termina em algum lugar.
  enfileirar(
    Tarefa{
      "Preparando os arquivos da arena",
      "Arena preparada: START e FINISH ja' existem, falta dizer onde eles ficam.",
      "init_service_areas",
      {"--map", arena, "--map-dir", MapasModule::mapsDir(),
        "--areas", "START", "FINISH", "--sync-docking"}});

  criarParedesVirtuais(arena);
}

QString ScriptsDoMapa::rotuloParedesVirtuais()
{
  return QStringLiteral("Criando as paredes virtuais");
}

void ScriptsDoMapa::criarParedesVirtuais(const QString & arena)
{
  enfileirar(
    Tarefa{
      rotuloParedesVirtuais(),
      "Paredes virtuais criadas (por enquanto o robo pode andar em todo lugar).",
      "init_keepout_mask.py",
      {"--map-name", arena, "--map-dir", MapasModule::mapsDir()}});
}

void ScriptsDoMapa::gravarEstacao(
  const QString & arena, const QString & id, const QString & tipo,
  double alturaDaMesa, const PoseEscolhida & pose)
{
  QStringList args{
    "--map", arena,
    "--map-dir", MapasModule::mapsDir(),
    "--name", id,
    "--type", tipo,
    "--height", numero(alturaDaMesa),
    // A tela ja' e' a confirmacao: sem --overwrite o script recusa regravar um
    // ponto que ja' tem pose, e corrigir um ponto torto e' justamente o motivo
    // mais comum de voltar aqui.
    "--overwrite"};
  if (pose.informada) {
    args << "--pose" << numero(pose.x) << numero(pose.y) << numero(pose.yaw);
  }
  enfileirar(
    Tarefa{
      QString("Gravando %1").arg(id),
      QString("%1 gravado.").arg(id),
      "save_service_area_pose",
      args});
}

void ScriptsDoMapa::gravarDock(
  const QString & arena, const QString & id, const QString & tipoDeDock,
  const PoseEscolhida & pose)
{
  QStringList args{
    "--map-name", arena,
    "--map-dir", MapasModule::mapsDir(),
    "--dock-id", id};
  if (!tipoDeDock.isEmpty()) {
    args << "--dock-type" << tipoDeDock;
  }
  if (pose.informada) {
    args << "--pose" << numero(pose.x) << numero(pose.y) << numero(pose.yaw);
  }
  enfileirar(
    Tarefa{
      QString("Gravando %1").arg(id),
      QString("%1 gravado.").arg(id),
      "save_dock_pose",
      args});
}
