#include "widgets/seletor_tipo_area.hpp"

namespace
{

struct TipoDeArea
{
  const char * valor;    // string do regulamento, e' o que vai para o YAML
  const char * rotulo;   // como o operador entende
  const char * ajuda;
};

// Fonte unica da verdade destes seis tipos. Qualquer tela que precise deles
// pergunta aqui -- nunca redigita a lista.
const TipoDeArea kTipos[] = {
  {"start", "Inicio da prova",
    "Onde o robo comeca a prova. Toda missao parte daqui."},
  {"finish", "Fim da prova",
    "Onde o robo tem que terminar a prova."},
  {"workstation", "Bancada de trabalho",
    "Mesa comum de onde o robo pega e onde deixa objetos."},
  {"shelf", "Prateleira",
    "Estacao alta: o robo para mais longe e o braco trabalha em outra altura."},
  {"precision_placement", "Encaixe de precisao",
    "Mesa com cavidades: o objeto tem que entrar no furo do formato certo."},
  {"rotating_table", "Mesa giratoria",
    "Mesa que gira sem parar; o robo tem que acertar o tempo da pega."},
};

const TipoDeArea * procurar(const QString & tipo)
{
  for (const TipoDeArea & t : kTipos) {
    if (tipo == QString::fromLatin1(t.valor)) {
      return &t;
    }
  }
  return nullptr;
}

}  // namespace

QStringList tiposDeAreaValidos()
{
  QStringList lista;
  for (const TipoDeArea & t : kTipos) {
    lista << QString::fromLatin1(t.valor);
  }
  return lista;
}

QString rotuloDeTipoDeArea(const QString & tipo)
{
  const TipoDeArea * achado = procurar(tipo);
  return achado ? QString::fromLatin1(achado->rotulo) : tipo;
}

QString ajudaDeTipoDeArea(const QString & tipo)
{
  const TipoDeArea * achado = procurar(tipo);
  return achado ? QString::fromLatin1(achado->ajuda) : QString();
}

bool tipoDeAreaValido(const QString & tipo)
{
  return procurar(tipo) != nullptr;
}

SeletorTipoArea::SeletorTipoArea(QWidget * parent)
: QComboBox(parent)
{
  setEditable(false);   // lista fixa: e' o ponto do widget

  for (const TipoDeArea & t : kTipos) {
    // Texto = rotulo humano + valor tecnico entre parenteses. O tecnico fica a
    // vista porque e' ele que aparece no arquivo e no log de erro; sem isso o
    // operador nao consegue casar a tela com o que leu no YAML.
    const QString valor = QString::fromLatin1(t.valor);
    addItem(QString("%1  (%2)").arg(QString::fromLatin1(t.rotulo), valor), valor);
    setItemData(count() - 1, QString::fromLatin1(t.ajuda), Qt::ToolTipRole);
  }
}

QString SeletorTipoArea::tipo() const
{
  return itemData(currentIndex()).toString();
}

void SeletorTipoArea::setTipo(const QString & tipo)
{
  for (int i = 0; i < count(); ++i) {
    if (itemData(i).toString() == tipo) {
      setCurrentIndex(i);
      return;
    }
  }
}
