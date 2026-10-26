#pragma once

// Seletor do tipo de uma service area -- e a UNICA lista valida desses tipos.
//
// Por que existe: as seis strings do regulamento (start, finish, workstation,
// shelf, precision_placement, rotating_table) estavam escritas a mao em duas
// telas diferentes (ferramenta de mapeamento e service areas). Duas copias de
// uma lista sao duas chances de divergir -- e quem paga e' o operador, que grava
// uma area com um tipo que o executor da missao nao reconhece e so' descobre na
// prova.
//
// Alem de centralizar, a lista aparece em portugues: "workstation" nao diz nada
// para quem nunca leu o rulebook. O valor tecnico continua visivel ao lado,
// porque e' ele que vai para o YAML e para o log.

#include <QComboBox>
#include <QString>
#include <QStringList>

// Os seis tipos validos, na ordem em que fazem sentido para o operador
// (comeco, fim, depois as estacoes de trabalho).
QStringList tiposDeAreaValidos();

// "workstation" -> "Bancada de trabalho". Tipo desconhecido volta como veio:
// melhor mostrar o valor cru do arquivo do que esconder que ele existe.
QString rotuloDeTipoDeArea(const QString & tipo);

// Uma frase explicando para que serve o tipo (vai para a dica da lista).
QString ajudaDeTipoDeArea(const QString & tipo);

bool tipoDeAreaValido(const QString & tipo);

class SeletorTipoArea : public QComboBox
{
  Q_OBJECT

public:
  explicit SeletorTipoArea(QWidget * parent = nullptr);

  // Valor tecnico escolhido (o que vai para o service_areas.yaml).
  QString tipo() const;

  // Seleciona pelo valor tecnico; ignora valor fora da lista, para nao inventar
  // um tipo que o executor da missao nao conhece.
  void setTipo(const QString & tipo);
};
