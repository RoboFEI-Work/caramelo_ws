#pragma once

// Seletor de arena: uma lista fixa das arenas que EXISTEM no robo.
//
// Antes, cinco telas diferentes pediam o mapa num campo de texto livre, cada uma
// com "arena2_520" cravado no codigo. Isso e' pior que feio: com o robo rodando
// no warehouse, salvar um waypoint gravava dentro do arena2_520 -- o operador
// nao teria como perceber, e o erro so' apareceria numa missao futura, na arena
// errada. Digitar o nome de uma arena inexistente tinha o mesmo destino
// silencioso.
//
// Este seletor comeca sempre na arena ATIVA do robo e marca qual e' ela. Trocar
// aqui vale so' para a tela em questao: mudar a arena do robo e' uma decisao
// explicita, feita em Mapas com "Usar esta arena".

#include <QComboBox>
#include <QString>

class RosBridge;

class SeletorArena : public QComboBox
{
  Q_OBJECT

public:
  explicit SeletorArena(RosBridge * bridge, QWidget * parent = nullptr);

  // Nome da arena selecionada, sem o sufixo "(ativa)".
  QString arena() const;

  void recarregar();

private:
  RosBridge * bridge_ = nullptr;
};
