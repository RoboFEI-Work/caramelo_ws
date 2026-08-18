#pragma once

// Item de aviso para combos vazios.
//
// Por que existe: combo vazio nao e' neutro. Ele parece um campo com um valor
// escolhido, o operador segue em frente, e a tela monta caminho ".../maps/" ou
// manda uma acao com id "" -- que falha calada, longe daqui. Um item visivel,
// escrito em portugues e IMPOSSIVEL de selecionar troca o silencio por uma
// frase.
//
// Header-only de proposito: e' um detalhe de widget compartilhado pelos
// seletores, nao merece unidade de compilacao propria.

#include <QComboBox>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QString>

// Acrescenta ao combo um item desabilitado, sem itemData -- os seletores leem o
// valor pelo itemData, entao um item de aviso nunca vira valor por acidente.
inline void adicionarAvisoDeListaVazia(QComboBox * combo, const QString & texto)
{
  if (!combo) {
    return;
  }
  const int indice = combo->count();
  combo->addItem(texto);
  if (auto * modelo = qobject_cast<QStandardItemModel *>(combo->model())) {
    if (QStandardItem * item = modelo->item(indice)) {
      item->setEnabled(false);
    }
  }
}
