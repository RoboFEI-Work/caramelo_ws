#pragma once

// Validacao do nome (id) de um ponto da arena, do jeito que o regulamento pede.
//
// Por que existe: o nome era um campo de texto livre. Digitar "ws 1", "Ws1" ou
// "mesa" gravava mesmo assim -- ou pior, a action de salvar abortava sem dizer
// nada e a lista da tela continuava mostrando o ponto como se ele existisse. O
// operador so' descobria na missao, quando o robo nao achava a estacao.
//
// A regra do rulebook e' esta:
//   START | FINISH | WS<n> | SH<n> | PP<n> | RT<n>     (n = numero, sem espacos)
//
// Header-only: e' regra pura, sem estado e sem Qt Widgets, e vale para qualquer
// tela que peca o nome de um dock, waypoint ou service area.

#include <QRegularExpression>
#include <QString>

// Expressao do regulamento. Ancorada nas duas pontas: "WS1 " ou "xWS1" nao
// passam -- o executor da missao compara o nome inteiro.
inline QRegularExpression regexIdDePonto()
{
  return QRegularExpression("^(START|FINISH|WS[0-9]+|SH[0-9]+|PP[0-9]+|RT[0-9]+)$");
}

// Tira espacos das pontas e sobe para maiusculas. O operador digita "ws1" o
// tempo todo, e recusar isso seria implicancia -- corrigir e' mais util.
inline QString idDePontoNormalizado(const QString & id)
{
  return id.trimmed().toUpper();
}

inline bool idDePontoValido(const QString & id)
{
  return regexIdDePonto().match(id).hasMatch();
}

// Mesma validacao, mas aceitando o que da' para corrigir sozinho (caixa e
// espacos). E' esta que as telas devem usar antes de gravar.
inline bool idDePontoAceitavel(const QString & id)
{
  return idDePontoValido(idDePontoNormalizado(id));
}

// Mensagem pronta para a tela: vazia quando o nome serve. Diz o formato e da'
// exemplos, porque "invalido" sozinho nao ensina ninguem a acertar.
inline QString erroDeIdDePonto(const QString & id)
{
  const QString limpo = id.trimmed();
  if (limpo.isEmpty()) {
    return QString(
      "Escreva o nome do ponto. Vale START, FINISH, ou a sigla da estacao com o "
      "numero: WS1 (bancada), SH1 (prateleira), PP1 (encaixe de precisao), "
      "RT1 (mesa giratoria).");
  }
  if (idDePontoValido(limpo)) {
    return QString();
  }
  if (idDePontoValido(idDePontoNormalizado(limpo))) {
    return QString(
      "Escreva \"%1\" em letras maiusculas e sem espacos: %2.")
           .arg(limpo, idDePontoNormalizado(limpo));
  }
  return QString(
    "\"%1\" nao serve como nome de ponto. Use START, FINISH, ou a sigla da "
    "estacao seguida do numero, sem espacos: WS1 (bancada), SH1 (prateleira), "
    "PP1 (encaixe de precisao), RT1 (mesa giratoria).").arg(limpo);
}
