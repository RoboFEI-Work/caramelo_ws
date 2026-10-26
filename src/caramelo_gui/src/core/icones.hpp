#pragma once

// Icones desenhados em codigo, nao carregados de arquivo.
//
// Duas razoes: o robo nao tem internet em competicao e nada pode depender de
// asset externo; e um icone vetorial desenhado na hora escala para qualquer
// tamanho de tela sem borrar. Cada icone e' um quadrado arredondado colorido
// com um glifo branco por cima -- a mesma gramatica visual dos menus de robo de
// servico (PUDU, Temi).

#include <QPixmap>

namespace icones
{

enum class Tipo
{
  Operacao,     // seta de navegacao
  Competicao,   // bandeira quadriculada
  Mapas,        // mapa dobrado
  Mapeamento,   // lapis
  Avancado,     // engrenagem
  Parar,        // quadrado de parada
  Base,         // seta de retorno
  Config,       // engrenagem pequena
};

// lado = largura/altura em pixels do quadrado arredondado.
QPixmap desenhar(Tipo tipo, int lado);

}  // namespace icones
