/* app.js -- a casca viva do painel: roteador, estado, WebSocket e cliente da API.
 *
 * O painel web segue a GUI Qt (caramelo_gui). A navegacao e' a mesma:
 *
 *     menu principal  ->  contexto  ->  voltar
 *
 * e nao uma barra de abas. O menu principal mostra o ESTADO de cada contexto
 * antes de entrar nele ("Bloqueado: sensores de navegacao sem dados"), que e' a
 * informacao que o operador precisa com o robo na arena; uma barra de abas
 * mostra cinco palavras iguais entre si e nao conta nada.
 *
 * Este arquivo nao desenha tela nenhuma. Ele:
 *   - troca a tela conforme o endereco (#/operacao), para dar' para recarregar
 *     a pagina sem perder o lugar;
 *   - mantem UM WebSocket de estado, com reconexao e recuo progressivo;
 *   - e o unico que fala com a API (chave, 403, 409, queda de rede);
 *   - entrega tudo isso as telas por um `ctx`.
 *
 * CONTRATO DE TELA (o mesmo em todos os arquivos de telas/):
 *
 *   export const tela = {
 *     id: 'operacao', titulo: 'Operacao', precisaDoMapa: true,
 *     montar(raiz, ctx) { ...; return {destruir(), aoEstado(estado)}; }
 *   };
 */

// As cinco telas do menu principal, na ordem da GUI, mais o proprio menu.
// O arquivo e' carregado sob demanda: uma tela com defeito derruba ela mesma e
// mostra o motivo, em vez de deixar o painel inteiro em branco.
const TELAS = {
  home:       { arquivo: './telas/home.js',       titulo: 'Caramelo' },
  operacao:   { arquivo: './telas/operacao.js',   titulo: 'Operacao' },
  competicao: { arquivo: './telas/competicao.js', titulo: 'Competicao' },
  mapas:      { arquivo: './telas/mapas.js',      titulo: 'Mapas' },
  mapeamento: { arquivo: './telas/mapeamento.js', titulo: 'Mapeamento' },
  avancado:   { arquivo: './telas/avancado.js',   titulo: 'Modo Avancado' },
};

// Sem quadro por mais que isto, o painel para de afirmar o que sabia. Um painel
// congelado mostrando tudo verde com o robo desligado e' o pior engano que esta
// tela pode cometer -- ja' aconteceu, com o cabo de rede caido.
const LIMITE_SEM_QUADRO = 3000;

// Recuo progressivo da reconexao. Comeca rapido (o caso comum e' o servidor ter
// reiniciado) e para de crescer em 8 s: o painel fica aberto o dia inteiro numa
// bancada e o Wi-Fi de competicao cai sozinho.
const RECUOS = [400, 800, 1500, 3000, 5000, 8000];

const SEM_CONTATO = 'Perdi contato com o painel do robô.';

// ------------------------------------------------------------------- chave
// A chave de acesso vem no proprio link (?t=...). NAO vai para localStorage:
// um tablet emprestado nao pode continuar comandando o robo depois que a aba
// fecha.
const CHAVE = new URLSearchParams(location.search).get('t') || '';

const CABECALHO_DA_CHAVE = 'X-Caramelo-Chave';

/** Rota com a chave grudada. Serve para <img>, WebSocket e link -- os tres
 *  lugares em que nao da' para mandar cabecalho. */
function comChave(rota) {
  if (!CHAVE) { return rota; }
  return rota + (rota.indexOf('?') >= 0 ? '&' : '?') + 't=' + encodeURIComponent(CHAVE);
}

// --------------------------------------------------------------- elementos
const palco = document.getElementById('palco');
const faixaTitulo = document.getElementById('titulo');
const botaoVoltar = document.getElementById('voltar');
const caixaDeTorradas = document.getElementById('torradas');
const CHIPS = {
  robo: document.getElementById('chipRobo'),
  arena: document.getElementById('chipArena'),
  navegacao: document.getElementById('chipNavegacao'),
  missao: document.getElementById('chipMissao'),
  chave: document.getElementById('chipChave'),
};

// ------------------------------------------------------------------ estado
const app = {
  autorizado: null,      // null = ainda nao perguntei ao robo
  telaAtual: null,       // {id, modulo, montado, raiz}
  geracao: 0,            // corrida entre dois toques rapidos no menu
  quadro: {},            // ultimo quadro entregue as telas (ja' completado)
  lentas: {},            // as chaves que so' chegam 1 Hz, guardadas
  ultimoQuadro: 0,
  ultimoEvento: 0,
  ligado: false,
};

// =========================================================== torradas
// Recado curto do robo. Some sozinho; nao rouba o toque de ninguem.
//
// REPETICAO VIRA CONTADOR, e nao uma pilha de torradas iguais: uma tela pode
// avisar por conta propria E o cliente da API avisar da mesma resposta (foi o
// que aconteceu entre a tela de mapeamento e este arquivo). Duas torradas
// identicas empilhadas parecem dois erros diferentes.
const torradasVivas = [];

function torrada(texto, bom) {
  texto = String(texto == null ? '' : texto).trim();
  if (!texto) { return; }
  const ruim = bom === false;

  const igual = torradasVivas.find((t) => t.texto === texto && t.ruim === ruim);
  if (igual) {
    igual.vezes += 1;
    igual.contador.textContent = '×' + igual.vezes;
    clearTimeout(igual.relogio);
    igual.relogio = setTimeout(() => fecharTorrada(igual), tempoDeLeitura(texto));
    return;
  }

  const no = document.createElement('div');
  no.className = ruim ? 'torrada ruim' : 'torrada';
  const corpo = document.createElement('span');
  corpo.textContent = texto;
  const contador = document.createElement('span');
  contador.className = 'vezes';
  no.appendChild(corpo);
  no.appendChild(contador);
  caixaDeTorradas.appendChild(no);

  const item = {texto, ruim, no, contador, vezes: 1, relogio: null};
  item.relogio = setTimeout(() => fecharTorrada(item), tempoDeLeitura(texto));
  torradasVivas.push(item);

  // Teto de cinco: mais que isso cobre a tela justamente quando muita coisa
  // esta dando errado e o operador precisa VER o robo.
  while (torradasVivas.length > 5) { fecharTorrada(torradasVivas[0]); }
}

/** Frase longa fica mais tempo: 3 s de piso, 90 ms por palavra. */
function tempoDeLeitura(texto) {
  return Math.min(12000, 3000 + texto.split(/\s+/).length * 90);
}

function fecharTorrada(item) {
  const i = torradasVivas.indexOf(item);
  if (i >= 0) { torradasVivas.splice(i, 1); }
  clearTimeout(item.relogio);
  if (item.no.parentNode) { item.no.parentNode.removeChild(item.no); }
}

// =========================================================== cliente da API
//
// Um lugar so' fala com o robo. As telas nao repetem tratamento de erro nem
// sabem onde mora a chave -- e por isso as tres respostas dificeis tem UMA
// redacao, e nao seis:
//
//   403  o painel foi aberto sem a chave. Nao e' falha: e' um painel de olhar.
//   409  o ROBO recusou. Nada quebrou; ele e' que nao estava em condicao de
//        obedecer. A frase e' dele, e vai inteira para a tela.
//   rede o PAINEL sumiu (Wi-Fi, servidor reiniciando). Distinguir isso de uma
//        recusa do robo e' o que evita o operador procurar defeito no robo.

async function pedir(rota, opcoes, escrita) {
  const cabecalhos = {Accept: 'application/json'};
  if (CHAVE) { cabecalhos[CABECALHO_DA_CHAVE] = CHAVE; }
  if (opcoes && opcoes.body) { cabecalhos['Content-Type'] = 'application/json'; }

  let resposta;
  try {
    resposta = await fetch(rota, Object.assign({headers: cabecalhos}, opcoes || {}));
  } catch (erro) {
    return {ok: false, http: 0, mensagem: SEM_CONTATO};
  }

  let corpo = {};
  try { corpo = (await resposta.json()) || {}; } catch (erro) { corpo = {}; }
  if (typeof corpo !== 'object' || corpo === null) { corpo = {valor: corpo}; }

  if (resposta.ok) {
    // Leitura devolve o corpo do robo INTACTO: as telas leem dados.arenas,
    // meta.docks, plano.linhas. Embrulhar num {ok, dados} quebraria todas.
    if (!escrita) { return corpo; }
    return Object.assign({ok: true, http: resposta.status}, corpo);
  }

  const saida = Object.assign({ok: false, http: resposta.status}, corpo);
  if (resposta.status === 403) {
    // A resposta do servidor ja' explica onde conseguir o link certo.
    if (app.autorizado !== false) { app.autorizado = false; pintarChipDaChave(); }
  } else if (!saida.mensagem) {
    saida.mensagem = resposta.status >= 500
      ? 'O painel do robô tropeçou ao executar isso. Tente de novo.'
      : 'O robô não aceitou este pedido.';
  }
  return saida;
}

async function ler(rota) {
  return pedir(comChave(rota), {method: 'GET'}, false);
}

async function comandar(rota, corpo) {
  const dados = await pedir(comChave(rota), {
    method: 'POST',
    body: JSON.stringify(corpo || {}),
  }, true);
  if (dados.mensagem) { torrada(dados.mensagem, dados.ok !== false); }
  return dados;
}

async function apagar(rota) {
  const dados = await pedir(comChave(rota), {method: 'DELETE'}, true);
  if (dados.mensagem) { torrada(dados.mensagem, dados.ok !== false); }
  return dados;
}

// As telas foram escritas em paralelo com este arquivo e cada uma procura o
// metodo pelo nome que imaginou. Os apelidos custam uma linha cada e evitam a
// tela cair no fetch cru -- que funciona, mas perde a torrada unica e o
// tratamento de 403.
const api = {
  url: comChave, comChave, rota: comChave,
  ler, get: ler, obter: ler, buscar: ler,
  comandar, post: comandar, enviar: comandar, postar: comandar,
  apagar, remover: apagar, del: apagar,
};

// ================================================================ roteador
//
// O lugar mora no endereco (#/operacao). Recarregar a pagina -- que e' o que
// alguem faz quando acha que travou -- devolve a mesma tela, e nao o menu.
// A secao vai junto (#/operacao/localizacao) para os atalhos entre telas
// ("marque onde o robo esta") caírem no lugar certo.

function lugarAtual() {
  const cru = (location.hash || '').replace(/^#\/?/, '');
  const partes = cru.split('/').filter(Boolean);
  const id = partes[0] || 'home';
  return {id: TELAS[id] ? id : 'home', secao: partes[1] || ''};
}

function irPara(id, opcoes) {
  const alvo = TELAS[id] ? id : 'home';
  const secao = (opcoes && opcoes.secao) ? '/' + opcoes.secao : '';
  const novo = '#/' + (alvo === 'home' ? '' : alvo + secao);
  if (location.hash === novo) { abrirLugar(); } else { location.hash = novo; }
  return true;
}

function voltar() { irPara('home', {}); }

async function abrirLugar() {
  const lugar = lugarAtual();
  const geracao = ++app.geracao;

  if (app.telaAtual) {
    try {
      if (app.telaAtual.montado && typeof app.telaAtual.montado.destruir === 'function') {
        app.telaAtual.montado.destruir();
      }
    } catch (erro) {
      // Tela que estoura ao sair nao pode impedir a proxima de abrir: o
      // operador ficaria preso na tela quebrada.
      console.error('a tela ' + app.telaAtual.id + ' nao saiu limpa:', erro);
    }
    app.telaAtual = null;
  }
  palco.textContent = '';

  const definicao = TELAS[lugar.id];
  faixaTitulo.textContent = definicao.titulo;
  document.title = lugar.id === 'home' ? 'Caramelo' : 'Caramelo — ' + definicao.titulo;
  botaoVoltar.hidden = lugar.id === 'home';

  let modulo;
  try {
    modulo = await import(definicao.arquivo);
  } catch (erro) {
    console.error('nao consegui carregar a tela ' + lugar.id + ':', erro);
    if (geracao === app.geracao) { mostrarTelaQuebrada(definicao.titulo); }
    return;
  }
  if (geracao !== app.geracao) { return; }   // o operador ja' pediu outra tela

  const tela = modulo.tela || modulo.default;
  if (!tela || typeof tela.montar !== 'function') {
    mostrarTelaQuebrada(definicao.titulo);
    return;
  }
  if (tela.titulo) {
    faixaTitulo.textContent = tela.titulo;
    document.title = lugar.id === 'home' ? 'Caramelo' : 'Caramelo — ' + tela.titulo;
  }

  const raiz = document.createElement('div');
  raiz.className = 'tela';
  raiz.dataset.tela = lugar.id;
  palco.appendChild(raiz);

  const ctx = criarContexto(lugar);
  let montado;
  try {
    montado = tela.montar(raiz, ctx) || {};
  } catch (erro) {
    console.error('a tela ' + lugar.id + ' nao subiu:', erro);
    mostrarTelaQuebrada(tela.titulo || definicao.titulo);
    return;
  }
  app.telaAtual = {id: lugar.id, modulo, montado, raiz};

  // A secao pedida por outra tela ("marque onde o robo esta" leva a Operacao,
  // secao Localizacao). Quem nao expuser irParaSecao simplesmente abre na
  // primeira secao -- o atalho perde a pontaria, mas nao quebra.
  if (lugar.secao && typeof montado.irParaSecao === 'function') {
    try { montado.irParaSecao(lugar.secao); } catch (erro) { /* secao inexistente */ }
  }

  // A tela nasce sabendo o que ja' se sabe do robo. Sem isto ela ficaria ate'
  // um segundo em branco -- e um segundo em branco parece painel travado.
  if (app.ligado && typeof montado.aoEstado === 'function' && app.quadro) {
    try { montado.aoEstado(app.quadro); } catch (erro) { console.error(erro); }
  }
}

function mostrarTelaQuebrada(titulo) {
  palco.textContent = '';
  const caixa = document.createElement('div');
  caixa.className = 'telaErro';
  const t = document.createElement('h2');
  t.textContent = 'A tela ' + titulo + ' não abriu.';
  const p1 = document.createElement('p');
  p1.textContent = 'O resto do painel continua funcionando. Volte ao menu '
    + 'principal e siga por outro caminho.';
  const p2 = document.createElement('p');
  p2.className = 'dica';
  p2.textContent = 'Se isto acontecer sempre nesta tela, avise quem cuida do '
    + 'robô: é defeito do painel, não do robô.';
  const bt = document.createElement('button');
  bt.type = 'button';
  bt.textContent = 'Voltar ao menu principal';
  bt.onclick = voltar;
  caixa.appendChild(t);
  caixa.appendChild(p1);
  caixa.appendChild(p2);
  caixa.appendChild(bt);
  palco.appendChild(caixa);
}

/** O `ctx` que cada tela recebe. Tudo o que ela precisa saber do resto do
 *  painel passa por aqui -- nenhuma tela fala com outra diretamente. */
function criarContexto(lugar) {
  return {
    api,
    chave: CHAVE,
    token: CHAVE,
    // Propriedade, e nao funcao, e `undefined` ENQUANTO NAO SE SABE.
    //
    // telas/operacao.js le' este valor uma vez so', na montagem, e so' confia
    // nele quando e' booleano (`typeof ctx.autorizado === 'boolean'`); se nao
    // for, ela mesma pergunta ao robo. Devolver um palpite booleano aqui faria
    // ela guardar o palpite para sempre -- e quem abriu o painel com uma chave
    // ERRADA passaria a tarde com os botoes acesos, descobrindo a recusa um
    // comando por vez. As outras telas tratam undefined como "pode" (otimista)
    // e reavaliam a cada quadro, que e' o comportamento certo para elas.
    get autorizado() { return app.autorizado === null ? undefined : app.autorizado; },
    get estado() { return app.quadro; },
    get arena() { return arenaDoQuadro(); },
    // A casca NAO tem mapa proprio. Cada tela que desenha mapa cria o seu
    // (operacao e mapeamento ja' fazem isso) porque os modos de toque sao
    // diferentes em cada uma: um mapa compartilhado teria que ser reconfigurado
    // a cada troca de tela, e um clique caindo no modo da tela anterior vira
    // comando de movimento na hora errada.
    mapa: null,
    secao: lugar.secao,
    opcoes: {secao: lugar.secao},
    torrada, avisar: torrada, aviso: torrada, recado: torrada, notificar: torrada,
    irPara, abrir: irPara, abrirTela: irPara, navegar: irPara, trocarTela: irPara,
    voltar,
  };
}

function arenaDoQuadro() {
  const cartoes = app.quadro.cartoes || {};
  return (cartoes.robo || {}).arena_nome || '';
}

// ============================================================== WebSocket
//
// Uma conexao so' para o painel inteiro, e ela e' de LEITURA. Cada quadro vai
// para a tela viva; o resto (chips, torradas dos eventos) fica aqui.

let socket = null;
let tentativa = 0;
let relogioDeReconexao = null;

function ligarEstado() {
  clearTimeout(relogioDeReconexao);
  relogioDeReconexao = null;
  if (socket && (socket.readyState === WebSocket.OPEN ||
                 socket.readyState === WebSocket.CONNECTING)) {
    return;
  }
  const protocolo = location.protocol === 'https:' ? 'wss' : 'ws';
  try {
    socket = new WebSocket(protocolo + '://' + location.host + '/ws/estado');
  } catch (erro) {
    agendarReconexao();
    return;
  }

  socket.onopen = () => {
    tentativa = 0;
    app.ligado = true;
    // O painel pode ter reiniciado com outra chave sorteada. Perguntar de novo
    // a cada volta e' o que evita botao aceso que so' responde 403.
    conferirAcesso();
  };

  socket.onmessage = (evento) => {
    let bruto;
    try { bruto = JSON.parse(evento.data); } catch (erro) { return; }
    app.ligado = true;
    app.ultimoQuadro = Date.now();
    entregarQuadro(bruto);
  };

  socket.onclose = () => {
    socket = null;
    app.ligado = false;
    agendarReconexao();
  };

  // Nao fecha aqui: o onclose vem logo atras e fechar duas vezes agendaria
  // duas reconexoes, que viram quatro, que viram oito.
  socket.onerror = () => {};
}

function agendarReconexao() {
  if (relogioDeReconexao) { return; }
  const espera = RECUOS[Math.min(tentativa, RECUOS.length - 1)];
  tentativa += 1;
  relogioDeReconexao = setTimeout(() => {
    relogioDeReconexao = null;
    ligarEstado();
  }, espera);
}

/** Completa o quadro parcial e entrega para a tela viva.
 *
 * O robo manda pose a 10 Hz e o bloco pesado (cartoes, sensores, capacidades,
 * componentes, diagnostico) a 1 Hz. Entregar o quadro cru faria uma tela que
 * le' `estado.capacidades.missao` piscar entre "pode" e "nao pode" nove vezes
 * por segundo. Aqui cada chave lenta que NAO veio e' a ultima que veio -- que
 * e' a verdade, ja' que nada ali muda dez vezes por segundo.
 *
 * pose e scan NUNCA sao completados: dado velho de posicao e' mentira, e o
 * proprio robo ja' marca `velho` quando e' o caso.
 */
const CHAVES_LENTAS = ['menu', 'cartoes', 'componentes', 'sensores',
                       'capacidades', 'diagnostico'];

function entregarQuadro(bruto) {
  CHAVES_LENTAS.forEach((chave) => {
    if (bruto[chave] !== undefined && bruto[chave] !== null) {
      app.lentas[chave] = bruto[chave];
    } else if (app.lentas[chave] !== undefined) {
      bruto[chave] = app.lentas[chave];
    }
  });
  app.quadro = bruto;

  anotarEventos(bruto.eventos, bruto.serie_evento);
  pintarChips(bruto);

  const viva = app.telaAtual;
  if (viva && viva.montado && typeof viva.montado.aoEstado === 'function') {
    try {
      viva.montado.aoEstado(bruto);
    } catch (erro) {
      // Um quadro que derruba a tela nao pode derrubar o painel: o proximo
      // chega em 100 ms e provavelmente passa.
      console.error('a tela ' + viva.id + ' tropecou num quadro:', erro);
    }
  }
}

/** Os recados do robo (destino aceito, missao terminada, dock falhou) viram
 *  torrada uma vez so'. A serie `n` e' o que impede o mesmo recado de aparecer
 *  de novo a cada quadro. */
function anotarEventos(eventos, serie) {
  // O painel do robo pode reiniciar com esta aba aberta; ai a serie recomeca do
  // zero e todo recado novo teria numero MENOR do que o ultimo que ja' foi
  // mostrado -- e nenhum recado apareceria nunca mais, sem sinal nenhum de que
  // algo esta errado. Serie que anda para tras significa "outro servidor".
  if (typeof serie === 'number' && serie < app.ultimoEvento) { app.ultimoEvento = 0; }
  (eventos || []).forEach((evento) => {
    if (!evento || evento.n <= app.ultimoEvento) { return; }
    app.ultimoEvento = evento.n;
    torrada(evento.texto, evento.ok !== false);
  });
}

// ================================================================== chips
// A faixa superior responde, sem ninguem entrar em tela nenhuma: o robo esta'
// vivo? em que arena? da' para mandar ele andar? tem prova rodando?

function pintarChip(no, texto, classe) {
  if (!no) { return; }
  no.textContent = texto;
  no.className = 'chip ' + (classe || 'mudo');
}

// A cor segue a GRAVIDADE, e nao um ciano fixo como no tema.qss: na web o chip
// fica sozinho na faixa, e "Sem conexao com o robo" escrito na mesma cor de
// "Pronto para operar" le-se como se estivesse tudo bem.
const COR_DO_ESTADO = {
  'Pronto para operar': 'ok',
  'Ligado, mas sem saber onde esta': 'aviso',
  'Sem conexao com o robo': 'erro',
  'Aguardando o robo responder': 'mudo',
};

function pintarChips(quadro) {
  const robo = (quadro.cartoes || {}).robo || {};
  if (robo.estado) {
    pintarChip(CHIPS.robo, robo.estado, COR_DO_ESTADO[robo.estado] || 'frio');
  }

  if (robo.arena_nome) {
    // Arena escolhida que nao existe mais no robo ja' aconteceu em campo, e a
    // tela dizia que estava tudo pronto.
    pintarChip(CHIPS.arena, 'arena: ' + robo.arena_nome,
               robo.arena_existe === false ? 'aviso' : 'frio');
  } else if (robo.arena) {
    pintarChip(CHIPS.arena, 'nenhuma arena escolhida', 'aviso');
  }

  const capacidades = quadro.capacidades || {};
  if (Object.keys(capacidades).length) {
    pintarChip(CHIPS.navegacao,
               capacidades.navegacao ? 'navegação pronta' : 'navegação fora do ar',
               capacidades.navegacao ? 'ok' : 'aviso');
  }

  const missao = quadro.missao;
  if (missao && missao.estado && missao.estado !== 'parado') {
    const passo = (missao.total ? ' ' + (missao.passo + 1) + '/' + missao.total : '');
    pintarChip(CHIPS.missao, 'prova: ' + missao.estado + passo,
               missao.estado === 'falhou' || missao.estado === 'abortada' ? 'erro'
                 : (missao.estado === 'concluida' ? 'ok' : 'frio'));
  } else {
    pintarChip(CHIPS.missao, 'sem prova', 'mudo');
  }
}

function pintarChipDaChave() {
  if (app.autorizado === null) {
    pintarChip(CHIPS.chave, CHAVE ? 'conferindo a chave…' : 'só para olhar', 'mudo');
    return;
  }
  pintarChip(CHIPS.chave,
             app.autorizado ? 'pode comandar' : 'só para olhar',
             app.autorizado ? 'ok' : 'aviso');
  CHIPS.chave.title = app.autorizado
    ? 'Este painel foi aberto com a chave: dá para comandar o robô.'
    : 'Este painel foi aberto sem a chave de acesso. Dá para acompanhar tudo, '
      + 'mas não dá para comandar o robô nem abrir o terminal. Peça o link '
      + 'completo a quem ligou o robô.';
}

/** Sem quadro ha' alguns segundos, a faixa PARA de afirmar. O que ela sabia
 *  era verdade ha' tres segundos; continuar mostrando verde seria inventar. */
function vigiarSinal() {
  if (!app.ultimoQuadro) { return; }
  if (Date.now() - app.ultimoQuadro < LIMITE_SEM_QUADRO) { return; }
  pintarChip(CHIPS.robo, 'sem contato com o painel', 'erro');
  pintarChip(CHIPS.navegacao, 'navegação: —', 'mudo');
  pintarChip(CHIPS.missao, 'sem prova', 'mudo');
}

// =================================================================== inicio

/** "Esta chave vale?" -- perguntado ao robo, e nao adivinhado aqui.
 *
 * Falha de rede NAO vira "nao vale": o painel diria "so' para olhar" a alguem
 * que tem a chave certa, e essa pessoa passaria a tarde achando que o link
 * dela quebrou. Enquanto nao ha' resposta, a pergunta continua sem resposta
 * (undefined) e as telas seguem otimistas -- quem recusa de verdade e' o
 * servidor, com a frase dele. */
async function conferirAcesso() {
  const resposta = await ler('/api/acesso');
  if (!resposta || resposta.autorizado === undefined) {
    setTimeout(conferirAcesso, 3000);
    return;
  }
  app.autorizado = !!resposta.autorizado;
  pintarChipDaChave();
}

function comecar() {
  botaoVoltar.addEventListener('click', voltar);
  window.addEventListener('hashchange', abrirLugar);

  // Voltar do sono (tablet com a tela apagada, aba escondida) fecha o socket
  // sem avisar. Tentar na hora e' o que evita o painel voltar congelado.
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) {
      tentativa = 0;
      ligarEstado();
    }
  });
  window.addEventListener('online', () => { tentativa = 0; ligarEstado(); });

  pintarChipDaChave();
  conferirAcesso();
  // O vigia comeca a contar AGORA, e nao no primeiro quadro. Sem isto, um
  // painel que nunca recebe quadro nenhum (o ROS nao subiu) ficaria com os
  // chips escritos "ligando..." para sempre, que le-se como pagina carregando
  // e nao como robo sem resposta.
  app.ultimoQuadro = Date.now();
  ligarEstado();
  setInterval(vigiarSinal, 1000);
  abrirLugar();
}

comecar();
