/* Contexto OPERACAO -- a mesma tela que a GUI Qt abre no cartao "Operacao".
 *
 * As secoes sao as de main_window.cpp (construirContextos), na mesma ordem e
 * com os mesmos nomes: Mapa, Navegar, Waypoints, Docking, Localizacao, Teleop.
 * O mapa fica no centro e as secoes num painel de 330 px a direita, que e o
 * arranjo do tema.qss (molduraMapa + painelControle): aqui o mapa e o
 * instrumento de trabalho, e formulario esticado em tela cheia fica ilegivel.
 *
 * Por que abas de secao e nao tudo empilhado: e o que a GUI faz
 * (ContextScreen empilha as secoes e mostra uma por vez). Com uma excecao
 * deliberada -- o FREIO fica fora da pilha, numa barra fixa embaixo do painel.
 * Um painel que sabe dar partida e nao sabe dar freio e pior que um que nao faz
 * nenhum dos dois, e o botao nao pode estar tres cliques longe quando o robo
 * esta andando para o lugar errado.
 *
 * DEPENDE DE OUTROS DOIS ARQUIVOS. Tudo que atravessa a fronteira passa pelas
 * duas pontes daqui de baixo (criarAcesso e ligarMapa) -- se a integracao mudar
 * um nome, muda AQUI, num lugar so:
 *
 *   ../mapa_canvas.js   criarMapaCanvas(hospedeiro, opcoes) -> carregarArena,
 *                       definirEstado, definirModo, destacar, definirCamada,
 *                       camadas, enquadrar, destruir. O gesto do "2D Goal"
 *                       volta em aoEscolherPose({x, y, yaw, arrastou, modo}),
 *                       ja em metros do mapa.
 *   ctx (do app.js)     acesso a API e ao estado. Usa-se ctx.api.comandar /
 *                       ctx.api.ler quando existirem; senao cai no fetch com a
 *                       chave da URL, que e a convencao do painel.
 *
 * A instancia do mapa e DESTA tela, e nao da casca: os retornos de chamada do
 * mapa_canvas so existem na hora de cria-lo, entao um mapa emprestado seria um
 * mapa em que o clique nao comanda nada. E a mesma escolha de mapas.js e
 * mapeamento.js.
 */
import * as MapaCanvas from '../mapa_canvas.js';

// --------------------------------------------------------------- constantes

// Mesmos ids e titulos de MENU_PRINCIPAL (ros_node.py), que por sua vez copia
// main_window.cpp. Repetido aqui so para a tela poder montar as abas antes do
// primeiro quadro do WebSocket chegar.
const SECOES = [
  { id: 'mapa', titulo: 'Mapa' },
  { id: 'navegar', titulo: 'Navegar' },
  { id: 'waypoints', titulo: 'Waypoints' },
  { id: 'docking', titulo: 'Docking' },
  { id: 'localizacao', titulo: 'Localizacao' },
  { id: 'teleop', titulo: 'Teleop' },
];

// O que um clique no mapa faz. Um botao que apenas ARMA uma ferramenta nao
// parece ter feito nada (a GUI ja tropecou nisso), entao o modo escolhido
// aparece pressionado E explicado em texto embaixo do mapa.
const MODOS = [
  {
    id: 'ir', rotulo: 'Mandar o robo ate o ponto',
    dica: 'Clique no mapa onde o robo deve chegar. Arrastando, a seta escolhe ' +
          'para que lado ele fica virado quando chegar.',
  },
  {
    id: 'pose', rotulo: 'Dizer onde o robo esta',
    dica: 'Clique em cima do lugar onde o robo esta de verdade e arraste na ' +
          'direcao para onde ele aponta. Depois gire o robo um pouco no lugar.',
  },
  {
    id: 'nenhum', rotulo: 'So olhar',
    dica: 'O clique nao comanda nada. Arraste para mover a vista; dois dedos ' +
          '(ou a roda do mouse) aproximam e afastam.',
  },
];

// Tetos do robo. Sao os mesmos do controller_server e os mesmos que o servidor
// aplica de novo por cima do que a tela mandar: quem dirige pela rede tem
// atraso de video e nao pode andar mais rapido do que o robo anda sozinho.
const LIMITES = { vx: 0.20, vy: 0.16, wz: 0.50 };

const MARCHAS = [
  { id: 'devagar', rotulo: 'Devagar', fator: 0.30 },
  { id: 'normal', rotulo: 'Normal', fator: 0.60 },
  { id: 'rapido', rotulo: 'Rapido', fator: 1.00 },
];

// 10 pulsos por segundo. O freio de verdade esta no servidor (300 ms sem
// comando novo e ele publica zero), entao o navegador so precisa manter o
// fluxo enquanto o dedo estiver no botao. Se a aba congelar, travar ou perder
// a rede, e o SILENCIO que freia o robo.
const PULSO_MS = 100;

// Sinal de cada direcao, em fracao do teto de cada eixo. y positivo e para a
// esquerda e wz positivo gira para a esquerda, como no resto do projeto.
const DIRECOES = {
  frente: { vx: 1, vy: 0, wz: 0 },
  frenteEsquerda: { vx: 1, vy: 1, wz: 0 },
  esquerda: { vx: 0, vy: 1, wz: 0 },
  trasEsquerda: { vx: -1, vy: 1, wz: 0 },
  tras: { vx: -1, vy: 0, wz: 0 },
  trasDireita: { vx: -1, vy: -1, wz: 0 },
  direita: { vx: 0, vy: -1, wz: 0 },
  frenteDireita: { vx: 1, vy: -1, wz: 0 },
  giroEsquerda: { vx: 0, vy: 0, wz: 1 },
  giroDireita: { vx: 0, vy: 0, wz: -1 },
};

const TECLAS = {
  w: 'frente', s: 'tras', a: 'esquerda', d: 'direita',
  q: 'giroEsquerda', e: 'giroDireita',
  ArrowUp: 'frente', ArrowDown: 'tras',
  ArrowLeft: 'esquerda', ArrowRight: 'direita',
};

// Estados de missao em que ainda ha o que abortar (ESTADOS_MISSAO do ros_node).
const MISSAO_ANDANDO = ['planejando', 'pre-flight', 'executando'];

const SEM_CHAVE =
  'Este painel foi aberto sem a chave de acesso: da para acompanhar, mas nao ' +
  'da para comandar. Peca o link completo a quem ligou o robo.';

// ------------------------------------------------------------------ estilo
//
// Fica aqui, e nao no tema.css, porque tema.css e de outro arquivo/dono; estas
// regras existem so enquanto esta tela estiver montada. As cores saem todas do
// tema.qss e vao por variavel com valor de reserva: se o tema.css definir a
// variavel, ela manda; se nao, a tela continua com a cor certa.
const ESTILO = `
.op { display:grid; grid-template-columns:minmax(0,1fr) 330px; gap:12px;
      height:100%; min-height:0; min-width:0; }
.op-col-mapa { display:flex; flex-direction:column; gap:6px; min-width:0; min-height:0; }
.op-moldura { position:relative; flex:1; min-height:0; overflow:hidden;
              background:var(--painel,#101e35); border:1px solid var(--borda,#1b3457);
              border-radius:16px; }
.op-mapa-tela { position:absolute; inset:0; }
.op-mapa-vazio { position:absolute; inset:0; display:flex; align-items:center;
                 justify-content:center; text-align:center; padding:24px;
                 color:var(--fraco,#7d9cc4); font-size:14px; line-height:1.5; }
.op-dica { color:var(--fraco,#7d9cc4); font-size:12px; padding:2px 4px; min-height:32px; }

.op-painel { display:flex; flex-direction:column; min-height:0; min-width:0;
             overflow:hidden; background:var(--painel,#101e35);
             border:1px solid var(--borda,#1b3457); border-radius:16px; }
.op-abas { display:flex; flex-wrap:wrap; gap:4px; padding:8px;
           border-bottom:1px solid #14294a; }
.op-abas button { flex:1 1 auto; min-width:86px; min-height:36px; padding:6px 8px;
                  border:none; border-radius:10px; background:transparent;
                  color:var(--fraco,#7d9cc4); font:inherit; font-size:13px;
                  font-weight:700; cursor:pointer; }
.op-abas button:hover { background:#12233f; color:#bfe3ff; }
.op-abas button[aria-selected="true"] { background:#103154; color:var(--ciano,#35c3f0); }

.op-corpo { flex:1; min-height:0; overflow-y:auto; overflow-x:hidden; padding:12px; }
.op-secao { display:none; flex-direction:column; gap:10px; }
.op-secao.ativa { display:flex; }
.op-pilha { display:flex; flex-direction:column; gap:8px; }

.op-titulo { color:var(--fraco,#7d9cc4); font-size:12px; font-weight:800;
             letter-spacing:1px; }
.op-texto { color:#8fb0d8; font-size:13px; line-height:1.45; }
.op-motivo { color:#ffd28f; font-size:12px; line-height:1.4; }
.op-motivo:empty { display:none; }
.op-linha { display:flex; justify-content:space-between; align-items:baseline;
            gap:8px; font-size:13px; }
.op-linha b { font-weight:700; color:#dcecff; text-align:right; }
.op-linha span { color:var(--fraco,#7d9cc4); }

.op-bt { min-height:44px; padding:10px 14px; border-radius:12px;
         border:1px solid #1f3a5f; background:#142643; color:#dcecff;
         font:inherit; font-weight:700; text-align:left; cursor:pointer; }
.op-bt:hover:not(:disabled) { background:#1b3457; border-color:var(--ciano,#35c3f0); color:#fff; }
.op-bt:disabled { color:#3d5a82; border-color:#14294a; cursor:not-allowed; }
.op-bt[aria-pressed="true"] { background:#103154; border-color:var(--ciano,#35c3f0);
                              color:var(--ciano,#35c3f0); }
.op-bt.primaria { background:var(--laranja,#ff9a2e); color:#2b1600; border:none;
                  font-weight:800; text-align:center; }
.op-bt.primaria:hover:not(:disabled) { background:#f07f0e; }
.op-bt.primaria:disabled { background:#3a2e1c; color:#7a6a52; }

.op-sel { width:100%; min-height:44px; padding:10px 12px; border-radius:10px;
          border:1px solid #1f3a5f; background:#0e1c31; color:var(--texto,#e6f1ff);
          font:inherit; font-weight:600; }
.op-sel:disabled { color:#40597d; background:#0a1424; }

.op-chip { align-self:flex-start; padding:5px 12px; border-radius:10px;
           font-size:12px; font-weight:800; color:#06121f; background:#7f93b0; }

.op-marchas { display:flex; gap:6px; }
.op-marchas button { flex:1; text-align:center; }
.op-volante { display:grid; grid-template-columns:repeat(3,1fr); gap:6px; }
.op-volante button { min-height:60px; font-size:20px; line-height:1.1;
                     display:flex; flex-direction:column; align-items:center;
                     justify-content:center; gap:2px; text-align:center;
                     touch-action:none; user-select:none; -webkit-user-select:none; }
.op-volante button small { font-size:10px; font-weight:600; color:var(--fraco,#7d9cc4); }
.op-volante button.parar { color:#ff9d9d; border-color:#7d2440; background:#3a1420;
                           font-size:14px; font-weight:800; }
.op-giro { display:grid; grid-template-columns:repeat(2,1fr); gap:6px; }
.op-giro button { min-height:52px; text-align:center; font-size:14px;
                  touch-action:none; user-select:none; -webkit-user-select:none; }

.op-rodape { border-top:1px solid #14294a; background:var(--topo,#081120);
             padding:10px; display:flex; flex-direction:column; gap:6px; }
.op-recado { font-size:12px; color:#8fb0d8; line-height:1.4; min-height:16px; }
.op-recado.ruim { color:#ff9d9d; }
.op-parar { min-height:52px; border-radius:12px; border:1px solid #7d2440;
            background:#3a1420; color:#ff9d9d; font:inherit; font-size:15px;
            font-weight:800; cursor:pointer; }
.op-parar:hover { background:#4d1a2a; }
.op-ajuda-parar { font-size:11px; color:#6d86a8; line-height:1.35; }

/* Tablet e telas estreitas: o mapa vira faixa em cima e o painel desce. A
   pagina continua sem rolagem horizontal -- quem rola e o corpo do painel. */
@media (max-width: 980px) {
  .op { grid-template-columns:minmax(0,1fr);
        grid-template-rows:minmax(200px,40%) minmax(0,1fr); }
}
`;

function injetarEstilo() {
  if (document.getElementById('estilo-tela-operacao')) return;
  const marca = document.createElement('style');
  marca.id = 'estilo-tela-operacao';
  marca.textContent = ESTILO;
  document.head.appendChild(marca);
}

// ------------------------------------------------------------------ auxilio

function no(tag, classe, texto) {
  const elemento = document.createElement(tag);
  if (classe) elemento.className = classe;
  // textContent e nao innerHTML: nome de arena, de dock e de waypoint vem de
  // arquivo no disco do robo, e o painel nao pode virar uma via de injecao por
  // causa de um yaml mal escrito.
  if (texto !== undefined && texto !== null) elemento.textContent = texto;
  return elemento;
}

function botao(classe, texto, aoClicar) {
  const b = no('button', classe, texto);
  b.type = 'button';
  if (aoClicar) b.addEventListener('click', aoClicar);
  return b;
}

function linha(rotulo) {
  const raiz = no('div', 'op-linha');
  raiz.appendChild(no('span', '', rotulo));
  const valor = no('b', '', '—');
  raiz.appendChild(valor);
  return { raiz, valor };
}

/** Numero em portugues: 1,25 e nao 1.25. */
function numero(valor, casas) {
  if (valor === null || valor === undefined || !isFinite(valor)) return '—';
  return Number(valor).toFixed(casas === undefined ? 2 : casas).replace('.', ',');
}

function graus(yaw) {
  if (yaw === null || yaw === undefined || !isFinite(yaw)) return '—';
  return Math.round((yaw * 180) / Math.PI) + '°';
}

function primeiroMetodo(objeto, nomes) {
  for (const nome of nomes) {
    if (objeto && typeof objeto[nome] === 'function') return objeto[nome].bind(objeto);
  }
  return null;
}

// ------------------------------------------------------------------ pontes

/** Fala com a API do robo. Usa o app.js quando ele oferece; senao, fetch. */
function criarAcesso(ctx) {
  const fonte = (ctx && ctx.api) || ctx || {};
  const enviarDoApp = primeiroMetodo(fonte, ['comandar', 'post', 'enviar']);
  const lerDoApp = primeiroMetodo(fonte, ['ler', 'get', 'buscar']);
  const urlDoApp = primeiroMetodo(fonte, ['comChave', 'url', 'rota']);

  const chave = new URLSearchParams(location.search).get('t') || '';
  const url = (rota) => {
    if (urlDoApp) return urlDoApp(rota);
    if (!chave) return rota;
    return rota + (rota.indexOf('?') >= 0 ? '&' : '?') + 't=' + encodeURIComponent(chave);
  };

  async function ler(rota) {
    if (lerDoApp) return (await lerDoApp(rota)) || {};
    try {
      const resposta = await fetch(url(rota));
      return (await resposta.json()) || {};
    } catch {
      return {};
    }
  }

  async function comandar(rota, corpo) {
    if (enviarDoApp) return (await enviarDoApp(rota, corpo || {})) || {};
    try {
      const resposta = await fetch(url(rota), {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(corpo || {}),
      });
      const dados = (await resposta.json().catch(() => ({}))) || {};
      if (dados.ok === undefined) dados.ok = resposta.ok;
      return dados;
    } catch {
      return { ok: false, mensagem: 'Perdi contato com o robo.' };
    }
  }

  return { ler, comandar, url };
}

/**
 * Liga no mapa_canvas.js. Metodo que nao existe nao pode derrubar a tela no
 * meio da prova, por isso tudo passa por primeiroMetodo e vira funcao vazia
 * quando falta.
 */
function ligarMapa(hospedeiro, aoEscolherPose, aoAviso) {
  const fabrica = MapaCanvas.criarMapaCanvas || MapaCanvas.criarMapa ||
                  MapaCanvas.montarMapa || MapaCanvas.default;

  // Os modos desta tela em nome de gente ("ir", "pose", "nenhum") viram os do
  // mapa. A traducao mora aqui: as secoes falam do que o operador quer fazer,
  // nao do vocabulario do desenho.
  const modosDoMapa = MapaCanvas.MODOS || {};
  const TRADUCAO = {
    ir: modosDoMapa.NAVEGAR || 'navegar',
    pose: modosDoMapa.POSICIONAR || 'posicionar',
    nenhum: modosDoMapa.OLHAR || 'olhar',
  };

  let instancia = null;
  if (typeof fabrica === 'function' && hospedeiro) {
    try {
      instancia = fabrica(hospedeiro, { interativo: true, aoEscolherPose, aoAviso });
    } catch (erro) {
      // Sem desenho a tela continua servindo para dockar, mandar o robo a um
      // ponto salvo e dirigir na mao. Ficar em branco seria pior.
      console.error('o desenho do mapa nao subiu:', erro);
      instancia = null;
    }
  }

  const carregarArena = primeiroMetodo(instancia, ['carregarArena', 'definirArena', 'setArena']);
  const definirEstado = primeiroMetodo(instancia, ['definirEstado', 'aoEstado', 'atualizar']);
  const definirModo = primeiroMetodo(instancia, ['definirModo', 'definirFerramenta']);
  const destacar = primeiroMetodo(instancia, ['destacar', 'realcar']);
  const definirCamada = primeiroMetodo(instancia, ['definirCamada', 'mostrarCamada']);
  const listarCamadas = primeiroMetodo(instancia, ['camadas', 'listarCamadas']);
  const enquadrar = primeiroMetodo(instancia, ['enquadrar', 'ajustar']);
  const destruir = primeiroMetodo(instancia, ['destruir', 'desmontar', 'fechar']);

  return {
    existe: !!instancia,
    temCamadas: !!(definirCamada && listarCamadas),
    temEnquadrar: !!enquadrar,
    // O painel ja leu os metadados para montar as listas; passar a meta pronta
    // evita o mapa buscar a mesma coisa de novo.
    carregarArena: (nome, meta) => { if (carregarArena) carregarArena(nome, meta); },
    definirEstado: (quadro) => { if (definirEstado) definirEstado(quadro); },
    definirModo: (modo) => { if (definirModo) definirModo(TRADUCAO[modo] || TRADUCAO.nenhum); },
    destacar: (tipo, id) => { if (destacar) destacar(tipo || '', id || ''); },
    definirCamada: (id, ligada) => { if (definirCamada) definirCamada(id, ligada); },
    camadas: () => (listarCamadas ? listarCamadas() || [] : []),
    enquadrar: () => { if (enquadrar) enquadrar(); },
    destruir: () => { if (destruir) destruir(); },
  };
}

// ------------------------------------------------------------------- a tela

export const tela = {
  id: 'operacao',
  titulo: 'Operacao',
  precisaDoMapa: true,

  montar(raiz, ctx) {
    injetarEstilo();

    const acesso = criarAcesso(ctx);
    const dados = {
      arena: '',
      meta: null,
      estado: {},
      capacidades: {},
      componentes: [],
      modo: 'ir',
      marcha: 'normal',
      indo: null,           // {rotulo, x, y} do ultimo destino aceito
      autorizado: !!(ctx && ctx.autorizado),
    };
    let ultimoEvento = 0;
    let secaoAtual = 'mapa';
    let vivo = true;

    // --- casca ---------------------------------------------------------
    const casca = no('div', 'op');
    const colunaMapa = no('div', 'op-col-mapa');
    const moldura = no('div', 'op-moldura');
    const hospedeiroMapa = no('div', 'op-mapa-tela');
    moldura.appendChild(hospedeiroMapa);
    // So aparece quando nao ha arena nenhuma para desenhar: o mapa_canvas ja
    // escreve o proprio recado, mas ele nao sabe que a arena se escolhe na tela
    // Mapas -- e "Escolha uma arena" sem dizer onde nao ajuda ninguem.
    const avisoSemMapa = no('div', 'op-mapa-vazio', '');
    avisoSemMapa.style.display = 'none';
    moldura.appendChild(avisoSemMapa);
    colunaMapa.appendChild(moldura);
    const dicaMapa = no('div', 'op-dica', '');
    colunaMapa.appendChild(dicaMapa);
    casca.appendChild(colunaMapa);

    const painelLateral = no('aside', 'op-painel');
    const abas = no('div', 'op-abas');
    const corpo = no('div', 'op-corpo');
    const rodape = no('div', 'op-rodape');
    painelLateral.appendChild(abas);
    painelLateral.appendChild(corpo);
    painelLateral.appendChild(rodape);
    casca.appendChild(painelLateral);
    raiz.appendChild(casca);

    // --- o freio, fora da pilha de secoes -------------------------------
    const recado = no('div', 'op-recado', 'Pronto.');
    const btParar = botao('op-parar', 'Parar o robo', () => frear());
    const ajudaParar = no(
      'div', 'op-ajuda-parar',
      'Cancela o destino atual, para a direcao manual e aborta a missao que ' +
      'estiver rodando. Nao e um botao de emergencia: este robo nao tem um.');
    rodape.appendChild(recado);
    rodape.appendChild(btParar);
    rodape.appendChild(ajudaParar);

    // --- painel compartilhado com as secoes ------------------------------
    const painel = {
      dados,
      acesso,
      dizer,
      comandar,
      componente,
      armar,
      destacar: (tipo, id) => mapa.destacar(tipo, id),
      enquadrar: () => mapa.enquadrar(),
      camadas: () => mapa.camadas(),
      definirCamada: (id, ligada) => mapa.definirCamada(id, ligada),
      temCamadas: false,
      temEnquadrar: false,
      atualizar: () => atualizarSecoes(),
      motivoSemChave: () => (dados.autorizado ? '' : SEM_CHAVE),
    };

    const mapa = ligarMapa(hospedeiroMapa, aoGestoDoMapa,
                          (texto) => dizer(texto, false));
    painel.temCamadas = mapa.temCamadas;
    painel.temEnquadrar = mapa.temEnquadrar;

    const secoes = [
      secaoMapa(painel),
      secaoNavegar(painel),
      secaoWaypoints(painel),
      secaoDocking(painel),
      secaoLocalizacao(painel),
      secaoTeleop(painel),
    ];
    const porId = {};
    secoes.forEach((secao) => {
      porId[secao.id] = secao;
      corpo.appendChild(secao.raiz);
    });

    const botoesAba = SECOES.map((definicao) => {
      const b = botao('', definicao.titulo, () => irParaSecao(definicao.id));
      b.setAttribute('aria-selected', 'false');
      abas.appendChild(b);
      return { id: definicao.id, b };
    });

    function irParaSecao(id) {
      // Sair do Teleop e um comando de PARAR: trocar de secao com o robo
      // andando e o jeito mais facil de deixa-lo solto sem ninguem olhando.
      if (secaoAtual !== id && porId[secaoAtual] && porId[secaoAtual].sair) {
        porId[secaoAtual].sair();
      }
      secaoAtual = id;
      botoesAba.forEach((item) => {
        item.b.setAttribute('aria-selected', String(item.id === id));
      });
      secoes.forEach((secao) => secao.raiz.classList.toggle('ativa', secao.id === id));
      atualizarSecoes();
    }

    // --- estado e recados -------------------------------------------------
    const torrada = primeiroMetodo(ctx || {}, ['torrada', 'aviso', 'recado', 'notificar']);

    function dizer(texto, ok) {
      if (!texto) return;
      // O recado fica na barra de baixo, que nunca rola junto com as secoes:
      // a resposta do robo tem que ser lida na secao em que o operador estiver,
      // e nao so naquela de onde ele mandou o comando.
      recado.textContent = texto;
      recado.classList.toggle('ruim', ok === false);
      if (torrada) torrada(texto, ok !== false);
    }

    async function comandar(rota, corpo) {
      if (!dados.autorizado) {
        dizer(SEM_CHAVE, false);
        return { ok: false };
      }
      const resposta = await acesso.comandar(rota, corpo);
      if (resposta && resposta.mensagem) dizer(resposta.mensagem, resposta.ok !== false);
      return resposta || {};
    }

    function componente(id) {
      return (dados.componentes || []).find((c) => c.id === id) || null;
    }

    function armar(modo) {
      dados.modo = modo;
      mapa.definirModo(modo);
      const escolhido = MODOS.find((m) => m.id === modo) || MODOS[0];
      if (dicaMapa) dicaMapa.textContent = escolhido.dica;
      atualizarSecoes();
    }

    function atualizarSecoes() {
      secoes.forEach((secao) => { if (secao.atualizar) secao.atualizar(); });
    }

    // --- o gesto no mapa --------------------------------------------------
    async function aoGestoDoMapa(gesto) {
      if (!vivo || !gesto) return;
      const arrastou = gesto.arrastou !== undefined ? gesto.arrastou
                     : (gesto.arrastado !== undefined ? gesto.arrastado : true);
      const x = Number(gesto.x);
      const y = Number(gesto.y);
      if (!isFinite(x) || !isFinite(y)) return;
      const yaw = isFinite(gesto.yaw) ? Number(gesto.yaw) : 0;

      if (dados.modo === 'ir') {
        // Sem arrasto o operador nao escolheu para que lado o robo termina. O
        // destino precisa de uma direcao de qualquer jeito, entao vale a que
        // ele ja tem: mandar zero faria o robo girar para o leste ao chegar,
        // sem ninguem ter pedido.
        const pose = dados.estado.pose;
        const direcao = arrastou ? yaw : (pose ? pose.yaw : 0);
        const resposta = await comandar('/api/meta', { x, y, yaw: direcao });
        if (resposta.ok !== false) {
          dados.indo = { rotulo: 'ponto marcado no mapa', x, y };
          atualizarSecoes();
        }
      } else if (dados.modo === 'pose') {
        // Aqui a direcao NAO pode ser chutada: dizer que o robo esta virado
        // para o leste quando ele esta virado para a parede poe a localizacao
        // pior do que estava.
        if (!arrastou) {
          dizer('Falta dizer para que lado o robo esta virado: clique em cima ' +
                'dele e ARRASTE na direcao para onde ele aponta.', false);
          return;
        }
        await comandar('/api/pose_inicial', { x, y, yaw });
      }
    }

    // --- freio ------------------------------------------------------------
    async function frear() {
      // Ordem igual a de MainWindow (o "Parar o robo" do menu principal):
      // primeiro o volante, depois o destino, depois a missao.
      const teleop = porId.teleop;
      if (teleop && teleop.parar) teleop.parar(true);
      dados.indo = null;
      atualizarSecoes();
      await comandar('/api/navegacao/cancelar');
      const missao = dados.estado.missao;
      const ocupada = dados.capacidades && dados.capacidades.missao_ocupada;
      if (ocupada || (missao && MISSAO_ANDANDO.indexOf(missao.estado) >= 0)) {
        await comandar('/api/missao/abortar');
      }
    }

    // --- arena -------------------------------------------------------------
    async function carregarArena(nome) {
      if (!nome) {
        dados.arena = '';
        dados.meta = null;
        mapa.carregarArena('', null);
        mostrarSemArena('Nenhuma arena escolhida ainda. Escolha em qual arena o ' +
                        'robo esta na tela Mapas.');
        secoes.forEach((secao) => { if (secao.aoArena) secao.aoArena(null); });
        atualizarSecoes();
        return;
      }
      const meta = await acesso.ler('/api/arenas/' + encodeURIComponent(nome) + '/meta');
      if (!vivo) return;
      if (!meta || meta.erro) {
        dados.arena = nome;
        dados.meta = null;
        // Limpa o desenho junto: mapa mostrando a arena anterior enquanto as
        // listas do painel estao vazias sao duas telas contando historias
        // diferentes sobre onde o robo esta.
        mapa.carregarArena('', null);
        mostrarSemArena((meta && meta.erro) ||
                        'Nao consegui abrir os dados desta arena.');
        secoes.forEach((secao) => { if (secao.aoArena) secao.aoArena(null); });
        atualizarSecoes();
        return;
      }
      dados.arena = nome;
      dados.meta = meta;
      mostrarSemArena('');
      mapa.carregarArena(nome, meta);
      secoes.forEach((secao) => { if (secao.aoArena) secao.aoArena(meta); });
      atualizarSecoes();
    }

    function mostrarSemArena(texto) {
      avisoSemMapa.textContent = texto || '';
      avisoSemMapa.style.display = texto ? 'flex' : 'none';
    }

    async function descobrirArena() {
      // Quando o app.js ja sabe em que arena o robo esta, essa e a resposta
      // boa: perguntar de novo abriria a porta para as duas telas mostrarem
      // arenas diferentes ao mesmo tempo.
      if (ctx && typeof ctx.arena === 'string' && ctx.arena) {
        await carregarArena(ctx.arena);
        return;
      }
      const lista = await acesso.ler('/api/arenas');
      if (!vivo) return;
      const nome = lista.ativa || (lista.arenas || [])[0] || '';
      await carregarArena(nome);
    }

    // --- ciclo de estado ---------------------------------------------------
    function aoEstado(quadro) {
      if (!vivo || !quadro) return;
      mapa.definirEstado(quadro);

      // O quadro do WebSocket e parcial de proposito: cartoes, componentes e
      // capacidades so vao uma vez por segundo. Guardar o ultimo que chegou e o
      // comportamento certo -- zerar o que faltou no quadro faria os botoes
      // piscarem entre "pode" e "nao pode" dez vezes por segundo.
      Object.assign(dados.estado, quadro);
      if (quadro.capacidades) dados.capacidades = quadro.capacidades;
      if (quadro.componentes) dados.componentes = quadro.componentes;

      const novos = (quadro.eventos || []).filter((e) => (e.n || 0) > ultimoEvento);
      if (novos.length) {
        ultimoEvento = novos[novos.length - 1].n || ultimoEvento;
        const ultimo = novos[novos.length - 1];
        dizer(ultimo.texto, ultimo.ok !== false);
        // Sem um "esta indo" no estado do robo, e o recado dele que fecha a
        // viagem: chegou, parou a pedido ou nao conseguiu -- os tres sao
        // eventos, e qualquer um deles encerra o acompanhamento do destino.
        dados.indo = null;
      }

      // A arena ativa muda em outra tela (Mapas). Reconferir de graca aqui
      // evita o painel continuar mostrando os docks da arena anterior.
      const robo = quadro.cartoes && quadro.cartoes.robo;
      if (robo && robo.arena_nome !== undefined && robo.arena_nome !== dados.arena) {
        carregarArena(robo.arena_nome);
      }

      atualizarSecoes();
    }

    // --- teclado e foco ----------------------------------------------------
    function aoTeclaBaixo(ev) {
      if (secaoAtual !== 'teleop' || ev.repeat) return;
      const alvo = ev.target;
      if (alvo && /^(INPUT|SELECT|TEXTAREA)$/.test(alvo.tagName)) return;
      if (ev.key === ' ') {
        ev.preventDefault();
        porId.teleop.parar(true);
        return;
      }
      const direcao = TECLAS[ev.key] || TECLAS[String(ev.key).toLowerCase()];
      if (direcao) {
        ev.preventDefault();
        porId.teleop.segurar(direcao);
      }
    }
    function aoTeclaCima(ev) {
      const direcao = TECLAS[ev.key] || TECLAS[String(ev.key).toLowerCase()];
      if (direcao) porId.teleop.soltar(direcao);
    }
    // Perder o foco da janela nao pode deixar o robo andando: sem foco o
    // navegador para de entregar o keyup e a tecla fica presa para sempre.
    function aoPerderFoco() { porId.teleop.parar(); }
    function aoEsconder() { if (document.hidden) porId.teleop.parar(); }

    window.addEventListener('keydown', aoTeclaBaixo);
    window.addEventListener('keyup', aoTeclaCima);
    window.addEventListener('blur', aoPerderFoco);
    document.addEventListener('visibilitychange', aoEsconder);

    // --- partida -----------------------------------------------------------
    (async () => {
      if (!(ctx && typeof ctx.autorizado === 'boolean')) {
        const resposta = await acesso.ler('/api/acesso');
        dados.autorizado = !!resposta.autorizado;
      }
      if (!vivo) return;
      if (!dados.autorizado) dizer(SEM_CHAVE, false);
      atualizarSecoes();
      await descobrirArena();
    })();

    if (ctx && ctx.estado) aoEstado(ctx.estado);
    irParaSecao('mapa');
    armar(dados.modo);

    return {
      aoEstado,
      // Exposto para a casca: outra tela pode pedir "abra Operacao na secao
      // Localizacao" (a Competicao faz isso quando o robo nao sabe onde esta),
      // e o endereco #/operacao/localizacao tem que cair na mesma secao depois
      // de um recarregamento.
      irParaSecao(id) {
        if (SECOES.some((definicao) => definicao.id === id)) irParaSecao(id);
      },
      destruir() {
        vivo = false;
        // Sair da tela freia o volante -- e so o volante. Cancelar navegacao
        // aqui frearia, sem ninguem pedir, um robo que estava indo sozinho.
        if (porId.teleop && porId.teleop.parar) porId.teleop.parar();
        window.removeEventListener('keydown', aoTeclaBaixo);
        window.removeEventListener('keyup', aoTeclaCima);
        window.removeEventListener('blur', aoPerderFoco);
        document.removeEventListener('visibilitychange', aoEsconder);
        mapa.destruir();
        if (casca.parentNode) casca.parentNode.removeChild(casca);
      },
    };
  },
};

// ------------------------------------------------------------- secao: Mapa

function secaoMapa(painel) {
  const raiz = no('div', 'op-secao');

  raiz.appendChild(no('div', 'op-titulo', 'O QUE O CLIQUE NO MAPA FAZ'));
  const grupoModos = no('div', 'op-pilha');
  const botoesModo = MODOS.map((modo) => {
    const b = botao('op-bt', modo.rotulo, () => painel.armar(modo.id));
    b.setAttribute('aria-pressed', 'false');
    grupoModos.appendChild(b);
    return { id: modo.id, b };
  });
  raiz.appendChild(grupoModos);
  const explicacao = no('div', 'op-texto', '');
  raiz.appendChild(explicacao);

  const grupoCamadas = no('div', 'op-pilha');
  grupoCamadas.appendChild(no('div', 'op-titulo', 'CAMADAS DO MAPA'));
  const caixaCamadas = no('div', 'op-pilha');
  grupoCamadas.appendChild(caixaCamadas);
  raiz.appendChild(grupoCamadas);
  grupoCamadas.style.display = 'none';

  const btEnquadrar = botao('op-bt', 'Enquadrar o mapa na tela', () => painel.enquadrar());
  raiz.appendChild(btEnquadrar);
  btEnquadrar.style.display = painel.temEnquadrar ? '' : 'none';

  raiz.appendChild(no('div', 'op-titulo', 'O QUE ESTA ARENA TEM'));
  const resumo = no('div', 'op-pilha');
  const linhas = {
    docks: linha('Estacoes (docks)'),
    areas: linha('Areas de trabalho'),
    waypoints: linha('Pontos salvos'),
    keepout: linha('Paredes virtuais'),
  };
  Object.values(linhas).forEach((l) => resumo.appendChild(l.raiz));
  raiz.appendChild(resumo);

  const avisos = no('div', 'op-pilha');
  raiz.appendChild(avisos);

  function aoArena(meta) {
    caixaCamadas.textContent = '';
    grupoCamadas.style.display = 'none';
    avisos.textContent = '';
    if (!meta) {
      Object.values(linhas).forEach((l) => { l.valor.textContent = '—'; });
      return;
    }
    linhas.docks.valor.textContent = String((meta.docks || []).length);
    linhas.areas.valor.textContent = String((meta.areas || []).length);
    linhas.waypoints.valor.textContent = String((meta.waypoints || []).length);
    linhas.keepout.valor.textContent = meta.tem_keepout ? 'sim' : 'nao';

    // A lista vem do proprio desenho, e nao dos metadados: la dentro estao
    // tambem as camadas AO VIVO (robo, LiDAR, caminho), que nao existem no
    // arquivo da arena e sao justamente as que o operador desliga quando o mapa
    // fica poluido. Camada so aparece se o mapa souber liga-la e desliga-la:
    // botao que existe e nao funciona e pior do que botao ausente.
    const camadas = painel.temCamadas ? painel.camadas() : [];
    if (camadas.length) {
      grupoCamadas.style.display = '';
      camadas.forEach((camada) => {
        const rotulo = no('label', 'op-texto');
        const caixa = document.createElement('input');
        caixa.type = 'checkbox';
        caixa.checked = camada.ligada !== false;
        caixa.style.marginRight = '8px';
        caixa.addEventListener('change', () => painel.definirCamada(camada.id, caixa.checked));
        rotulo.appendChild(caixa);
        rotulo.appendChild(document.createTextNode(camada.rotulo || camada.id));
        caixaCamadas.appendChild(rotulo);
      });
    }

    (meta.avisos || []).forEach((texto) => avisos.appendChild(no('div', 'op-motivo', texto)));
  }

  function atualizar() {
    const modo = painel.dados.modo;
    botoesModo.forEach((item) => item.b.setAttribute('aria-pressed', String(item.id === modo)));
    const escolhido = MODOS.find((m) => m.id === modo) || MODOS[0];
    explicacao.textContent = escolhido.dica;
  }

  return { id: 'mapa', raiz, atualizar, aoArena };
}

// ---------------------------------------------------------- secao: Navegar

function secaoNavegar(painel) {
  const raiz = no('div', 'op-secao');

  raiz.appendChild(no('div', 'op-titulo', 'NAVEGACAO'));
  const chip = no('div', 'op-chip', 'SEM DADOS');
  raiz.appendChild(chip);
  const situacao = no('div', 'op-texto', '');
  raiz.appendChild(situacao);
  const acao = no('div', 'op-motivo', '');
  raiz.appendChild(acao);

  const btEscolher = botao('op-bt primaria', 'Escolher destino no mapa',
                           () => painel.armar('ir'));
  raiz.appendChild(btEscolher);
  raiz.appendChild(no('div', 'op-texto',
    'Depois de escolher, clique no mapa onde o robo deve chegar. Arrastando, a ' +
    'seta escolhe para que lado ele fica virado quando chegar.'));

  raiz.appendChild(no('div', 'op-titulo', 'DESTINO ATUAL'));
  const alvo = linha('Indo para');
  const falta = linha('Falta andar');
  raiz.appendChild(alvo.raiz);
  raiz.appendChild(falta.raiz);

  const btCancelar = botao('op-bt', 'Cancelar o destino',
                           () => painel.comandar('/api/navegacao/cancelar'));
  raiz.appendChild(btCancelar);

  raiz.appendChild(no('div', 'op-titulo', 'QUANDO O ROBO TRAVA SEM MOTIVO'));
  raiz.appendChild(no('div', 'op-texto',
    'O robo guarda na memoria os obstaculos que ja viu. Se alguem passou na ' +
    'frente dele e foi embora, o vulto pode ter ficado la e o caminho parece ' +
    'fechado. Esquecer faz ele considerar so o que os sensores mostram agora.'));
  const btCostmaps = botao('op-bt', 'Esquecer os obstaculos ja vistos',
                           () => painel.comandar('/api/navegacao/costmaps'));
  raiz.appendChild(btCostmaps);
  const motivoCostmaps = no('div', 'op-motivo', '');
  raiz.appendChild(motivoCostmaps);

  function atualizar() {
    const dados = painel.dados;
    const nav = painel.componente('caramelo/nav2');
    chip.textContent = nav ? nav.rotulo : 'SEM DADOS';
    chip.style.background = nav ? nav.cor : '#7f93b0';
    situacao.textContent = nav ? nav.mensagem : 'Ainda nao sei como esta a navegacao.';
    acao.textContent = nav ? (nav.acao || '') : '';

    const semChave = painel.motivoSemChave();
    const podeNavegar = dados.capacidades.navegacao !== false;
    btEscolher.disabled = !!semChave;
    btEscolher.setAttribute('aria-pressed', String(dados.modo === 'ir'));
    btCancelar.disabled = !!semChave;
    btCostmaps.disabled = !!semChave || dados.capacidades.costmaps === false;

    if (semChave) {
      motivoCostmaps.textContent = semChave;
    } else if (dados.capacidades.costmaps === false) {
      motivoCostmaps.textContent =
        'A navegacao nao esta no ar: nao ha memoria de obstaculos para apagar.';
    } else {
      motivoCostmaps.textContent = '';
    }
    if (!podeNavegar && !semChave) {
      acao.textContent =
        'A navegacao nao esta no ar: o robo nao sabe ir a lugar nenhum. Ligue a ' +
        'navegacao no robo antes de marcar um destino.';
    }

    const indo = dados.indo;
    alvo.valor.textContent = indo
      ? indo.rotulo + ' (' + numero(indo.x) + ' m, ' + numero(indo.y) + ' m)'
      : 'nada agora';
    const pose = dados.estado.pose;
    falta.valor.textContent = (indo && pose)
      ? numero(Math.hypot(indo.x - pose.x, indo.y - pose.y), 1) + ' m'
      : '—';
  }

  return { id: 'navegar', raiz, atualizar };
}

// -------------------------------------------------------- secao: Waypoints

function secaoWaypoints(painel) {
  const raiz = no('div', 'op-secao');

  raiz.appendChild(no('div', 'op-titulo', 'PONTOS SALVOS NESTA ARENA'));
  raiz.appendChild(no('div', 'op-texto',
    'Sao os pontos ja marcados no mapa. Escolha um da lista e mande o robo ate ' +
    'la. Para criar um ponto novo, use o Mapeamento.'));

  const lista = no('select', 'op-sel');
  lista.size = 1;
  raiz.appendChild(lista);
  const motivo = no('div', 'op-motivo', '');
  raiz.appendChild(motivo);

  const btIr = botao('op-bt primaria', 'Ir ate o ponto escolhido', () => irAoPonto());
  raiz.appendChild(btIr);

  raiz.appendChild(no('div', 'op-titulo', 'ANDAMENTO'));
  const alvo = linha('Indo para');
  const falta = linha('Falta andar');
  raiz.appendChild(alvo.raiz);
  raiz.appendChild(falta.raiz);

  let pontos = [];

  function escolhido() {
    return pontos.find((p) => p.id === lista.value) || null;
  }

  async function irAoPonto() {
    const ponto = escolhido();
    if (!ponto) return;
    const resposta = await painel.comandar('/api/meta',
      { x: ponto.x, y: ponto.y, yaw: ponto.yaw || 0 });
    if (resposta.ok !== false) {
      painel.dados.indo = { rotulo: ponto.id, x: ponto.x, y: ponto.y };
      painel.atualizar();
    }
  }

  lista.addEventListener('change', () => {
    const ponto = escolhido();
    painel.destacar('waypoint', ponto ? ponto.id : '');
    painel.atualizar();
  });

  function aoArena(meta) {
    pontos = (meta && meta.waypoints) || [];
    lista.textContent = '';
    if (!pontos.length) {
      const vazia = no('option', '', 'nenhum ponto salvo nesta arena');
      vazia.value = '';
      lista.appendChild(vazia);
    } else {
      pontos.forEach((ponto) => {
        const opcao = no('option', '',
          ponto.id + (ponto.sem_pose ? '  (sem posicao gravada)' : ''));
        opcao.value = ponto.id;
        lista.appendChild(opcao);
      });
    }
  }

  function atualizar() {
    const dados = painel.dados;
    const ponto = escolhido();
    const semChave = painel.motivoSemChave();
    lista.disabled = !pontos.length;

    let impedimento = '';
    if (semChave) impedimento = semChave;
    else if (!dados.meta) impedimento = 'Escolha primeiro em qual arena o robo esta, na tela Mapas.';
    else if (!pontos.length) {
      impedimento = 'Esta arena ainda nao tem nenhum ponto salvo. Leve o robo ate ' +
                    'o lugar e grave o ponto no Mapeamento.';
    } else if (!ponto) impedimento = 'Escolha na lista para onde o robo deve ir.';
    else if (ponto.sem_pose) {
      // Sintoma que isto evita: o ponto existia no arquivo com posicao zerada,
      // o robo aceitava a ordem e saia rumo a origem do mapa, atravessando a
      // arena inteira.
      impedimento = 'O ponto "' + ponto.id + '" esta na lista mas nunca teve a ' +
                    'posicao gravada. Leve o robo ate ele e grave o ponto no ' +
                    'Mapeamento antes de manda-lo para la.';
    } else if (dados.capacidades.navegacao === false) {
      impedimento = 'A navegacao nao esta no ar: o robo nao sabe ir a lugar nenhum.';
    }

    btIr.disabled = !!impedimento;
    motivo.textContent = impedimento;

    const indo = dados.indo;
    alvo.valor.textContent = indo ? indo.rotulo : 'nada agora';
    const pose = dados.estado.pose;
    falta.valor.textContent = (indo && pose)
      ? numero(Math.hypot(indo.x - pose.x, indo.y - pose.y), 1) + ' m'
      : '—';
  }

  return { id: 'waypoints', raiz, atualizar, aoArena };
}

// ---------------------------------------------------------- secao: Docking

function secaoDocking(painel) {
  const raiz = no('div', 'op-secao');

  raiz.appendChild(no('div', 'op-titulo', 'ENCOSTAR NUMA ESTACAO'));
  raiz.appendChild(no('div', 'op-texto',
    'Encostar o robo numa estacao ja marcada no mapa. Se a estacao que voce ' +
    'procura nao esta na lista, ela ainda nao foi marcada: use o Mapeamento.'));

  const listaDocks = no('select', 'op-sel');
  raiz.appendChild(listaDocks);
  const motivo = no('div', 'op-motivo', '');
  raiz.appendChild(motivo);
  const btDockar = botao('op-bt primaria', 'Encostar na estacao', () => dockar());
  raiz.appendChild(btDockar);

  raiz.appendChild(no('div', 'op-titulo', 'SAIR DA ESTACAO'));
  raiz.appendChild(no('div', 'op-texto',
    'O jeito de sair depende de onde o robo esta encostado. Se ele acabou de ' +
    'encostar, "De onde estiver encostado" resolve.'));
  const listaTipos = no('select', 'op-sel');
  raiz.appendChild(listaTipos);
  const ajudaTipo = no('div', 'op-texto', '');
  raiz.appendChild(ajudaTipo);
  const btUndock = botao('op-bt', 'Sair da estacao', () => desdockar());
  raiz.appendChild(btUndock);
  const motivoUndock = no('div', 'op-motivo', '');
  raiz.appendChild(motivoUndock);

  let docks = [];
  let tipos = [];

  function dockEscolhido() {
    return docks.find((d) => d.id === listaDocks.value) || null;
  }

  async function dockar() {
    const dock = dockEscolhido();
    if (!dock) return;
    const resposta = await painel.comandar('/api/dock', { dock_id: dock.id });
    if (resposta.ok !== false) {
      painel.dados.indo = { rotulo: dock.id, x: dock.x, y: dock.y };
      painel.atualizar();
    }
  }

  function desdockar() {
    return painel.comandar('/api/undock', { tipo: listaTipos.value || '' });
  }

  listaDocks.addEventListener('change', () => {
    const dock = dockEscolhido();
    painel.destacar('dock', dock ? dock.id : '');
    painel.atualizar();
  });
  listaTipos.addEventListener('change', () => painel.atualizar());

  function aoArena(meta) {
    docks = (meta && meta.docks) || [];
    tipos = (meta && meta.tipos_dock) || [];

    listaDocks.textContent = '';
    if (!docks.length) {
      const vazia = no('option', '', 'esta arena nao tem estacao marcada');
      vazia.value = '';
      listaDocks.appendChild(vazia);
    } else {
      docks.forEach((dock) => {
        const opcao = no('option', '',
          dock.id + (dock.sem_pose ? '  (sem posicao gravada)' : ''));
        opcao.value = dock.id;
        listaDocks.appendChild(opcao);
      });
    }

    listaTipos.textContent = '';
    const padrao = no('option', '', 'De onde estiver encostado');
    padrao.value = '';
    listaTipos.appendChild(padrao);
    tipos.forEach((tipo) => {
      const opcao = no('option', '', tipo.rotulo || tipo.valor);
      opcao.value = tipo.valor;
      listaTipos.appendChild(opcao);
    });
  }

  function atualizar() {
    const dados = painel.dados;
    const dock = dockEscolhido();
    const semChave = painel.motivoSemChave();
    listaDocks.disabled = !docks.length;

    // Os motivos sao os mesmos de DockingModule::atualizarDisponibilidade:
    // cinza sem explicacao le como "quebrado".
    let impedimento = '';
    if (semChave) impedimento = semChave;
    else if (!dados.meta) impedimento = 'Escolha primeiro em qual arena o robo esta, na tela Mapas.';
    else if (!docks.length) {
      impedimento = 'Esta arena nao tem nenhuma estacao marcada. Leve o robo ate a ' +
                    'estacao e grave o ponto no Mapeamento.';
    } else if (!dock) impedimento = 'Escolha em qual estacao o robo deve encostar.';
    else if (dock.sem_pose) {
      impedimento = 'A estacao "' + dock.id + '" esta na lista mas nunca teve a ' +
                    'posicao gravada, por isso mandar o robo encostar esta ' +
                    'desligado. Leve o robo ate ela e grave a pose no Mapeamento.';
    } else if (dados.capacidades.dock === false) {
      impedimento = 'O encaixe automatico nao esta ligado neste robo.';
    }
    btDockar.disabled = !!impedimento;
    motivo.textContent = impedimento;

    const tipo = tipos.find((t) => t.valor === listaTipos.value);
    ajudaTipo.textContent = tipo ? (tipo.ajuda || '') : '';

    let impedeUndock = '';
    if (semChave) impedeUndock = semChave;
    else if (dados.capacidades.undock === false) {
      impedeUndock = 'O desencaixe automatico nao esta ligado neste robo.';
    }
    btUndock.disabled = !!impedeUndock;
    motivoUndock.textContent = impedeUndock;
  }

  return { id: 'docking', raiz, atualizar, aoArena };
}

// ------------------------------------------------------- secao: Localizacao

function secaoLocalizacao(painel) {
  const raiz = no('div', 'op-secao');

  raiz.appendChild(no('div', 'op-titulo', 'O ROBO SABE ONDE ESTA?'));
  const chip = no('div', 'op-chip', 'SEM DADOS');
  raiz.appendChild(chip);
  const situacao = no('div', 'op-texto', '');
  raiz.appendChild(situacao);
  const acao = no('div', 'op-motivo', '');
  raiz.appendChild(acao);

  const ondeX = linha('Onde ele acha que esta');
  const paraOnde = linha('Para que lado aponta');
  raiz.appendChild(ondeX.raiz);
  raiz.appendChild(paraOnde.raiz);

  raiz.appendChild(no('div', 'op-titulo', 'DIZER ONDE ELE ESTA'));
  raiz.appendChild(no('div', 'op-texto',
    'Use quando o robo se perder. Clique no mapa em cima do lugar onde ele esta ' +
    'de verdade e arraste na direcao para onde ele aponta. Depois gire o robo um ' +
    'pouco no lugar para a localizacao assentar.'));
  const btArmar = botao('op-bt primaria', 'Dizer onde o robo esta',
                        () => painel.armar('pose'));
  raiz.appendChild(btArmar);
  const motivo = no('div', 'op-motivo', '');
  raiz.appendChild(motivo);

  function atualizar() {
    const dados = painel.dados;
    const tf = painel.componente('caramelo/tf');
    chip.textContent = tf ? tf.rotulo : 'SEM DADOS';
    chip.style.background = tf ? tf.cor : '#7f93b0';
    situacao.textContent = tf ? tf.mensagem : 'Ainda nao sei se o robo se localizou.';
    acao.textContent = tf ? (tf.acao || '') : '';

    const pose = dados.estado.pose;
    if (pose) {
      ondeX.valor.textContent = numero(pose.x) + ' m, ' + numero(pose.y) + ' m';
      paraOnde.valor.textContent = graus(pose.yaw);
      // Dado velho e diferente de sem dado: uma tela congelada nao pode
      // parecer um robo parado no lugar certo.
      if (pose.velho) {
        ondeX.valor.textContent += '  (parou de atualizar)';
      }
    } else {
      ondeX.valor.textContent = 'nao sabe';
      paraOnde.valor.textContent = '—';
    }

    const semChave = painel.motivoSemChave();
    btArmar.disabled = !!semChave;
    btArmar.setAttribute('aria-pressed', String(dados.modo === 'pose'));
    motivo.textContent = semChave;
  }

  return { id: 'localizacao', raiz, atualizar };
}

// ----------------------------------------------------------- secao: Teleop

function secaoTeleop(painel) {
  const raiz = no('div', 'op-secao');

  raiz.appendChild(no('div', 'op-titulo', 'DIRIGIR NA MAO'));
  raiz.appendChild(no('div', 'op-texto',
    'O robo anda enquanto o botao estiver pressionado, e para quando voce soltar. ' +
    'Ele tambem para sozinho se esta tela sair da frente ou se a rede cair.'));

  const grupoMarchas = no('div', 'op-marchas');
  const botoesMarcha = MARCHAS.map((marcha) => {
    const b = botao('op-bt', marcha.rotulo, () => {
      painel.dados.marcha = marcha.id;
      painel.atualizar();
    });
    b.setAttribute('aria-pressed', 'false');
    grupoMarchas.appendChild(b);
    return { id: marcha.id, b };
  });
  raiz.appendChild(grupoMarchas);

  const volante = no('div', 'op-volante');
  // Seta sozinha nao diz de que movimento se trata numa base que anda de lado:
  // a legenda embaixo e a diferenca entre "vai para a esquerda" e "gira".
  const TECLADO_VISUAL = [
    ['frenteEsquerda', '↖', 'diagonal', 'Andar para a frente e para a esquerda'],
    ['frente', '↑', 'frente', 'Andar para a frente'],
    ['frenteDireita', '↗', 'diagonal', 'Andar para a frente e para a direita'],
    ['esquerda', '←', 'de lado', 'Andar de lado para a esquerda'],
    [null, 'PARAR', '', 'Parar o robo agora'],
    ['direita', '→', 'de lado', 'Andar de lado para a direita'],
    ['trasEsquerda', '↙', 'diagonal', 'Andar para tras e para a esquerda'],
    ['tras', '↓', 'de re', 'Andar para tras'],
    ['trasDireita', '↘', 'diagonal', 'Andar para tras e para a direita'],
  ];
  const botoesDirecao = [];
  TECLADO_VISUAL.forEach(([direcao, simbolo, legenda, descricao]) => {
    const b = botao('op-bt' + (direcao ? '' : ' parar'), simbolo);
    if (legenda) b.appendChild(no('small', '', legenda));
    b.title = descricao;
    b.setAttribute('aria-label', descricao);
    if (direcao) {
      ligarBotaoDeDirecao(b, direcao);
      botoesDirecao.push(b);
    } else {
      b.addEventListener('click', () => parar(true));
      botoesDirecao.push(b);
    }
    volante.appendChild(b);
  });
  raiz.appendChild(volante);

  const giro = no('div', 'op-giro');
  [['giroEsquerda', 'Girar ↺', 'Girar no lugar para a esquerda'],
   ['giroDireita', 'Girar ↻', 'Girar no lugar para a direita']].forEach(
    ([direcao, rotulo, descricao]) => {
      const b = botao('op-bt', rotulo);
      b.title = descricao;
      b.setAttribute('aria-label', descricao);
      ligarBotaoDeDirecao(b, direcao);
      botoesDirecao.push(b);
      giro.appendChild(b);
    });
  raiz.appendChild(giro);

  const estadoVolante = linha('Volante');
  raiz.appendChild(estadoVolante.raiz);
  const velocidadeAtual = linha('Velocidade escolhida');
  raiz.appendChild(velocidadeAtual.raiz);
  const motivo = no('div', 'op-motivo', '');
  raiz.appendChild(motivo);

  raiz.appendChild(no('div', 'op-texto',
    'Pelo teclado: W, A, S, D andam; Q e E giram; barra de espaco para. So ' +
    'funciona com esta secao aberta.'));

  // --- o motor do volante ---------------------------------------------
  const pressionados = new Set();
  let repetidor = null;
  let dirigindo = false;

  function ligarBotaoDeDirecao(b, direcao) {
    b.addEventListener('pointerdown', (ev) => {
      ev.preventDefault();
      try { b.setPointerCapture(ev.pointerId); } catch { /* mouse antigo */ }
      segurar(direcao);
    });
    ['pointerup', 'pointercancel', 'pointerleave'].forEach((evento) => {
      b.addEventListener(evento, () => soltar(direcao));
    });
  }

  function vetor() {
    const marcha = MARCHAS.find((m) => m.id === painel.dados.marcha) || MARCHAS[1];
    let ex = 0;
    let ey = 0;
    let ez = 0;
    pressionados.forEach((nome) => {
      const direcao = DIRECOES[nome];
      if (!direcao) return;
      ex += direcao.vx;
      ey += direcao.vy;
      ez += direcao.wz;
    });
    ex = Math.max(-1, Math.min(1, ex));
    ey = Math.max(-1, Math.min(1, ey));
    ez = Math.max(-1, Math.min(1, ez));
    // Na diagonal os dois eixos no teto andariam ~28% mais rapido que a maior
    // velocidade em que o robo anda sozinho. Normalizar mantem a direcao e
    // respeita o teto de cada eixo.
    const modulo = Math.hypot(ex, ey);
    if (modulo > 1) { ex /= modulo; ey /= modulo; }
    const marchaFator = marcha.fator;
    return {
      vx: ex * LIMITES.vx * marchaFator,
      vy: ey * LIMITES.vy * marchaFator,
      wz: ez * LIMITES.wz * marchaFator,
    };
  }

  function enviarPulso() {
    if (!pressionados.size) return;
    // Vai por fetch cru, e nao pelo comandar() do painel: a resposta de cada
    // pulso viraria um recado novo dez vezes por segundo e cobriria a tela.
    fetch(painel.acesso.url('/api/teleop'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(vetor()),
    }).catch(() => {});
  }

  function segurar(direcao) {
    if (!DIRECOES[direcao]) return;
    if (painel.motivoSemChave()) {
      painel.dizer(SEM_CHAVE, false);
      return;
    }
    dirigindo = true;
    pressionados.add(direcao);
    if (!repetidor) repetidor = setInterval(enviarPulso, PULSO_MS);
    enviarPulso();
    atualizar();
  }

  function soltar(direcao) {
    if (!pressionados.has(direcao)) return;
    pressionados.delete(direcao);
    if (!pressionados.size) parar();
    else { enviarPulso(); atualizar(); }
  }

  /**
   * Freia o volante. So manda o zero se ALGUEM estava dirigindo (ou se e um
   * pedido explicito): o comando manual tem prioridade sobre a navegacao, e um
   * "parar" gratuito -- trocar de secao, minimizar a janela -- frearia por meio
   * segundo um robo que estava navegando sozinho, sem ninguem ter pedido.
   */
  function parar(explicito) {
    pressionados.clear();
    if (repetidor) { clearInterval(repetidor); repetidor = null; }
    const precisa = dirigindo || explicito;
    dirigindo = false;
    atualizar();
    if (!precisa || painel.motivoSemChave()) return;
    // Zero explicito, e nao apenas parar de mandar: o ESC desta base le
    // ausencia de comando como re em velocidade maxima.
    fetch(painel.acesso.url('/api/teleop'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ parar: true }),
    }).catch(() => {});
  }

  function atualizar() {
    const dados = painel.dados;
    const semChave = painel.motivoSemChave();
    botoesMarcha.forEach((item) => {
      item.b.setAttribute('aria-pressed', String(item.id === dados.marcha));
      item.b.disabled = !!semChave;
    });
    botoesDirecao.forEach((b) => { b.disabled = !!semChave; });
    motivo.textContent = semChave;

    const marcha = MARCHAS.find((m) => m.id === dados.marcha) || MARCHAS[1];
    velocidadeAtual.valor.textContent =
      numero(LIMITES.vx * marcha.fator) + ' m/s para a frente, ' +
      numero(LIMITES.wz * marcha.fator) + ' rad/s de giro';
    estadoVolante.valor.textContent = pressionados.size
      ? 'andando' : (dados.estado.teleop_ligado ? 'freando' : 'parado');
  }

  return { id: 'teleop', raiz, atualizar, segurar, soltar, parar, sair: () => parar() };
}
