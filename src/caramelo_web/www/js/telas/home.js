"use strict";

// MENU PRINCIPAL -- a primeira coisa que alguem ve ao abrir o painel.
//
// Espelho da tela inicial da GUI (caramelo_gui/src/core/home_screen.cpp): mesma
// coluna do robo a esquerda, mesma grade de cinco cartoes de ACAO, mesma barra
// de acoes rapidas embaixo. Nao e' uma barra de abas: abas sao todas iguais
// entre si e nao contam nada sobre o que da' para fazer agora. O cartao conta,
// porque carrega ESTADO e MOTIVO.
//
// Quem decide o estado de cada cartao e' o ROBO, nao esta tela: o bloco
// `cartoes` do /ws/estado e' traducao 1:1 de HomeScreen::reavaliar. Refazer a
// conta aqui daria duas respostas diferentes no dia em que so uma das duas
// fosse corrigida -- e a GUI e a web precisam responder igual. Os textos
// copiados abaixo servem apenas ENQUANTO o primeiro quadro nao chegou.
//
// Nenhuma frase desta tela cita topico, action, no' ou nome de pacote: quem le'
// nao sabe ROS e nao vai abrir terminal por vontade propria.

// ---------------------------------------------------------------- constantes

// Os cinco cartoes, com os titulos e subtitulos de home_screen.cpp. O robo
// manda esta mesma lista no primeiro quadro (bloco `menu`); esta copia e' o
// desenho da tela antes disso, para o painel nao abrir vazio.
const CARTOES_PADRAO = [
  {id: "operacao", titulo: "Operacao", subtitulo: "Navegar, dockar e teleoperar"},
  {id: "competicao", titulo: "Competicao", subtitulo: "Rodar uma prova de ponta a ponta"},
  {id: "mapas", titulo: "Mapas", subtitulo: "Escolher a arena em que o robo esta"},
  {id: "mapeamento", titulo: "Mapeamento", subtitulo: "Criar mapa, waypoints e docks"},
  {id: "avancado", titulo: "Modo Avancado", subtitulo: "Sensores, sistema e diagnostico"},
];

// Rotulo e cor de cada estado, iguais aos de home_screen.cpp. O robo manda os
// dois junto com o cartao; isto aqui e' so' o que vale antes do primeiro quadro
// (e a rede de seguranca se um estado novo aparecer).
const ESTADOS = {
  pronto: {rotulo: "PRONTO", cor: "#27ae60"},
  degradado: {rotulo: "ATENCAO", cor: "#f2994a"},
  bloqueado: {rotulo: "BLOQUEADO", cor: "#eb5757"},
  desconhecido: {rotulo: "SEM DADOS", cor: "#7f93b0"},
};

// Quanto tempo sem quadro nenhum ate' a tela admitir que esta cega. O
// /ws/estado manda dez por segundo; seis segundos e' folga larga para um
// engasgo de rede, e curto o bastante para nao mentir por muito tempo.
const LIMITE_SEM_QUADRO = 6000;

// Enquanto o robo nao responde, "nao sei" -- nunca "esta tudo bem". Um painel
// congelado mostrando PRONTO em verde com o robo desligado e' pior que um
// painel que assume que nao sabe.
const CARTAO_INICIAL = {estado: "desconhecido", motivo: "Aguardando o robo responder."};

const ROBO_INICIAL = {
  nome: "Caramelo",
  equipe: "RoboFEI@Work",
  estado: "Aguardando o robo responder",
  arena: "Arena: —",
  arena_existe: false,
};

const ACOES_INICIAIS = {
  parar: {
    rotulo: "Parar o robo",
    habilitado: true,
    ajuda: "Cancela o destino atual e aborta a missao. Nao e um botao de "
         + "emergencia: este robo nao tem um.",
  },
  base: {
    rotulo: "Levar para a base",
    habilitado: false,
    motivo: "Sem base definida — crie um dock START em Mapeamento.",
    dock_id: "",
  },
  diagnostico: {rotulo: "Diagnostico", habilitado: true, contexto: "avancado"},
};

// Painel aberto sem a chave do link: da' para acompanhar, nao da' para
// comandar. Botao que existe e nao funciona e' pior que botao ausente -- o
// operador aperta, nada acontece, e passa a duvidar do painel inteiro.
const SEM_CHAVE = "Este painel foi aberto sem a chave de acesso: dá para "
                + "acompanhar, mas não dá para comandar. Peça o link completo a "
                + "quem ligou o robô.";

// Cor do chip do robo. A GUI pinta esse chip sempre de ciano (tema.qss), mas na
// web ele fica sozinho no alto da coluna: "Sem conexao com o robo" escrito em
// ciano, a mesma cor de PRONTO, ja' foi lido como se estivesse tudo bem. O
// texto continua sendo o do robo; so' a cor segue a gravidade.
const COR_DO_ESTADO_DO_ROBO = {
  "Pronto para operar": "#35c3f0",
  "Ligado, mas sem saber onde esta": "#f2994a",
  "Sem conexao com o robo": "#eb5757",
  "Aguardando o robo responder": "#7f93b0",
};

// ------------------------------------------------------------------- icones
//
// Os mesmos simbolos que icones.cpp desenha em QPainter, redesenhados em SVG.
// Nao ha' arquivo de imagem para reaproveitar e nao ha' internet em competicao:
// icone e' codigo, como la'. As coordenadas sao as mesmas fracoes do .cpp num
// viewBox de 100 -- 0.50*s virou 50 -- para conferir lado a lado com a fonte.
//
// O segundo tom de cada par e' o cor.lighter(118) do Qt (valor HSV x 1.18),
// calculado uma vez e fixado: o gradiente vai do claro (topo) ao cheio (base).
const CORES_ICONE = {
  operacao: ["#338aff", "#2f80ed"],
  competicao: ["#2ecd71", "#27ae60"],
  mapas: ["#ffa14e", "#f2994a"],
  mapeamento: ["#b05cff", "#9b51e0"],
  avancado: ["#00d3c1", "#00b3a4"],
  parar: ["#ff5e5e", "#eb5757"],
  base: ["#5bd7ff", "#56ccf2"],
  config: ["#96aed0", "#7f93b0"],
};

// Seta de navegacao (aviaozinho de papel).
function glifoOperacao() {
  return {defs: "", corpo: '<path d="M50 16 L82 84 L50 66 L18 84 Z" fill="#fff"/>'};
}

// Bandeira quadriculada.
function glifoCompeticao() {
  let quadros = '<rect x="20" y="14" width="5.5" height="72" fill="#fff"/>';
  const x0 = 28, y0 = 18, c = 11.5;
  for (let l = 0; l < 3; l++) {
    for (let col = 0; col < 4; col++) {
      if ((l + col) % 2 === 0) {
        quadros += '<rect x="' + (x0 + col * c) + '" y="' + (y0 + l * c)
                 + '" width="' + c + '" height="' + c + '" fill="#fff"/>';
      }
    }
  }
  return {defs: "", corpo: quadros};
}

// Mapa dobrado.
function glifoMapas() {
  return {
    defs: "",
    corpo:
      '<path d="M14 30 L38 20 L62 30 L86 20 L86 76 L62 86 L38 76 L14 86 Z"'
      + ' fill="rgba(255,255,255,0.92)"/>'
      + '<g stroke="rgba(0,0,0,0.27)" stroke-width="3.5">'
      + '<line x1="38" y1="20" x2="38" y2="76"/>'
      + '<line x1="62" y1="30" x2="62" y2="86"/></g>',
  };
}

// Lapis inclinado (criar/editar).
function glifoMapeamento() {
  return {
    defs: "",
    corpo:
      '<g transform="translate(50,50) rotate(-45)" fill="#fff">'
      + '<rect x="-10" y="-34" width="20" height="50"/>'
      + '<path d="M-10 16 L10 16 L0 36 Z"/></g>',
  };
}

// Engrenagem. O furo do meio e' recortado por mascara e nao pintado por cima:
// o fundo do icone e' um gradiente, entao um circulo de cor chapada no lugar do
// furo apareceria como uma mancha em vez de um vazio.
function glifoEngrenagem(uid, escala) {
  const raio = 26 * escala;
  const dente = 10 * escala;
  let dentes = "";
  for (let i = 0; i < 8; i++) {
    dentes += '<rect x="' + (-dente / 2) + '" y="' + (-(raio + dente * 0.9))
            + '" width="' + dente + '" height="' + (dente * 1.6)
            + '" rx="' + (dente * 0.25) + '" transform="rotate(' + (i * 45) + ')"/>';
  }
  const mascara = "furo" + uid;
  return {
    defs:
      '<mask id="' + mascara + '" maskUnits="userSpaceOnUse"'
      + ' x="0" y="0" width="100" height="100">'
      + '<g transform="translate(50,50)" fill="#fff">'
      + '<circle r="' + raio + '"/>' + dentes + '</g>'
      + '<circle cx="50" cy="50" r="' + (raio * 0.42) + '" fill="#000"/></mask>',
    corpo: '<rect width="100" height="100" fill="#fff" mask="url(#' + mascara + ')"/>',
  };
}

// Quadrado de parada.
function glifoParar() {
  return {
    defs: "",
    corpo: '<rect x="28" y="28" width="44" height="44" rx="6" fill="#fff"/>',
  };
}

// Seta de retorno (meia-volta). O arco vem do drawArc de 20 a 310 graus; em SVG
// o mesmo giro e' sweep-flag 0 porque o y cresce para baixo.
function glifoBase() {
  return {
    defs: "",
    corpo:
      '<path d="M76.31 40.48 A28 22 0 1 0 68 64.85" fill="none" stroke="#fff"'
      + ' stroke-width="10" stroke-linecap="round"/>'
      + '<path d="M30 30 L30 56 L54 42 Z" fill="#fff"/>',
  };
}

const GLIFOS = {
  operacao: glifoOperacao,
  competicao: glifoCompeticao,
  mapas: glifoMapas,
  mapeamento: glifoMapeamento,
  avancado: (uid) => glifoEngrenagem(uid, 1.0),
  config: (uid) => glifoEngrenagem(uid, 0.9),
  parar: glifoParar,
  base: glifoBase,
};

let contadorId = 0;

// Um icone: ladrilho arredondado com gradiente + glifo branco, como em
// icones::desenhar. O raio e' 0.24 do lado (72px viram raio 17, como na GUI).
function iconeSvg(tipo, lado) {
  const uid = ++contadorId;
  const cores = CORES_ICONE[tipo] || CORES_ICONE.config;
  const glifo = (GLIFOS[tipo] || glifoEngrenagem)(uid, 1.0);
  const grad = "grad" + uid;
  return '<svg class="home-icone" viewBox="0 0 100 100" width="' + lado
    + '" height="' + lado + '" aria-hidden="true" focusable="false">'
    + '<defs><linearGradient id="' + grad + '" x1="0" y1="0" x2="0" y2="1">'
    + '<stop offset="0" stop-color="' + cores[0] + '"/>'
    + '<stop offset="1" stop-color="' + cores[1] + '"/></linearGradient>'
    + glifo.defs + '</defs>'
    + '<rect width="100" height="100" rx="24" ry="24" fill="url(#' + grad + ')"/>'
    + glifo.corpo + '</svg>';
}

// Cadeado pequeno, ao lado do chip, quando o cartao nao abre. So' a cor
// vermelha do chip nao diz que o clique nao vai levar a lugar nenhum.
const CADEADO =
  '<svg class="home-cadeado" viewBox="0 0 24 24" width="14" height="14"'
  + ' aria-hidden="true" focusable="false">'
  + '<path d="M8 10V7.5a4 4 0 0 1 8 0V10" fill="none" stroke="currentColor"'
  + ' stroke-width="2.2" stroke-linecap="round"/>'
  + '<rect x="5" y="10" width="14" height="10" rx="2.5" fill="currentColor"/></svg>';

// A silhueta do robo, no pe' da coluna. Desenhada aqui pelo mesmo motivo dos
// icones: nada e' baixado. Frente do Caramelo -- base mecanum, torre com a
// tela, o sensor de distancia girando no topo da base e o bracinho recolhido.
const SILHUETA =
  '<svg class="home-silhueta" viewBox="0 0 160 200" role="img"'
  + ' aria-label="Desenho do robo Caramelo">'
  + '<ellipse cx="80" cy="186" rx="58" ry="8" fill="#050d18" opacity="0.55"/>'
  // varredura do sensor de distancia
  + '<g fill="none" stroke="#35c3f0" opacity="0.32" stroke-width="2.5"'
  + ' stroke-linecap="round">'
  + '<path d="M40 118 A44 44 0 0 1 120 118"/>'
  + '<path d="M52 124 A30 30 0 0 1 108 124"/></g>'
  // torre
  + '<rect x="50" y="30" width="60" height="86" rx="18" fill="#16294a"'
  + ' stroke="#1b3457" stroke-width="2"/>'
  // tela / rosto
  + '<rect x="58" y="40" width="44" height="32" rx="10" fill="#050d18"'
  + ' stroke="#1b3457" stroke-width="2"/>'
  + '<circle cx="71" cy="56" r="4.5" fill="#35c3f0"/>'
  + '<circle cx="89" cy="56" r="4.5" fill="#35c3f0"/>'
  // faixa laranja Caramelo
  + '<rect x="50" y="86" width="60" height="8" rx="4" fill="#ff9a2e"/>'
  // braco recolhido
  + '<g fill="none" stroke="#7d9cc4" stroke-width="7" stroke-linecap="round"'
  + ' stroke-linejoin="round" opacity="0.85">'
  + '<path d="M112 78 L128 88 L124 108"/></g>'
  // sensor de distancia
  + '<rect x="64" y="112" width="32" height="14" rx="6" fill="#0e1c31"'
  + ' stroke="#35c3f0" stroke-width="2"/>'
  // chassi
  + '<rect x="22" y="126" width="116" height="40" rx="12" fill="#16294a"'
  + ' stroke="#1b3457" stroke-width="2"/>'
  // rodas mecanum, com os rolos na diagonal
  + '<g fill="#0e1c31" stroke="#2a4a75" stroke-width="2">'
  + '<rect x="12" y="138" width="22" height="38" rx="9"/>'
  + '<rect x="126" y="138" width="22" height="38" rx="9"/></g>'
  + '<g stroke="#35c3f0" stroke-width="2" opacity="0.5" stroke-linecap="round">'
  + '<path d="M15 150 L31 142"/><path d="M15 160 L31 152"/><path d="M15 170 L31 162"/>'
  + '<path d="M129 150 L145 142"/><path d="M129 160 L145 152"/>'
  + '<path d="M129 170 L145 162"/></g>'
  + '</svg>';

// -------------------------------------------------------------------- visual
//
// O visual do painel mora em www/tema.css. Este bloco entra como PADRAO: e'
// inserido no comeco do <head>, portanto qualquer regra de tema.css para as
// mesmas classes ganha dele. Existe para a tela nunca abrir sem forma se o tema
// ainda nao conhecer as classes desta tela; as cores sao lidas das variaveis do
// tema, com os valores do tema.qss como reserva.
const ESTILO = `
.tela-home{display:flex;flex-direction:column;gap:18px;height:100%;min-height:0;
      padding:22px 28px 18px;overflow:hidden}
.home-meio{flex:1 1 auto;min-height:0;display:grid;
           grid-template-columns:260px minmax(0,1fr);gap:22px;
           overflow-y:auto;overflow-x:hidden;padding-right:4px}

/* Coluna do robo */
.home-robo{display:flex;flex-direction:column;gap:10px;padding:22px 20px;
           border:1px solid var(--borda,#1b3457);border-radius:20px;
           background:linear-gradient(180deg,#14284a 0%,#0e1c31 100%)}
.home-nome{font-size:30px;font-weight:800;color:#fff;line-height:1.1;margin:0}
.home-equipe{font-size:14px;color:var(--fraco,#7d9cc4);margin:0}
.home-chip-robo{align-self:flex-start;margin-top:10px;font-size:14px;
                font-weight:700;color:#06121f;background:var(--ciano,#35c3f0);
                border-radius:10px;padding:8px 12px;max-width:100%}
.home-arena{font-size:14px;color:#a9c6e8;margin:0;padding-top:4px;
            overflow-wrap:anywhere}
.home-arena[data-alerta="1"]{color:#ffd28f}
.home-silhueta{margin-top:auto;width:100%;max-width:180px;align-self:center;
               height:auto;padding-top:12px}

/* Grade dos cartoes */
.home-grade{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));
            gap:18px;align-content:start}
.home-icone{flex:none;display:block;align-self:flex-start}
.home-cartao{display:flex;flex-direction:column;gap:8px;text-align:left;
             white-space:normal;touch-action:manipulation;
             -webkit-appearance:none;appearance:none;
             min-height:200px;padding:18px 18px 14px;font:inherit;
             color:inherit;cursor:pointer;
             background:var(--painel,#101e35);
             border:1px solid var(--borda,#1b3457);border-radius:20px;
             transition:background .12s ease,border-color .12s ease}
.home-cartao:hover{background:#16294a;border-color:var(--ciano,#35c3f0)}
.home-cartao:focus-visible{outline:3px solid var(--ciano,#35c3f0);
                           outline-offset:2px}
.home-cartao[aria-disabled="true"]{cursor:not-allowed;background:#0f1a2c;
                                   border-color:#4a2331}
.home-cartao[aria-disabled="true"]:hover{background:#131f33;border-color:#6b3243}
.home-cartao[aria-disabled="true"] .home-icone{opacity:.45;filter:saturate(.5)}
.home-titulo-cartao{font-size:21px;font-weight:800;color:#fff}
.home-sub-cartao{font-size:13px;color:#8fb0d8;line-height:1.35}
.home-rodape-cartao{margin-top:auto;display:flex;flex-direction:column;gap:6px}
.home-selo{display:flex;align-items:center;gap:6px;align-self:flex-start;
           font-size:11px;font-weight:800;letter-spacing:.4px;color:#06121f;
           background:#7f93b0;border-radius:9px;padding:3px 12px}
.home-cadeado{flex:none}
.home-motivo{font-size:12px;color:#ffd28f;line-height:1.4}

/* Barra de acoes rapidas */
.home-barra{flex:none;display:flex;align-items:center;gap:28px;
            padding:10px 18px;background:var(--topo,#081120);
            border:1px solid #14294a;border-radius:16px;flex-wrap:wrap}
.home-acao{display:flex;align-items:center;gap:10px;min-height:52px;flex:none;
           -webkit-appearance:none;appearance:none;
           padding:8px 18px;border:none;border-radius:12px;background:transparent;
           color:#cfe4fb;font:inherit;font-size:15px;font-weight:700;
           cursor:pointer;white-space:nowrap;touch-action:manipulation}
.home-acao:hover:not(:disabled){background:#12233f}
.home-acao:focus-visible{outline:3px solid var(--ciano,#35c3f0);outline-offset:2px}
.home-acao:disabled{color:#40597d;cursor:not-allowed}
.home-acao:disabled .home-icone{opacity:.45;filter:saturate(.35)}
.home-acao.parar{color:#ff9d9d;font-weight:800}
.home-acao.parar:hover:not(:disabled){background:#3a1620}
.home-par{display:flex;align-items:center;gap:12px;flex:1 1 320px;min-width:0}
.home-recado-barra{flex:1 1 auto;min-width:0;font-size:12px;
                   color:#6d86a8;line-height:1.35}
.home-recado-barra[data-alerta="1"]{color:#ffd28f}
.home-espaco{flex:1 1 auto}

/* De 1024 px a 2560 px, e tambem em tablet: o conteudo se reorganiza, nada e
   cortado e a pagina nunca rola na horizontal. */
@media (max-width:1280px){
  .home-grade{grid-template-columns:repeat(2,minmax(0,1fr))}
}
@media (max-width:900px){
  .tela-home{padding:14px 14px 12px;gap:12px}
  .home-meio{grid-template-columns:minmax(0,1fr);gap:14px}
  .home-robo{flex-direction:row;align-items:center;flex-wrap:wrap;gap:8px 18px}
  .home-nome{font-size:24px}
  .home-chip-robo{margin-top:0}
  .home-silhueta{margin:0 0 0 auto;max-width:92px;padding-top:0}
  .home-cartao{min-height:0}
  .home-espaco{display:none}
}
@media (max-width:640px){
  .home-grade{grid-template-columns:minmax(0,1fr)}
  .home-barra{gap:10px 16px}
  .home-par{flex:1 1 100%}
  .home-acao{padding:8px 12px}
}
@media (max-height:620px){
  .home-silhueta{display:none}
  .home-cartao{min-height:0}
}
`;

function garantirEstilo() {
  if (document.getElementById("estilo-tela-home")) return;
  const tag = document.createElement("style");
  tag.id = "estilo-tela-home";
  tag.textContent = ESTILO;
  // No COMECO do <head> de proposito: assim o tema.css, que vem depois, manda
  // em qualquer regra que ele tambem escreva.
  document.head.insertBefore(tag, document.head.firstChild);
}

// -------------------------------------------------------------- ponte com o app
//
// O contrato combinado entre as telas e o app cobre montar/destruir/aoEstado. O
// resto (abrir contexto, comandar, avisar) e' chamado pelo nome que o app
// oferecer: estas tres funcoes procuram o nome e, se nao acharem nenhum, fazem
// o minimo por conta propria. Uma tela muda porque o nome de um metodo nao
// bateu e' pior do que meia duzia de linhas de tolerancia.

// Primeiro nome que existir, entre os que o roteador pode ter usado. As telas
// vizinhas fazem a mesma procura, com a mesma lista.
function primeira(alvo, nomes) {
  if (!alvo) return null;
  for (const nome of nomes) {
    if (typeof alvo[nome] === "function") return alvo[nome];
  }
  return null;
}

function abrirContexto(ctx, id) {
  const ir = primeira(ctx, ["irPara", "abrir", "abrirTela", "navegar", "trocarTela"]);
  if (ir) {
    ir.call(ctx, id, {});
    return;
  }
  location.hash = "#/" + id;
}

function avisar(ctx, texto, bom) {
  const t = primeira(ctx, ["torrada", "avisar", "aviso", "recado", "notificar"]);
  if (t) t.call(ctx, texto, bom !== false);
  else console.warn(texto);
}

// O app sabe se a chave do link vale (ele pergunta ao robo em /api/acesso).
// Quando ele nao expuser essa informacao, assume-se que da' para comandar: e'
// melhor um botao que responde com a recusa do robo do que um painel inteiro
// desabilitado por engano.
function podeComandar(ctx) {
  if (!ctx) return true;
  const v = typeof ctx.autorizado === "function" ? ctx.autorizado() : ctx.autorizado;
  return (v === undefined || v === null) ? true : !!v;
}

// A chave de acesso vem no proprio link (?t=...). Nao vai para localStorage: um
// tablet emprestado nao deve continuar comandando o robo depois que a aba fecha.
function chaveDoLink() {
  return new URLSearchParams(location.search).get("t") || "";
}

async function comandar(ctx, rota, corpo) {
  const api = (ctx && ctx.api) ? ctx.api : (ctx || {});
  const fn = primeira(api, ["comandar", "post", "enviar", "postar"]);
  if (fn) return (await fn.call(api, rota, corpo || {})) || {};

  const chave = (ctx && ctx.chave) || chaveDoLink();
  const url = chave
    ? rota + (rota.includes("?") ? "&" : "?") + "t=" + encodeURIComponent(chave)
    : rota;
  try {
    const r = await fetch(url, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(corpo || {}),
    });
    const d = await r.json().catch(() => ({}));
    // Rota que nao existe responde 404 sem corpo: sem esta linha o "ok" ficaria
    // indefinido e a tela daria o comando por aceito.
    if (d.ok === undefined) d.ok = r.ok;
    if (d.mensagem) avisar(ctx, d.mensagem, r.ok && d.ok !== false);
    else if (!r.ok) avisar(ctx, "O robô não aceitou o pedido.", false);
    return d;
  } catch (erro) {
    avisar(ctx, "Perdi contato com o robô.", false);
    return {ok: false};
  }
}

// --------------------------------------------------------------------- tela

export const tela = {
  id: "home",
  // A faixa superior mostra "Caramelo" no menu principal, como na GUI.
  titulo: "Caramelo",
  precisaDoMapa: false,

  montar(raiz, ctx) {
    garantirEstilo();

    raiz.innerHTML =
      '<section class="tela-home">'
      + '<div class="home-meio">'
      + '<aside class="home-robo">'
      + '<p class="home-nome" data-campo="nome">Caramelo</p>'
      + '<p class="home-equipe" data-campo="equipe">RoboFEI@Work</p>'
      + '<p class="home-chip-robo" data-campo="estado" role="status"></p>'
      + '<p class="home-arena" data-campo="arena"></p>'
      + SILHUETA
      + '</aside>'
      + '<div class="home-grade" data-campo="grade"></div>'
      + '</div>'
      + '<footer class="home-barra">'
      + '<div class="home-par">'
      + '<button type="button" class="home-acao parar" data-acao="parar">'
      + iconeSvg("parar", 28) + '<span data-campo="rotulo">Parar o robo</span></button>'
      + '<p class="home-recado-barra" data-campo="ajuda-parar"></p>'
      + '</div>'
      + '<div class="home-par">'
      + '<button type="button" class="home-acao" data-acao="base" disabled>'
      + iconeSvg("base", 28) + '<span data-campo="rotulo">Levar para a base</span></button>'
      + '<p class="home-recado-barra" data-campo="motivo-base"></p>'
      + '</div>'
      + '<span class="home-espaco"></span>'
      + '<button type="button" class="home-acao" data-acao="diagnostico">'
      + iconeSvg("config", 28) + '<span data-campo="rotulo">Diagnostico</span></button>'
      + '</footer>'
      + '</section>';

    const grade = raiz.querySelector('[data-campo="grade"]');
    const campoNome = raiz.querySelector('[data-campo="nome"]');
    const campoEquipe = raiz.querySelector('[data-campo="equipe"]');
    const campoEstado = raiz.querySelector('[data-campo="estado"]');
    const campoArena = raiz.querySelector('[data-campo="arena"]');
    const botaoParar = raiz.querySelector('[data-acao="parar"]');
    const botaoBase = raiz.querySelector('[data-acao="base"]');
    const botaoDiag = raiz.querySelector('[data-acao="diagnostico"]');
    const ajudaParar = raiz.querySelector('[data-campo="ajuda-parar"]');
    const motivoBase = raiz.querySelector('[data-campo="motivo-base"]');

    let vivo = true;
    let cartoes = {};        // id -> {frame, selo, rotuloSelo, motivo, estado}
    let idsDaGrade = "";     // para so' remontar a grade quando a lista mudar
    let acoes = ACOES_INICIAIS;
    let comanda = podeComandar(ctx);   // a chave do link vale?
    let ultimoRobo = ROBO_INICIAL;

    // ------------------------------------------------------------- grade
    function montarGrade(lista) {
      const chave = lista.map((c) => c.id).join(",");
      if (chave !== idsDaGrade) {
        idsDaGrade = chave;
        grade.textContent = "";
        cartoes = {};
        lista.forEach((def) => {
          const frame = document.createElement("button");
          frame.type = "button";
          frame.className = "home-cartao";
          frame.dataset.cartao = def.id;
          frame.innerHTML =
            iconeSvg(def.id, 72)
            + '<span class="home-titulo-cartao"></span>'
            + '<span class="home-sub-cartao"></span>'
            + '<span class="home-rodape-cartao">'
            + '<span class="home-selo"><span data-campo="rotulo-selo">SEM DADOS</span></span>'
            + '<span class="home-motivo"></span></span>';
          frame.addEventListener("click", () => aoClicarNoCartao(def));
          grade.appendChild(frame);
          cartoes[def.id] = {
            frame,
            titulo: frame.querySelector(".home-titulo-cartao"),
            sub: frame.querySelector(".home-sub-cartao"),
            selo: frame.querySelector(".home-selo"),
            rotuloSelo: frame.querySelector('[data-campo="rotulo-selo"]'),
            motivo: frame.querySelector(".home-motivo"),
            estado: "desconhecido",
            texto: "",
          };
          pintarCartao(def.id, CARTAO_INICIAL);
        });
      }
      // Titulo e subtitulo sao sempre os que o robo mandou, mesmo quando a
      // grade nao foi remontada: se um dia a GUI mudar um titulo, a web muda
      // junto, sem ninguem lembrar de vir corrigir a copia daqui.
      lista.forEach((def) => {
        const c = cartoes[def.id];
        if (!c) return;
        if (def.titulo) c.titulo.textContent = def.titulo;
        c.sub.textContent = def.subtitulo || "";
      });
    }

    function pintarCartao(id, dados) {
      const c = cartoes[id];
      if (!c) return;
      const nome = String(dados.estado || "desconhecido");
      const padrao = ESTADOS[nome] || ESTADOS.desconhecido;
      const motivo = dados.motivo || "";
      c.estado = nome;
      c.texto = motivo;
      c.rotuloSelo.textContent = dados.rotulo || padrao.rotulo;
      c.selo.style.background = dados.cor || padrao.cor;
      // Estado sozinho nao ajuda ninguem: "BLOQUEADO" sem motivo so' informa
      // que algo deu errado. O motivo e' a parte util -- e fica no proprio
      // cartao, nao numa tela de detalhe que ninguem vai abrir.
      c.motivo.textContent = motivo;
      c.motivo.style.display = motivo ? "" : "none";
      const travado = nome === "bloqueado";
      c.frame.setAttribute("aria-disabled", travado ? "true" : "false");
      const cadeado = c.selo.querySelector(".home-cadeado");
      if (travado && !cadeado) {
        c.selo.insertAdjacentHTML("afterbegin", CADEADO);
      } else if (!travado && cadeado) {
        cadeado.remove();
      }
    }

    function aoClicarNoCartao(def) {
      const c = cartoes[def.id];
      // Cartao bloqueado nao abre. Mas tambem nao pode so' ignorar o toque: sem
      // resposta nenhuma, o operador aperta de novo e conclui que a tela travou.
      if (c && c.estado === "bloqueado") {
        avisar(ctx, c.texto || (def.titulo + " não dá para abrir agora."), false);
        return;
      }
      abrirContexto(ctx, def.id);
    }

    // ------------------------------------------------------- barra inferior
    function pintarBarra(dados) {
      acoes = dados || acoes;
      const base = acoes.base || ACOES_INICIAIS.base;
      const parar = acoes.parar || ACOES_INICIAIS.parar;
      const diag = acoes.diagnostico || ACOES_INICIAIS.diagnostico;

      botaoParar.querySelector('[data-campo="rotulo"]').textContent = parar.rotulo;
      // O rotulo NAO promete parada de emergencia: este robo nao tem botao de
      // emergencia de hardware, e prometer um seria pior do que nao ter nenhum.
      botaoParar.title = parar.ajuda || ACOES_INICIAIS.parar.ajuda;

      botaoBase.querySelector('[data-campo="rotulo"]').textContent = base.rotulo;
      botaoDiag.querySelector('[data-campo="rotulo"]').textContent = diag.rotulo;

      comanda = podeComandar(ctx);
      botaoParar.disabled = !comanda;
      botaoBase.disabled = !comanda || !base.habilitado;

      // O que cada frase explica fica GRUDADO no seu botao: uma frase solta no
      // meio da barra seria lida como explicacao do botao errado.
      ajudaParar.textContent = comanda
        ? (parar.ajuda || ACOES_INICIAIS.parar.ajuda) : SEM_CHAVE;
      // Botao cinza sem explicacao faz o operador achar que a interface esta
      // quebrada; com o motivo ao lado, ele sabe o que fazer.
      const motivo = base.habilitado ? "" : (base.motivo || "");
      motivoBase.textContent = motivo || (comanda ? "" : SEM_CHAVE);
      motivoBase.dataset.alerta = motivo ? "1" : "0";
    }

    async function pararRobo() {
      // UM pedido, e nao dois. O robo faz a sequencia inteira do lado dele, na
      // ordem da GUI (main_window.cpp): zera o teleop, cancela o destino em voo
      // e aborta a missao.
      //
      // O teleop e' o motivo de nao mandar isto daqui em duas chamadas: ele
      // precisa ser ZERADO, e parar de publicar nao e' parar -- o ESC desta
      // base le' ausencia de PWM como re' em velocidade maxima. Uma aba que
      // perdesse a rede entre o primeiro e o segundo pedido pararia metade do
      // robo e ninguem saberia qual metade.
      //
      // Continua NAO sendo parada de emergencia: este robo nao tem uma.
      botaoParar.disabled = true;
      try {
        await comandar(ctx, "/api/parar", {});
      } finally {
        if (vivo) botaoParar.disabled = !comanda;
      }
    }

    async function levarParaBase() {
      const base = acoes.base || {};
      if (!base.habilitado || !base.dock_id) {
        avisar(ctx, base.motivo || ACOES_INICIAIS.base.motivo, false);
        return;
      }
      botaoBase.disabled = true;
      try {
        // Rota propria em vez de /api/dock com o id que esta tela guardou: o
        // robo confere de novo se da' para ir, e devolve o MESMO motivo que
        // aparece aqui embaixo do botao. A arena pode ter mudado (pela GUI,
        // pelo terminal) depois que este botao ficou verde.
        await comandar(ctx, "/api/base", {});
      } finally {
        if (vivo && comanda && (acoes.base || {}).habilitado) botaoBase.disabled = false;
      }
    }

    // Vigia da propria tela: se PARAR de chegar quadro (o painel caiu, o Wi-Fi
    // sumiu, a aba dormiu no tablet), a tela nao pode continuar afirmando
    // PRONTO em verde. Sem isto, um painel congelado com o robo desligado
    // parece um robo pronto -- o pior de todos os enganos possiveis aqui.
    let ultimoQuadro = 0;
    let semSinal = false;
    const vigia = setInterval(() => {
      if (!vivo || !ultimoQuadro || semSinal) return;
      if (Date.now() - ultimoQuadro < LIMITE_SEM_QUADRO) return;
      semSinal = true;
      Object.keys(cartoes).forEach((id) => pintarCartao(id, CARTAO_INICIAL));
      // A arena escolhida nao deixa de ser a arena escolhida so' porque a
      // ligacao caiu: o que muda e' o que se sabe do robo.
      pintarRobo(Object.assign({}, ultimoRobo, {estado: ROBO_INICIAL.estado}));
    }, 1000);

    botaoParar.addEventListener("click", pararRobo);
    botaoBase.addEventListener("click", levarParaBase);
    botaoDiag.addEventListener("click", () => {
      abrirContexto(ctx, (acoes.diagnostico || {}).contexto || "avancado");
    });

    // ---------------------------------------------------------- estado inicial
    montarGrade(CARTOES_PADRAO);
    pintarRobo(ROBO_INICIAL);
    pintarBarra(ACOES_INICIAIS);

    function pintarRobo(robo) {
      ultimoRobo = robo;
      campoNome.textContent = robo.nome || ROBO_INICIAL.nome;
      campoEquipe.textContent = robo.equipe || ROBO_INICIAL.equipe;
      const texto = robo.estado || ROBO_INICIAL.estado;
      campoEstado.textContent = texto;
      campoEstado.style.background = COR_DO_ESTADO_DO_ROBO[texto] || "#35c3f0";
      campoArena.textContent = robo.arena || ROBO_INICIAL.arena;
      // Arena escolhida que nao existe mais no robo ja' aconteceu em campo, e a
      // tela dizia que estava tudo pronto. Agora ela aparece em ambar.
      const some = robo.arena_nome && robo.arena_existe === false;
      campoArena.dataset.alerta = some ? "1" : "0";
    }

    return {
      // Chamado a cada mensagem do WebSocket. O bloco pesado (cartoes) so' vem
      // uma vez por segundo: quando ele nao vier, a tela fica com o ultimo que
      // recebeu, que e' o certo -- nada ali muda dez vezes por segundo.
      aoEstado(estado) {
        if (!vivo || !estado) return;
        ultimoQuadro = Date.now();
        semSinal = false;

        // O menu chega inteiro no primeiro quadro de cada conexao. Ele manda
        // nos titulos: assim a web nao pode divergir da GUI por um titulo
        // redigitado errado aqui.
        const menu = estado.menu;
        if (menu && Array.isArray(menu.cartoes) && menu.cartoes.length) {
          montarGrade(menu.cartoes.map((c) => ({
            id: c.id, titulo: c.titulo, subtitulo: c.subtitulo,
          })));
        }

        // A resposta de /api/acesso chega depois da tela montar; sem isto os
        // botoes ficariam cinzas ate' o proximo bloco de acoes -- que so' vem
        // uma vez por segundo, e pode nao vir nunca.
        if (comanda !== podeComandar(ctx)) pintarBarra(acoes);

        const bloco = estado.cartoes;
        if (!bloco) return;
        if (bloco.robo) pintarRobo(bloco.robo);
        const mapa = bloco.cartoes || bloco;
        Object.keys(cartoes).forEach((id) => {
          if (mapa[id]) pintarCartao(id, mapa[id]);
        });
        if (bloco.acoes) pintarBarra(bloco.acoes);
      },

      destruir() {
        vivo = false;
        clearInterval(vigia);
        raiz.textContent = "";
        // A folha de estilo fica: ela e' a mesma toda vez que o menu volta, e
        // retira-la a cada troca de tela so' daria trabalho ao navegador.
      },
    };
  },
};
