// Ferramenta de Mapeamento -- criar uma arena do zero, do jeito que ela e'
// criada de verdade.
//
// Espelha caramelo_gui/src/modules/mapeamento/ferramenta_mapeamento.cpp: mesma
// sequencia, mesmos programas, mesmo formato de arquivo. Um mapa feito aqui
// tem que abrir na GUI e vice-versa.
//
// A SEQUENCIA TEM ORDEM, e a ordem e' do operador:
//   1. Mapear            o robo anda, o mapa cresce na tela
//   2. Limpar o ruido    a pessoa que passou na frente do laser vira chao
//   3. Marcar os pontos  as estacoes, o START e o FINISH
//   4. Paredes virtuais  o que o laser nao enxerga (fita de piso, degrau, vidro)
//   5. Conferir          como ficou e o que ainda falta
//
// POR QUE TRILHO E NAO ABAS. Abas sao todas iguais entre si e nao contam nada
// sobre ordem. Sintoma medido com quem nunca usou o robo: clicava na terceira
// antes da primeira, marcava um ponto sem mapa existir, recebia uma recusa que
// nao entendia e concluia que o painel estava quebrado. O trilho mostra tres
// estados (feita, atual, travada) e a travada diz o MOTIVO em texto.
//
// A ordem so' trava para arena NOVA. Editar uma arena que ja' existe e' livre:
// quem volta para arrumar um dock torto nao vai remapear a sala inteira.
//
// ONDE SE COMECOU A MAPEAR NAO E' O START. Nenhum texto desta tela assume isso,
// e a etapa 3 diz isso com todas as letras -- ja' houve prova perdida porque o
// START ficou na porta do laboratorio, que era so' onde o notebook estava.
//
// precisaDoMapa e' FALSE de proposito. Nas outras telas o mapa e' espectador;
// aqui ele e' a mesa de trabalho -- recebe clique, arrasto e pincel. A
// ferramenta traz o proprio mapa, que e' o arranjo que o tema desenha
// (trilho | mapa | painel). Pedir tambem o mapa da casca poria DOIS mapas na
// mesma tela, e o operador editaria no que nao responde.
//
// O QUE ESTA TELA PEDE DE mapa_canvas.js (ver adaptador mais abaixo):
//   criarMapa(elemento, {estilo, camadas})  -> mapa
//   mapa.definirArena(meta, {mapa, keepout})
//   mapa.definirEstado({pose, scan})
//   mapa.paraMundo(px, py) -> [x, y]        obrigatorio: e' o que traduz o dedo
//   mapa.paraTela(x, y)    -> [px, py]      obrigatorio: e' o que desenha a previa
//   mapa.redesenhar() / mapa.destruir()
// O resto do desenho de edicao (traco pendente, ponto provisorio, cursor do
// pincel) e' feito numa camada propria por cima: e' desenho que so' existe
// enquanto nao foi gravado, e nao deve entrar no mapa compartilhado.

import * as mapaCanvas from '../mapa_canvas.js';

// ---------------------------------------------------------------- constantes

const ROTAS = {
  estado: '/api/mapeamento/estado',
  iniciar: '/api/mapeamento/iniciar',
  cancelar: '/api/mapeamento/cancelar',
  salvar: '/api/mapeamento/salvar',
  ruido: '/api/mapeamento/ruido',
  paredes: '/api/mapeamento/paredes',
  arenas: '/api/arenas',
  teleop: '/api/teleop',
};

const ETAPAS = [
  {id: 'mapear', numero: 1, nome: 'Mapear'},
  {id: 'limpar', numero: 2, nome: 'Limpar o ruido'},
  {id: 'pontos', numero: 3, nome: 'Marcar os pontos'},
  {id: 'paredes', numero: 4, nome: 'Paredes virtuais'},
  {id: 'conferir', numero: 5, nome: 'Conferir'},
];

// Mesma regra do servidor (REGRA_DE_NOME_DE_MAPA). Repetida aqui so' para
// avisar ENQUANTO se digita; quem recusa de verdade continua sendo o robo.
const REGRA_DE_NOME_DE_MAPA = /^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$/;

// As siglas do regulamento. Servem para montar a lista de nomes quando o robo
// ainda nao tiver a rota que a monta -- nunca para validar no lugar dele.
const PREFIXOS_DE_PONTO = [
  {prefixo: 'WS', rotulo: 'Bancada', tipo: 'workstation'},
  {prefixo: 'SH', rotulo: 'Prateleira', tipo: 'shelf'},
  {prefixo: 'PP', rotulo: 'Encaixe de precisao', tipo: 'precision_placement'},
  {prefixo: 'RT', rotulo: 'Mesa giratoria', tipo: 'rotating_table'},
];

// Tres velocidades, iguais as do resto do painel. O padrao aqui e' o DEVAGAR:
// mapa bom se faz andando devagar, e quem esta mapeando quase sempre esta
// olhando para a tela e nao para o robo.
const VELOCIDADES = {
  devagar: {rotulo: 'Devagar', v: 0.06, w: 0.15},
  normal: {rotulo: 'Normal', v: 0.12, w: 0.30},
  rapido: {rotulo: 'Rapido', v: 0.20, w: 0.50},
};

// Estilo de reserva. O bom vem do robo, dentro dos metadados da arena
// (arenas.ESTILO_DO_MAPA), justamente para nao existirem duas listas de cores.
// Este aqui so' evita tela preta antes da primeira arena carregar.
const ESTILO_RESERVA = {
  edicao: '#f2c94c',
  keepout: 'rgba(235,87,87,0.43)',
  sem_pose: '#eb5757',
  robo: '#ff9a2e',
  waypoint: '#35c3f0',
  area: '#56ccf2',
  dock: '#f2994a',
  dock_inicio: '#27ae60',
  dock_fim: '#9b51e0',
};

// ---------------------------------------------------------------- utilidades

function el(tag, classe, texto) {
  const no = document.createElement(tag);
  if (classe) { no.className = classe; }
  if (texto !== undefined && texto !== null) { no.textContent = texto; }
  return no;
}

function limpar(no) {
  while (no.firstChild) { no.removeChild(no.firstChild); }
}

function metros(valor) {
  return (Math.round(valor * 100) / 100).toFixed(2);
}

// "virado para onde" em palavra, porque radiano nao diz nada a quem esta de pe'
// no meio da arena com o robo na mao.
function paraOndeAponta(yaw) {
  const graus = ((yaw * 180 / Math.PI) % 360 + 360) % 360;
  const nomes = ['direita', 'diagonal cima-direita', 'cima', 'diagonal cima-esquerda',
                 'esquerda', 'diagonal baixo-esquerda', 'baixo', 'diagonal baixo-direita'];
  return nomes[Math.round(graus / 45) % 8];
}

function idDePontoNormalizado(ident) {
  return String(ident || '').trim().toUpperCase();
}

// --------------------------------------------------------- ponte com o app

// O roteador entrega um ctx com a API e o estado. Os nomes de metodo sao dele,
// e esta tela e' escrita em paralelo com ele: por isso a ponte aceita mais de
// um nome e, se nao achar nenhum, cai no fetch cru com a chave da URL. Uma tela
// que morre com "ctx.api.get is not a function" abre em branco, sem uma linha
// de explicacao para quem esta na arena.
function criarPonte(ctx) {
  const bruto = (ctx && ctx.api) ? ctx.api : (ctx || {});
  const obterDoCtx = bruto.obter || bruto.get || bruto.buscar;
  const enviarDoCtx = bruto.enviar || bruto.post || bruto.comandar;
  const apagarDoCtx = bruto.apagar || bruto.remover || bruto.del;
  const avisarDoCtx = (ctx && (ctx.torrada || ctx.aviso || ctx.notificar)) || null;

  const chave = (ctx && ctx.chave) ||
    new URLSearchParams(location.search).get('t') || '';

  function comChave(rota) {
    if (!chave) { return rota; }
    return rota + (rota.includes('?') ? '&' : '?') + 't=' + encodeURIComponent(chave);
  }

  async function cru(rota, metodo, corpo) {
    const opcoes = {method: metodo};
    if (corpo !== undefined) {
      opcoes.headers = {'Content-Type': 'application/json'};
      opcoes.body = JSON.stringify(corpo);
    }
    const resposta = await fetch(comChave(rota), opcoes);
    let dados = {};
    try { dados = await resposta.json(); } catch (erro) { dados = {}; }
    if (dados.ok === undefined) { dados.ok = resposta.ok; }
    if (!dados.ok && !dados.mensagem) {
      dados.mensagem = resposta.status === 403
        ? "Este painel esta so para olhar. Peca o link completo a quem ligou o robo."
        : 'O robo nao respondeu a este pedido.';
    }
    return dados;
  }

  return {
    url: comChave,
    async obter(rota) {
      if (obterDoCtx) { return await obterDoCtx.call(bruto, rota); }
      return await cru(rota, 'GET');
    },
    async enviar(rota, corpo) {
      if (enviarDoCtx) { return await enviarDoCtx.call(bruto, rota, corpo || {}); }
      return await cru(rota, 'POST', corpo || {});
    },
    async apagar(rota) {
      if (apagarDoCtx) { return await apagarDoCtx.call(bruto, rota); }
      return await cru(rota, 'DELETE');
    },
    avisar(texto, ok) {
      if (avisarDoCtx) { avisarDoCtx(texto, ok !== false); }
    },
  };
}

// ------------------------------------------------------- adaptador do mapa

// Resolve a fabrica do mapa_canvas.js sem depender do nome exato do export --
// os dois arquivos estao sendo escritos ao mesmo tempo. Se nao houver fabrica
// nenhuma, devolve null e a tela mostra o motivo em portugues no lugar do mapa,
// em vez de estourar e apagar a tela inteira.
function abrirMapa(elemento, opcoes) {
  const nomes = ['criarMapa', 'criarMapaCanvas', 'montarMapa', 'criar', 'default'];
  let fabrica = null;
  for (const nome of nomes) {
    if (typeof mapaCanvas[nome] === 'function') { fabrica = mapaCanvas[nome]; break; }
  }
  if (!fabrica) { return null; }
  try {
    return fabrica(elemento, opcoes);
  } catch (erro) {
    console.error('mapa_canvas nao subiu:', erro);
    return null;
  }
}

// Chama o primeiro metodo que existir, sem quebrar quando nenhum existe.
function noMapa(mapa, nomes, ...args) {
  if (!mapa) { return undefined; }
  for (const nome of nomes) {
    if (typeof mapa[nome] === 'function') {
      try { return mapa[nome](...args); } catch (erro) { console.error(nome, erro); return undefined; }
    }
  }
  return undefined;
}

// ------------------------------------------------- o mapa crescendo ao vivo

// Enquanto o mapeamento roda, o mapa da arena AINDA NAO EXISTE como arquivo: e'
// exatamente isso que se esta' fazendo. O mapa_canvas.js desenha uma arena
// salva (metadados + imagem), que e' outro objeto. Entao a etapa 1 tem a
// propria tela: uma grade montada no navegador com o que o laser ja' varreu.
//
// Nao e' o mapa oficial (esse quem faz e' o robo, na hora de salvar) e nao
// precisa ser: serve para o operador ver o desenho crescer e saber onde ainda
// nao passou -- que e' a unica pergunta que ele faz enquanto dirige.
function criarVarredura(canvas) {
  const CELULA = 0.10;          // 10 cm por celula: fino o bastante para ver porta
  const LADO = 600;             // 60 m de lado; nao existe arena @Work maior
  const NUNCA = 0, LIVRE = 1, PAREDE = 2;

  const grade = new Uint8Array(LADO * LADO);
  const ctx = canvas.getContext('2d');
  const buffer = document.createElement('canvas');
  buffer.width = LADO;
  buffer.height = LADO;
  const bufferCtx = buffer.getContext('2d');
  const pintura = bufferCtx.createImageData(LADO, LADO);

  let centro = null;            // canto inferior esquerdo da grade, em metros
  let livres = 0;
  let caminho = [];             // pose do robo, para o rastro e a distancia
  let percorrido = 0;
  let sujo = true;
  let limites = null;           // {ix0, iy0, ix1, iy1} do que ja' foi pintado

  function paraCelula(x, y) {
    return [Math.floor((x - centro[0]) / CELULA), Math.floor((y - centro[1]) / CELULA)];
  }

  function marcar(ix, iy, valor) {
    if (ix < 0 || iy < 0 || ix >= LADO || iy >= LADO) { return; }
    const indice = iy * LADO + ix;
    // Parede nunca vira livre de volta pelo raio de outro angulo: o laser passa
    // rente a quina o tempo todo e a parede piscaria.
    if (grade[indice] === valor || (grade[indice] === PAREDE && valor === LIVRE)) { return; }
    if (grade[indice] === NUNCA && valor === LIVRE) { livres += 1; }
    grade[indice] = valor;

    // A imagem cresce para BAIXO e o mundo cresce para CIMA: o y e' invertido
    // aqui, senao a varredura sai espelhada em relacao ao mapa salvo depois.
    const linha = (LADO - 1 - iy) * LADO + ix;
    const p = linha * 4;
    if (valor === PAREDE) {
      pintura.data[p] = 226; pintura.data[p + 1] = 240; pintura.data[p + 2] = 255;
      pintura.data[p + 3] = 255;
    } else {
      pintura.data[p] = 20; pintura.data[p + 1] = 42; pintura.data[p + 2] = 70;
      pintura.data[p + 3] = 255;
    }
    limites = limites || {ix0: ix, iy0: iy, ix1: ix, iy1: iy};
    limites.ix0 = Math.min(limites.ix0, ix); limites.ix1 = Math.max(limites.ix1, ix);
    limites.iy0 = Math.min(limites.iy0, iy); limites.iy1 = Math.max(limites.iy1, iy);
    sujo = true;
  }

  function riscar(ix0, iy0, ix1, iy1) {
    // Bresenham. O ultimo ponto fica de fora: quem marca o fim e' quem chamou,
    // e ele e' parede, nao chao.
    let dx = Math.abs(ix1 - ix0), dy = Math.abs(iy1 - iy0);
    const sx = ix0 < ix1 ? 1 : -1, sy = iy0 < iy1 ? 1 : -1;
    let erro = dx - dy;
    let x = ix0, y = iy0;
    let passos = 0;
    while ((x !== ix1 || y !== iy1) && passos < 1200) {
      marcar(x, y, LIVRE);
      const dobro = 2 * erro;
      if (dobro > -dy) { erro -= dy; x += sx; }
      if (dobro < dx) { erro += dx; y += sy; }
      passos += 1;
    }
  }

  return {
    // Um quadro de estado: pose + laser. Sem pose nao da' para colocar o laser
    // no mundo, entao o quadro e' descartado inteiro.
    alimentar(estado) {
      const pose = estado && estado.pose;
      if (!pose || pose.velho) { return; }
      if (!centro) { centro = [pose.x - LADO * CELULA / 2, pose.y - LADO * CELULA / 2]; }

      const ultimo = caminho[caminho.length - 1];
      if (!ultimo || Math.hypot(pose.x - ultimo[0], pose.y - ultimo[1]) > 0.05) {
        if (ultimo) { percorrido += Math.hypot(pose.x - ultimo[0], pose.y - ultimo[1]); }
        caminho.push([pose.x, pose.y]);
        if (caminho.length > 4000) { caminho.shift(); }
        sujo = true;
      }

      const scan = estado.scan;
      if (!scan || !scan.ranges) { return; }
      const [rx, ry] = paraCelula(pose.x, pose.y);
      // O servidor ja' manda o laser decimado 1:2, entao o passo real e' o
      // dobro do incremento anunciado.
      const passo = scan.angle_increment * 2;
      const alcance = scan.range_max || 12.0;
      for (let i = 0; i < scan.ranges.length; i += 1) {
        const r = scan.ranges[i];
        if (r === null || r === undefined || !(r > 0.05)) { continue; }
        const ang = pose.yaw + scan.angle_min + i * passo;
        const distancia = Math.min(r, alcance);
        const [ex, ey] = paraCelula(pose.x + Math.cos(ang) * distancia,
                                    pose.y + Math.sin(ang) * distancia);
        riscar(rx, ry, ex, ey);
        if (r < alcance - 0.01) { marcar(ex, ey, PAREDE); }
      }
    },

    desenhar() {
      const largura = canvas.clientWidth, altura = canvas.clientHeight;
      if (!largura || !altura) { return; }
      const dpr = window.devicePixelRatio || 1;
      if (canvas.width !== Math.round(largura * dpr)) {
        canvas.width = Math.round(largura * dpr);
        canvas.height = Math.round(altura * dpr);
        sujo = true;
      }
      if (!sujo) { return; }
      sujo = false;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, largura, altura);
      if (!limites || !centro) { return; }

      bufferCtx.putImageData(pintura, 0, 0);

      // Enquadra o que ja' foi varrido, com uma folga, para o desenho nao ficar
      // colado na borda enquanto ainda e' pequeno.
      const folga = 12;
      const ix0 = Math.max(0, limites.ix0 - folga), ix1 = Math.min(LADO - 1, limites.ix1 + folga);
      const iy0 = Math.max(0, limites.iy0 - folga), iy1 = Math.min(LADO - 1, limites.iy1 + folga);
      const larguraCel = ix1 - ix0 + 1, alturaCel = iy1 - iy0 + 1;
      const escala = Math.min(largura / larguraCel, altura / alturaCel) * 0.95;
      const dx = (largura - larguraCel * escala) / 2 - ix0 * escala;
      const dy = (altura - alturaCel * escala) / 2 - (LADO - 1 - iy1) * escala;

      ctx.imageSmoothingEnabled = escala < 3;
      ctx.drawImage(buffer, dx, dy, LADO * escala, LADO * escala);

      const naTela = (x, y) => {
        const cx = (x - centro[0]) / CELULA, cy = (y - centro[1]) / CELULA;
        return [dx + cx * escala, dy + (LADO - cy) * escala];
      };

      if (caminho.length > 1) {
        ctx.beginPath();
        caminho.forEach(([x, y], i) => {
          const [px, py] = naTela(x, y);
          if (i === 0) { ctx.moveTo(px, py); } else { ctx.lineTo(px, py); }
        });
        ctx.strokeStyle = 'rgba(53,195,240,0.55)';
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      const fim = caminho[caminho.length - 1];
      if (fim) {
        const [px, py] = naTela(fim[0], fim[1]);
        ctx.beginPath();
        ctx.arc(px, py, 7, 0, Math.PI * 2);
        ctx.fillStyle = ESTILO_RESERVA.robo;
        ctx.fill();
      }
    },

    resumo() {
      return {
        area: livres * CELULA * CELULA,
        percorrido: percorrido,
        comecou: caminho.length > 1,
      };
    },

    zerar() {
      grade.fill(NUNCA);
      pintura.data.fill(0);
      livres = 0; caminho = []; percorrido = 0; centro = null; limites = null;
      sujo = true;
    },
  };
}

// ------------------------------------------------------------- volante

// Direcao manual na propria tela. Passa pela rota /api/teleop, que e' a que tem
// watchdog no robo -- e' a UNICA que pode ser usada: o ESC desta base entende
// ausencia de comando como re' a fundo e nao existe botao de emergencia de
// hardware, entao qualquer atalho que pule esse caminho e' um robo solto.
function criarVolante(ponte) {
  const pressionados = new Set();
  let velocidade = 'devagar';
  let repetidor = null;
  let dirigindo = false;
  let aoMudar = null;

  const raiz = el('div', 'volante');

  const barraVel = el('div', 'segmentado velocidadeManual');
  Object.keys(VELOCIDADES).forEach((chave) => {
    const bt = el('button', 'segmento', VELOCIDADES[chave].rotulo);
    bt.type = 'button';
    bt.dataset.velocidade = chave;
    bt.setAttribute('aria-pressed', String(chave === velocidade));
    bt.onclick = () => {
      velocidade = chave;
      barraVel.querySelectorAll('button').forEach(
        (b) => b.setAttribute('aria-pressed', String(b === bt)));
    };
    barraVel.appendChild(bt);
  });
  raiz.appendChild(barraVel);

  const cruz = el('div', 'cruzetaVolante');
  const BOTOES = [
    {dir: 'giroEsq', texto: 'Girar\npara a esquerda', curto: 'Girar esq.'},
    {dir: 'frente', texto: 'Frente', curto: 'Frente'},
    {dir: 'giroDir', texto: 'Girar\npara a direita', curto: 'Girar dir.'},
    {dir: 'esquerda', texto: 'Esquerda', curto: 'Esquerda'},
    {dir: 'parar', texto: 'Parar', curto: 'Parar'},
    {dir: 'direita', texto: 'Direita', curto: 'Direita'},
    {dir: '', texto: '', curto: ''},
    {dir: 'tras', texto: 'Tras', curto: 'Tras'},
    {dir: '', texto: '', curto: ''},
  ];
  BOTOES.forEach((def) => {
    if (!def.dir) { cruz.appendChild(el('span', 'vagoVolante')); return; }
    const bt = el('button', def.dir === 'parar' ? 'volanteBotao volanteParar' : 'volanteBotao',
                  def.curto);
    bt.type = 'button';
    bt.dataset.dir = def.dir;
    cruz.appendChild(bt);
  });
  raiz.appendChild(cruz);

  const estadoTexto = el('p', 'estadoVolante', 'Parado.');
  raiz.appendChild(estadoTexto);

  function vetor() {
    const {v, w} = VELOCIDADES[velocidade];
    let vx = 0, vy = 0, wz = 0;
    if (pressionados.has('frente')) { vx += v; }
    if (pressionados.has('tras')) { vx -= v; }
    if (pressionados.has('esquerda')) { vy += v; }
    if (pressionados.has('direita')) { vy -= v; }
    if (pressionados.has('giroEsq')) { wz += w; }
    if (pressionados.has('giroDir')) { wz -= w; }
    return {vx, vy, wz};
  }

  function pulsar() {
    if (!pressionados.size) { return; }
    ponte.enviar(ROTAS.teleop, vetor());
    estadoTexto.textContent = 'Andando.';
    if (aoMudar) { aoMudar(true); }
  }

  function segurar(dir) {
    if (dir === 'parar') { parar(true); return; }
    dirigindo = true;
    pressionados.add(dir);
    if (!repetidor) { repetidor = setInterval(pulsar, 100); }
    pulsar();
  }

  function soltar(dir) {
    pressionados.delete(dir);
    if (!pressionados.size) { parar(false); }
  }

  // O "parar" so' vai quando ALGUEM estava dirigindo. Nao e' economia de
  // pedido: o comando manual tem prioridade sobre a navegacao, entao um parar
  // gratuito (trocar de etapa, a tela apagar) frearia por meio segundo um robo
  // que estava navegando sozinho, sem ninguem ter pedido nada.
  function parar(forcar) {
    pressionados.clear();
    if (repetidor) { clearInterval(repetidor); repetidor = null; }
    estadoTexto.textContent = 'Parado.';
    if (aoMudar) { aoMudar(false); }
    if (!dirigindo && !forcar) { return; }
    dirigindo = false;
    ponte.enviar(ROTAS.teleop, {parar: true});
  }

  cruz.querySelectorAll('button[data-dir]').forEach((bt) => {
    const dir = bt.dataset.dir;
    bt.addEventListener('pointerdown', (ev) => {
      ev.preventDefault();
      bt.setPointerCapture(ev.pointerId);
      segurar(dir);
    });
    ['pointerup', 'pointercancel', 'pointerleave'].forEach(
      (nome) => bt.addEventListener(nome, () => soltar(dir)));
  });

  const TECLAS = {
    w: 'frente', s: 'tras', a: 'esquerda', d: 'direita', q: 'giroEsq', e: 'giroDir',
    ArrowUp: 'frente', ArrowDown: 'tras', ArrowLeft: 'esquerda', ArrowRight: 'direita',
  };
  function digitando(alvo) {
    return alvo && alvo.matches && alvo.matches('input, textarea, select');
  }
  function aoApertar(ev) {
    if (!raiz.isConnected || ev.repeat || digitando(ev.target)) { return; }
    if (ev.key === ' ') { ev.preventDefault(); parar(true); return; }
    const dir = TECLAS[ev.key] || TECLAS[String(ev.key).toLowerCase()];
    if (dir) { ev.preventDefault(); segurar(dir); }
  }
  function aoSoltar(ev) {
    const dir = TECLAS[ev.key] || TECLAS[String(ev.key).toLowerCase()];
    if (dir) { soltar(dir); }
  }
  // Perder o foco da janela nao pode deixar o robo andando: o navegador para de
  // entregar o keyup e a tecla ficaria presa para sempre.
  function aoPerderFoco() { parar(false); }
  function aoEsconder() { if (document.hidden) { parar(false); } }

  window.addEventListener('keydown', aoApertar);
  window.addEventListener('keyup', aoSoltar);
  window.addEventListener('blur', aoPerderFoco);
  document.addEventListener('visibilitychange', aoEsconder);

  return {
    raiz,
    definirAviso(fn) { aoMudar = fn; },
    destruir() {
      parar(false);
      window.removeEventListener('keydown', aoApertar);
      window.removeEventListener('keyup', aoSoltar);
      window.removeEventListener('blur', aoPerderFoco);
      document.removeEventListener('visibilitychange', aoEsconder);
    },
  };
}

// ------------------------------------------------- camada de edicao do mapa

// Tudo o que ainda NAO foi gravado mora aqui: o traco que esta sendo pintado, o
// ponto que foi marcado mas nao gravado, o cursor do pincel. Fica numa camada
// propria por cima do mapa porque e' desenho provisorio -- se fosse pintado
// dentro do mapa compartilhado, um traco descartado continuaria na tela das
// outras telas.
//
// Os gestos sao ouvidos na CAPTURA do container: assim o dedo com um dedo so'
// vem para ca' e o mapa nem chega a ver, enquanto dois dedos, botao direito e
// roda continuam passando direto para o mapa (que e' quem sabe dar zoom e
// arrastar).
function criarCamadaDeEdicao(container, mapa, obterEstilo) {
  const canvas = el('canvas', 'camadaEdicao');
  // Geometria, nao tema: a camada precisa ficar exatamente em cima do mapa e
  // nao pode roubar o dedo de quem estiver dando zoom. Cor nenhuma aqui.
  canvas.style.position = 'absolute';
  canvas.style.left = '0';
  canvas.style.top = '0';
  canvas.style.width = '100%';
  canvas.style.height = '100%';
  canvas.style.pointerEvents = 'none';
  if (getComputedStyle(container).position === 'static') {
    container.style.position = 'relative';
  }
  container.appendChild(canvas);
  const ctx = canvas.getContext('2d');

  let modo = 'ver';             // 'ver' | 'apontar' | 'pintar'
  let pincel = {valor: 'livre', raio_m: 0.10, cor: ESTILO_RESERVA.edicao};
  let aoApontar = null;
  let aoPintar = null;
  let gesto = null;             // {px0, py0, px1, py1} em pixels da camada
  let provisorio = null;        // {x, y, yaw} ja' escolhido, ainda nao gravado
  let tracos = [];              // grupos de pinceladas, um grupo por arrasto
  let cursor = null;            // ultima posicao do dedo, para o circulo do pincel
  let animacao = null;

  function relativo(ev) {
    const r = canvas.getBoundingClientRect();
    return [ev.clientX - r.left, ev.clientY - r.top];
  }

  function paraMundo(px, py) {
    const saida = noMapa(mapa, ['paraMundo', 'telaParaMundo', 'paraMetros'], px, py);
    return Array.isArray(saida) ? saida : (saida && [saida.x, saida.y]);
  }

  function paraTela(x, y) {
    const saida = noMapa(mapa, ['paraTela', 'mundoParaTela', 'paraPixel'], x, y);
    return Array.isArray(saida) ? saida : (saida && [saida.px, saida.py]);
  }

  // Um dedo/botao esquerdo edita; o resto e' do mapa (zoom e arrastar).
  function meu(ev) {
    return modo !== 'ver' && ev.isPrimary && ev.button === 0 &&
      !(ev.buttons & 2) && !(ev.buttons & 4);
  }

  function aoPressionar(ev) {
    if (!meu(ev)) { return; }
    ev.stopPropagation();
    ev.preventDefault();
    container.setPointerCapture(ev.pointerId);
    const [px, py] = relativo(ev);
    gesto = {px0: px, py0: py, px1: px, py1: py};
    cursor = [px, py];
    if (modo === 'pintar') {
      const mundo = paraMundo(px, py);
      if (!mundo) { gesto = null; return; }
      // Um toque parado tambem pinta: quem so' encostou quer apagar aquele
      // ponto, e exigir arrasto faria o toque parecer que nao funcionou.
      tracos.push([{valor: pincel.valor, raio_m: pincel.raio_m, x: mundo[0], y: mundo[1]}]);
      gesto.ultimo = mundo;
    }
  }

  function aoMover(ev) {
    const [px, py] = relativo(ev);
    cursor = [px, py];
    if (!gesto) { return; }
    if (!meu(ev) && ev.buttons !== 1) { return; }
    ev.stopPropagation();
    gesto.px1 = px;
    gesto.py1 = py;
    if (modo !== 'pintar') { return; }
    const mundo = paraMundo(px, py);
    if (!mundo || !gesto.ultimo) { return; }
    const avanco = Math.hypot(mundo[0] - gesto.ultimo[0], mundo[1] - gesto.ultimo[1]);
    // Nao guarda um segmento por pixel de mouse: o servidor recusa mais de 5000
    // pinceladas de uma vez, e um traco de dois metros gastaria a cota inteira.
    if (avanco < Math.max(0.02, pincel.raio_m * 0.6)) { return; }
    const grupo = tracos[tracos.length - 1];
    grupo.push({valor: pincel.valor, raio_m: pincel.raio_m,
                x0: gesto.ultimo[0], y0: gesto.ultimo[1], x1: mundo[0], y1: mundo[1]});
    gesto.ultimo = mundo;
  }

  function aoLargar(ev) {
    if (!gesto) { return; }
    ev.stopPropagation();
    const fim = gesto;
    gesto = null;
    try { container.releasePointerCapture(ev.pointerId); } catch (erro) { /* ja' solto */ }
    if (modo !== 'apontar' || !aoApontar) { return; }
    const mundo = paraMundo(fim.px0, fim.py0);
    if (!mundo) { return; }
    const dx = fim.px1 - fim.px0, dy = fim.py1 - fim.py0;
    // Arrasto curto e' clique: quem so' encostou nao quis escolher direcao.
    // Sem esta tolerancia o tremor do dedo num tablet vira direcao aleatoria.
    const arrastou = Math.hypot(dx, dy) > 12;
    provisorio = {
      x: mundo[0], y: mundo[1],
      yaw: arrastou ? Math.atan2(-dy, dx) : 0,
      temDirecao: arrastou,
    };
    aoApontar(Object.assign({}, provisorio));
  }

  container.addEventListener('pointerdown', aoPressionar, true);
  container.addEventListener('pointermove', aoMover, true);
  container.addEventListener('pointerup', aoLargar, true);
  container.addEventListener('pointercancel', aoLargar, true);
  container.addEventListener('pointerleave', () => { cursor = null; });

  function desenharPincelada(p, escalaPorMetro, cor) {
    const raio = Math.max(1.5, p.raio_m * escalaPorMetro);
    ctx.strokeStyle = cor;
    ctx.fillStyle = cor;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.lineWidth = raio * 2;
    if (p.x1 !== undefined) {
      const a = paraTela(p.x0, p.y0), b = paraTela(p.x1, p.y1);
      if (!a || !b) { return; }
      ctx.beginPath();
      ctx.moveTo(a[0], a[1]);
      ctx.lineTo(b[0], b[1]);
      ctx.stroke();
    } else {
      const a = paraTela(p.x, p.y);
      if (!a) { return; }
      ctx.beginPath();
      ctx.arc(a[0], a[1], raio, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Quantos pixels de tela cabem num metro. Sai da propria conversao do mapa,
  // para o pincel na tela ter o tamanho que ele vai ter no chao.
  function escalaPorMetro() {
    const a = paraTela(0, 0), b = paraTela(1, 0);
    if (!a || !b) { return 40; }
    const escala = Math.hypot(b[0] - a[0], b[1] - a[1]);
    return escala > 0.01 ? escala : 40;
  }

  function desenhar() {
    const largura = canvas.clientWidth, altura = canvas.clientHeight;
    if (!largura || !altura) { return; }
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(largura * dpr)) {
      canvas.width = Math.round(largura * dpr);
      canvas.height = Math.round(altura * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, largura, altura);
    if (modo === 'ver' && !provisorio) { return; }

    const estilo = obterEstilo();
    const porMetro = escalaPorMetro();

    tracos.forEach((grupo) => grupo.forEach(
      (p) => desenharPincelada(p, porMetro, pincel.cor)));

    if (modo === 'pintar' && cursor) {
      const raio = Math.max(2, pincel.raio_m * porMetro);
      ctx.beginPath();
      ctx.arc(cursor[0], cursor[1], raio, 0, Math.PI * 2);
      ctx.strokeStyle = estilo.edicao || ESTILO_RESERVA.edicao;
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    if (modo === 'apontar' && gesto) {
      ctx.beginPath();
      ctx.moveTo(gesto.px0, gesto.py0);
      ctx.lineTo(gesto.px1, gesto.py1);
      ctx.strokeStyle = estilo.edicao || ESTILO_RESERVA.edicao;
      ctx.lineWidth = 3;
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(gesto.px0, gesto.py0, 8, 0, Math.PI * 2);
      ctx.fillStyle = estilo.edicao || ESTILO_RESERVA.edicao;
      ctx.fill();
    }

    if (provisorio) {
      const a = paraTela(provisorio.x, provisorio.y);
      if (a) {
        ctx.beginPath();
        ctx.arc(a[0], a[1], 9, 0, Math.PI * 2);
        ctx.fillStyle = estilo.edicao || ESTILO_RESERVA.edicao;
        ctx.fill();
        ctx.strokeStyle = '#06121f';
        ctx.lineWidth = 2;
        ctx.stroke();
        if (provisorio.temDirecao) {
          ctx.beginPath();
          ctx.moveTo(a[0], a[1]);
          ctx.lineTo(a[0] + Math.cos(provisorio.yaw) * 26,
                     a[1] - Math.sin(provisorio.yaw) * 26);
          ctx.strokeStyle = estilo.edicao || ESTILO_RESERVA.edicao;
          ctx.lineWidth = 3;
          ctx.stroke();
        }
      }
    }
  }

  function laco() {
    desenhar();
    animacao = requestAnimationFrame(laco);
  }
  animacao = requestAnimationFrame(laco);

  return {
    definirModo(novo) {
      modo = novo;
      canvas.style.pointerEvents = 'none';   // quem ouve e' o container
      container.style.cursor = novo === 'ver' ? '' : 'crosshair';
    },
    definirPincel(novo) { pincel = Object.assign({}, pincel, novo); },
    definirAoApontar(fn) { aoApontar = fn; },
    definirProvisorio(novo) { provisorio = novo; },
    quantasPinceladas() { return tracos.reduce((soma, g) => soma + g.length, 0); },
    quantosTracos() { return tracos.length; },
    pinceladas() { return tracos.reduce((lista, g) => lista.concat(g), []); },
    desfazer() { tracos.pop(); },
    descartar() { tracos = []; },
    destruir() {
      cancelAnimationFrame(animacao);
      container.removeEventListener('pointerdown', aoPressionar, true);
      container.removeEventListener('pointermove', aoMover, true);
      container.removeEventListener('pointerup', aoLargar, true);
      container.removeEventListener('pointercancel', aoLargar, true);
      container.style.cursor = '';
      canvas.remove();
    },
  };
}

// ------------------------------------------------------- pecas de interface

function grupo(titulo) {
  const secao = el('section', 'grupoPainel');
  if (titulo) { secao.appendChild(el('h3', 'tituloPainel', titulo)); }
  return secao;
}

function texto(conteudo, classe) {
  return el('p', classe || 'textoPainel', conteudo);
}

function botao(rotulo, classe, aoClicar) {
  const bt = el('button', classe || '', rotulo);
  bt.type = 'button';
  if (aoClicar) { bt.onclick = aoClicar; }
  return bt;
}

// Controle segmentado: as opcoes TODAS a vista, lado a lado. Um combo esconde
// as outras atras de um clique, e nesta tela a escolha entre duas maneiras de
// trabalhar e' justamente o que precisa ser enxergado de uma vez.
function segmentado(opcoes, escolhido, aoEscolher) {
  const raiz = el('div', 'segmentado');
  raiz.setAttribute('role', 'group');
  const botoes = opcoes.map((opcao) => {
    const bt = el('button', 'segmento');
    bt.type = 'button';
    bt.dataset.valor = opcao.valor;
    bt.appendChild(el('span', 'rotuloSegmento', opcao.rotulo));
    if (opcao.dica) { bt.appendChild(el('small', 'dicaSegmento', opcao.dica)); }
    bt.setAttribute('aria-pressed', String(opcao.valor === escolhido));
    bt.onclick = () => {
      botoes.forEach((b) => b.setAttribute('aria-pressed', String(b === bt)));
      aoEscolher(opcao.valor);
    };
    raiz.appendChild(bt);
    return bt;
  });
  return {raiz, definir(valor) {
    botoes.forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.valor === valor)));
  }};
}

function listaDeOpcoes(opcoes, escolhido, aoEscolher) {
  const lista = el('select', 'listaOpcoes');
  opcoes.forEach((opcao) => {
    const item = el('option', '', opcao.rotulo);
    item.value = opcao.valor;
    if (opcao.valor === escolhido) { item.selected = true; }
    lista.appendChild(item);
  });
  lista.onchange = () => aoEscolher(lista.value);
  return lista;
}

function campoComRotulo(rotulo, controle) {
  const linha = el('label', 'campoPainel');
  linha.appendChild(el('span', 'rotuloCampo', rotulo));
  linha.appendChild(controle);
  return linha;
}

// Confirmacao em dois toques, no proprio botao. Modal de confirmacao em tela de
// robo e' pior: some o contexto e o dedo ja' vem descendo no "sim".
function botaoConfirmando(rotulo, rotuloArmado, classe, aoConfirmar) {
  let armado = null;
  const bt = botao(rotulo, classe, () => {
    if (armado) {
      clearTimeout(armado);
      armado = null;
      bt.textContent = rotulo;
      bt.classList.remove('armado');
      aoConfirmar();
      return;
    }
    bt.textContent = rotuloArmado;
    bt.classList.add('armado');
    armado = setTimeout(() => {
      armado = null;
      bt.textContent = rotulo;
      bt.classList.remove('armado');
    }, 6000);
  });
  return bt;
}

function deslizantePincel(aoMudar) {
  const raiz = el('div', 'campoPainel');
  const rotulo = el('span', 'rotuloCampo', 'Espessura do pincel: 20 cm');
  const barra = el('input', 'deslizante');
  barra.type = 'range';
  barra.min = '5';
  barra.max = '150';
  barra.step = '5';
  barra.value = '20';
  barra.oninput = () => {
    rotulo.textContent = 'Espessura do pincel: ' + barra.value + ' cm';
    aoMudar(Number(barra.value) / 200);   // raio em metros
  };
  raiz.appendChild(rotulo);
  raiz.appendChild(barra);
  return raiz;
}

// ------------------------------------------------------------ dados da arena

// Um ponto e' UMA coisa, mesmo morando em dois arquivos no robo. Juntar aqui
// evita a lista mostrar "WS1" duas vezes, que ja' fez operador apagar o que
// achou que era duplicado.
function pontosDaArena(meta) {
  const juntos = new Map();
  (meta.areas || []).forEach((a) => juntos.set(a.id, {
    id: a.id, tipo: a.tipo, tipo_rotulo: a.tipo_rotulo, sem_pose: !!a.sem_pose,
    x: a.x, y: a.y, yaw: a.yaw, papel: '',
  }));
  (meta.docks || []).forEach((d) => {
    const ja = juntos.get(d.id);
    if (ja) {
      ja.sem_pose = ja.sem_pose || !!d.sem_pose;
      ja.papel = d.papel || ja.papel;
      return;
    }
    juntos.set(d.id, {
      id: d.id, tipo: '', tipo_rotulo: d.tipo_rotulo, sem_pose: !!d.sem_pose,
      x: d.x, y: d.y, yaw: d.yaw, papel: d.papel || '',
    });
  });
  return [...juntos.values()].sort((a, b) => a.id.localeCompare(b.id));
}

// Mesma deducao que o robo faz a partir do nome (type_from_area_id).
function tipoPeloNome(ident) {
  const nome = idDePontoNormalizado(ident);
  if (nome === 'START') { return 'start'; }
  if (nome === 'FINISH') { return 'finish'; }
  const achado = PREFIXOS_DE_PONTO.find((p) => nome.startsWith(p.prefixo));
  return achado ? achado.tipo : '';
}

// Lista de nomes de reserva, para o caso de o robo ainda nao ter a rota que a
// monta. So' de reserva: se o painel ficasse sem lista nenhuma, nao haveria
// como nomear um ponto, e a tela viraria um beco sem saida.
function nomesSugeridosLocal(meta) {
  const existentes = new Set();
  ['docks', 'areas', 'waypoints'].forEach((chave) => {
    (meta[chave] || []).forEach((item) => existentes.add(idDePontoNormalizado(item.id)));
  });
  const saida = [...existentes].sort().map((id) => ({id, existe: true}));
  ['START', 'FINISH'].forEach((fixo) => {
    if (!existentes.has(fixo)) { saida.push({id: fixo, existe: false, rotulo: fixo}); }
  });
  PREFIXOS_DE_PONTO.forEach((p) => {
    const usados = [...existentes]
      .filter((i) => i.startsWith(p.prefixo) && /^[0-9]+$/.test(i.slice(p.prefixo.length)))
      .map((i) => parseInt(i.slice(p.prefixo.length), 10));
    const proximo = usados.length ? Math.max(...usados) + 1 : 1;
    saida.push({id: p.prefixo + proximo, existe: false,
                rotulo: p.rotulo + ' ' + proximo, tipo_sugerido: p.tipo});
  });
  return saida;
}

// Os avisos da etapa 5, um por linha e cada um levando a etapa que resolve.
// Contagem ("2 docks sem posicao") nao serve aqui: quem esta' com o robo na mao
// precisa saber QUAL ponto abrir.
function oQueFalta(meta) {
  const faltas = [];
  if (!meta || meta.erro) {
    return [{texto: 'Nenhum mapa salvo ainda para conferir.', etapa: 'mapear'}];
  }
  const pontos = pontosDaArena(meta);
  if (!pontos.length) {
    faltas.push({texto: 'Nenhum ponto marcado nesta arena: nao da para rodar prova aqui.',
                 etapa: 'pontos'});
  }
  if (!pontos.some((p) => p.id === 'START')) {
    faltas.push({texto: 'Nenhum dock START definido: a prova nao tem de onde comecar.',
                 etapa: 'pontos'});
  }
  if (!pontos.some((p) => p.id === 'FINISH')) {
    faltas.push({texto: 'Nenhum ponto FINISH definido: a prova nao tem onde terminar.',
                 etapa: 'pontos'});
  }
  pontos.filter((p) => p.sem_pose).forEach((p) => {
    faltas.push({texto: p.id + ' sem pose gravada: a missao aborta ao chegar nele.',
                 etapa: 'pontos'});
  });
  (meta.areas || []).filter((a) => a.tipo && !a.tipo_valido).forEach((a) => {
    faltas.push({texto: a.id + ' esta marcada com um tipo de estacao que o robo nao conhece.',
                 etapa: 'pontos'});
  });
  if (!meta.tem_keepout) {
    faltas.push({texto: 'Sem paredes virtuais nesta arena. Fita de piso, degrau baixo e ' +
                        'vidro sao invisiveis para o LiDAR.', etapa: 'paredes'});
  }
  // Os avisos que so' o robo sabe dar (arquivo ilegivel, altura de bancada que
  // falta) entram como vieram; os que ja' viraram linha por ponto, nao.
  (meta.avisos || []).forEach((aviso) => {
    if (/posicao gravada|paredes virtuais|nenhum dock/i.test(aviso)) { return; }
    faltas.push({texto: aviso, etapa: /parede/i.test(aviso) ? 'paredes'
      : (/imagem|mapa/i.test(aviso) ? 'mapear' : 'pontos')});
  });
  return faltas;
}

// =========================================================== ETAPA 1: mapear

function painelMapear(t) {
  const raiz = el('div', 'conteudoEtapa');
  raiz.appendChild(el('h2', 'tituloEtapa', '1. Mapear'));
  raiz.appendChild(texto(
    'Ligue o mapeamento e dirija o robo DEVAGAR pelo ambiente, passando por ' +
    'todos os cantos e entrando em cada corredor. O desenho ao lado cresce ' +
    'conforme o LiDAR enxerga.', 'explicacaoEtapa'));

  // --- em que arena estamos mexendo ---
  const gArena = grupo('Arena');
  const opcoesArena = [{valor: '', rotulo: 'Criar uma arena nova'}]
    .concat(t.arenasSalvas.map((nome) => ({valor: nome, rotulo: nome})));
  gArena.appendChild(campoComRotulo(
    'Estou trabalhando em',
    listaDeOpcoes(opcoesArena, t.arena, (valor) => t.trocarArena(valor))));
  const notaArena = texto('', 'notaPainel');
  gArena.appendChild(notaArena);
  raiz.appendChild(gArena);

  // --- ligar / desligar ---
  const gLigar = grupo('Mapeamento');
  const btLigar = botao('Ligar o mapeamento', 'acaoPrimaria', async () => {
    btLigar.disabled = true;
    const ligado = t.mapeamentoLigado();
    const resposta = await t.ponte.enviar(ligado ? ROTAS.cancelar : ROTAS.iniciar, {});
    t.ponte.avisar(resposta.mensagem, resposta.ok);
    if (!ligado && resposta.ok) { t.varredura.zerar(); }
    await t.recarregarMapeamento();
  });
  gLigar.appendChild(btLigar);
  const estadoSlam = texto('', 'estadoPainel');
  gLigar.appendChild(estadoSlam);
  const cobertura = texto('', 'coberturaPainel');
  gLigar.appendChild(cobertura);
  raiz.appendChild(gLigar);

  // --- dirigir ---
  const gDirigir = grupo('Dirigir o robo');
  gDirigir.appendChild(texto(
    'Tambem funciona pelo teclado: W A S D para andar de lado, Q e E para ' +
    'girar, barra de espaco para parar.', 'notaPainel'));
  gDirigir.appendChild(t.volante.raiz);
  raiz.appendChild(gDirigir);

  // --- salvar ---
  const gSalvar = grupo('Salvar o mapa');
  gSalvar.appendChild(texto(
    'Salve ANTES de desligar o mapeamento: e o robo quem entrega o mapa, e ele ' +
    'so tem o mapa na memoria enquanto o mapeamento esta ligado.', 'avisoPainel'));

  // O UNICO campo de texto livre desta tela -- um mapa novo, por definicao,
  // ainda nao esta em lista nenhuma.
  const campoNome = el('input', 'campoTexto');
  campoNome.type = 'text';
  campoNome.placeholder = 'arena_regional';
  campoNome.autocomplete = 'off';
  campoNome.spellcheck = false;
  campoNome.value = t.arena || '';
  gSalvar.appendChild(campoComRotulo('Nome do mapa', campoNome));
  const erroNome = texto('', 'erroPainel');
  gSalvar.appendChild(erroNome);

  function conferirNome() {
    const nome = campoNome.value.trim();
    if (!nome) {
      erroNome.textContent = 'De um nome ao mapa antes de salvar.';
      return {nome: '', valido: false, existe: false};
    }
    if (!REGRA_DE_NOME_DE_MAPA.test(nome)) {
      erroNome.textContent = 'Use letras, numeros, hifen e sublinhado, sem espacos ' +
        'e sem acentos -- por exemplo arena_regional.';
      return {nome, valido: false, existe: false};
    }
    const existe = t.arenasSalvas.includes(nome);
    erroNome.textContent = existe
      ? 'Ja existe uma arena com este nome. Salvar vai substituir o mapa dela.'
      : '';
    return {nome, valido: true, existe};
  }
  campoNome.oninput = () => { conferirNome(); atualizarSalvar(); };

  const btSalvar = botao('Salvar o mapa', 'acaoPrimaria', async () => {
    const conferido = conferirNome();
    if (!conferido.valido) { return; }
    if (conferido.existe && btSalvar.dataset.confirmado !== conferido.nome) {
      btSalvar.dataset.confirmado = conferido.nome;
      btSalvar.textContent = 'Confirmar: salvar por cima de ' + conferido.nome;
      btSalvar.classList.add('armado');
      return;
    }
    btSalvar.disabled = true;
    const resposta = await t.ponte.enviar(ROTAS.salvar, {nome: conferido.nome});
    btSalvar.classList.remove('armado');
    btSalvar.dataset.confirmado = '';
    btSalvar.textContent = 'Salvar o mapa';
    t.ponte.avisar(resposta.mensagem, resposta.ok);
    limpar(avisosSalvar);
    (resposta.avisos || []).forEach((aviso) => avisosSalvar.appendChild(
      texto(aviso, 'avisoPainel')));
    if (resposta.ok) {
      await t.recarregarArenas();
      await t.trocarArena(resposta.arena || conferido.nome);
      proximo.hidden = false;
    }
    await t.recarregarMapeamento();
  });
  gSalvar.appendChild(btSalvar);
  const avisosSalvar = el('div', 'avisosPainel');
  gSalvar.appendChild(avisosSalvar);

  const proximo = botao('Proximo passo: limpar o ruido', 'acaoSecundaria',
                        () => t.irParaEtapa('limpar'));
  proximo.hidden = true;
  gSalvar.appendChild(proximo);
  raiz.appendChild(gSalvar);

  function atualizarSalvar() {
    const conferido = campoNome.value.trim();
    btSalvar.disabled = !conferido || !t.mapeamentoLigado();
    if (btSalvar.dataset.confirmado && btSalvar.dataset.confirmado !== conferido) {
      btSalvar.dataset.confirmado = '';
      btSalvar.textContent = 'Salvar o mapa';
      btSalvar.classList.remove('armado');
    }
  }

  function atualizar() {
    const estado = t.estadoMapeamento || {};
    const ligado = !!estado.ligado;
    btLigar.textContent = ligado ? 'Desligar o mapeamento' : 'Ligar o mapeamento';
    btLigar.disabled = !!estado.ligado_por_fora;
    estadoSlam.textContent = estado.mensagem || 'Perguntando ao robo...';

    notaArena.textContent = t.arena
      ? 'Mapear de novo substitui o mapa desta arena. Os pontos ja marcados ' +
        'continuam, mas confira se ainda caem no lugar certo.'
      : 'Nenhuma arena escolhida: o mapa que voce fizer agora nasce novo, com o ' +
        'nome que voce der abaixo.';

    const resumo = t.varredura.resumo();
    if (!ligado && !resumo.comecou) {
      cobertura.textContent = '';
    } else if (!resumo.comecou) {
      cobertura.textContent = 'Ainda nao chegou nada do LiDAR. Confira se o robo ' +
        'esta ligado e respondendo.';
    } else {
      cobertura.textContent = 'Ja varrido: cerca de ' + Math.round(resumo.area) +
        ' m2 de chao. Robo dirigido: ' + resumo.percorrido.toFixed(1) + ' m.';
    }
    atualizarSalvar();
  }

  conferirNome();
  atualizar();
  return {raiz, atualizar};
}

// ==================================================== ETAPA 2: limpar o ruido

function painelLimpar(t) {
  const raiz = el('div', 'conteudoEtapa');
  raiz.appendChild(el('h2', 'tituloEtapa', '2. Limpar o ruido'));
  raiz.appendChild(texto(
    'Durante o mapeamento, gente andando na frente do robo, uma cadeira que ' +
    'saiu do lugar e o reflexo de um vidro viram parede no mapa. O robo vai ' +
    'desviar dessas paredes que nao existem. Apague o que sobrou.',
    'explicacaoEtapa'));

  // --- automatico ---
  const gAuto = grupo('Tirar as manchas soltas');
  gAuto.appendChild(texto(
    'Tira do mapa as manchinhas que estao soltas no meio do nada. Parede de ' +
    'verdade nunca esta solta: ela encosta em outra parede, e por isso nao sai.',
    'notaPainel'));
  let nivel = 'medio';
  gAuto.appendChild(segmentado([
    {valor: 'leve', rotulo: 'Leve', dica: 'so o cisco'},
    {valor: 'medio', rotulo: 'Media', dica: 'o comum'},
    {valor: 'forte', rotulo: 'Forte', dica: 'pega ate pessoa'},
  ], nivel, (valor) => { nivel = valor; }).raiz);
  gAuto.appendChild(botao('Limpar as manchas soltas', 'acaoSecundaria', async () => {
    const resposta = await t.ponte.enviar(ROTAS.ruido, {arena: t.arena, nivel});
    t.ponte.avisar(resposta.mensagem, resposta.ok);
    if (resposta.ok) { t.limpezaFeita = true; await t.recarregarMeta(); }
  }));
  raiz.appendChild(gAuto);

  // --- a mao ---
  const gMao = grupo('Apagar a mao');
  gMao.appendChild(texto(
    'Pinte por cima do mapa. A borracha vira chao livre; o lapis marca uma ' +
    'parede que existe de verdade e o LiDAR nao pegou.', 'notaPainel'));

  let ferramenta = 'livre';
  const troca = segmentado([
    {valor: 'livre', rotulo: 'Borracha', dica: 'vira chao livre'},
    {valor: 'parede', rotulo: 'Lapis', dica: 'vira parede'},
  ], ferramenta, (valor) => {
    ferramenta = valor;
    t.camada.definirPincel({
      valor,
      cor: valor === 'parede' ? t.estilo().sem_pose : t.estilo().edicao,
    });
  });
  gMao.appendChild(troca.raiz);
  gMao.appendChild(deslizantePincel((raio) => t.camada.definirPincel({raio_m: raio})));

  const pendentes = texto('', 'estadoPainel');
  gMao.appendChild(pendentes);

  const btDesfazer = botao('Desfazer o ultimo traco', 'acaoSecundaria', () => {
    t.camada.desfazer();
    atualizar();
  });
  gMao.appendChild(btDesfazer);
  const btDescartar = botao('Descartar tudo o que pintei', 'acaoSecundaria', () => {
    t.camada.descartar();
    atualizar();
  });
  gMao.appendChild(btDescartar);

  const btAplicar = botao('Aplicar no mapa', 'acaoPrimaria', async () => {
    const pinceladas = t.camada.pinceladas();
    if (!pinceladas.length) { return; }
    btAplicar.disabled = true;
    const resposta = await t.ponte.enviar(ROTAS.ruido, {arena: t.arena, pinceladas});
    t.ponte.avisar(resposta.mensagem, resposta.ok);
    if (resposta.ok) {
      t.camada.descartar();
      t.limpezaFeita = true;
      await t.recarregarMeta();
    }
    atualizar();
  });
  gMao.appendChild(btAplicar);
  gMao.appendChild(texto(
    'A versao anterior do mapa fica guardada no robo, entao da para voltar atras ' +
    'se algo sair errado.', 'notaPainel'));
  raiz.appendChild(gMao);

  raiz.appendChild(botao('Proximo passo: marcar os pontos', 'acaoSecundaria',
                         () => t.irParaEtapa('pontos')));

  function atualizar() {
    const quantos = t.camada.quantosTracos();
    pendentes.textContent = quantos
      ? quantos + ' traco(s) pintado(s) e ainda nao aplicado(s).'
      : 'Nada pintado ainda. Arraste em cima do mapa.';
    btDesfazer.disabled = !quantos;
    btDescartar.disabled = !quantos;
    btAplicar.disabled = !quantos;
  }

  t.camada.definirModo('pintar');
  t.camada.definirPincel({valor: 'livre', raio_m: 0.10, cor: t.estilo().edicao});
  atualizar();
  return {raiz, atualizar, aoDesenhar: atualizar};
}

// ================================================= ETAPA 3: marcar os pontos

function painelPontos(t) {
  const raiz = el('div', 'conteudoEtapa');
  raiz.appendChild(el('h2', 'tituloEtapa', '3. Marcar os pontos'));
  raiz.appendChild(texto(
    'Cada estacao da arena precisa de um ponto gravado: e dali que o robo ' +
    'trabalha. Marque tambem o START, onde a prova comeca, e o FINISH, onde ' +
    'ela termina.', 'explicacaoEtapa'));
  raiz.appendChild(texto(
    'O lugar onde voce ligou o mapeamento NAO e o START. Marque o START onde a ' +
    'prova realmente comeca, mesmo que seja do outro lado da arena.',
    'avisoPainel'));

  let origem = 'no_mapa';
  let escolhido = null;          // {x, y, yaw, temDirecao} vindo do mapa
  let nomeAtual = '';
  let tipoAtual = '';
  let alturaAtual = '';

  // --- as DUAS maneiras, lado a lado ---
  const gComo = grupo('Como marcar');
  const troca = segmentado([
    {valor: 'no_mapa', rotulo: 'No mapa',
     dica: 'clique e arraste; o robo nao sai do lugar'},
    {valor: 'robo_aqui', rotulo: 'Levando o robo',
     dica: 'dirija ate la e grave onde ele esta'},
  ], origem, (valor) => {
    origem = valor;
    escolhido = null;
    t.camada.definirProvisorio(null);
    aplicarOrigem();
    atualizar();
  });
  gComo.appendChild(troca.raiz);
  const explicaOrigem = texto('', 'notaPainel');
  gComo.appendChild(explicaOrigem);
  raiz.appendChild(gComo);

  // --- dirigir, para a segunda maneira ---
  const gDirigir = grupo('Dirigir o robo');
  gDirigir.appendChild(texto(
    'Encoste o robo na estacao do jeito que ele vai ficar na prova, e so ' +
    'entao grave. Tambem funciona pelo teclado: W A S D, Q e E, espaco para parar.',
    'notaPainel'));
  gDirigir.appendChild(t.volante.raiz);
  raiz.appendChild(gDirigir);

  // --- nome, tipo, altura ---
  const gPonto = grupo('O ponto');
  const listaNomes = el('select', 'listaOpcoes');
  gPonto.appendChild(campoComRotulo('Nome do ponto', listaNomes));
  const ajudaNome = texto('', 'notaPainel');
  gPonto.appendChild(ajudaNome);

  const listaTipos = el('select', 'listaOpcoes');
  gPonto.appendChild(campoComRotulo('Tipo de estacao', listaTipos));
  const ajudaTipo = texto('', 'notaPainel');
  gPonto.appendChild(ajudaTipo);

  const campoAltura = campoComRotulo('Altura da mesa', el('select', 'listaOpcoes'));
  const listaAlturas = campoAltura.querySelector('select');
  gPonto.appendChild(campoAltura);

  const posicao = texto('', 'estadoPainel');
  gPonto.appendChild(posicao);

  const btGravar = botao('Gravar o ponto', 'acaoPrimaria', gravar);
  gPonto.appendChild(btGravar);
  raiz.appendChild(gPonto);

  // --- o que ja esta marcado ---
  const gLista = grupo('Ja marcados nesta arena');
  const listaPontos = el('div', 'listaPontos');
  gLista.appendChild(listaPontos);
  raiz.appendChild(gLista);

  raiz.appendChild(botao('Proximo passo: paredes virtuais', 'acaoSecundaria',
                         () => t.irParaEtapa('paredes')));

  listaNomes.onchange = () => {
    nomeAtual = listaNomes.value;
    const sugerido = tipoPeloNome(nomeAtual);
    if (sugerido) { tipoAtual = sugerido; listaTipos.value = sugerido; }
    atualizar();
  };
  listaTipos.onchange = () => { tipoAtual = listaTipos.value; atualizar(); };
  listaAlturas.onchange = () => { alturaAtual = listaAlturas.value; };

  function aplicarOrigem() {
    t.camada.definirModo(origem === 'no_mapa' ? 'apontar' : 'ver');
    gDirigir.hidden = origem !== 'robo_aqui';
    explicaOrigem.textContent = origem === 'no_mapa'
      ? 'Clique no mapa onde o robo deve PARAR e arraste para dizer para que ' +
        'lado ele fica virado. Serve para marcar um lugar aonde ainda nao da ' +
        'para levar o robo.'
      : 'Dirija o robo ate o lugar, ajuste a pose com calma e grave onde ele ' +
        'esta. E o unico jeito de acertar uma mesa de encaixe de precisao.';
  }

  function preencherNomes() {
    const anterior = listaNomes.value;
    limpar(listaNomes);
    (t.nomesSugeridos || []).forEach((n) => {
      const item = el('option', '', n.existe
        ? n.id + '  (ja marcado)'
        : n.id + (n.rotulo && n.rotulo !== n.id ? '  (' + n.rotulo + ')' : ''));
      item.value = n.id;
      listaNomes.appendChild(item);
    });
    if (anterior && [...listaNomes.options].some((o) => o.value === anterior)) {
      listaNomes.value = anterior;
    }
    nomeAtual = listaNomes.value || '';
    const sugerido = tipoPeloNome(nomeAtual);
    if (sugerido) { tipoAtual = sugerido; }
  }

  function preencherTipos() {
    const tipos = (t.meta && t.meta.tipos_area) || [];
    limpar(listaTipos);
    tipos.forEach((tipo) => {
      const item = el('option', '', tipo.rotulo);
      item.value = tipo.valor;
      item.title = tipo.ajuda || '';
      listaTipos.appendChild(item);
    });
    if (tipoAtual) { listaTipos.value = tipoAtual; }
    tipoAtual = listaTipos.value || '';
  }

  function preencherAlturas() {
    const alturas = ((t.estadoMapeamento || {}).opcoes || {}).alturas_de_mesa || [];
    limpar(listaAlturas);
    const nenhuma = el('option', '', 'Nao dizer agora');
    nenhuma.value = '';
    listaAlturas.appendChild(nenhuma);
    alturas.forEach((altura) => {
      const item = el('option', '', Math.round(altura * 100) + ' cm');
      item.value = String(altura);
      listaAlturas.appendChild(item);
    });
    listaAlturas.value = alturaAtual || '';
  }

  function preencherLista() {
    limpar(listaPontos);
    const pontos = t.meta ? pontosDaArena(t.meta) : [];
    if (!pontos.length) {
      listaPontos.appendChild(texto('Nenhum ponto marcado ainda.', 'notaPainel'));
      return;
    }
    pontos.forEach((ponto) => {
      const linha = el('div', 'itemPonto');
      const titulo = el('span', 'nomePonto', ponto.id);
      linha.appendChild(titulo);
      linha.appendChild(el('span', 'tipoPonto', ponto.tipo_rotulo || ''));
      if (ponto.sem_pose) {
        linha.appendChild(el('span', 'marcaSemPose', 'sem pose gravada'));
      } else {
        linha.appendChild(el('span', 'posePonto',
          'x ' + metros(ponto.x) + ' m, y ' + metros(ponto.y) + ' m'));
      }
      linha.appendChild(botaoConfirmando(
        'Apagar', 'Confirmar: apagar ' + ponto.id, 'acaoDestrutiva', async () => {
          const resposta = await t.ponte.apagar(
            ROTAS.arenas + '/' + encodeURIComponent(t.arena) + '/ponto/' +
            encodeURIComponent(ponto.id));
          t.ponte.avisar(resposta.mensagem, resposta.ok);
          if (resposta.ok) { await t.recarregarMeta(); }
        }));
      listaPontos.appendChild(linha);
    });
  }

  async function gravar() {
    if (!nomeAtual) { return; }
    const pedido = {id: nomeAtual, tipo: tipoAtual, origem};
    if (origem === 'no_mapa') {
      if (!escolhido) { return; }
      pedido.x = escolhido.x;
      pedido.y = escolhido.y;
      pedido.yaw = escolhido.yaw;
    }
    if (alturaAtual !== '') { pedido.altura = Number(alturaAtual); }
    btGravar.disabled = true;
    const resposta = await t.ponte.enviar(
      ROTAS.arenas + '/' + encodeURIComponent(t.arena) + '/ponto', pedido);
    t.ponte.avisar(resposta.mensagem, resposta.ok);
    (resposta.avisos || []).forEach((aviso) => t.ponte.avisar(aviso, false));
    if (resposta.ok) {
      escolhido = null;
      t.camada.definirProvisorio(null);
      await t.recarregarMeta();
    }
    atualizar();
  }

  function atualizar() {
    const pose = (t.ultimoEstado || {}).pose;
    if (origem === 'no_mapa') {
      posicao.textContent = escolhido
        ? 'Marcado em x ' + metros(escolhido.x) + ' m, y ' + metros(escolhido.y) +
          ' m, virado para ' + (escolhido.temDirecao ? paraOndeAponta(escolhido.yaw)
            : 'nenhum lado (arraste no mapa para dizer a direcao)') + '.'
        : 'Clique no mapa para escolher o lugar.';
      btGravar.textContent = 'Gravar o ponto no lugar marcado';
      btGravar.disabled = !escolhido || !nomeAtual;
    } else if (!pose) {
      posicao.textContent = 'O robo nao esta dizendo onde ele esta. Ligue o ' +
        'mapeamento ou a localizacao antes de gravar.';
      btGravar.textContent = 'Gravar aqui, onde o robo esta';
      btGravar.disabled = true;
    } else {
      posicao.textContent = 'O robo esta em x ' + metros(pose.x) + ' m, y ' +
        metros(pose.y) + ' m, virado para ' + paraOndeAponta(pose.yaw) + '.' +
        (pose.velho ? ' (esta informacao parou de chegar; nao grave assim)' : '');
      btGravar.textContent = 'Gravar aqui, onde o robo esta';
      btGravar.disabled = !nomeAtual || !!pose.velho;
    }

    const tipos = (t.meta && t.meta.tipos_area) || [];
    const achado = tipos.find((tipo) => tipo.valor === tipoAtual);
    ajudaTipo.textContent = achado ? achado.ajuda : '';
    const jaExiste = (t.nomesSugeridos || []).some((n) => n.id === nomeAtual && n.existe);
    ajudaNome.textContent = jaExiste
      ? 'Este ponto ja existe nesta arena. Gravar de novo corrige a pose dele.'
      : 'Nomes valem pelo regulamento: START, FINISH, WS (bancada), SH ' +
        '(prateleira), PP (encaixe de precisao) e RT (mesa giratoria).';
    campoAltura.hidden = (tipoAtual === 'start' || tipoAtual === 'finish' || !tipoAtual);
  }

  function aoTrocarArena() {
    preencherNomes();
    preencherTipos();
    preencherAlturas();
    preencherLista();
    atualizar();
  }

  t.camada.definirAoApontar((ponto) => {
    escolhido = ponto;
    t.camada.definirProvisorio(ponto);
    atualizar();
  });
  aplicarOrigem();
  aoTrocarArena();
  return {raiz, atualizar, aoTrocarArena};
}

// =============================================== ETAPA 4: paredes virtuais

function painelParedes(t) {
  const raiz = el('div', 'conteudoEtapa');
  raiz.appendChild(el('h2', 'tituloEtapa', '4. Paredes virtuais'));
  raiz.appendChild(texto(
    'Parede virtual e um pedaco do chao que o robo passa a tratar como parede, ' +
    'mesmo sem nada aparecer ali no mapa: fita colada no piso, um degrau baixo, ' +
    'uma porta de vidro. O LiDAR nao enxerga nenhuma dessas coisas.',
    'explicacaoEtapa'));

  let ferramenta = 'parede';
  const gPincel = grupo('Pintar');
  const troca = segmentado([
    {valor: 'parede', rotulo: 'Pincel', dica: 'o robo nao entra aqui'},
    {valor: 'livre', rotulo: 'Borracha', dica: 'volta a poder entrar'},
  ], ferramenta, (valor) => {
    ferramenta = valor;
    t.camada.definirPincel({
      valor,
      cor: valor === 'parede' ? t.estilo().keepout : t.estilo().edicao,
    });
  });
  gPincel.appendChild(troca.raiz);
  gPincel.appendChild(deslizantePincel((raio) => t.camada.definirPincel({raio_m: raio})));

  const pendentes = texto('', 'estadoPainel');
  gPincel.appendChild(pendentes);

  const btDesfazer = botao('Desfazer o ultimo traco', 'acaoSecundaria', () => {
    t.camada.desfazer();
    atualizar();
  });
  gPincel.appendChild(btDesfazer);
  const btDescartar = botao('Descartar tudo o que pintei', 'acaoSecundaria', () => {
    t.camada.descartar();
    atualizar();
  });
  gPincel.appendChild(btDescartar);

  const btGravar = botao('Gravar as paredes virtuais', 'acaoPrimaria', async () => {
    const pinceladas = t.camada.pinceladas();
    if (!pinceladas.length) { return; }
    btGravar.disabled = true;
    const resposta = await t.ponte.enviar(ROTAS.paredes, {arena: t.arena, pinceladas});
    t.ponte.avisar(resposta.mensagem, resposta.ok);
    if (resposta.ok) {
      t.camada.descartar();
      bloqueado.textContent = 'Area bloqueada nesta arena: ' +
        (resposta.area_bloqueada_m2 || 0).toFixed(2) + ' m2.';
      await t.recarregarMeta();
    }
    atualizar();
  });
  gPincel.appendChild(btGravar);
  const bloqueado = texto('', 'notaPainel');
  gPincel.appendChild(bloqueado);
  raiz.appendChild(gPincel);

  const gApagar = grupo('Recomecar');
  gApagar.appendChild(botaoConfirmando(
    'Apagar todas as paredes virtuais',
    'Confirmar: apagar TODAS as paredes virtuais',
    'acaoDestrutiva', async () => {
      const resposta = await t.ponte.enviar(ROTAS.paredes, {arena: t.arena, limpar: true});
      t.ponte.avisar(resposta.mensagem, resposta.ok);
      if (resposta.ok) { t.camada.descartar(); await t.recarregarMeta(); }
      atualizar();
    }));
  raiz.appendChild(gApagar);

  raiz.appendChild(botao('Proximo passo: conferir', 'acaoSecundaria',
                         () => t.irParaEtapa('conferir')));

  function atualizar() {
    const quantos = t.camada.quantosTracos();
    pendentes.textContent = quantos
      ? quantos + ' traco(s) pintado(s) e ainda nao gravado(s).'
      : 'Nada pintado ainda. Arraste em cima do mapa, por cima da fita ou do degrau.';
    btDesfazer.disabled = !quantos;
    btDescartar.disabled = !quantos;
    btGravar.disabled = !quantos;
  }

  t.camada.definirModo('pintar');
  t.camada.definirPincel({valor: 'parede', raio_m: 0.10, cor: t.estilo().keepout});
  atualizar();
  return {raiz, atualizar};
}

// ========================================================= ETAPA 5: conferir

function painelConferir(t) {
  const raiz = el('div', 'conteudoEtapa');
  raiz.appendChild(el('h2', 'tituloEtapa', '5. Conferir'));
  raiz.appendChild(texto(
    'Este e o mapa como ele ficou salvo no robo. O que estiver faltando aparece ' +
    'abaixo; toque no aviso para ir direto ao passo que resolve.',
    'explicacaoEtapa'));

  const gResumo = grupo('A arena');
  const resumo = el('div', 'resumoArena');
  gResumo.appendChild(resumo);
  raiz.appendChild(gResumo);

  const gFalta = grupo('O que ainda falta');
  const faltas = el('div', 'listaFaltas');
  gFalta.appendChild(faltas);
  raiz.appendChild(gFalta);

  function linhaResumo(rotulo, valor) {
    const linha = el('div', 'linhaResumo');
    linha.appendChild(el('span', 'rotuloResumo', rotulo));
    linha.appendChild(el('span', 'valorResumo', String(valor)));
    return linha;
  }

  function atualizar() {
    limpar(resumo);
    limpar(faltas);
    const meta = t.meta;
    if (!meta || meta.erro) {
      resumo.appendChild(texto('Nenhum mapa salvo para conferir.', 'notaPainel'));
    } else {
      const pontos = pontosDaArena(meta);
      resumo.appendChild(linhaResumo('Nome', meta.nome));
      resumo.appendChild(linhaResumo('Tamanho', Math.round(
        (meta.largura_px || 0) * (meta.resolucao || 0)) + ' m por ' + Math.round(
        (meta.altura_px || 0) * (meta.resolucao || 0)) + ' m'));
      resumo.appendChild(linhaResumo('Pontos marcados', pontos.length));
      resumo.appendChild(linhaResumo('Sem pose gravada',
                                     pontos.filter((p) => p.sem_pose).length));
      resumo.appendChild(linhaResumo('Pontos de passagem', (meta.waypoints || []).length));
      resumo.appendChild(linhaResumo('Paredes virtuais', meta.tem_keepout ? 'sim' : 'nao'));
    }

    const lista = oQueFalta(meta);
    if (!lista.length) {
      faltas.appendChild(texto('Nada a apontar: esta arena esta completa.',
                               'estadoPainel'));
      return;
    }
    lista.forEach((falta) => {
      const bt = botao(falta.texto, 'itemFalta', () => t.irParaEtapa(falta.etapa));
      faltas.appendChild(bt);
    });
  }

  t.camada.definirModo('ver');
  atualizar();
  return {raiz, atualizar, aoTrocarArena: atualizar};
}

// ================================================================== a tela

// Quando cada etapa pode ser considerada FEITA. So' entra aqui o que da' para
// conferir de verdade no robo -- marcar "feita" por palpite seria pior que nao
// marcar: o operador confiaria e a arena chegaria incompleta na prova.
function etapaFeita(t, id) {
  const meta = t.meta;
  if (id === 'mapear') { return !!(meta && meta.largura_px); }
  if (!meta || meta.erro) { return false; }
  if (id === 'limpar') { return !!t.limpezaFeita; }
  if (id === 'pontos') {
    const pontos = pontosDaArena(meta);
    return pontos.length > 0 && pontos.some((p) => p.id === 'START') &&
      !pontos.some((p) => p.sem_pose);
  }
  if (id === 'paredes') { return !!meta.tem_keepout; }
  if (id === 'conferir') { return oQueFalta(meta).length === 0; }
  return false;
}

// Por que a etapa esta' travada, em texto. Vazio = liberada.
//
// A unica ordem que o robo IMPOE e' "primeiro precisa existir um mapa": os
// pontos e as paredes sao gravados dentro da pasta da arena, que so' nasce no
// salvamento. Editar uma arena que ja' existe nao trava nada -- quem voltou
// para arrumar um dock torto nao vai remapear a sala inteira.
function travaDaEtapa(t, id) {
  if (id === 'mapear') { return ''; }
  if (!t.arena) {
    return 'Ainda nao existe mapa salvo. Faca a etapa 1: ligue o mapeamento, ' +
      'dirija o robo pela arena e salve o mapa com um nome.';
  }
  if ((id === 'limpar' || id === 'paredes') && t.mapeamentoLigado()) {
    return 'O mapeamento ainda esta ligado. Desligue e salve o mapa antes de ' +
      'desenhar em cima dele, senao o desenho se perde no proximo salvamento.';
  }
  return '';
}

export const tela = {
  id: 'mapeamento',
  titulo: 'Mapeamento',
  // A ferramenta traz o proprio mapa: aqui ele nao e' um espectador, e' a mesa
  // de trabalho (recebe clique, arrasto e pincel). Se a casca ja' oferecer um
  // mapa em ctx.mapa, este arquivo usa o dela e nao cria um segundo.
  precisaDoMapa: true,

  montar(raiz, ctx) {
    const ponte = criarPonte(ctx);

    const t = {
      ponte,
      etapaAtual: 'mapear',
      arena: '',
      arenasSalvas: [],
      meta: null,
      nomesSugeridos: [],
      estadoMapeamento: null,
      ultimoEstado: (ctx && ctx.estado) || {},
      limpezaFeita: false,
      avisosDoSalvamento: [],
      gancho: null,
      estilo() { return (t.meta && t.meta.estilo) || ESTILO_RESERVA; },
      mapeamentoLigado() {
        const estado = t.estadoMapeamento || {};
        return !!(estado.ligado || estado.ligado_por_fora);
      },
    };

    // ------------------------------------------------------------ estrutura
    const casca = el('div', 'ferramentaMapeamento');
    const trilho = el('nav', 'trilhoEtapas');
    trilho.setAttribute('aria-label', 'Etapas do mapeamento');
    casca.appendChild(trilho);

    // A casca pode ja' ter um mapa proprio. Se tiver, e' o dela que e' usado --
    // dois mapas na mesma tela fariam o operador desenhar no que nao responde.
    const mapaEmprestado = ctx && ctx.mapa &&
      (ctx.mapa.elemento || ctx.mapa.raiz || ctx.mapa.container);
    let moldura;
    let dicaGesto;
    if (mapaEmprestado) {
      t.mapa = ctx.mapa;
      moldura = mapaEmprestado;
      dicaGesto = el('p', 'dicaGesto');
      moldura.appendChild(dicaGesto);
    } else {
      const area = el('div', 'areaDoMapa');
      moldura = el('div', 'molduraMapa');
      area.appendChild(moldura);
      dicaGesto = el('p', 'dicaGesto');
      area.appendChild(dicaGesto);
      casca.appendChild(area);
      t.mapa = abrirMapa(moldura, {
        estilo: ESTILO_RESERVA,
        camadas: ['keepout', 'grade', 'areas', 'docks', 'waypoints', 'rotulos'],
      });
      if (!t.mapa) {
        moldura.appendChild(texto(
          'Nao consegui desenhar o mapa nesta versao do painel. O resto da ' +
          'ferramenta continua funcionando.', 'avisoPainel'));
      }
    }

    // A varredura ao vivo mora na mesma moldura e cobre o mapa salvo na etapa 1:
    // enquanto o mapeamento roda, o mapa salvo (se existir) e' o ANTIGO, e
    // mostrar os dois juntos faria acreditar que a arena ja' esta' pronta.
    const telaVarredura = el('canvas', 'telaVarredura');
    telaVarredura.style.position = 'absolute';
    telaVarredura.style.left = '0';
    telaVarredura.style.top = '0';
    telaVarredura.style.width = '100%';
    telaVarredura.style.height = '100%';
    telaVarredura.style.pointerEvents = 'none';
    telaVarredura.style.background = t.estilo().fundo || ESTILO_RESERVA.fundo;
    if (getComputedStyle(moldura).position === 'static') {
      moldura.style.position = 'relative';
    }
    moldura.appendChild(telaVarredura);

    const painel = el('aside', 'painelControle');
    const rolagem = el('div', 'painelRolagem');
    painel.appendChild(rolagem);
    casca.appendChild(painel);
    raiz.appendChild(casca);

    t.varredura = criarVarredura(telaVarredura);
    t.camada = criarCamadaDeEdicao(moldura, t.mapa, () => t.estilo());
    t.volante = criarVolante(ponte);

    // ------------------------------------------------------------- trilho
    function montarTrilho() {
      limpar(trilho);
      const cabecalho = el('div', 'cabecalhoTrilho');
      cabecalho.appendChild(el('span', 'rotuloTrilho', 'Arena'));
      cabecalho.appendChild(el('strong', 'arenaTrilho',
                               t.arena || 'nova, ainda sem nome'));
      trilho.appendChild(cabecalho);

      ETAPAS.forEach((etapa) => {
        const motivo = travaDaEtapa(t, etapa.id);
        let estado = '';
        if (etapa.id === t.etapaAtual) { estado = 'atual'; }
        else if (motivo) { estado = 'travada'; }
        else if (etapaFeita(t, etapa.id)) { estado = 'feita'; }

        const bt = el('button', 'etapaTrilho');
        bt.type = 'button';
        if (estado) { bt.dataset.estado = estado; }
        const numero = el('span', 'numeroEtapa', String(etapa.numero));
        if (estado) { numero.dataset.estado = estado; }
        bt.appendChild(numero);
        bt.appendChild(el('span', 'nomeEtapa', etapa.nome));
        bt.disabled = !!motivo;
        bt.onclick = () => irParaEtapa(etapa.id);
        trilho.appendChild(bt);

        // Cinza sem explicacao le-se como "quebrado". O motivo em ambar e' o
        // que transforma uma etapa travada em uma instrucao.
        if (motivo) { trilho.appendChild(el('p', 'motivoEtapa', motivo)); }
      });
    }

    // ------------------------------------------------------------- painel
    const CONSTRUTORES = {
      mapear: painelMapear,
      limpar: painelLimpar,
      pontos: painelPontos,
      paredes: painelParedes,
      conferir: painelConferir,
    };

    const DICAS = {
      mapear: 'O desenho cresce sozinho enquanto o robo anda. Ele ainda nao e o ' +
        'mapa salvo: o mapa so existe depois de voce salvar.',
      limpar: 'Arraste em cima do mapa para apagar. Com dois dedos (ou o botao ' +
        'direito do mouse) da para mover e ampliar.',
      pontos: 'Com dois dedos (ou o botao direito do mouse) da para mover e ' +
        'ampliar o mapa.',
      paredes: 'Arraste para pintar a parede virtual. Com dois dedos (ou o botao ' +
        'direito do mouse) da para mover e ampliar.',
      conferir: 'Com dois dedos (ou o botao direito do mouse) da para mover e ' +
        'ampliar o mapa.',
    };

    function montarPainel() {
      t.volante.parar();
      t.camada.definirModo('ver');
      t.camada.definirProvisorio(null);
      t.camada.descartar();
      limpar(rolagem);
      const construtor = CONSTRUTORES[t.etapaAtual] || painelMapear;
      t.gancho = construtor(t);
      rolagem.appendChild(t.gancho.raiz);
      rolagem.scrollTop = 0;
      telaVarredura.hidden = t.etapaAtual !== 'mapear';
      definirDica(DICAS[t.etapaAtual] || '');
    }

    function definirDica(conteudo) { dicaGesto.textContent = conteudo; }
    t.definirDica = definirDica;

    function irParaEtapa(id) {
      const motivo = travaDaEtapa(t, id);
      if (motivo) { ponte.avisar(motivo, false); return; }
      t.etapaAtual = id;
      montarPainel();
      montarTrilho();
    }
    t.irParaEtapa = irParaEtapa;

    // -------------------------------------------------------------- dados
    async function recarregarArenas() {
      const resposta = await ponte.obter(ROTAS.arenas);
      t.arenasSalvas = (resposta && resposta.arenas) || [];
    }
    t.recarregarArenas = recarregarArenas;

    async function recarregarMeta() {
      if (!t.arena) {
        t.meta = null;
        t.nomesSugeridos = [];
        noMapa(t.mapa, ['limparArena', 'limpar']);
      } else {
        const alvo = ROTAS.arenas + '/' + encodeURIComponent(t.arena);
        t.meta = await ponte.obter(alvo + '/meta');
        if (t.meta && !t.meta.erro) {
          noMapa(t.mapa, ['definirArena', 'carregarArena', 'definirMeta'], t.meta, {
            mapa: ponte.url(alvo + '/mapa.png'),
            keepout: ponte.url(alvo + '/keepout.png'),
          });
        }
        t.nomesSugeridos = await nomesSugeridos(alvo);
      }
      montarTrilho();
      if (t.gancho && t.gancho.aoTrocarArena) { t.gancho.aoTrocarArena(); }
      else if (t.gancho) { t.gancho.atualizar(); }
    }
    t.recarregarMeta = recarregarMeta;

    // A lista de nomes e' do robo. A de reserva so' entra se a rota ainda nao
    // existir -- sem lista nenhuma nao haveria como nomear um ponto, e a etapa
    // 3 viraria um beco sem saida.
    async function nomesSugeridos(alvo) {
      try {
        const resposta = await ponte.obter(alvo + '/nomes');
        const lista = (resposta && (resposta.nomes || resposta.pontos)) ||
          (Array.isArray(resposta) ? resposta : null);
        if (lista && lista.length) { return lista; }
      } catch (erro) { /* cai na lista de reserva */ }
      return nomesSugeridosLocal(t.meta || {});
    }

    async function trocarArena(nome) {
      if (nome === t.arena) { return; }
      t.arena = nome || '';
      t.limpezaFeita = false;
      t.avisosDoSalvamento = [];
      t.camada.descartar();
      t.camada.definirProvisorio(null);
      await recarregarMeta();
      montarPainel();
      montarTrilho();
    }
    t.trocarArena = trocarArena;

    async function recarregarMapeamento() {
      t.estadoMapeamento = await ponte.obter(ROTAS.estado);
      // Se o mapeamento subiu enquanto o operador estava desenhando, a etapa
      // deixa de valer: melhor tirar ele de la' dizendo por que.
      const motivo = travaDaEtapa(t, t.etapaAtual);
      if (motivo) {
        ponte.avisar(motivo, false);
        t.etapaAtual = 'mapear';
        montarPainel();
      }
      montarTrilho();
      if (t.gancho) { t.gancho.atualizar(); }
    }
    t.recarregarMapeamento = recarregarMapeamento;

    // ------------------------------------------------------------- relogios
    // O estado do mapeamento nao vem pelo fluxo do robo (ele e' do painel, nao
    // do robo), entao e' perguntado de tempos em tempos.
    const relogioMapeamento = setInterval(() => {
      recarregarMapeamento().catch(() => {});
    }, 2000);

    // Tique curto so' para os numeros que mudam enquanto o dedo esta na tela
    // (tracos pendentes, area varrida). Nao remonta nada.
    const relogioTela = setInterval(() => {
      if (t.gancho) { t.gancho.atualizar(); }
    }, 500);

    let animacao = null;
    function laco() {
      if (t.etapaAtual === 'mapear') { t.varredura.desenhar(); }
      animacao = requestAnimationFrame(laco);
    }
    animacao = requestAnimationFrame(laco);

    // ------------------------------------------------------------- arranque
    montarPainel();
    montarTrilho();
    (async () => {
      await recarregarArenas();
      t.estadoMapeamento = await ponte.obter(ROTAS.estado);
      const ativa = (t.estadoMapeamento && t.estadoMapeamento.arena_ativa) || '';
      // Abrir ja' na arena em que o robo esta' e' o caso comum: quase todo
      // mapeamento que nao e' o primeiro comeca com "preciso arrumar uma coisa
      // nesta arena aqui".
      if (ativa && t.arenasSalvas.includes(ativa)) {
        t.arena = ativa;
        await recarregarMeta();
      }
      montarPainel();
      montarTrilho();
    })().catch(() => {});

    return {
      aoEstado(estado) {
        t.ultimoEstado = estado || {};
        if (t.etapaAtual === 'mapear') { t.varredura.alimentar(t.ultimoEstado); }
        noMapa(t.mapa, ['definirEstado', 'atualizarEstado'], t.ultimoEstado);
        if (t.gancho && t.etapaAtual === 'pontos') { t.gancho.atualizar(); }
      },

      destruir() {
        clearInterval(relogioMapeamento);
        clearInterval(relogioTela);
        cancelAnimationFrame(animacao);
        t.volante.destruir();
        t.camada.destruir();
        telaVarredura.remove();
        if (!mapaEmprestado) { noMapa(t.mapa, ['destruir', 'fechar']); }
        else { dicaGesto.remove(); }
        casca.remove();
      },
    };
  },
};
