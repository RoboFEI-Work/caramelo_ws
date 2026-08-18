// O desenho do mapa do painel, em canvas puro. Sem biblioteca, sem build.
//
// Este arquivo e' o GEMEO de caramelo_gui/src/widgets/map_preview.cpp. O
// operador pediu que a web siga a GUI: se as duas telas desenham a MESMA arena
// de jeitos diferentes, uma delas esta mentindo, e o erro so' aparece com o
// robo andando. Por isso a conta de coordenadas, a ordem das camadas, as cores,
// o raio do marcador e ate' o gesto do "2D Goal" sao os mesmos de la'.
//
// O que a web tem a mais que a GUI: o robo AO VIVO por cima do mapa (pose,
// LiDAR, caminho planejado) e o pincel de paredes virtuais no mesmo canvas.
//
// De onde vem cada coisa:
//   desenho da arena    /api/arenas/<nome>/meta  (arenas.py, gemeo do parser da GUI)
//   imagem do mapa      /api/arenas/<nome>/mapa.png
//   paredes virtuais    /api/arenas/<nome>/keepout.png  (mascara ja' tingida)
//   robo ao vivo        /ws/estado  (pose 10 Hz, LiDAR 5 Hz)
//
// Nao ha' laco de animacao: cada mudanca agenda UM quadro. O painel roda no
// mesmo computador da navegacao e costuma ficar aberto por horas.

export const MODOS = Object.freeze({
  // So' olhar: arrastar com qualquer botao move o mapa. E' o modo das telas de
  // consulta, onde um toque errado nao pode virar comando de movimento.
  OLHAR: 'olhar',
  // Clicar escolhe destino; arrastar escolhe tambem para que lado o robo fica
  // virado ao chegar.
  NAVEGAR: 'navegar',
  // Mesmo gesto, outro significado: "o robo esta aqui".
  POSICIONAR: 'posicionar',
  // Pegar um ponto salvo: arrastar o circulo move, arrastar a ponta da seta
  // gira, botao direito (ou dedo parado) abre as acoes do ponto.
  EDITAR: 'editar',
  // Pincel e borracha da mascara de keepout.
  PAREDES: 'paredes',
});

// Tokens do tema (caramelo_gui/resources/tema.qss). Nao e' uma segunda lista de
// estilo: e' a mesma origem de onde arenas.py copiou o ESTILO_DO_MAPA. Quando
// os metadados chegam, quem manda e' meta.estilo -- estes valores so' seguram o
// desenho nos milissegundos antes disso e quando nenhuma arena carregou.
const PALETA = Object.freeze({
  fundo: '#0b1626',
  fraco: '#7d9cc4',
  ciano: '#35c3f0',
  laranja: '#ff9a2e',
  verde: '#27ae60',
  ambar: '#f2994a',
  vermelho: '#eb5757',
  roxo: '#9b51e0',
  amarelo: '#f2c94c',
  azulClaro: '#56ccf2',
  tinta: '#06121f',
});

const ESTILO_RESERVA = Object.freeze({
  fundo: PALETA.fundo,
  grade: 'rgba(255,255,255,0.15)',
  grade_passo_m: 1.0,
  grade_passo_minimo_px: 12.0,
  keepout: 'rgba(235,87,87,0.43)',
  sem_pose: PALETA.vermelho,
  dock: PALETA.ambar,
  dock_inicio: PALETA.verde,
  dock_fim: PALETA.roxo,
  area: PALETA.azulClaro,
  area_preenchimento: 'rgba(86,204,242,0.18)',
  area_lado_px: 22.0,
  waypoint: PALETA.ciano,
  edicao: PALETA.amarelo,
  robo: PALETA.laranja,
  raio_marcador_px: 7.0,
  comprimento_seta_px: 22.0,
  tolerancia_toque_px: 12.0,
  rotulo_sem_pose: ' (sem pose)',
});

// Cores do que e' AO VIVO. Nao vem dos metadados de proposito: arenas.py
// descreve a arena salva, e nada disto esta salvo em arena nenhuma.
const ESTILO_AO_VIVO = Object.freeze({
  lidar: 'rgba(53,195,240,0.85)',
  cego: 'rgba(235,87,87,0.18)',
  cego_texto: '#ff9d9d',
  // Dado velho aparece apagado: uma tela congelada nao pode parecer um robo
  // parado no lugar certo.
  velho: '#7f93b0',
  // Verde e nao ciano: o caminho passa por cima dos pontos do LiDAR, e duas
  // coisas cianas empilhadas viram uma mancha so'.
  caminho: PALETA.verde,
});

// Camadas que nao vem do arquivo da arena (o servidor so' lista as que estao
// salvas no disco). Ficam aqui porque sao do painel, nao da arena.
const CAMADAS_AO_VIVO = Object.freeze([
  { id: 'robo', rotulo: 'Robô' },
  { id: 'lidar', rotulo: 'LiDAR ao vivo' },
  { id: 'caminho', rotulo: 'Caminho planejado' },
]);

const CAMADAS_RESERVA = Object.freeze([
  { id: 'keepout', rotulo: 'Paredes virtuais' },
  { id: 'grade', rotulo: 'Grade (1 m)' },
  { id: 'areas', rotulo: 'Service areas' },
  { id: 'docks', rotulo: 'Docks' },
  { id: 'waypoints', rotulo: 'Waypoints' },
  { id: 'rotulos', rotulo: 'Nomes' },
]);

// Tolerancia do toque, em pixels. Dedo em tela de robo nao acerta 7 px.
const TOLERANCIA_SETA_PX = 10;
// Arrasto curto e' clique: sem esta folga o tremor do dedo num tablet viraria
// uma direcao aleatoria gravada como pose.
const LIMIAR_DIRECAO_PX = 12;
// Para ponto ja' salvo o limiar e' menor: mover 4 px ja' e' intencao de mover.
const LIMIAR_ARRASTO_PX = 4;
// Dedo parado sobre um ponto abre as acoes dele. Tablet nao tem botao direito,
// e sem isto metade dos comandos de um ponto seria inalcancavel no tablet.
const ESPERA_ACOES_MS = 550;

const ZOOM_MIN = 0.05;
const ZOOM_MAX = 60;
const PASSO_DA_RODA = 1.12;

// O servidor recusa mais que isso de uma vez (mapeamento.py). Avisar antes e'
// melhor que deixar o operador desenhar dez minutos e perder tudo no envio.
const MAX_PINCELADAS = 5000;

// Footprint real do Caramelo, em metros (bridge/manual_localization.cpp). Ver o
// retangulo em escala e' o que permite julgar se o robo passa no corredor;
// um triangulo de tamanho fixo nao responde essa pergunta.
const COMPRIMENTO_ROBO_M = 0.71;
const LARGURA_ROBO_M = 0.47;

// O setor cego e' infinito no chao; desenhar 30 m de leque cobriria a arena
// inteira. O que interessa e' a direcao e o pedaco onde o robo bate primeiro.
const ALCANCE_DO_CEGO_M = 3.0;

// ---------------------------------------------------------------------------
// O LiDAR do Caramelo esta' montado DE CABECA PARA BAIXO e virado.
//
// No URDF (base_tophat.xacro) ele vem com rpy = (pi, 0, pi), que da' a matriz
// diag(-1, +1, -1). No plano XY isso NAO e' uma rotacao: e' um ESPELHO
// (x troca de sinal, y nao). Em angulo:
//
//     angulo_no_robo = pi - angulo_no_laser
//
// Antes daqui o desenho somava o angulo do laser direto no yaw do robo, como
// se os dois frames fossem o mesmo. Como espelho nao e' rotacao, o erro nao
// era um deslocamento constante que ninguem notaria: uma parede 2 m A FRENTE
// aparecia 2 m ATRAS no mapa. E o setor cego era pintado do lado errado.
//
// Conferido nos numeros do robo (scan_normalizer.yaml recorta o /scan em
// [1.5708, 4.7124] rad no frame do laser):
//     laser  90 graus -> robo  90 graus (esquerda)
//     laser 180 graus -> robo   0 graus (FRENTE)
//     laser 270 graus -> robo -90 graus (direita)
// Ou seja: o Caramelo ENXERGA A FRENTE. Quem nao tem leitura e' a TRASEIRA.
const anguloNoRobo = (anguloNoLaser) => Math.PI - anguloNoLaser;

// O sensor tambem nao fica no centro do robo: +0.245 m para a frente. Sem
// somar isso, todo ponto sai deslocado um quarto de metro.
const LIDAR_X_NO_ROBO = 0.245;

const FONTE = '"Noto Sans", system-ui, sans-serif';

function distancia(ax, ay, bx, by) {
  return Math.hypot(ax - bx, ay - by);
}

// Clareia mantendo matiz e saturacao (o mesmo que QColor::lighter faz com o V
// do HSV). Usado so' no texto dos rotulos, que precisa saltar do fundo escuro.
function clarear(cor, fator) {
  const m = /^#([0-9a-f]{6})$/i.exec(String(cor || '').trim());
  if (!m) return cor;
  const n = parseInt(m[1], 16);
  const canal = (v) => Math.min(255, Math.round(v * fator));
  return `rgb(${canal((n >> 16) & 255)},${canal((n >> 8) & 255)},${canal(n & 255)})`;
}

function carregarImagem(url) {
  return new Promise((resolve) => {
    const img = new Image();
    img.onload = () => resolve(img);
    img.onerror = () => resolve(null);
    img.src = url;
  });
}

/**
 * Cria o mapa dentro de `hospedeiro` (qualquer elemento com tamanho proprio).
 *
 * opcoes:
 *   aoEscolherPose({x, y, yaw, arrastou, modo})  gesto do "2D Goal" terminado
 *   aoClicarMarcador({tipo, id, pose})           tocou num ponto ja' salvo
 *   aoArrastarMarcador({tipo, id}, {x, y, yaw})  moveu ou girou um ponto salvo
 *   aoAbrirAcoes({tipo, id, pose}, {x, y})       botao direito / dedo parado
 *   aoPintar({segmentos, grupos})                mudou o desenho das paredes
 *   aoMudarVista({escala})                       zoom/pan mudou
 *   aoAviso(texto)                               frase pronta para o operador
 *   urlMeta/urlMapa/urlParedes(nome)             para trocar as rotas nos testes
 *
 * Devolve o controlador que as telas usam. Nenhum metodo grava arquivo: o mapa
 * so' avisa, quem decide se salva e' a tela (mesma regra do MapPreview da GUI).
 */
export function criarMapaCanvas(hospedeiro, opcoes = {}) {
  const urlMeta = opcoes.urlMeta || ((n) => `/api/arenas/${encodeURIComponent(n)}/meta`);
  const urlMapa = opcoes.urlMapa || ((n) => `/api/arenas/${encodeURIComponent(n)}/mapa.png`);
  const urlParedes = opcoes.urlParedes ||
    ((n) => `/api/arenas/${encodeURIComponent(n)}/keepout.png`);

  const cv = document.createElement('canvas');
  cv.className = 'mapaCanvas';
  cv.style.display = 'block';
  cv.style.width = '100%';
  cv.style.height = '100%';
  // touch-action none: sem isto o navegador engole o gesto de dois dedos e o
  // tablet dava zoom na PAGINA enquanto o mapa ficava parado.
  cv.style.touchAction = 'none';
  hospedeiro.appendChild(cv);
  const pincel2d = cv.getContext('2d');

  let meta = null;
  let arena = '';
  let serieDeCarga = 0;          // descarta imagem de arena que o operador ja' trocou
  let imagemMapa = null;
  let imagemParedes = null;
  let mensagem = 'Escolha uma arena';

  let estado = {};               // ultimo quadro do /ws/estado
  let scanVivo = null;           // ultimo LiDAR recebido
  let scanChegouEm = 0;
  let caminho = [];              // caminho planejado, em metros

  const vista = { escala: 1, dx: 0, dy: 0, enquadrado: false };
  let ultimoTamanho = { larg: 0, alt: 0 };
  const camadas = new Map();
  for (const c of CAMADAS_RESERVA) camadas.set(c.id, true);
  for (const c of CAMADAS_AO_VIVO) camadas.set(c.id, true);

  let modo = MODOS.OLHAR;
  let destaque = { tipo: '', id: '' };
  let poseProvisoria = null;     // {x, y, yaw} desenhada durante e apos o gesto
  let gesto = null;              // gesto do botao esquerdo em andamento
  let pan = null;
  let cursorNoMapa = null;       // ultima posicao do ponteiro, em pixels

  // --- paredes virtuais ---
  let pincel = { valor: 'parede', raio_m: 0.10 };
  let grupos = [];               // uma pincelada por vez que o operador desceu o dedo
  let tracoAtual = null;
  let camadaParedes = null;      // composicao (mascara salva + o que ainda nao foi)
  let rascunhoParedes = null;    // a mesma composicao mais o traco em andamento

  const dedos = new Map();
  let doisDedos = null;
  let cronometroAcoes = 0;

  let quadroPedido = 0;
  let vivo = true;

  // ------------------------------------------------------------- utilidades
  function estilo() {
    return Object.assign({}, ESTILO_RESERVA, (meta && meta.estilo) || {});
  }

  function chamar(nome, ...args) {
    const f = opcoes[nome];
    if (typeof f === 'function') f(...args);
  }

  function avisar(texto) {
    chamar('aoAviso', texto);
  }

  function temMapa() {
    return !!(meta && imagemMapa && meta.largura_px && meta.altura_px);
  }

  function agendarDesenho() {
    if (!vivo || quadroPedido) return;
    quadroPedido = requestAnimationFrame(() => {
      quadroPedido = 0;
      pintar();
    });
  }

  // map.yaml: origin e' a pose do pixel INFERIOR esquerdo da imagem, e a imagem
  // cresce para BAIXO. Errar este sinal espelha todos os marcadores do mapa --
  // e marcador espelhado e' o robo indo para o lado oposto da bancada.
  function paraTela(x, y) {
    if (!meta) return [0, 0];
    const px = (x - meta.origem[0]) / meta.resolucao;
    const py = meta.altura_px - (y - meta.origem[1]) / meta.resolucao;
    return [vista.dx + px * vista.escala, vista.dy + py * vista.escala];
  }

  // Inversa exata de paraTela. Sem ela nao existe clique-no-mapa, so' desenho.
  function paraMapa(cx, cy) {
    if (!meta || vista.escala <= 0) return [0, 0];
    const px = (cx - vista.dx) / vista.escala;
    const py = (cy - vista.dy) / vista.escala;
    return [px * meta.resolucao + meta.origem[0],
            (meta.altura_px - py) * meta.resolucao + meta.origem[1]];
  }

  // Angulo do arrasto com o Y da tela ja' invertido (a tela cresce para baixo,
  // o mapa cresce para cima). Errar isto grava todo yaw espelhado.
  function yawDoArrasto(x0, y0, x1, y1) {
    return Math.atan2(-(y1 - y0), x1 - x0);
  }

  function local(ev) {
    const r = cv.getBoundingClientRect();
    return [ev.clientX - r.left, ev.clientY - r.top];
  }

  function larguraVisivel() { return cv.clientWidth || 0; }
  function alturaVisivel() { return cv.clientHeight || 0; }

  // ---------------------------------------------------------- tamanho/vista
  function ajustarTamanho() {
    const larg = larguraVisivel();
    const alt = alturaVisivel();
    if (!larg || !alt) return;
    // devicePixelRatio: sem isto o mapa fica borrado em tablet e em notebook de
    // tela densa. O contexto passa a trabalhar em pixels CSS, entao toda a
    // matematica de coordenadas acima continua valendo sem mudanca.
    const dpr = window.devicePixelRatio || 1;
    const w = Math.round(larg * dpr);
    const h = Math.round(alt * dpr);
    if (cv.width !== w || cv.height !== h) {
      cv.width = w;
      cv.height = h;
    }
    pincel2d.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function enquadrar() {
    ajustarTamanho();
    if (!temMapa()) return;
    const larg = larguraVisivel();
    const alt = alturaVisivel();
    if (!larg || !alt) return;
    vista.escala = Math.min(larg / meta.largura_px, alt / meta.altura_px) * 0.95;
    vista.dx = (larg - meta.largura_px * vista.escala) / 2;
    vista.dy = (alt - meta.altura_px * vista.escala) / 2;
    vista.enquadrado = true;
    chamar('aoMudarVista', { escala: vista.escala });
    agendarDesenho();
  }

  function ampliar(fator, cx, cy) {
    const larg = larguraVisivel();
    const alt = alturaVisivel();
    const ax = (cx === undefined) ? larg / 2 : cx;
    const ay = (cy === undefined) ? alt / 2 : cy;
    const nova = Math.max(ZOOM_MIN, Math.min(ZOOM_MAX, vista.escala * fator));
    const real = nova / vista.escala;
    // Zoom ancorado no ponteiro: sem isto o mapa foge da tela ao ampliar.
    vista.dx = ax - (ax - vista.dx) * real;
    vista.dy = ay - (ay - vista.dy) * real;
    vista.escala = nova;
    chamar('aoMudarVista', { escala: vista.escala });
    agendarDesenho();
  }

  function aoRedimensionar() {
    const larg = larguraVisivel();
    const alt = alturaVisivel();
    if (!larg || !alt) return;
    if (!temMapa() || !vista.enquadrado) {
      ajustarTamanho();
      enquadrar();
      ultimoTamanho = { larg, alt };
      return;
    }
    // O ponto que estava no meio continua no meio. Sem isto, girar o tablet ou
    // abrir um painel lateral jogava fora o zoom que o operador acabara de
    // fazer -- e ele tinha que reenquadrar no meio da prova.
    const meio = ultimoTamanho.larg
      ? paraMapa(ultimoTamanho.larg / 2, ultimoTamanho.alt / 2) : null;
    ajustarTamanho();
    if (meio) {
      const [mx, my] = paraTela(meio[0], meio[1]);
      vista.dx += larg / 2 - mx;
      vista.dy += alt / 2 - my;
    }
    ultimoTamanho = { larg, alt };
    agendarDesenho();
  }

  // ------------------------------------------------------------- marcadores
  function candidatos() {
    // Na MESMA ordem em que sao pintados (area, dock, waypoint): assim o ultimo
    // da lista e' o que aparece por cima, e o desempate do toque segue o que o
    // operador esta vendo. So' entra camada ligada -- pegar um ponto invisivel
    // pareceria que o mapa mexeu sozinho em algo que ninguem ve.
    const lista = [];
    if (!meta) return lista;
    if (camadas.get('areas')) {
      for (const a of meta.areas || []) lista.push({ tipo: 'area', id: a.id, pose: a });
    }
    if (camadas.get('docks')) {
      for (const d of meta.docks || []) lista.push({ tipo: 'dock', id: d.id, pose: d });
    }
    if (camadas.get('waypoints')) {
      for (const w of meta.waypoints || []) lista.push({ tipo: 'waypoint', id: w.id, pose: w });
    }
    return lista;
  }

  function temSeta(c) {
    const destacado = (c.tipo === destaque.tipo && c.id === destaque.id);
    return destacado || (!c.pose.sem_pose && c.tipo !== 'area');
  }

  function buscarMarcador(cx, cy, folga) {
    const est = estilo();
    const escalaToque = folga || 1;
    let achado = null;
    let naSeta = false;

    // 1) corpo do marcador (mover). Vence a seta: o circulo e' o alvo obvio.
    // Empate: ganha o ultimo da lista, que e' o desenhado por cima. Acontece
    // com varios pontos ainda em [0,0,0], todos empilhados na origem do mapa --
    // o operador tira um de la' por vez.
    let melhor = (est.tolerancia_toque_px || 12) * escalaToque;
    for (const c of candidatos()) {
      const [px, py] = paraTela(c.pose.x, c.pose.y);
      const d = distancia(cx, cy, px, py);
      if (d <= melhor) { melhor = d; achado = c; }
    }
    if (achado) return { marcador: achado, naSeta: false };

    // 2) ponta da seta (girar) -- so' das setas que estao desenhadas.
    melhor = TOLERANCIA_SETA_PX * escalaToque;
    const comprimento = est.comprimento_seta_px || 22;
    for (const c of candidatos()) {
      if (!temSeta(c)) continue;
      const [px, py] = paraTela(c.pose.x, c.pose.y);
      const d = distancia(cx, cy,
        px + Math.cos(c.pose.yaw) * comprimento,
        py - Math.sin(c.pose.yaw) * comprimento);
      if (d <= melhor) { melhor = d; achado = c; naSeta = true; }
    }
    return achado ? { marcador: achado, naSeta } : null;
  }

  function poseDoMarcador(tipo, id) {
    if (!meta || !id) return null;
    const fonte = tipo === 'dock' ? meta.docks
      : tipo === 'area' ? meta.areas
      : tipo === 'waypoint' ? meta.waypoints : null;
    if (!fonte) return null;
    return fonte.find((p) => p.id === id) || null;
  }

  // ---------------------------------------------------------------- desenho
  function textoComFundo(texto, x, y, cor, centralizado) {
    if (!texto) return;
    pincel2d.font = `bold 12px ${FONTE}`;
    pincel2d.textBaseline = 'middle';
    pincel2d.textAlign = 'left';
    const larg = pincel2d.measureText(texto).width;
    const alvo = centralizado ? x - larg / 2 : x;
    // Preso dentro do canvas: o operador reclamou de coisa cortada na borda, e
    // um nome pela metade e' o mesmo que nome nenhum.
    const px = Math.max(4, Math.min(alvo, larguraVisivel() - larg - 10));
    const py = Math.max(12, Math.min(y, alturaVisivel() - 12));
    pincel2d.fillStyle = 'rgba(6,18,31,0.80)';
    pincel2d.beginPath();
    if (pincel2d.roundRect) {
      pincel2d.roundRect(px - 5, py - 10, larg + 10, 20, 4);
    } else {
      // Navegador antigo nao tem roundRect. Em competicao nao ha' como
      // atualizar navegador: canto reto e' melhor que rotulo nenhum.
      pincel2d.rect(px - 5, py - 10, larg + 10, 20);
    }
    pincel2d.fill();
    pincel2d.fillStyle = cor;
    pincel2d.fillText(texto, px, py);
  }

  function desenharSeta(cx, cy, yaw, comprimento, cor, grossura) {
    // O eixo Y da tela aponta para baixo: o seno entra negativo.
    const dx = Math.cos(yaw);
    const dy = -Math.sin(yaw);
    const px = cx + dx * comprimento;
    const py = cy + dy * comprimento;
    pincel2d.beginPath();
    pincel2d.moveTo(cx, cy);
    pincel2d.lineTo(px, py);
    pincel2d.strokeStyle = cor;
    pincel2d.lineWidth = grossura || 3;
    pincel2d.stroke();
    const lx = -dy;
    const ly = dx;
    pincel2d.beginPath();
    pincel2d.moveTo(px, py);
    pincel2d.lineTo(px - dx * 9 + lx * 5, py - dy * 9 + ly * 5);
    pincel2d.lineTo(px - dx * 9 - lx * 5, py - dy * 9 - ly * 5);
    pincel2d.closePath();
    pincel2d.fillStyle = cor;
    pincel2d.fill();
  }

  function desenharMarcador(pose, cor, rotulo, comSeta) {
    const est = estilo();
    // Pose nao gravada em vermelho: e' a diferenca entre uma arena pronta e uma
    // que vai abortar a missao no primeiro deslocamento.
    const usada = pose.sem_pose ? est.sem_pose : cor;
    const [cx, cy] = paraTela(pose.x, pose.y);
    const raio = est.raio_marcador_px || 7;

    if (comSeta && !pose.sem_pose) {
      const comprimento = est.comprimento_seta_px || 22;
      pincel2d.beginPath();
      pincel2d.moveTo(cx, cy);
      pincel2d.lineTo(cx + Math.cos(pose.yaw) * comprimento,
                      cy - Math.sin(pose.yaw) * comprimento);
      pincel2d.strokeStyle = usada;
      pincel2d.lineWidth = 3;
      pincel2d.stroke();
    }

    pincel2d.beginPath();
    pincel2d.arc(cx, cy, raio, 0, Math.PI * 2);
    pincel2d.fillStyle = usada;
    pincel2d.fill();
    pincel2d.strokeStyle = PALETA.tinta;
    pincel2d.lineWidth = 2;
    pincel2d.stroke();

    if (camadas.get('rotulos') && rotulo) {
      const texto = pose.sem_pose ? rotulo + (est.rotulo_sem_pose || ' (sem pose)') : rotulo;
      textoComFundo(texto, cx + 12, cy - 14, clarear(usada, 1.4));
    }
  }

  function desenharGrade() {
    const est = estilo();
    const passoM = est.grade_passo_m || 1.0;
    const passo = (passoM / meta.resolucao) * vista.escala;
    if (passo < (est.grade_passo_minimo_px || 12)) {
      return;   // grade mais densa que isso vira ruido
    }
    const largura = meta.largura_px * vista.escala;
    const altura = meta.altura_px * vista.escala;
    pincel2d.strokeStyle = est.grade;
    pincel2d.lineWidth = 1;
    pincel2d.beginPath();
    for (let x = vista.dx; x <= vista.dx + largura; x += passo) {
      pincel2d.moveTo(x, vista.dy);
      pincel2d.lineTo(x, vista.dy + altura);
    }
    for (let y = vista.dy; y <= vista.dy + altura; y += passo) {
      pincel2d.moveTo(vista.dx, y);
      pincel2d.lineTo(vista.dx + largura, y);
    }
    pincel2d.stroke();
  }

  function retanguloDaMascara() {
    // A mascara costuma nascer com a geometria do mapa, mas ler a dela evita um
    // desenho deslocado quando alguem reeditar so' um dos dois arquivos.
    const geo = (meta && meta.keepout) || null;
    const res = geo ? geo.resolucao : meta.resolucao;
    const ox = geo ? geo.origem[0] : meta.origem[0];
    const oy = geo ? geo.origem[1] : meta.origem[1];
    const [x, y] = paraTela(ox, oy + meta.altura_px * res);
    return [x, y,
            meta.largura_px * (res / meta.resolucao) * vista.escala,
            meta.altura_px * (res / meta.resolucao) * vista.escala];
  }

  function desenharParedes() {
    if (!camadaParedes) return;
    let camada = camadaParedes;
    if (tracoAtual) {
      // O traco em andamento e' composto FORA da tela, junto com a mascara.
      // Pintar a borracha direto no canvas principal abria buraco no mapa e no
      // fundo: aparecia a pagina atras do desenho.
      camada = camadaDoRascunho();
    }
    const [x, y, w, h] = retanguloDaMascara();
    pincel2d.drawImage(camada, x, y, w, h);
  }

  function camadaDoRascunho() {
    if (!rascunhoParedes) rascunhoParedes = document.createElement('canvas');
    if (rascunhoParedes.width !== camadaParedes.width ||
        rascunhoParedes.height !== camadaParedes.height) {
      rascunhoParedes.width = camadaParedes.width;
      rascunhoParedes.height = camadaParedes.height;
    }
    const c = rascunhoParedes.getContext('2d');
    c.clearRect(0, 0, rascunhoParedes.width, rascunhoParedes.height);
    c.drawImage(camadaParedes, 0, 0);
    aplicarGrupoNaCamada(c, tracoAtual);
    return rascunhoParedes;
  }

  function desenharAreas() {
    const est = estilo();
    const lado = est.area_lado_px || 22;
    for (const a of meta.areas || []) {
      const [cx, cy] = paraTela(a.x, a.y);
      const cor = a.sem_pose ? est.sem_pose : est.area;
      pincel2d.fillStyle = est.area_preenchimento;
      pincel2d.fillRect(cx - lado / 2, cy - lado / 2, lado, lado);
      pincel2d.strokeStyle = cor;
      pincel2d.lineWidth = 2;
      pincel2d.strokeRect(cx - lado / 2, cy - lado / 2, lado, lado);
      // A GUI nao rotula service area; aqui rotula porque na web o mesmo
      // quadrado e' o alvo do toque, e um quadrado sem nome nao diz em qual
      // bancada o operador esta mexendo.
      if (camadas.get('rotulos')) {
        const texto = a.id + (a.sem_pose ? (est.rotulo_sem_pose || ' (sem pose)') : '');
        textoComFundo(texto, cx + lado / 2 + 6, cy - 12, clarear(cor, 1.4));
      }
    }
  }

  function desenharDocks() {
    const est = estilo();
    for (const d of meta.docks || []) {
      // START e FINISH sao o comeco e o fim da prova: cor propria para o
      // operador achar o ponto de inicializacao de relance.
      const cor = d.papel === 'inicio' ? est.dock_inicio
        : d.papel === 'fim' ? est.dock_fim : est.dock;
      const rotulo = d.papel === 'inicio' ? `${d.id} (início)` : d.id;
      desenharMarcador(d, cor, rotulo, true);
    }
  }

  function desenharWaypoints() {
    const est = estilo();
    for (const w of meta.waypoints || []) desenharMarcador(w, est.waypoint, w.id, true);
  }

  function desenharCaminho() {
    if (caminho.length < 2) return;
    pincel2d.beginPath();
    caminho.forEach((p, i) => {
      const [x, y] = paraTela(p[0], p[1]);
      if (i === 0) pincel2d.moveTo(x, y); else pincel2d.lineTo(x, y);
    });
    pincel2d.strokeStyle = ESTILO_AO_VIVO.caminho;
    pincel2d.lineWidth = 3;
    pincel2d.lineJoin = 'round';
    pincel2d.lineCap = 'round';
    pincel2d.stroke();
    const fim = caminho[caminho.length - 1];
    const [fx, fy] = paraTela(fim[0], fim[1]);
    pincel2d.beginPath();
    pincel2d.arc(fx, fy, 5, 0, Math.PI * 2);
    pincel2d.fillStyle = ESTILO_AO_VIVO.caminho;
    pincel2d.fill();
  }

  function scanUtil() {
    if (!scanVivo || !scanVivo.ranges) return null;
    // Segurar o ultimo LiDAR por 2 s: o quadro vem a 10 Hz e o LiDAR so' em
    // metade deles, e sem isto os pontos piscavam na tela. Passou disso, some:
    // ponto velho desenhado como se fosse de agora e' obstaculo fantasma.
    if ((Date.now() - scanChegouEm) > 2000) return null;
    return scanVivo;
  }

  function desenharLidar(pose) {
    const scan = scanUtil();
    if (!scan) return;
    const metroEmPx = vista.escala / meta.resolucao;

    // A TRASEIRA do Caramelo nao tem visao: o LiDAR e' recortado nos 180 graus
    // da frente. Desenhar so' os pontos daria a impressao de cobertura total,
    // que e' exatamente o engano que provoca colisao ao dar re'.
    const abertura = scan.angle_max - scan.angle_min;
    if (abertura > 0 && abertura < 2 * Math.PI - 0.05) {
      const [rx, ry] = paraTela(pose.x, pose.y);
      const alcance = Math.min(scan.range_max || ALCANCE_DO_CEGO_M, ALCANCE_DO_CEGO_M);
      // Piso em pixels: de longe o setor ficaria menor que o icone do robo e o
      // aviso simplesmente sumiria da tela.
      const raio = Math.max(48, alcance * metroEmPx);
      // O espelho INVERTE a ordem dos extremos: o angulo_max do laser vira o
      // MENOR angulo no robo. Trocar os dois e' o que faz o setor cego cair
      // atras do robo, que e' onde ele fica de verdade.
      const inicio = pose.yaw + anguloNoRobo(scan.angle_min);
      const fim = pose.yaw + anguloNoRobo(scan.angle_max) + 2 * Math.PI;
      pincel2d.beginPath();
      pincel2d.moveTo(rx, ry);
      // O canvas mede angulo no sentido horario (Y cresce para baixo); por isso
      // o sinal trocado e o sentido anti-horario ligado.
      pincel2d.arc(rx, ry, raio, -inicio, -fim, true);
      pincel2d.closePath();
      pincel2d.fillStyle = ESTILO_AO_VIVO.cego;
      pincel2d.fill();

      // O rotulo segue o DADO, nao a crenca de quem escreveu a tela: se um dia
      // o recorte do LiDAR mudar de lado, a frase para de dizer "frente"
      // sozinha, em vez de continuar mentindo por cima do mapa.
      const meio = (inicio + fim) / 2;
      const dentroDoCego = (alvo) => {
        const d = ((alvo - inicio) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        return d <= (fim - inicio);
      };
      const rotulo = dentroDoCego(pose.yaw) ? 'frente sem visão'
        : dentroDoCego(pose.yaw + Math.PI) ? 'traseira sem visão'
        : 'sem visão aqui';
      textoComFundo(
        rotulo,
        rx + Math.cos(meio) * raio * 0.6, ry - Math.sin(meio) * raio * 0.6,
        ESTILO_AO_VIVO.cego_texto, true);
    }

    // O servidor manda um ponto sim, outro nao (ros_node.py decima 1:2). Sem
    // recalcular o passo, metade do scan aparecia comprimida em 90 graus.
    const total = scan.total || scan.ranges.length;
    const fator = Math.max(1, Math.round(total / scan.ranges.length));
    const passo = scan.angle_increment * fator;
    const tamanho = Math.max(2, Math.min(4, vista.escala * 0.12));
    pincel2d.fillStyle = ESTILO_AO_VIVO.lidar;
    for (let i = 0; i < scan.ranges.length; i += 1) {
      const r = scan.ranges[i];
      if (r === null || r === undefined) continue;
      // laser -> robo -> mapa, nesta ordem. Pular a primeira etapa era o bug
      // do espelho; pular a segunda deslocava tudo 0,245 m.
      const angRobo = anguloNoRobo(scan.angle_min + i * passo);
      const xNoRobo = LIDAR_X_NO_ROBO + Math.cos(angRobo) * r;
      const yNoRobo = Math.sin(angRobo) * r;
      const [px, py] = paraTela(
        pose.x + Math.cos(pose.yaw) * xNoRobo - Math.sin(pose.yaw) * yNoRobo,
        pose.y + Math.sin(pose.yaw) * xNoRobo + Math.cos(pose.yaw) * yNoRobo);
      pincel2d.fillRect(px - tamanho / 2, py - tamanho / 2, tamanho, tamanho);
    }
  }

  function desenharRobo(pose) {
    const est = estilo();
    const [rx, ry] = paraTela(pose.x, pose.y);
    const cor = pose.velho ? ESTILO_AO_VIVO.velho : est.robo;
    const metroEmPx = vista.escala / meta.resolucao;
    const comprimento = COMPRIMENTO_ROBO_M * metroEmPx;
    const largura = LARGURA_ROBO_M * metroEmPx;

    pincel2d.save();
    pincel2d.translate(rx, ry);
    pincel2d.rotate(-pose.yaw);
    if (comprimento >= 14) {
      // Footprint em escala: e' o que responde "o robo passa nesse corredor?".
      pincel2d.fillStyle = 'rgba(255,154,46,0.35)';
      pincel2d.strokeStyle = cor;
      pincel2d.lineWidth = 2;
      pincel2d.beginPath();
      pincel2d.rect(-comprimento / 2, -largura / 2, comprimento, largura);
      pincel2d.fill();
      pincel2d.stroke();
      pincel2d.beginPath();
      pincel2d.moveTo(comprimento / 2, 0);
      pincel2d.lineTo(comprimento / 6, -largura / 4);
      pincel2d.lineTo(comprimento / 6, largura / 4);
      pincel2d.closePath();
      pincel2d.fillStyle = cor;
      pincel2d.fill();
    } else {
      // De longe o footprint viraria um ponto: vale mais um icone legivel, que
      // ao menos mostra para que lado o robo aponta.
      pincel2d.beginPath();
      pincel2d.moveTo(13, 0);
      pincel2d.lineTo(-8, -8);
      pincel2d.lineTo(-8, 8);
      pincel2d.closePath();
      pincel2d.fillStyle = cor;
      pincel2d.fill();
    }
    pincel2d.restore();
  }

  function desenharEdicao() {
    const est = estilo();
    const cor = est.edicao;
    const comprimento = est.comprimento_seta_px || 22;

    // Halo do ponto destacado: sobrevive ao fim do gesto, para o operador nao
    // perder de vista qual dos vinte marcadores a tela esta editando.
    const alvo = poseDoMarcador(destaque.tipo, destaque.id);
    if (alvo) {
      const [cx, cy] = paraTela(alvo.x, alvo.y);
      pincel2d.beginPath();
      pincel2d.arc(cx, cy, 16, 0, Math.PI * 2);
      pincel2d.setLineDash([5, 4]);
      pincel2d.strokeStyle = 'rgba(242,201,76,0.55)';
      pincel2d.lineWidth = 2;
      pincel2d.stroke();
      pincel2d.setLineDash([]);
      if (!poseProvisoria) {
        // A seta do ponto destacado e' a alca de giro: sem ela o operador
        // estaria arrastando um ponto invisivel da tela.
        desenharSeta(cx, cy, alvo.yaw, comprimento, 'rgba(242,201,76,0.85)');
      }
    }

    if (!poseProvisoria) return;
    const [cx, cy] = paraTela(poseProvisoria.x, poseProvisoria.y);
    desenharSeta(cx, cy, poseProvisoria.yaw, comprimento + 6, cor);
    pincel2d.beginPath();
    pincel2d.arc(cx, cy, 8, 0, Math.PI * 2);
    pincel2d.fillStyle = cor;
    pincel2d.fill();
    pincel2d.strokeStyle = PALETA.tinta;
    pincel2d.lineWidth = 2;
    pincel2d.stroke();

    // Numeros do que esta sendo escolhido: o operador confere antes de soltar,
    // em vez de salvar e conferir depois.
    const texto = `x ${poseProvisoria.x.toFixed(2)} m   y ${poseProvisoria.y.toFixed(2)} m`
      + `   giro ${Math.round(poseProvisoria.yaw * 180 / Math.PI)}°`;
    textoComFundo(texto, cx + 14, cy + 20, clarear(cor, 1.3));
  }

  function desenharPincel() {
    if (modo !== MODOS.PAREDES || !cursorNoMapa) return;
    const raio = Math.max(2, (pincel.raio_m / meta.resolucao) * vista.escala);
    pincel2d.beginPath();
    pincel2d.arc(cursorNoMapa[0], cursorNoMapa[1], raio, 0, Math.PI * 2);
    pincel2d.strokeStyle = pincel.valor === 'parede'
      ? PALETA.vermelho : PALETA.ciano;
    pincel2d.lineWidth = 1.5;
    pincel2d.stroke();
  }

  function pintar() {
    const larg = larguraVisivel();
    const alt = alturaVisivel();
    if (!larg || !alt) return;
    const est = estilo();
    pincel2d.clearRect(0, 0, larg, alt);
    pincel2d.fillStyle = est.fundo;
    pincel2d.fillRect(0, 0, larg, alt);

    if (!temMapa()) {
      pincel2d.fillStyle = PALETA.fraco;
      pincel2d.font = `600 14px ${FONTE}`;
      pincel2d.textAlign = 'center';
      pincel2d.textBaseline = 'middle';
      pincel2d.fillText(mensagem, larg / 2, alt / 2);
      pincel2d.textAlign = 'left';
      return;
    }

    pincel2d.textAlign = 'left';
    pincel2d.imageSmoothingEnabled = vista.escala < 4;
    pincel2d.drawImage(imagemMapa, vista.dx, vista.dy,
                       meta.largura_px * vista.escala, meta.altura_px * vista.escala);

    // Mesma ordem do MapPreview da GUI. Quem esta por cima no desenho e' quem
    // ganha o toque quando dois pontos se sobrepoem.
    // No modo de pintar, a camada aparece mesmo desligada: esconder o que a
    // mao esta desenhando seria desenhar no escuro.
    if (camadas.get('keepout') || modo === MODOS.PAREDES) desenharParedes();
    if (camadas.get('grade')) desenharGrade();
    if (camadas.get('areas')) desenharAreas();
    if (camadas.get('docks')) desenharDocks();
    if (camadas.get('waypoints')) desenharWaypoints();
    if (camadas.get('caminho')) desenharCaminho();

    const pose = estado.pose;
    if (pose) {
      if (camadas.get('lidar')) desenharLidar(pose);
      if (camadas.get('robo')) desenharRobo(pose);
    }

    // Sempre por cima: o ponto em edicao nao pode ficar escondido atras de um
    // vizinho justamente na hora em que o operador esta mexendo nele.
    desenharEdicao();
    desenharPincel();
  }

  // ------------------------------------------------------- paredes virtuais
  function refazerCamadaDeParedes() {
    if (!meta || !meta.largura_px || !meta.altura_px) return;
    if (!camadaParedes) camadaParedes = document.createElement('canvas');
    if (camadaParedes.width !== meta.largura_px || camadaParedes.height !== meta.altura_px) {
      camadaParedes.width = meta.largura_px;
      camadaParedes.height = meta.altura_px;
    }
    const c = camadaParedes.getContext('2d');
    c.clearRect(0, 0, camadaParedes.width, camadaParedes.height);
    if (imagemParedes) {
      c.drawImage(imagemParedes, 0, 0, camadaParedes.width, camadaParedes.height);
    }
    for (const g of grupos) aplicarGrupoNaCamada(c, g);
  }

  function paraPixelDaMascara(x, y) {
    // A mesma conta que o servidor usa para pintar o PGM (mapeamento.py). Se as
    // duas contas discordarem, o operador desenha num lugar e a parede virtual
    // nasce em outro -- e o robo entra exatamente onde nao pode.
    const geo = (meta && meta.keepout) || null;
    const res = geo ? geo.resolucao : meta.resolucao;
    const ox = geo ? geo.origem[0] : meta.origem[0];
    const oy = geo ? geo.origem[1] : meta.origem[1];
    return [(x - ox) / res, meta.altura_px - (y - oy) / res];
  }

  function aplicarGrupoNaCamada(c, grupo) {
    if (!grupo.pontos.length) return;
    const est = estilo();
    const geo = (meta && meta.keepout) || null;
    const res = geo ? geo.resolucao : meta.resolucao;
    const raio = Math.max(0.5, grupo.raio_m / res);
    c.save();
    c.beginPath();
    grupo.pontos.forEach((p, i) => {
      const [px, py] = paraPixelDaMascara(p[0], p[1]);
      if (i === 0) c.moveTo(px, py); else c.lineTo(px, py);
    });
    if (grupo.pontos.length === 1) {
      const [px, py] = paraPixelDaMascara(grupo.pontos[0][0], grupo.pontos[0][1]);
      c.lineTo(px + 0.01, py);
    }
    c.lineCap = 'round';
    c.lineJoin = 'round';
    c.lineWidth = raio * 2;
    if (grupo.valor === 'parede') {
      c.strokeStyle = est.keepout;
    } else {
      c.globalCompositeOperation = 'destination-out';
      c.strokeStyle = 'rgba(0,0,0,1)';
    }
    c.stroke();
    c.restore();
  }

  function segmentosDasPinceladas() {
    // O servidor quer METROS, nao pixel: a mesma pincelada tem que valer igual
    // num mapa de 2 cm e num de 5 cm por celula (mapeamento.py).
    const lista = [];
    for (const g of grupos) {
      if (g.pontos.length === 1) {
        lista.push({ valor: g.valor, raio_m: g.raio_m, x: g.pontos[0][0], y: g.pontos[0][1] });
        continue;
      }
      for (let i = 1; i < g.pontos.length; i += 1) {
        lista.push({
          valor: g.valor, raio_m: g.raio_m,
          x0: g.pontos[i - 1][0], y0: g.pontos[i - 1][1],
          x1: g.pontos[i][0], y1: g.pontos[i][1],
        });
      }
    }
    return lista;
  }

  function avisarPinceladas() {
    chamar('aoPintar', { segmentos: segmentosDasPinceladas(), grupos: grupos.length });
  }

  function comecarTraco(cx, cy) {
    const [x, y] = paraMapa(cx, cy);
    tracoAtual = { valor: pincel.valor, raio_m: pincel.raio_m, pontos: [[x, y]] };
    agendarDesenho();
  }

  function seguirTraco(cx, cy) {
    if (!tracoAtual) return;
    const [x, y] = paraMapa(cx, cy);
    const ultimo = tracoAtual.pontos[tracoAtual.pontos.length - 1];
    // Um ponto a cada meio pincel: guardar cada pixel do mouse encheria o envio
    // com milhares de segmentos que desenham exatamente a mesma linha.
    if (distancia(x, y, ultimo[0], ultimo[1]) < tracoAtual.raio_m / 2) return;
    tracoAtual.pontos.push([x, y]);
    agendarDesenho();
  }

  function terminarTraco() {
    if (!tracoAtual) return;
    const grupo = tracoAtual;
    tracoAtual = null;
    grupos.push(grupo);
    if (camadaParedes) aplicarGrupoNaCamada(camadaParedes.getContext('2d'), grupo);
    else refazerCamadaDeParedes();
    const quantos = segmentosDasPinceladas().length;
    if (quantos > MAX_PINCELADAS) {
      avisar('O desenho ficou grande demais para gravar de uma vez. '
             + 'Grave o que já foi feito antes de continuar.');
    }
    avisarPinceladas();
    agendarDesenho();
  }

  // ---------------------------------------------------------------- cursor
  function atualizarCursor(cx, cy) {
    if (modo === MODOS.PAREDES) { cv.style.cursor = 'crosshair'; return; }
    if (modo === MODOS.OLHAR) { cv.style.cursor = 'grab'; return; }
    if (modo === MODOS.EDITAR) {
      const achado = temMapa() ? buscarMarcador(cx, cy, 1) : null;
      // Sem marcador embaixo, arrastar move o mapa -- o cursor tem que dizer isso.
      cv.style.cursor = !achado ? 'grab' : (achado.naSeta ? 'pointer' : 'move');
      return;
    }
    cv.style.cursor = 'crosshair';
  }

  // ---------------------------------------------------------------- gestos
  function medirDois() {
    const [a, b] = [...dedos.values()];
    return {
      dist: distancia(a.x, a.y, b.x, b.y),
      mx: (a.x + b.x) / 2,
      my: (a.y + b.y) / 2,
    };
  }

  function aplicarDoisDedos() {
    if (!doisDedos) return;
    const agora = medirDois();
    if (agora.dist > 8 && doisDedos.dist > 8) {
      ampliar(agora.dist / doisDedos.dist, agora.mx, agora.my);
    }
    vista.dx += agora.mx - doisDedos.mx;
    vista.dy += agora.my - doisDedos.my;
    doisDedos = agora;
    agendarDesenho();
  }

  function cancelarAcoes() {
    if (cronometroAcoes) { clearTimeout(cronometroAcoes); cronometroAcoes = 0; }
  }

  function abandonarGesto() {
    cancelarAcoes();
    // Sem limpar a pose provisoria fica um ponto amarelo no mapa sem gesto
    // nenhum por tras dele, e o operador acha que gravou alguma coisa.
    if (gesto) poseProvisoria = null;
    gesto = null;
    tracoAtual = null;
    agendarDesenho();
  }

  function aoDescer(ev) {
    cv.setPointerCapture(ev.pointerId);
    const [x, y] = local(ev);
    dedos.set(ev.pointerId, { x, y, tipo: ev.pointerType });
    cursorNoMapa = [x, y];

    if (dedos.size === 2) {
      // Dois dedos e' navegar o mapa, nunca comandar: o segundo dedo cancela o
      // que o primeiro tinha comecado.
      abandonarGesto();
      pan = null;
      doisDedos = medirDois();
      return;
    }
    if (dedos.size > 2) return;

    const botaoDeArrasto = (ev.button === 1 || ev.button === 2);
    if (botaoDeArrasto || modo === MODOS.OLHAR) {
      pan = { x, y };
      cv.style.cursor = 'grabbing';
      return;
    }
    if (!temMapa()) return;

    if (modo === MODOS.PAREDES) { comecarTraco(x, y); return; }

    if (modo === MODOS.EDITAR) {
      const folga = ev.pointerType === 'touch' ? 1.6 : 1;
      const achado = buscarMarcador(x, y, folga);
      if (!achado) { pan = { x, y }; cv.style.cursor = 'grabbing'; return; }
      const pose = achado.marcador.pose;
      destaque = { tipo: achado.marcador.tipo, id: achado.marcador.id };
      poseProvisoria = { x: pose.x, y: pose.y, yaw: pose.yaw };
      gesto = {
        tipo: achado.naSeta ? 'girar' : 'mover',
        x0: x, y0: y, moveu: false,
        alvo: { tipo: achado.marcador.tipo, id: achado.marcador.id },
      };
      chamar('aoClicarMarcador',
             { tipo: achado.marcador.tipo, id: achado.marcador.id, pose });
      // Tablet nao tem botao direito: dedo parado sobre o ponto abre as acoes.
      cancelarAcoes();
      cronometroAcoes = setTimeout(() => {
        cronometroAcoes = 0;
        if (!gesto || gesto.moveu) return;
        gesto = null;
        // O ponto ja' esta desenhado no lugar; deixar tambem a bolinha amarela
        // em cima dele so' faria parecer que existem dois pontos ali.
        poseProvisoria = null;
        chamar('aoAbrirAcoes',
               { tipo: achado.marcador.tipo, id: achado.marcador.id, pose },
               { x: ev.clientX, y: ev.clientY });
      }, ESPERA_ACOES_MS);
      agendarDesenho();
      return;
    }

    // Navegar / Posicionar: gesto do "2D Goal". O toque ja' define a posicao; a
    // direcao so' existe depois de arrastar, por isso comeca em zero.
    const [mx, my] = paraMapa(x, y);
    poseProvisoria = { x: mx, y: my, yaw: 0 };
    gesto = { tipo: 'nova', x0: x, y0: y, moveu: false, alvo: null };
    agendarDesenho();
  }

  function aoMoverPonteiro(ev) {
    const [x, y] = local(ev);
    if (!dedos.has(ev.pointerId)) {
      cursorNoMapa = [x, y];
      atualizarCursor(x, y);
      if (modo === MODOS.PAREDES) agendarDesenho();
      return;
    }
    dedos.set(ev.pointerId, { x, y, tipo: ev.pointerType });
    cursorNoMapa = [x, y];

    if (dedos.size >= 2) { aplicarDoisDedos(); return; }
    if (pan) {
      vista.dx += x - pan.x;
      vista.dy += y - pan.y;
      pan = { x, y };
      agendarDesenho();
      return;
    }
    if (tracoAtual) { seguirTraco(x, y); return; }
    if (!gesto) return;

    const limiar = gesto.tipo === 'nova' ? LIMIAR_DIRECAO_PX : LIMIAR_ARRASTO_PX;
    if (distancia(x, y, gesto.x0, gesto.y0) > limiar) {
      gesto.moveu = true;
      cancelarAcoes();
    }
    if (gesto.tipo === 'nova') {
      if (gesto.moveu) poseProvisoria.yaw = yawDoArrasto(gesto.x0, gesto.y0, x, y);
    } else if (gesto.tipo === 'mover') {
      const [mx, my] = paraMapa(x, y);
      poseProvisoria.x = mx;
      poseProvisoria.y = my;
    } else if (gesto.tipo === 'girar') {
      const [cx, cy] = paraTela(poseProvisoria.x, poseProvisoria.y);
      poseProvisoria.yaw = yawDoArrasto(cx, cy, x, y);
    }
    agendarDesenho();
  }

  function aoSubir(ev) {
    const [x, y] = local(ev);
    dedos.delete(ev.pointerId);
    if (dedos.size < 2) doisDedos = null;
    cancelarAcoes();

    if (pan) { pan = null; atualizarCursor(x, y); }
    if (tracoAtual) { terminarTraco(); return; }
    if (!gesto) { agendarDesenho(); return; }

    const terminado = gesto;
    gesto = null;
    if (terminado.tipo === 'nova') {
      // Sem arrasto o operador nao escolheu direcao. Quem decide o que fazer
      // com isso e' a tela: navegar aceita "tanto faz", dizer onde o robo esta
      // nao aceita.
      chamar('aoEscolherPose', {
        x: poseProvisoria.x, y: poseProvisoria.y, yaw: poseProvisoria.yaw,
        arrastou: terminado.moveu, modo,
      });
    } else if (terminado.moveu) {
      // Toque curto num ponto salvo e' selecao (ja' avisada no press); so' vira
      // alteracao quando o operador de fato arrastou.
      chamar('aoArrastarMarcador', terminado.alvo, {
        x: poseProvisoria.x, y: poseProvisoria.y, yaw: poseProvisoria.yaw,
      });
    }
    atualizarCursor(x, y);
    agendarDesenho();
  }

  function aoCancelarPonteiro(ev) {
    dedos.delete(ev.pointerId);
    if (dedos.size < 2) doisDedos = null;
    pan = null;
    if (tracoAtual) terminarTraco();
    abandonarGesto();
  }

  function aoRoda(ev) {
    ev.preventDefault();
    if (!temMapa()) return;
    const [x, y] = local(ev);
    ampliar(ev.deltaY < 0 ? PASSO_DA_RODA : 1 / PASSO_DA_RODA, x, y);
  }

  function aoMenuDoSistema(ev) {
    ev.preventDefault();
    if (modo !== MODOS.EDITAR || !temMapa()) return;
    const [x, y] = local(ev);
    const achado = buscarMarcador(x, y, 1);
    if (!achado) return;
    destaque = { tipo: achado.marcador.tipo, id: achado.marcador.id };
    chamar('aoAbrirAcoes',
           { tipo: achado.marcador.tipo, id: achado.marcador.id, pose: achado.marcador.pose },
           { x: ev.clientX, y: ev.clientY });
    agendarDesenho();
  }

  function aoVoltarAVer() {
    if (!document.hidden) agendarDesenho();
  }

  cv.addEventListener('pointerdown', aoDescer);
  cv.addEventListener('pointermove', aoMoverPonteiro);
  cv.addEventListener('pointerup', aoSubir);
  cv.addEventListener('pointercancel', aoCancelarPonteiro);
  cv.addEventListener('pointerleave', (ev) => {
    if (!dedos.has(ev.pointerId)) { cursorNoMapa = null; agendarDesenho(); }
  });
  cv.addEventListener('wheel', aoRoda, { passive: false });
  cv.addEventListener('contextmenu', aoMenuDoSistema);
  document.addEventListener('visibilitychange', aoVoltarAVer);

  const observador = new ResizeObserver(() => aoRedimensionar());
  observador.observe(hospedeiro);

  // ------------------------------------------------------------------- API
  async function carregarArena(nome, metaPronta) {
    const serie = ++serieDeCarga;
    arena = nome || '';
    if (!arena) {
      meta = null; imagemMapa = null; imagemParedes = null; camadaParedes = null;
      mensagem = 'Escolha uma arena';
      agendarDesenho();
      return null;
    }
    let dados = metaPronta || null;
    if (!dados) {
      try {
        dados = await (await fetch(urlMeta(arena))).json();
      } catch (e) {
        dados = { erro: 'Não consegui falar com o robô para abrir esta arena.' };
      }
    }
    if (serie !== serieDeCarga) return null;   // o operador ja' trocou de arena
    if (!dados || dados.erro) {
      meta = null; imagemMapa = null; imagemParedes = null; camadaParedes = null;
      mensagem = (dados && dados.erro) || 'Não consegui abrir esta arena.';
      agendarDesenho();
      return dados || null;
    }

    meta = dados;
    grupos = [];
    tracoAtual = null;
    poseProvisoria = null;
    destaque = { tipo: '', id: '' };
    vista.enquadrado = false;

    const mapa = await carregarImagem(urlMapa(arena));
    // Sem a guarda de serie, trocar de arena duas vezes seguidas deixava o
    // desenho da anterior por cima dos pontos da nova.
    if (serie !== serieDeCarga) return meta;
    imagemMapa = mapa;
    if (!imagemMapa) {
      mensagem = 'Não consegui carregar o desenho desta arena.';
      agendarDesenho();
      return meta;
    }
    enquadrar();

    imagemParedes = null;
    // A composicao nasce mesmo sem mascara no disco: da' para desenhar parede
    // virtual numa arena que ainda nao tem nenhuma (o servidor cria o arquivo
    // na hora de gravar).
    refazerCamadaDeParedes();
    if (meta.tem_keepout) {
      const mascara = await carregarImagem(urlParedes(arena));
      if (serie !== serieDeCarga) return meta;
      imagemParedes = mascara;
      refazerCamadaDeParedes();
      agendarDesenho();
    }
    return meta;
  }

  async function recarregarParedes() {
    if (!meta || !arena) return;
    // Parametro de versao: sem ele o navegador devolve a mascara do cache e o
    // operador jura que o "gravar paredes" nao funcionou.
    const mascara = meta.tem_keepout
      ? await carregarImagem(`${urlParedes(arena)}?v=${Date.now()}`) : null;
    imagemParedes = mascara;
    grupos = [];
    tracoAtual = null;
    refazerCamadaDeParedes();
    avisarPinceladas();
    agendarDesenho();
  }

  function definirEstado(quadro) {
    if (!quadro) return;
    estado = quadro;
    // O quadro chega a 10 Hz e o LiDAR so' na metade deles (ros_node.py). Sem
    // segurar o ultimo, os pontos piscavam na tela.
    if (quadro.scan && quadro.scan.ranges) {
      scanVivo = quadro.scan;
      scanChegouEm = Date.now();
    }
    agendarDesenho();
  }

  function definirCaminho(pontos) {
    const lista = [];
    for (const p of pontos || []) {
      if (Array.isArray(p) && p.length >= 2) lista.push([Number(p[0]), Number(p[1])]);
      else if (p && typeof p === 'object') lista.push([Number(p.x), Number(p.y)]);
    }
    caminho = lista.filter((p) => Number.isFinite(p[0]) && Number.isFinite(p[1]));
    agendarDesenho();
  }

  function definirModo(novo) {
    if (modo === novo) return;
    modo = novo;
    abandonarGesto();
    pan = null;
    if (novo !== MODOS.EDITAR && novo !== MODOS.NAVEGAR && novo !== MODOS.POSICIONAR) {
      poseProvisoria = null;
    }
    atualizarCursor(cursorNoMapa ? cursorNoMapa[0] : 0, cursorNoMapa ? cursorNoMapa[1] : 0);
    agendarDesenho();
  }

  function listarCamadas() {
    // Lista vazia nao serve como lista: uma arena antiga pode vir sem o campo,
    // e a tela ficaria sem nenhum interruptor de camada para oferecer.
    const doArquivo = (meta && meta.camadas && meta.camadas.length)
      ? meta.camadas : CAMADAS_RESERVA;
    return [...doArquivo, ...CAMADAS_AO_VIVO].map(
      (c) => ({ id: c.id, rotulo: c.rotulo, ligada: camadas.get(c.id) !== false }));
  }

  function destruir() {
    vivo = false;
    if (quadroPedido) cancelAnimationFrame(quadroPedido);
    cancelarAcoes();
    observador.disconnect();
    document.removeEventListener('visibilitychange', aoVoltarAVer);
    cv.remove();
  }

  atualizarCursor(0, 0);
  agendarDesenho();

  return {
    // --- arena ---
    carregarArena,
    recarregarParedes,
    meta: () => meta,
    arena: () => arena,
    estiloDoMapa: estilo,

    // --- dado ao vivo ---
    definirEstado,
    definirCaminho,

    // --- vista ---
    enquadrar,
    ampliar,
    escala: () => vista.escala,
    paraTela,
    paraMapa,
    redesenhar: agendarDesenho,

    // --- camadas ---
    camadas: listarCamadas,
    definirCamada(id, ligada) { camadas.set(id, !!ligada); agendarDesenho(); },
    camadaLigada: (id) => camadas.get(id) !== false,

    // --- modos e edicao ---
    MODOS,
    definirModo,
    modo: () => modo,
    destacar(tipo, id) { destaque = { tipo: tipo || '', id: id || '' }; agendarDesenho(); },
    limparEdicao() {
      destaque = { tipo: '', id: '' };
      poseProvisoria = null;
      abandonarGesto();
    },
    poseEmEdicao: () => (poseProvisoria ? Object.assign({}, poseProvisoria) : null),
    marcadorEm(cx, cy) {
      const achado = temMapa() ? buscarMarcador(cx, cy, 1) : null;
      return achado ? achado.marcador : null;
    },

    // --- paredes virtuais ---
    definirPincel(novo) {
      if (novo && novo.valor) pincel.valor = novo.valor;
      if (novo && Number.isFinite(novo.raio_m)) {
        pincel.raio_m = Math.max(0.01, Math.min(2.0, novo.raio_m));
      }
      agendarDesenho();
    },
    pincel: () => Object.assign({}, pincel),
    pinceladas: segmentosDasPinceladas,
    temPinceladas: () => grupos.length > 0,
    desfazer() {
      if (!grupos.length) return false;
      grupos.pop();
      refazerCamadaDeParedes();
      avisarPinceladas();
      agendarDesenho();
      return true;
    },
    limparPinceladas() {
      grupos = [];
      tracoAtual = null;
      refazerCamadaDeParedes();
      avisarPinceladas();
      agendarDesenho();
    },

    destruir,
  };
}
