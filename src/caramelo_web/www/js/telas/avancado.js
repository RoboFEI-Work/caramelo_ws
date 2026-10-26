// Modo Avancado: o contexto tecnico do painel, espelhando
// caramelo_gui/src/modules/avancado/sensores_module.* e
// caramelo_gui/src/modules/inicio/inicio_module.*, mais as duas coisas que so
// existem na web -- o terminal e o espelho da tela do robo.
//
// POR QUE ele e maior que na GUI: quem esta na frente do robo tem o teclado
// dele. Quem esta do outro lado do galpao, com um tablet, nao tem. Terminal e
// espelho de tela existem para essa pessoa, e nao para dar mais poder a quem ja
// esta sentado no robo.
//
// A regra de texto vale aqui tambem, e e a mais facil de esquecer justamente
// nesta tela: quem abre o Modo Avancado pode saber mais, mas nao
// necessariamente sabe ROS. "O LiDAR nao esta respondendo" e texto de tela;
// "/scan STALE" nao e. Nome de topico so aparece dentro do terminal, que e o
// unico lugar onde ele e o assunto.
//
// Secoes (as mesmas que o servidor anuncia em MENU_PRINCIPAL, contexto
// "avancado"): Estado do robo, Sensores ao vivo, Service areas, Terminal e
// Tela do robo.

const ESTILO_ID = "estilo-tela-avancado";

// GIF de 1x1 transparente. Fechar um MJPEG e trocar o src da imagem, e trocar
// por "" faz o navegador pedir a PAGINA de novo (src vazio resolve para a URL
// atual). Com o gif embutido a conexao morre na hora e nada e pedido -- e e
// disso que depende a vaga do espelho voltar: o teto e de dois.
const IMAGEM_VAZIA =
  "data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7";

const ESTILO = `
.av{display:flex;flex-direction:column;gap:12px;height:100%;min-height:480px;
    color:var(--texto,#e6f1ff);font-family:"Noto Sans",system-ui,sans-serif}
.av *{box-sizing:border-box}
.av-corpo{flex:1;min-height:0;display:grid;grid-template-columns:222px minmax(0,1fr);
          gap:14px}
.av-trilho{display:flex;flex-direction:column;gap:4px;padding:8px;
           background:var(--topo,#081120);border:1px solid var(--borda,#1b3457);
           border-radius:16px;overflow:auto}
.av-trilho button{display:block;width:100%;text-align:left;background:transparent;
                  border:none;border-radius:12px;color:var(--fraco,#7d9cc4);
                  font:inherit;font-weight:600;font-size:14px;padding:13px 16px;
                  cursor:pointer;white-space:nowrap}
.av-trilho button:hover{background:#12233f;color:#bfe3ff}
.av-trilho button[aria-selected="true"]{background:#103154;color:var(--ciano,#35c3f0);
                                        font-weight:700}
.av-palco{min-width:0;min-height:0;overflow:auto;padding-right:4px}
.av-secao{display:none;flex-direction:column;gap:12px;min-height:100%}
.av-secao.ativa{display:flex}
.av-cartao{background:var(--painel,#101e35);border:1px solid var(--borda,#1b3457);
           border-radius:16px;padding:14px;min-width:0}
.av-cartao h3{margin:0 0 10px;font-size:12px;font-weight:800;letter-spacing:.6px;
              text-transform:uppercase;color:var(--fraco,#7d9cc4)}
.av-cartao h4{margin:0;font-size:16px;font-weight:700;color:#dcecff}
.av-grade{display:grid;grid-template-columns:repeat(auto-fill,minmax(268px,1fr));
          gap:12px}
.av-duas{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:12px;
         align-items:start}
.av-dica{color:var(--fraco,#7d9cc4);font-size:12.5px;line-height:1.5}
.av-motivo{color:#ffd28f;font-size:12.5px;line-height:1.5}
.av-topo{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.av-espaco{flex:1}
.av-farol{width:14px;height:14px;border-radius:7px;flex:none;background:#7f93b0}
.av-chip{padding:4px 10px;border-radius:9px;font-weight:800;font-size:11px;
         background:#7f93b0;color:#06121f;white-space:nowrap}
.av-linha{display:flex;justify-content:space-between;gap:12px;padding:5px 0;
          border-bottom:1px solid #16294a;font-size:13px}
.av-linha:last-child{border-bottom:none}
.av-linha .rot{color:var(--fraco,#7d9cc4)}
.av-linha .val{font-weight:700;text-align:right}
.av-bt{background:#142643;color:#dcecff;border:1px solid #1f3a5f;border-radius:12px;
       padding:10px 16px;font:inherit;font-weight:600;cursor:pointer}
.av-bt:hover:not(:disabled){background:#1b3457;border-color:var(--ciano,#35c3f0);
                            color:#fff}
.av-bt:disabled{opacity:.45;cursor:not-allowed}
.av-bt.primaria{background:linear-gradient(#ff9a2e,#f07f0e);border:none;
                color:#1a0e00;font-weight:800}
.av-bt.primaria:hover:not(:disabled){background:#ffab4d}
.av-acoes{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
.av-sel{background:#0e1c31;color:var(--texto,#e6f1ff);border:1px solid #1f3a5f;
        border-radius:10px;padding:9px 10px;font:inherit;font-weight:600;
        max-width:100%}
.av-rolagem{overflow-x:auto;max-width:100%}
.av-tabela{border-collapse:collapse;width:100%;min-width:520px;font-size:13px}
.av-tabela th{text-align:left;color:var(--fraco,#7d9cc4);font-size:11px;
              text-transform:uppercase;letter-spacing:.5px;padding:6px 10px;
              border-bottom:1px solid var(--borda,#1b3457);white-space:nowrap}
.av-tabela td{padding:7px 10px;border-bottom:1px solid #16294a;vertical-align:top}
.av-tabela tr:last-child td{border-bottom:none}
.av-barra{height:10px;border-radius:5px;background:#0e1c31;overflow:hidden;
          border:1px solid #1f3a5f}
.av-barra i{display:block;height:100%;background:var(--verde,#27ae60);width:0}
.av-visor{width:100%;display:block;background:#081120;border-radius:12px;
          border:1px solid var(--borda,#1b3457)}
.av-imagem{width:100%;max-height:52vh;object-fit:contain;background:#050d18;
           border-radius:12px;border:1px solid var(--borda,#1b3457);display:block}
.av-mapa{position:relative;min-height:300px;height:42vh;background:#101e35;
         border:1px solid var(--borda,#1b3457);border-radius:12px;overflow:hidden}
.av-tabela tr[data-destacada="1"] td{background:#123a56}
.av-terminal{flex:1;min-height:320px;background:#000;border-radius:12px;padding:6px;
             border:1px solid var(--borda,#1b3457)}
.av-pilha{display:flex;flex-direction:column;gap:10px;flex:1;min-height:0}
.av-janela{position:fixed;z-index:80;background:var(--painel,#101e35);
           border:1px solid var(--borda,#1b3457);border-radius:16px;
           box-shadow:0 18px 60px rgba(0,0,0,.65);display:flex;
           flex-direction:column;overflow:hidden;max-width:96vw;max-height:92vh}
.av-janela header{display:flex;align-items:center;gap:10px;padding:8px 12px;
                  background:var(--topo,#081120);border-bottom:1px solid #14294a;
                  cursor:move;user-select:none;font-size:13px;font-weight:700}
.av-janela .corpo{padding:8px;overflow:auto;background:#050d18;min-height:120px}
.av-janela img{max-width:100%;display:block;border-radius:8px;margin:0 auto}
.av-janela .av-bt{padding:6px 12px;font-size:13px}
@media (max-width:900px){
  .av-corpo{grid-template-columns:minmax(0,1fr);gap:10px}
  .av-trilho{flex-direction:row;overflow-x:auto;padding:6px}
  .av-trilho button{width:auto;padding:10px 14px}
  .av-duas{grid-template-columns:minmax(0,1fr)}
  .av-terminal{min-height:60vh}
}
`;

function garantirEstilo() {
  if (document.getElementById(ESTILO_ID)) return;
  const est = document.createElement("style");
  est.id = ESTILO_ID;
  est.textContent = ESTILO;
  document.head.appendChild(est);
}

// --------------------------------------------------------------- utilidades
// Tudo que vem do robo entra por textContent, nunca por innerHTML: nome de
// arena, de dock e de componente sao dados, e dado nao pode virar marcacao.
function el(tag, attr, filhos) {
  const no = document.createElement(tag);
  for (const chave in (attr || {})) {
    const valor = attr[chave];
    if (chave === "texto") no.textContent = valor;
    else if (chave === "classe") no.className = valor;
    else if (chave === "estilo") no.style.cssText = valor;
    else if (chave.startsWith("ao")) no.addEventListener(chave.slice(2).toLowerCase(), valor);
    else if (valor !== null && valor !== undefined) no.setAttribute(chave, valor);
  }
  (filhos || []).forEach((f) => no.appendChild(typeof f === "string"
    ? document.createTextNode(f) : f));
  return no;
}

const numero = (v, casas) => (typeof v === "number" && isFinite(v))
  ? v.toFixed(casas === undefined ? 1 : casas) : "—";

function duracao(segundos) {
  if (typeof segundos !== "number" || !isFinite(segundos) || segundos < 0) return "—";
  const d = Math.floor(segundos / 86400);
  const h = Math.floor((segundos % 86400) / 3600);
  const m = Math.floor((segundos % 3600) / 60);
  if (d) return `${d} d ${h} h`;
  if (h) return `${h} h ${m} min`;
  if (m) return `${m} min`;
  return `${Math.floor(segundos)} s`;
}

// A ponte com o resto do painel. O contrato da tela diz que `ctx` da acesso a
// API e ao estado, mas nao fixa os nomes dos metodos -- e esta tela nao pode
// morrer por causa disso. Se o app oferecer, usa-se o dele (uma torrada so,
// uma chave so); se nao, cai no fetch direto, que e o mesmo caminho.
function criarPonte(ctx) {
  ctx = ctx || {};
  const api = ctx.api || {};
  const chave = ctx.chave || ctx.token ||
    new URLSearchParams(location.search).get("t") || "";

  const comChave = (rota) => chave
    ? rota + (rota.includes("?") ? "&" : "?") + "t=" + encodeURIComponent(chave)
    : rota;

  const obter = async (rota) => {
    if (typeof api.obter === "function") return api.obter(rota);
    const r = await fetch(comChave(rota));
    const dados = await r.json().catch(() => ({}));
    if (!r.ok) return Object.assign({ok: false, http: r.status}, dados);
    return dados;
  };

  const comandar = async (rota, corpo) => {
    if (typeof api.comandar === "function") return api.comandar(rota, corpo);
    try {
      const r = await fetch(comChave(rota), {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(corpo || {}),
      });
      const dados = await r.json().catch(() => ({}));
      return Object.assign({ok: r.ok, http: r.status}, dados);
    } catch {
      return {ok: false, http: 0, mensagem: "Perdi contato com o robô."};
    }
  };

  const torrada = (texto, bom) => {
    if (typeof ctx.torrada === "function") ctx.torrada(texto, bom);
  };

  const autorizado = () => {
    const a = ctx.autorizado;
    if (typeof a === "function") return !!a();
    if (typeof a === "boolean") return a;
    // Sem resposta do app, a chave no link e o melhor palpite: o servidor
    // recusa de qualquer jeito, e a tela avisa com a frase que ele mandar.
    return !!chave;
  };

  return {chave, comChave, obter, comandar, torrada, autorizado};
}

// ======================================================== Estado do robo
// Traducao do InicioModule da GUI: um cartao por parte do robo, farol colorido,
// mensagem em portugues e a acao a tomar. O "Modo Engenharia" da GUI virou uma
// caixinha que mostra os numeros do diagnostico -- eles ficam escondidos por
// padrao porque frequencia em hertz nao ajuda quem so quer saber se da para
// rodar a prova.
function criarEstado(ponte) {
  const cartoes = new Map();
  let mostrarNumeros = false;
  let visivel = false;
  let relogio = null;
  let temSistema = null;   // null = ainda nao perguntei; false = este robo nao informa

  const chipEstado = el("span", {classe: "av-chip", texto: "—"});
  const rotuloArena = el("span", {classe: "av-dica", texto: "Arena: —"});

  const barraBateria = el("i");
  const textoBateria = el("div", {classe: "av-linha"}, [
    el("span", {classe: "rot", texto: "Carga"}),
    el("span", {classe: "val", texto: "—"}),
  ]);
  const linhasBateria = el("div");

  const linhasSistema = el("div");
  const dicaSistema = el("div", {classe: "av-dica"});

  const grade = el("div", {classe: "av-grade"});
  const listaNos = el("div", {classe: "av-dica", texto: "—"});
  const contagemNos = el("span", {classe: "av-chip", texto: "—"});

  const caixaNumeros = el("input", {type: "checkbox", id: "avNumeros"});
  caixaNumeros.addEventListener("change", () => {
    mostrarNumeros = caixaNumeros.checked;
    cartoes.forEach((c) => {
      c.numeros.style.display = (mostrarNumeros && c.temNumeros) ? "" : "none";
    });
  });

  const no = el("div", {classe: "av-secao"}, [
    el("div", {classe: "av-cartao"}, [
      el("div", {classe: "av-topo"}, [
        el("h4", {texto: "Estado do robô"}),
        chipEstado,
        el("span", {classe: "av-espaco"}),
        rotuloArena,
      ]),
      el("div", {classe: "av-dica", texto:
        "Cada parte do robô com o que ela está fazendo agora e, quando algo "
        + "está errado, o que fazer a respeito."}),
    ]),
    el("div", {classe: "av-duas"}, [
      el("div", {classe: "av-cartao"}, [
        el("h3", {texto: "Bateria"}),
        el("div", {classe: "av-barra"}, [barraBateria]),
        textoBateria,
        linhasBateria,
      ]),
      el("div", {classe: "av-cartao"}, [
        el("h3", {texto: "Computador do robô"}),
        linhasSistema,
        dicaSistema,
      ]),
    ]),
    el("div", {classe: "av-cartao"}, [
      el("div", {classe: "av-topo"}, [
        el("h3", {texto: "Partes do robô", estilo: "margin:0"}),
        el("span", {classe: "av-espaco"}),
        el("label", {classe: "av-dica", for: "avNumeros",
                     estilo: "display:flex;gap:6px;align-items:center;cursor:pointer"}, [
          caixaNumeros, "mostrar os números de cada parte",
        ]),
      ]),
      el("div", {estilo: "height:10px"}),
      grade,
    ]),
    el("div", {classe: "av-cartao"}, [
      el("div", {classe: "av-topo"}, [
        el("h3", {texto: "Programas no ar", estilo: "margin:0"}),
        contagemNos,
      ]),
      el("div", {estilo: "height:8px"}),
      listaNos,
      el("div", {classe: "av-dica", texto:
        "São os programas que o robô subiu sozinho ao ligar. Serve para "
        + "responder \"o robô terminou de subir?\"."}),
    ]),
  ]);

  function cartaoDe(comp) {
    let c = cartoes.get(comp.id);
    if (c) return c;
    const farol = el("span", {classe: "av-farol"});
    const titulo = el("strong", {texto: comp.nome});
    const rotulo = el("span", {classe: "av-chip", texto: ""});
    const msg = el("div", {classe: "av-dica", estilo: "margin-top:8px"});
    const acao = el("div", {classe: "av-motivo", estilo: "margin-top:6px"});
    const numeros = el("div", {classe: "av-dica", estilo: "margin-top:8px;display:none"});
    const frame = el("div", {classe: "av-cartao"}, [
      el("div", {classe: "av-topo"}, [farol, titulo, el("span", {classe: "av-espaco"}), rotulo]),
      msg, acao, numeros,
    ]);
    c = {frame, farol, titulo, rotulo, msg, acao, numeros, temNumeros: false};
    cartoes.set(comp.id, c);
    grade.appendChild(frame);
    return c;
  }

  function aplicarComponentes(lista) {
    (lista || []).forEach((comp) => {
      const c = cartaoDe(comp);
      c.titulo.textContent = comp.nome;
      c.farol.style.background = comp.cor || "#7f93b0";
      c.rotulo.textContent = comp.rotulo || "";
      c.rotulo.style.background = comp.cor || "#7f93b0";
      c.rotulo.style.color = comp.estado === "falha" ? "#fff" : "#06121f";
      c.msg.textContent = comp.mensagem || "";
      c.acao.textContent = comp.acao || "";
      const numeros = comp.numeros || [];
      c.temNumeros = numeros.length > 0;
      c.numeros.textContent = numeros
        .map((n) => `${n.rotulo}: ${n.valor}`).join("   •   ");
      c.numeros.style.display = (mostrarNumeros && c.temNumeros) ? "" : "none";
    });
  }

  function aplicarBateria(bateria) {
    const b = bateria || {};
    const pct = (typeof b.percentual === "number") ? b.percentual : null;
    barraBateria.style.width = (pct === null ? 0 : Math.max(0, Math.min(100, pct))) + "%";
    barraBateria.style.background = pct === null ? "#7f93b0"
      : (pct >= 50 ? "var(--verde,#27ae60)"
        : (pct >= 20 ? "var(--ambar,#f2994a)" : "var(--vermelho,#eb5757)"));
    textoBateria.lastChild.textContent = pct === null
      ? "sem leitura" : `${Math.round(pct)}%`;

    linhasBateria.textContent = "";
    if (b.tem_dado) {
      if (typeof b.tensao === "number") {
        linhasBateria.appendChild(linha("Tensão", numero(b.tensao, 1) + " V"));
      }
      if (typeof b.corrente === "number") {
        linhasBateria.appendChild(linha("Corrente", numero(b.corrente, 2) + " A"));
      }
      linhasBateria.appendChild(linha("Situação",
        b.carregando ? "carregando" : "em uso"));
    } else {
      linhasBateria.appendChild(el("div", {classe: "av-dica", texto:
        "O robô não está informando a carga da bateria. Confira no medidor da "
        + "própria bateria antes de começar uma prova."}));
    }
  }

  function linha(rot, val) {
    return el("div", {classe: "av-linha"}, [
      el("span", {classe: "rot", texto: rot}),
      el("span", {classe: "val", texto: val}),
    ]);
  }

  // Uso de processador, memoria e tempo ligado sao um EXTRA: o painel funciona
  // sem eles. Por isso a rota e perguntada uma vez e, se este robo nao a tiver,
  // a tela diz isso numa frase em vez de ficar batendo numa porta fechada.
  function aplicarSistema(sis) {
    linhasSistema.textContent = "";
    if (!sis) {
      dicaSistema.textContent = temSistema === false
        ? "Este robô não informa uso de processador e memória."
        : "Consultando…";
      return;
    }
    dicaSistema.textContent = "";
    const tempo = sis.tempo_ligado !== undefined ? duracao(sis.tempo_ligado)
      : (sis.tempo_ligado_texto || null);
    if (tempo) linhasSistema.appendChild(linha("Ligado há", tempo));
    const cpu = (sis.cpu !== undefined) ? sis.cpu : sis.cpu_percentual;
    if (typeof cpu === "number") {
      linhasSistema.appendChild(linha("Processador", Math.round(cpu) + "% em uso"));
    }
    const mem = (sis.memoria !== undefined) ? sis.memoria : sis.memoria_percentual;
    if (typeof mem === "number") {
      linhasSistema.appendChild(linha("Memória", Math.round(mem) + "% em uso"));
    }
    if (sis.memoria_usada_mb && sis.memoria_total_mb) {
      linhasSistema.appendChild(linha("Memória usada",
        `${Math.round(sis.memoria_usada_mb)} de ${Math.round(sis.memoria_total_mb)} MB`));
    }
    if (sis.temperatura_c !== undefined && sis.temperatura_c !== null) {
      linhasSistema.appendChild(linha("Temperatura", numero(sis.temperatura_c, 0) + " °C"));
    }
    if (!linhasSistema.childElementCount) {
      dicaSistema.textContent = "Este robô não informa uso de processador e memória.";
    }
  }

  async function consultar() {
    if (!visivel) return;
    const saude = await ponte.obter("/api/saude").catch(() => null);
    if (saude && Array.isArray(saude.nos)) {
      contagemNos.textContent = `${saude.nos.length} no ar`;
      listaNos.textContent = saude.nos.length ? saude.nos.join(", ")
        : "Nenhum programa respondendo: o robô ainda não terminou de subir.";
    }
    if (temSistema === false) return;
    const sis = await ponte.obter("/api/sistema").catch(() => null);
    const valido = sis && sis.ok !== false && !sis.http;
    temSistema = !!valido;
    aplicarSistema(valido ? sis : null);
  }

  return {
    no,
    aoEntrar() {
      visivel = true;
      consultar();
      // 5 s: consultar o grafo do ROS custa CPU do mesmo computador que navega,
      // e a lista de programas no ar praticamente nao muda depois que sobe.
      relogio = setInterval(consultar, 5000);
    },
    aoSair() {
      visivel = false;
      clearInterval(relogio);
      relogio = null;
    },
    aoEstado(estado) {
      const robo = (estado.cartoes || {}).robo || {};
      if (robo.estado) chipEstado.textContent = robo.estado;
      if (robo.arena) rotuloArena.textContent = robo.arena;
      if (estado.componentes) aplicarComponentes(estado.componentes);
      if (estado.sensores) aplicarBateria(estado.sensores.bateria);
      if (estado.sistema) { temSistema = true; aplicarSistema(estado.sistema); }
    },
    destruir() {
      clearInterval(relogio);
      relogio = null;
    },
  };
}

// ======================================================== Sensores ao vivo
// Espelha o SensoresModule da GUI. A pergunta desta secao NAO e "a mensagem
// chega?" -- isso os faroes da secao anterior ja respondem -- e sim "o dado
// presta?". Um LiDAR pode publicar na frequencia certa e enxergar so ruido, e
// um sensor de inclinacao pode responder com a orientacao travada.
function criarSensores(ponte) {
  let scan = null;          // ultimo scan recebido (o quadro nao traz um em todo ciclo)
  let resumo = {};          // estado.sensores
  let visivel = false;
  let cameraLigada = false;
  let fontesAtuais = "";    // assinatura da lista, para nao reconstruir o seletor a toda hora

  // ------------------------------------------------------------------ LiDAR
  const visorLidar = el("canvas", {classe: "av-visor", estilo: "aspect-ratio:1/1;max-height:52vh"});
  const resumoLidar = el("div", {classe: "av-dica", texto: "Sem dados."});

  function desenharLidar() {
    const cv = visorLidar;
    const larguraCss = cv.clientWidth || 320;
    const alturaCss = cv.clientHeight || larguraCss;
    // Sem multiplicar pelo devicePixelRatio o desenho sai borrado em tela de
    // alta densidade (tablet). O contexto passa a trabalhar em pixels CSS.
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(larguraCss * dpr) || cv.height !== Math.round(alturaCss * dpr)) {
      cv.width = Math.round(larguraCss * dpr);
      cv.height = Math.round(alturaCss * dpr);
    }
    const p = cv.getContext("2d");
    p.setTransform(dpr, 0, 0, dpr, 0, 0);
    p.clearRect(0, 0, larguraCss, alturaCss);
    p.fillStyle = "#081120";
    p.fillRect(0, 0, larguraCss, alturaCss);

    const cx = larguraCss / 2;
    const cy = alturaCss / 2;
    const raio = Math.min(larguraCss, alturaCss) / 2 - 22;
    if (raio <= 10) return;

    const lidar = resumo.lidar || {};
    if (!scan || !scan.ranges || !scan.ranges.length) {
      p.fillStyle = "#7d9cc4";
      p.font = "13px 'Noto Sans', system-ui, sans-serif";
      p.textAlign = "center";
      p.fillText("Sem dados do LiDAR", cx, cy);
      return;
    }

    const alcance = Math.max(1, scan.range_max || 1);
    const velho = scan.velho || lidar.tem_dado === false;

    // Aneis de distancia com rotulo em metros: sem escala nao da para julgar se
    // o obstaculo esta a 30 cm ou a 3 m.
    const passoAnel = alcance > 6 ? 2 : 1;
    p.strokeStyle = "rgba(255,255,255,.16)";
    p.fillStyle = "#456b98";
    p.font = "10px 'Noto Sans', system-ui, sans-serif";
    p.textAlign = "center";
    p.lineWidth = 1;
    for (let m = passoAnel; m <= alcance; m += passoAnel) {
      const r = raio * (m / alcance);
      p.beginPath();
      p.arc(cx, cy, r, 0, Math.PI * 2);
      p.stroke();
      p.fillText(m + " m", cx, cy - r - 3);
    }

    // O SETOR CEGO em destaque. Este robo le so os 180 graus TRASEIROS: a frente
    // e cega. Desenhar o circulo inteiro daria a impressao de cobertura total, e
    // essa impressao e exatamente o que termina em colisao frontal -- por isso a
    // regiao sem visao aparece marcada, e nao escondida.
    const angMin = scan.angle_min || 0;
    const angMax = (scan.angle_max !== undefined) ? scan.angle_max : angMin + Math.PI * 2;
    const abertura = angMax - angMin;
    if (abertura < Math.PI * 2 - 0.05) {
      // O eixo Y da tela cresce para baixo, entao o angulo do robo entra
      // negado no canvas; o setor cego e o complemento do coberto.
      p.beginPath();
      p.moveTo(cx, cy);
      p.arc(cx, cy, raio, -angMax, -(angMin + Math.PI * 2), true);
      p.closePath();
      p.fillStyle = "rgba(235,87,87,.16)";
      p.fill();
      p.strokeStyle = "rgba(235,87,87,.45)";
      p.stroke();

      const meio = (angMax + angMin + Math.PI * 2) / 2;
      p.fillStyle = "#ff9d9d";
      p.font = "700 12px 'Noto Sans', system-ui, sans-serif";
      p.textAlign = "center";
      p.fillText("sem visão aqui",
        cx + Math.cos(meio) * raio * 0.58, cy - Math.sin(meio) * raio * 0.58);
    }

    // Pontos do scan. A lista chega decimada pela metade; o passo angular real
    // sai da razao entre o total original e o que veio, senao o desenho gira.
    const pontos = scan.ranges;
    const fator = (scan.total && pontos.length) ? (scan.total / pontos.length) : 2;
    const inc = (scan.angle_increment || 0) * fator;
    p.fillStyle = velho ? "rgba(53,195,240,.3)" : "#35c3f0";
    for (let i = 0; i < pontos.length; i++) {
      const d = pontos[i];
      if (d === null || d === undefined) continue;
      const ang = angMin + i * inc;
      const r = raio * (d / alcance);
      p.beginPath();
      p.arc(cx + Math.cos(ang) * r, cy - Math.sin(ang) * r, 1.6, 0, Math.PI * 2);
      p.fill();
    }

    // O robo no centro, apontando para a frente dele (+X, a direita da tela).
    p.fillStyle = "#f2994a";
    p.beginPath();
    p.moveTo(cx + 11, cy);
    p.lineTo(cx - 7, cy - 7);
    p.lineTo(cx - 7, cy + 7);
    p.closePath();
    p.fill();
    p.fillStyle = "#7d9cc4";
    p.font = "10px 'Noto Sans', system-ui, sans-serif";
    p.textAlign = "left";
    p.fillText("frente", cx + 16, cy + 4);
  }

  function atualizarResumoLidar() {
    const l = resumo.lidar || {};
    if (!l.tem_dado) {
      resumoLidar.textContent = "O LiDAR não está respondendo agora.";
      return;
    }
    resumoLidar.textContent =
      `${numero(l.hz, 1)} leituras por segundo   •   ${l.validos} de ${l.total} pontos `
      + `aproveitáveis   •   enxerga ${l.abertura_graus}° dos 360°   •   `
      + `alcance até ${numero(l.alcance_max, 1)} m`;
  }

  // -------------------------------------------------------------------- IMU
  const visorImu = el("canvas", {classe: "av-visor",
                                 estilo: "height:150px;max-width:100%"});
  const linhasImu = el("div");
  const resumoImu = el("div", {classe: "av-dica", texto: "Sem dados."});

  function desenharHorizonte() {
    const cv = visorImu;
    const larguraCss = cv.clientWidth || 280;
    const alturaCss = cv.clientHeight || 150;
    const dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(larguraCss * dpr) || cv.height !== Math.round(alturaCss * dpr)) {
      cv.width = Math.round(larguraCss * dpr);
      cv.height = Math.round(alturaCss * dpr);
    }
    const p = cv.getContext("2d");
    p.setTransform(dpr, 0, 0, dpr, 0, 0);
    p.clearRect(0, 0, larguraCss, alturaCss);

    const imu = resumo.imu || {};
    const cx = larguraCss / 2;
    const cy = alturaCss / 2;
    const raio = Math.min(larguraCss, alturaCss) / 2 - 6;

    p.save();
    p.beginPath();
    p.arc(cx, cy, raio, 0, Math.PI * 2);
    p.clip();

    if (!imu.tem_dado) {
      p.fillStyle = "#081120";
      p.fillRect(0, 0, larguraCss, alturaCss);
      p.restore();
      p.strokeStyle = "#1b3457";
      p.beginPath();
      p.arc(cx, cy, raio, 0, Math.PI * 2);
      p.stroke();
      p.fillStyle = "#7d9cc4";
      p.font = "12px 'Noto Sans', system-ui, sans-serif";
      p.textAlign = "center";
      p.fillText("sem leitura", cx, cy + 4);
      return;
    }

    const roll = (imu.roll || 0) * Math.PI / 180;
    // 2,4 px por grau: com o robo no chao a linha fica no meio e uma rampa de
    // 10 graus ja e visivel sem sair do circulo.
    const deslocamento = (imu.pitch || 0) * 2.4;

    p.translate(cx, cy);
    p.rotate(-roll);
    p.translate(0, deslocamento);
    p.fillStyle = "#14406b";
    p.fillRect(-raio * 2, -raio * 3, raio * 4, raio * 3);
    p.fillStyle = "#5a3d1b";
    p.fillRect(-raio * 2, 0, raio * 4, raio * 3);
    p.strokeStyle = "#e6f1ff";
    p.lineWidth = 2;
    p.beginPath();
    p.moveTo(-raio * 2, 0);
    p.lineTo(raio * 2, 0);
    p.stroke();
    p.restore();

    // Referencia fixa do robo, por cima do ceu e do chao.
    p.strokeStyle = "#ff9a2e";
    p.lineWidth = 2;
    p.beginPath();
    p.moveTo(cx - 26, cy);
    p.lineTo(cx - 8, cy);
    p.moveTo(cx + 8, cy);
    p.lineTo(cx + 26, cy);
    p.moveTo(cx, cy - 5);
    p.lineTo(cx, cy + 5);
    p.stroke();
    p.strokeStyle = "#1b3457";
    p.lineWidth = 1;
    p.beginPath();
    p.arc(cx, cy, raio, 0, Math.PI * 2);
    p.stroke();
  }

  function atualizarImu() {
    const imu = resumo.imu || {};
    linhasImu.textContent = "";
    if (!imu.tem_dado) {
      resumoImu.textContent = "O sensor de inclinação não está respondendo agora.";
      return;
    }
    resumoImu.textContent = `${numero(imu.hz, 1)} leituras por segundo`;
    [
      ["Inclinação lateral", numero(imu.roll, 1) + "°"],
      ["Inclinação frontal", numero(imu.pitch, 1) + "°"],
      ["Direção para onde aponta", numero(imu.yaw, 1) + "°"],
      // rad/s nao diz nada para quem opera; graus por segundo, sim.
      ["Girando sobre o próprio eixo", (typeof imu.giro_z === "number")
        ? numero(imu.giro_z * 180 / Math.PI, 0) + "°/s" : "—"],
      ["Aceleração (frente / lado / cima)",
        `${numero(imu.acel_x, 2)} / ${numero(imu.acel_y, 2)} / ${numero(imu.acel_z, 2)} m/s²`],
    ].forEach(([rot, val]) => {
      linhasImu.appendChild(el("div", {classe: "av-linha"}, [
        el("span", {classe: "rot", texto: rot}),
        el("span", {classe: "val", texto: val}),
      ]));
    });
  }

  // ----------------------------------------------------------------- camera
  // A camera NAO viaja no quadro de estado: ela sai em MJPEG, sob demanda, e
  // nasce DESLIGADA. Trinta quadros por segundo dentro do JSON tirariam da
  // navegacao a banda inteira da rede de competicao -- e ninguem olha a camera
  // o tempo todo.
  const seletorCamera = el("select", {classe: "av-sel"});
  const btCamera = el("button", {classe: "av-bt", texto: "Ligar a câmera"});
  const recadoCamera = el("div", {classe: "av-dica"});
  const imagemCamera = el("img", {
    classe: "av-imagem", alt: "Imagem ao vivo da câmera do robô", src: IMAGEM_VAZIA,
  });

  function desenharListaDeCameras(camera) {
    const fontes = camera.fontes || [];
    const assinatura = fontes.map((f) => f.id).join("|");
    if (assinatura !== fontesAtuais) {
      fontesAtuais = assinatura;
      const escolhida = seletorCamera.value;
      seletorCamera.textContent = "";
      fontes.forEach((fonte) => {
        const opcao = el("option", {value: fonte.id, texto: fonte.rotulo || fonte.id});
        seletorCamera.appendChild(opcao);
      });
      if (fontes.some((f) => f.id === escolhida)) seletorCamera.value = escolhida;
      else if (camera.ativa) seletorCamera.value = camera.ativa;
    }
    seletorCamera.disabled = !fontes.length || cameraLigada;
    btCamera.disabled = !fontes.length || !ponte.autorizado();
    btCamera.textContent = cameraLigada ? "Desligar a câmera" : "Ligar a câmera";

    if (!ponte.autorizado() && fontes.length) {
      recadoCamera.textContent =
        "Este painel está só para olhar: sem a chave de acesso não dá para "
        + "ligar a câmera. Peça o link completo a quem ligou o robô.";
    } else if (cameraLigada) {
      recadoCamera.textContent = camera.hz
        ? `${numero(camera.hz, 1)} imagens por segundo${camera.largura ?
            `   •   ${camera.largura}×${camera.altura}` : ""}`
        : "Ligando a câmera…";
    } else {
      recadoCamera.textContent = camera.mensagem
        || "A câmera fica desligada até alguém pedir: ligada, ela consome a rede "
           + "que a navegação usa.";
    }
  }

  async function alternarCamera() {
    if (cameraLigada) {
      desligarCamera();
      await ponte.comandar("/api/camera", {desligar: true});
      return;
    }
    const topico = seletorCamera.value;
    if (!topico) return;
    const resposta = await ponte.comandar("/api/camera", {topico});
    // A recusa do robo ja' vira torrada no cliente da API (app.js): repetir
    // aqui mostraria a mesma frase duas vezes, como se fossem dois problemas.
    if (resposta.ok === false) {
      recadoCamera.textContent = resposta.mensagem || "Não consegui ligar a câmera.";
      return;
    }
    cameraLigada = true;
    // Trocar o src e' o que ABRE a conexao do MJPEG. O carimbo de tempo evita o
    // navegador reaproveitar um stream que ja' morreu.
    imagemCamera.src = ponte.comChave("/api/camera.mjpeg") + "&_=" + Date.now();
    desenharListaDeCameras(resumo.camera || {});
  }

  function desligarCamera() {
    cameraLigada = false;
    // GIF vazio, e nao "": src vazio faz o navegador pedir a PAGINA de novo.
    imagemCamera.src = IMAGEM_VAZIA;
    desenharListaDeCameras(resumo.camera || {});
  }

  btCamera.addEventListener("click", alternarCamera);
  imagemCamera.addEventListener("error", () => {
    if (!cameraLigada) return;
    desligarCamera();
    recadoCamera.textContent =
      "A imagem da câmera parou de chegar. Ligue de novo para tentar outra vez.";
  });

  // ------------------------------------------------------------------ corpo
  const no = el("div", {classe: "av-secao"}, [
    el("div", {classe: "av-cartao"}, [
      el("h4", {texto: "Sensores ao vivo"}),
      el("div", {classe: "av-dica", texto:
        "Os cartões da tela anterior dizem se o sensor está respondendo. Aqui dá "
        + "para ver se o que ele responde presta: um sensor pode publicar na "
        + "hora certa e enxergar só ruído."}),
    ]),
    el("div", {classe: "av-duas"}, [
      el("div", {classe: "av-cartao"}, [
        el("h3", {texto: "LiDAR — o que o robô enxerga agora"}),
        visorLidar,
        resumoLidar,
      ]),
      el("div", {classe: "av-cartao"}, [
        el("h3", {texto: "Inclinação"}),
        visorImu,
        resumoImu,
        linhasImu,
      ]),
    ]),
    el("div", {classe: "av-cartao"}, [
      el("h3", {texto: "Câmera"}),
      el("div", {classe: "av-acoes"}, [seletorCamera, btCamera]),
      el("div", {estilo: "height:8px"}),
      imagemCamera,
      recadoCamera,
    ]),
  ]);

  // O desenho acompanha o QUADRO que chega (5 Hz para o LiDAR), e nao um
  // requestAnimationFrame a 60 Hz: redesenhar 1000 pontos sessenta vezes por
  // segundo esquenta o tablet sem mostrar nada de novo.
  function redesenhar() {
    if (!visivel) return;
    desenharLidar();
    desenharHorizonte();
  }

  const aoRedimensionar = () => redesenhar();

  return {
    no,
    aoEntrar() {
      visivel = true;
      window.addEventListener("resize", aoRedimensionar);
      redesenhar();
    },
    aoSair() {
      visivel = false;
      window.removeEventListener("resize", aoRedimensionar);
      // Sair da secao DESLIGA a camera: e' a mesma regra do hideEvent da GUI.
      // Camera ligada continua consumindo CPU do computador que roda a
      // navegacao, e ninguem esta olhando.
      if (cameraLigada) {
        desligarCamera();
        ponte.comandar("/api/camera", {desligar: true});
      }
    },
    aoEstado(estado) {
      if (estado.scan) scan = estado.scan;
      if (estado.sensores) {
        resumo = estado.sensores;
        atualizarResumoLidar();
        atualizarImu();
        const camera = resumo.camera || {};
        // O robo e' quem sabe se a assinatura caiu (ele larga a camera sozinho
        // depois de alguns segundos sem ninguem pedindo quadro).
        if (cameraLigada && camera.ligada === false) desligarCamera();
        desenharListaDeCameras(camera);
      }
      redesenhar();
    },
    destruir() {
      window.removeEventListener("resize", aoRedimensionar);
      if (cameraLigada) {
        desligarCamera();
        ponte.comandar("/api/camera", {desligar: true});
      }
    },
  };
}

// ======================================================== Service areas
// A lista de pontos da arena ATIVA, so' para conferir. Criar, mover e apagar
// ponto e' da tela de Mapeamento -- duas telas gravando o mesmo arquivo seria
// duas versoes da arena, e a que valesse dependeria de quem salvou por ultimo.
function criarAreas(ponte) {
  let arenaLida = "";
  let arenaAtual = "";
  let visivel = false;

  const chipArena = el("span", {classe: "av-chip", texto: "—"});
  const corpo = el("tbody");
  const recado = el("div", {classe: "av-dica"});
  const avisos = el("div", {classe: "av-motivo"});

  const tabela = el("table", {classe: "av-tabela"}, [
    el("thead", {}, [
      el("tr", {}, [
        el("th", {texto: "Ponto"}),
        el("th", {texto: "O que é"}),
        el("th", {texto: "Onde fica"}),
        el("th", {texto: "Situação"}),
      ]),
    ]),
    corpo,
  ]);

  // O contexto Avancado MOSTRA O MAPA na GUI (main_window.cpp, mostra_mapa
  // true). Ele entra aqui, na secao dos pontos, porque e' aqui que ele responde
  // alguma coisa: uma linha de tabela dizendo "x 3,20 m" nao diz se a mesa esta
  // no corredor certo; o desenho diz.
  const caixaDoMapa = el("div", {classe: "av-mapa"});
  let mapa = null;

  const no = el("div", {classe: "av-secao"}, [
    el("div", {classe: "av-cartao"}, [
      el("div", {classe: "av-topo"}, [
        el("h4", {texto: "Pontos da arena"}),
        chipArena,
      ]),
      el("div", {classe: "av-dica", texto:
        "São os lugares que o robô sabe alcançar nesta arena. Para criar, mover "
        + "ou apagar um ponto, use Mapeamento."}),
    ]),
    el("div", {classe: "av-duas"}, [
      caixaDoMapa,
      el("div", {classe: "av-cartao"}, [
        el("div", {classe: "av-rolagem"}, [tabela]),
        recado,
        avisos,
      ]),
    ]),
  ]);

  // Carregado sob demanda e sem derrubar a secao se falhar: sem o desenho, a
  // tabela continua respondendo o que ela responde.
  (async () => {
    try {
      const modulo = await import("../mapa_canvas.js");
      const fabrica = modulo.criarMapaCanvas || modulo.criarMapa || modulo.default;
      if (typeof fabrica !== "function") return;
      mapa = fabrica(caixaDoMapa, {
        // Modo de olhar: um toque nesta tela nao pode virar comando de
        // movimento. Quem manda o robo andar e' a tela de Operacao.
        aoClicarMarcador: (marcador) => destacarLinha(marcador && marcador.id),
      });
      if (arenaAtual && mapa.carregarArena) mapa.carregarArena(arenaAtual);
    } catch (erro) {
      console.error("o desenho do mapa nao subiu no Modo Avancado:", erro);
      caixaDoMapa.appendChild(el("div", {classe: "av-dica", estilo: "padding:14px",
        texto: "O desenho do mapa não carregou nesta versão do painel."}));
    }
  })();

  function destacarLinha(id) {
    [...corpo.children].forEach((linha) => {
      linha.dataset.destacada = (id && linha.dataset.ponto === id) ? "1" : "0";
    });
  }

  async function carregar() {
    if (!visivel || !arenaAtual || arenaAtual === arenaLida) return;
    arenaLida = arenaAtual;
    const meta = await ponte.obter(
      "/api/arenas/" + encodeURIComponent(arenaAtual) + "/meta");
    corpo.textContent = "";
    avisos.textContent = "";
    if (!meta || meta.erro) {
      recado.textContent = (meta && meta.erro) || "Não consegui ler esta arena.";
      return;
    }
    const juntos = new Map();
    (meta.areas || []).forEach((a) => juntos.set(a.id, {
      id: a.id, rotulo: a.tipo_rotulo || "Estação", sem_pose: !!a.sem_pose,
      x: a.x, y: a.y,
    }));
    (meta.docks || []).forEach((d) => {
      const ja = juntos.get(d.id);
      if (ja) { ja.sem_pose = ja.sem_pose || !!d.sem_pose; return; }
      juntos.set(d.id, {
        id: d.id, rotulo: d.tipo_rotulo || "Dock", sem_pose: !!d.sem_pose,
        x: d.x, y: d.y,
      });
    });
    const pontos = [...juntos.values()].sort((a, b) => a.id.localeCompare(b.id));

    pontos.forEach((p) => {
      const linha = el("tr", {"data-ponto": p.id}, [
        el("td", {}, [el("strong", {texto: p.id})]),
        el("td", {texto: p.rotulo}),
        el("td", {texto: p.sem_pose ? "—"
          : `x ${numero(p.x, 2)} m, y ${numero(p.y, 2)} m`}),
        el("td", {}, [el("span", {
          classe: p.sem_pose ? "av-motivo" : "av-dica",
          texto: p.sem_pose ? "sem posição gravada: a missão aborta ao chegar nele"
                            : "pronto",
        })]),
      ]);
      linha.addEventListener("click", () => {
        destacarLinha(p.id);
        if (mapa && mapa.destacar) mapa.destacar("", p.id);
      });
      corpo.appendChild(linha);
    });
    recado.textContent = pontos.length ? ""
      : "Esta arena ainda não tem nenhum ponto marcado.";
    (meta.avisos || []).forEach((aviso) => {
      avisos.appendChild(el("div", {texto: aviso}));
    });
  }

  return {
    no,
    aoEntrar() { visivel = true; carregar(); },
    aoSair() { visivel = false; },
    aoEstado(estado) {
      if (mapa && mapa.definirEstado && visivel) mapa.definirEstado(estado);
      const robo = (estado.cartoes || {}).robo || {};
      const nome = robo.arena_nome || "";
      if (nome === arenaAtual) return;
      arenaAtual = nome;
      chipArena.textContent = nome || "nenhuma arena escolhida";
      chipArena.style.background = nome ? "#35c3f0" : "#f2994a";
      arenaLida = "";
      if (mapa && mapa.carregarArena && nome) mapa.carregarArena(nome);
      carregar();
    },
    destruir() {
      visivel = false;
      if (mapa && mapa.destruir) mapa.destruir();
      mapa = null;
    },
  };
}

// ============================================================== Terminal
// Uma das duas coisas que so' a web tem. Quem esta na frente do robo tem o
// teclado dele; quem esta do outro lado do galpao com um tablet, nao.
//
// Nome de topico e de no' pode aparecer AQUI dentro -- e o unico lugar do
// painel onde eles sao o assunto, e nao ruido.
function criarTerminal(ponte) {
  let term = null;
  let ajuste = null;
  let socket = null;
  let observador = null;

  const caixa = el("div", {classe: "av-terminal"});
  const botao = el("button", {classe: "av-bt primaria", texto: "Abrir terminal"});
  const recado = el("div", {classe: "av-dica", texto:
    "Abre um terminal de verdade no computador do robô, com o ambiente já "
    + "pronto. Fecha sozinho quando esta tela sai."});

  const no = el("div", {classe: "av-secao"}, [
    el("div", {classe: "av-cartao"}, [
      el("div", {classe: "av-topo"}, [botao, el("span", {classe: "av-espaco"}), recado]),
    ]),
    el("div", {classe: "av-pilha"}, [caixa]),
  ]);

  function ajustarTamanho() {
    if (!ajuste || !term) return;
    try { ajuste.fit(); } catch (erro) { /* caixa ainda sem tamanho */ }
    if (socket && socket.readyState === 1) {
      socket.send(JSON.stringify({t: "tam", colunas: term.cols, linhas: term.rows}));
    }
  }

  function abrir() {
    if (socket) { socket.close(); return; }
    if (!ponte.autorizado()) {
      recado.textContent =
        "Sem a chave de acesso não dá para abrir o terminal. Peça o link "
        + "completo a quem ligou o robô.";
      return;
    }
    // A biblioteca esta versionada em vendor/ e vem por <script> comum, entao
    // ela e' global. Se faltar, isto explica em vez de estourar.
    if (typeof window.Terminal !== "function") {
      recado.textContent =
        "O terminal não está disponível nesta versão do painel.";
      return;
    }

    if (!term) {
      term = new window.Terminal({
        fontSize: 13, cursorBlink: true, scrollback: 4000,
        fontFamily: "'JetBrains Mono', Menlo, Consolas, 'DejaVu Sans Mono', monospace",
        theme: {background: "#000000", foreground: "#e6f1ff", cursor: "#35c3f0"},
      });
      if (window.FitAddon && window.FitAddon.FitAddon) {
        ajuste = new window.FitAddon.FitAddon();
        term.loadAddon(ajuste);
      }
      term.open(caixa);
      observador = new ResizeObserver(() => ajustarTamanho());
      observador.observe(caixa);
      term.onData((d) => {
        if (socket && socket.readyState === 1) {
          socket.send(JSON.stringify({t: "in", d}));
        }
      });
    }
    ajustarTamanho();

    const protocolo = location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(protocolo + "://" + location.host + ponte.comChave(
      `/ws/terminal?colunas=${term.cols}&linhas=${term.rows}`));
    // arraybuffer e nao blob: o xterm escreve Uint8Array direto, sem uma
    // leitura assincrona a cada pedaco de saida.
    socket.binaryType = "arraybuffer";
    socket.onopen = () => {
      botao.textContent = "Fechar terminal";
      recado.textContent = "Terminal aberto no computador do robô.";
      ajustarTamanho();
    };
    socket.onmessage = (evento) => {
      if (typeof evento.data === "string") {
        let pacote = {};
        try { pacote = JSON.parse(evento.data); } catch (erro) { pacote = {}; }
        if (pacote.texto) {
          term.writeln("\r\n\x1b[33m" + pacote.texto + "\x1b[0m");
          recado.textContent = pacote.texto;
        }
        return;
      }
      term.write(new Uint8Array(evento.data));
    };
    socket.onclose = () => {
      socket = null;
      botao.textContent = "Abrir terminal";
      recado.textContent = "Terminal fechado.";
    };
  }

  botao.addEventListener("click", abrir);

  function fechar() {
    if (socket) { socket.close(); socket = null; }
  }

  return {
    no,
    aoEntrar() { ajustarTamanho(); },
    // Sair da secao NAO mata o terminal: um comando longo (colcon build) tem
    // que sobreviver a uma olhada no mapa. Quem mata e' sair da tela.
    aoSair() {},
    aoEstado() {},
    destruir() {
      fechar();
      if (observador) { observador.disconnect(); observador = null; }
      if (term) { term.dispose(); term = null; }
    },
  };
}

// =========================================================== Tela do robo
// A outra coisa que so' a web tem: o espelho do desktop do computador do robo,
// para quem esta longe dele poder ler uma janela que ficou aberta.
function criarTelaDoRobo(ponte) {
  let ligado = false;
  const imagem = el("img", {classe: "av-imagem", alt: "Tela do computador do robô",
                            src: IMAGEM_VAZIA});
  const botao = el("button", {classe: "av-bt primaria", texto: "Mostrar a tela"});
  const tamanho = el("select", {classe: "av-sel"});
  [["1280", "grande (1280 px)"], ["960", "média (960 px)"], ["640", "pequena (640 px)"]]
    .forEach(([valor, rotulo]) => {
      tamanho.appendChild(el("option", {value: valor, texto: rotulo}));
    });
  tamanho.value = "960";
  const recado = el("div", {classe: "av-dica"});
  const emJanela = el("button", {classe: "av-bt", texto: "Abrir em outra janela"});

  const no = el("div", {classe: "av-secao"}, [
    el("div", {classe: "av-cartao"}, [
      el("div", {classe: "av-topo"}, [botao, tamanho, emJanela]),
      el("div", {estilo: "height:8px"}),
      recado,
    ]),
    el("div", {classe: "av-cartao"}, [imagem]),
  ]);

  function desligar() {
    ligado = false;
    imagem.src = IMAGEM_VAZIA;
    botao.textContent = "Mostrar a tela";
  }

  async function ligar() {
    if (ligado) { desligar(); return; }
    if (!ponte.autorizado()) {
      recado.textContent =
        "Sem a chave de acesso não dá para ver a tela do robô. Ela mostra a "
        + "máquina inteira, e não só o robô. Peça o link completo a quem ligou o robô.";
      return;
    }
    const situacao = await ponte.obter("/api/tela/estado");
    if (situacao && situacao.disponivel === false) {
      recado.textContent = situacao.mensagem
        || "Este robô não está com uma tela ligada para espelhar.";
      return;
    }
    ligado = true;
    botao.textContent = "Parar de mostrar";
    recado.textContent = "Espelhando a tela do computador do robô.";
    imagem.src = ponte.comChave("/api/tela.mjpeg")
      + "&largura=" + encodeURIComponent(tamanho.value) + "&_=" + Date.now();
  }

  botao.addEventListener("click", ligar);
  tamanho.addEventListener("change", () => { if (ligado) { desligar(); ligar(); } });
  emJanela.addEventListener("click", () => {
    window.open(ponte.comChave("/tela.html"), "_blank", "noopener");
  });
  imagem.addEventListener("error", () => {
    if (!ligado) return;
    desligar();
    recado.textContent =
      "A imagem da tela parou de chegar. Tente mostrar de novo.";
  });

  return {
    no,
    aoEntrar() {},
    // O espelho tem duas vagas so'. Sair da secao devolve a vaga na hora, em
    // vez de deixar uma aba esquecida segurando metade do que existe.
    aoSair() { if (ligado) desligar(); },
    aoEstado() {},
    destruir() { desligar(); },
  };
}

// ================================================================== a tela
// As secoes sao as mesmas que o robo anuncia em MENU_PRINCIPAL, contexto
// "avancado". Os titulos estao aqui em copia so' para a tela desenhar ANTES do
// primeiro quadro; quando o menu chega, ele manda.
const SECOES = [
  {id: "estado", titulo: "Estado do robô", criar: criarEstado},
  {id: "sensores", titulo: "Sensores ao vivo", criar: criarSensores},
  {id: "areas", titulo: "Service areas", criar: criarAreas},
  {id: "terminal", titulo: "Terminal", criar: criarTerminal},
  {id: "tela", titulo: "Tela do robô", criar: criarTelaDoRobo},
];

export const tela = {
  id: "avancado",
  titulo: "Modo Avancado",
  precisaDoMapa: false,
  montar(raiz, ctx) {
    garantirEstilo();
    const ponte = criarPonte(ctx);

    const trilho = el("nav", {classe: "av-trilho", "aria-label": "Seções do modo avançado"});
    const palco = el("div", {classe: "av-palco"});
    const casca = el("div", {classe: "av"}, [
      el("div", {classe: "av-corpo"}, [trilho, palco]),
    ]);
    raiz.appendChild(casca);

    const partes = SECOES.map((definicao) => {
      const parte = definicao.criar(ponte);
      parte.id = definicao.id;
      palco.appendChild(parte.no);
      const botao = el("button", {
        texto: definicao.titulo, type: "button", "aria-selected": "false",
      });
      botao.addEventListener("click", () => irParaSecao(definicao.id));
      trilho.appendChild(botao);
      parte.botao = botao;
      return parte;
    });

    let atual = "";
    let ultimoEstado = (ctx && ctx.estado) || {};

    function irParaSecao(id) {
      if (!partes.some((p) => p.id === id)) return;
      if (atual === id) return;
      partes.forEach((parte) => {
        const ligada = parte.id === id;
        if (!ligada && parte.id === atual && parte.aoSair) parte.aoSair();
        parte.no.classList.toggle("ativa", ligada);
        parte.botao.setAttribute("aria-selected", String(ligada));
      });
      atual = id;
      const parte = partes.find((p) => p.id === id);
      // A secao que estava escondida nao recebeu quadro nenhum: entrega o
      // ultimo antes de mostrar, senao ela abre vazia por ate um segundo.
      if (parte) {
        if (parte.aoEstado) { try { parte.aoEstado(ultimoEstado); } catch (e) { console.error(e); } }
        if (parte.aoEntrar) parte.aoEntrar();
      }
    }

    irParaSecao("estado");
    if (ctx && ctx.secao) irParaSecao(ctx.secao);

    return {
      irParaSecao,
      aoEstado(estado) {
        if (!estado) return;
        ultimoEstado = estado;
        partes.forEach((parte) => {
          if (!parte.aoEstado) return;
          try { parte.aoEstado(estado); } catch (erro) { console.error(erro); }
        });
      },
      destruir() {
        partes.forEach((parte) => {
          try { if (parte.aoSair) parte.aoSair(); } catch (erro) { console.error(erro); }
          try { if (parte.destruir) parte.destruir(); } catch (erro) { console.error(erro); }
        });
        raiz.textContent = "";
      },
    };
  },
};
