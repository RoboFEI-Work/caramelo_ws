// Tela Mapas do painel web -- o gemeo de caramelo_gui/src/modules/mapas.
//
// Mesma leitura da esquerda para a direita da GUI: ESCOLHER a arena, VER o que
// esta salvo dentro dela, AGIR sobre o que estiver errado.
//
// O preview nao e miniatura do desenho do mapa. Miniatura mostra as paredes e
// esconde justamente o que decide se a missao roda: dock sem posicao gravada,
// service area zerada, ausencia de parede virtual, onde fica o ponto de inicio.
// Por isso o desenho aqui e o mesmo do resto do painel (mapa_canvas.js), com as
// camadas por cima -- foi o operador que pediu isso com todas as letras.
//
// "Usar esta arena" grava a escolha no robo e ela sobrevive ao desligamento. O
// que esta tela NAO faz e trocar o mapa da navegacao que ja esta no ar: nao ha
// caminho para isso pelo painel. A frase na tela diz isso em portugues, em vez
// de deixar o operador achando que trocou tudo e descobrir na prova.

export const tela = {
  id: 'mapas',
  titulo: 'Mapas',
  // Sem o mapa da casca de proposito, igual a GUI (main_window.cpp): o mapa
  // desta tela e o da arena ESCOLHIDA, que quase nunca e a arena ativa com o
  // robo dentro. Dois mapas na mesma tela seria pergunta sem resposta ("qual
  // desses e o de verdade?").
  precisaDoMapa: false,
  montar,
};

// Rotulos iguais aos de mapas_module.cpp (rotuloDoTipo).
const ROTULO_DO_TIPO = {
  area: 'Service area (mesa)',
  dock: 'Dock (estacao)',
  waypoint: 'Waypoint',
};

// As quatro correcoes que o operador listou. Cada uma abre o Mapeamento na
// etapa certa, com a arena ja escolhida. Arena que JA EXISTE se edita em
// qualquer ordem: nada aqui exige comecar do primeiro passo -- travar a ordem
// obrigaria a remapear uma arena inteira para mover um unico ponto.
const AJUSTES = [
  {
    etapa: 'pontos', acao: 'adicionar',
    rotulo: 'Adicionar ou mover um ponto',
    ajuda: 'Marca uma estacao, um dock ou um ponto de passagem, ou leva para o '
         + 'lugar certo um que ficou torto. Nao precisa levar o robo ate la.',
  },
  {
    etapa: 'limpar', acao: 'limpar',
    rotulo: 'Limpar ruido do mapa',
    ajuda: 'Apaga do desenho o que nao e parede: gente que passou na frente do '
         + 'sensor, reflexo de vidro, sujeira de mapeamento.',
  },
  {
    etapa: 'paredes', acao: 'pintar',
    rotulo: 'Paredes virtuais',
    ajuda: 'Marca onde o robo nao pode entrar mesmo estando livre no mapa: fita '
         + 'no chao, degrau, area da outra equipe.',
  },
  {
    etapa: 'pontos', acao: 'remover',
    rotulo: 'Remover um ponto',
    ajuda: 'Tira da arena um ponto que nao existe mais. Nada e apagado sem '
         + 'confirmar, e o arquivo antigo fica guardado.',
  },
];

const escapar = (t) => String(t == null ? '' : t)
  .replace(/[&<>"]/g, (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));

// Ponte com o app.js. O roteador e o dono do acesso a API, das torradas e da
// navegacao entre telas; aqui so pedimos essas quatro coisas pelo nome mais
// provavel e caimos para o fetch cru se nada existir. Sem isto, uma tela
// carregada sozinha (ou um nome trocado na integracao) derruba o painel inteiro
// em vez de mostrar o mapa.
function ligarContexto(ctx) {
  const c = ctx || {};
  const api = c.api || c;
  const chave = c.chave || new URLSearchParams(location.search).get('t') || '';
  const comChave = (rota) =>
    chave ? rota + (rota.includes('?') ? '&' : '?') + 't=' + encodeURIComponent(chave) : rota;
  const primeira = (alvo, nomes) => nomes.map((n) => alvo && alvo[n])
    .find((f) => typeof f === 'function');

  return {
    estado: () => c.estado || {},
    async ler(rota) {
      const f = primeira(api, ['ler', 'get', 'buscar']);
      if (f) return f.call(api, rota);
      const r = await fetch(rota, {headers: {Accept: 'application/json'}});
      return r.json();
    },
    async comandar(rota, corpo) {
      const f = primeira(api, ['comandar', 'post', 'enviar']);
      if (f) return f.call(api, rota, corpo || {});
      const r = await fetch(comChave(rota), {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(corpo || {}),
      });
      const d = await r.json().catch(() => ({}));
      // Rota que nao existe responde 404 sem corpo: sem esta linha o "ok"
      // ficaria indefinido e a tela daria o comando por aceito.
      if (d.ok === undefined) d.ok = r.ok;
      if (d.mensagem) this.avisar(d.mensagem, r.ok && d.ok !== false);
      else if (!r.ok) this.avisar('O robo nao aceitou o pedido.', false);
      return d;
    },
    // O app sabe se a chave do link vale (ele pergunta em /api/acesso). Quando
    // ele nao disser nada, assume-se que da para comandar: um painel inteiro
    // desabilitado por engano e pior que um botao que volta com a recusa do
    // robo escrita.
    podeComandar() {
      const v = typeof c.autorizado === 'function' ? c.autorizado() : c.autorizado;
      return (v === undefined || v === null) ? true : !!v;
    },
    avisar(texto, ok) {
      const f = primeira(c, ['torrada', 'avisar', 'aviso', 'recado', 'notificar']);
      if (f) f.call(c, texto, ok !== false); else console.warn(texto);
    },
    irPara(idDaTela, opcoes) {
      const f = primeira(c, ['irPara', 'abrir', 'abrirTela', 'navegar', 'trocarTela']);
      if (f) { f.call(c, idDaTela, opcoes || {}); return true; }
      // Sem roteador a vista, o endereco ainda leva a tela certa (e o que a
      // tela inicial faz). O que se perde e' o atalho: a outra tela abre na
      // arena ativa, e nao na que estava selecionada aqui.
      location.hash = '#/' + idDaTela;
      return true;
    },
  };
}

// O desenho e de mapa_canvas.js, que tem dono proprio. Chamamos so o que ele
// expuser: metodo que nao existe nao pode derrubar a tela no meio da prova.
function chamar(alvo, nomes, ...args) {
  for (const nome of nomes) {
    if (alvo && typeof alvo[nome] === 'function') return alvo[nome](...args);
  }
  return undefined;
}

const MOLDE = `
<style>
/* So arranjo. As cores vem do tema; os poucos ajustes visuais daqui estao
   dentro de :where(), que tem peso ZERO -- se o tema.css definir a mesma
   classe, o tema ganha. Sem isso esta tela comecaria a divergir do resto do
   painel no dia em que alguem mexesse no tema. */
.tela-mapas { display: flex; flex-direction: column; height: 100%; min-height: 0; gap: 12px; }
.tela-mapas .mapas-grade {
  display: grid;
  grid-template-columns: 264px minmax(0, 1fr) 344px;
  gap: 12px;
  flex: 1 1 auto;
  min-height: 0;
}
.tela-mapas .mapas-col {
  display: flex; flex-direction: column; gap: 12px;
  min-width: 0; min-height: 0;
  overflow-y: auto; overflow-x: hidden;
  padding: 14px;
  background: var(--painel, #101e35);
  border: 1px solid var(--borda, #1b3457);
  border-radius: 16px;
}
.tela-mapas .mapas-mapa {
  flex: 1 1 auto; min-height: 320px;
  position: relative; overflow: hidden;
  border-radius: 12px;
  background: var(--fundo, #0b1626);
  border: 1px solid var(--borda, #1b3457);
}
.tela-mapas .mapas-mapa > * { width: 100%; height: 100%; display: block; }
.tela-mapas .mapas-vazio {
  display: flex; align-items: center; justify-content: center;
  height: 100%; padding: 24px; text-align: center;
  color: var(--texto-fraco, #7d9cc4);
}
.tela-mapas .mapas-linha-arena {
  display: flex; align-items: center; justify-content: space-between; gap: 8px;
  width: 100%; text-align: left;
  padding: 10px 12px; border-radius: 12px;
  border: 1px solid var(--borda, #1b3457);
  background: var(--fundo, #0b1626);
  color: var(--texto, #e6f1ff);
  font: inherit; font-weight: 600; cursor: pointer;
}
.tela-mapas .mapas-linha-arena:hover { border-color: var(--ciano, #35c3f0); }
.tela-mapas .mapas-linha-arena[aria-pressed="true"] {
  background: #103154; border-color: var(--ciano, #35c3f0); color: var(--ciano, #35c3f0);
}
.tela-mapas .mapas-rolagem {
  max-height: 260px; overflow-y: auto; overflow-x: hidden;
  display: flex; flex-direction: column; gap: 6px;
}
.tela-mapas .mapas-item {
  width: 100%; text-align: left; font: inherit;
  padding: 8px 10px; border-radius: 10px;
  border: 1px solid transparent; background: transparent;
  color: var(--texto, #e6f1ff); cursor: pointer;
}
.tela-mapas .mapas-item:hover { background: #142643; }
.tela-mapas .mapas-item[aria-pressed="true"] { background: #103154; border-color: var(--ciano, #35c3f0); }
.tela-mapas .mapas-item.sem-pose { color: #ffd28f; }
.tela-mapas .mapas-camadas { display: flex; flex-wrap: wrap; gap: 6px 16px; align-items: center; }
.tela-mapas .mapas-camadas label { display: flex; align-items: center; gap: 6px; white-space: nowrap; }
.tela-mapas .mapas-ajuste { display: flex; flex-direction: column; gap: 4px; }
.tela-mapas .mapas-resumo { display: flex; flex-direction: column; gap: 4px; }
.tela-mapas .mapas-resumo div { display: flex; justify-content: space-between; gap: 10px; }
:where(.tela-mapas) .mapas-secao { display: flex; flex-direction: column; gap: 8px; }
:where(.tela-mapas) h2 { font-size: 21px; font-weight: 800; color: #fff; margin: 0; }
:where(.tela-mapas) h3 { font-size: 15px; font-weight: 700; color: #dcecff; margin: 0; }
:where(.tela-mapas) .msgCartao { color: #8fb0d8; font-size: 13px; margin: 0; }
:where(.tela-mapas) .motivoCartao { color: #ffd28f; font-size: 12px; margin: 0; }
:where(.tela-mapas) .dica { color: #7d9cc4; font-size: 12px; margin: 0; }
:where(.tela-mapas) .valor { color: #cfe4fb; font-weight: 700; }
:where(.tela-mapas) button {
  background: #142643; color: #dcecff;
  border: 1px solid #1f3a5f; border-radius: 12px;
  padding: 10px 14px; font: inherit; font-weight: 600; cursor: pointer;
}
:where(.tela-mapas) button:hover:not(:disabled) { border-color: #35c3f0; color: #fff; }
:where(.tela-mapas) button:disabled { color: #3d5a82; border-color: #14294a; cursor: not-allowed; }
:where(.tela-mapas) .acaoPrimaria {
  background: #ff9a2e; color: #1a0e00; border: none; font-weight: 800;
}
:where(.tela-mapas) .acaoPrimaria:hover:not(:disabled) { background: #f07f0e; }
:where(.tela-mapas) .chip {
  font-size: 11px; font-weight: 800; border-radius: 8px;
  padding: 3px 8px; color: #06121f; background: #35c3f0;
}
/* Telas menores (o painel roda de 1024 a 2560 px, e tambem em tablet): as tres
   colunas viram duas e depois uma. A rolagem passa para a tela inteira, nunca
   para os lados -- barra horizontal em pagina de robo esconde botao. */
@media (max-width: 1440px) {
  .tela-mapas .mapas-grade {
    grid-template-columns: 244px minmax(0, 1fr);
    grid-template-rows: minmax(320px, 1fr) auto;
  }
  .tela-mapas [data-col="detalhe"] { grid-column: 1 / -1; max-height: 42vh; }
}
@media (max-width: 1024px) {
  .tela-mapas { overflow-y: auto; }
  .tela-mapas .mapas-grade { grid-template-columns: minmax(0, 1fr); grid-template-rows: none; }
  .tela-mapas .mapas-col { overflow: visible; max-height: none; }
  .tela-mapas [data-col="detalhe"] { max-height: none; }
  .tela-mapas .mapas-mapa { min-height: 60vh; }
}
</style>

<div class="tela-mapas">
  <div class="mapas-grade">

    <aside class="mapas-col" data-col="arenas">
      <div class="mapas-secao">
        <h2>Arenas</h2>
        <p class="msgCartao" data-el="arenaAtiva">Arena ativa no robo: —</p>
      </div>
      <div class="mapas-rolagem" data-el="lista" aria-label="Arenas salvas neste robo"></div>
      <button class="acaoPrimaria" data-el="usar" disabled>Usar esta arena</button>
      <p class="motivoCartao" data-el="motivoUsar"></p>
      <p class="dica">A escolha fica gravada no robo e continua valendo depois de
        desligar. A navegacao que ja esta ligada segue com a arena anterior ate
        ser ligada de novo.</p>
      <button data-el="atualizar">Atualizar lista</button>
    </aside>

    <section class="mapas-col" data-col="mapa">
      <div class="mapas-secao">
        <h3 data-el="tituloMapa">Mapa da arena</h3>
        <p class="msgCartao" data-el="estadoDaArena">Escolha uma arena na lista.</p>
      </div>
      <div class="mapas-mapa" data-el="mapa">
        <div class="mapas-vazio" data-el="mapaVazio">Escolha uma arena para ver o mapa dela.</div>
      </div>
      <div class="mapas-camadas" data-el="camadas"></div>
      <div style="display:flex; gap:8px; flex-wrap:wrap;">
        <button data-el="enquadrar">Enquadrar arena</button>
      </div>
      <p class="dica">Arraste para mover a vista e use a roda do mouse (ou dois
        dedos) para aproximar. Toque num ponto para saber qual e.</p>
    </section>

    <aside class="mapas-col" data-col="detalhe">
      <div class="mapas-secao">
        <h3>O que esta salvo nesta arena</h3>
        <div class="mapas-rolagem" data-el="conteudo"></div>
      </div>
      <div class="mapas-secao">
        <h3>Resumo</h3>
        <div class="mapas-resumo" data-el="resumo"></div>
      </div>
      <div class="mapas-secao">
        <h3>Atencao</h3>
        <div data-el="avisos"></div>
      </div>
      <div class="mapas-secao">
        <h3>Ajustar esta arena</h3>
        <p class="msgCartao">Cada botao abre o Mapeamento ja nesta arena. Arena
          que ja existe pode ser corrigida em qualquer ordem.</p>
        <div class="mapas-secao" data-el="ajustes"></div>
      </div>
    </aside>

  </div>
</div>`;

function montar(raiz, ctx) {
  const app = ligarContexto(ctx);
  raiz.innerHTML = MOLDE;
  const el = (nome) => raiz.querySelector(`[data-el="${nome}"]`);

  let arenas = [];
  let ativa = '';
  let escolhida = '';
  let meta = null;
  let mapa = null;              // instancia de mapa_canvas.js
  let camadasDesligadas = new Set();
  let selecionado = null;       // {tipo, id} destacado no desenho
  let ocupado = false;          // ha missao em andamento
  let vivo = true;

  // --------------------------------------------------------------- desenho
  // Carregado sob demanda: se o desenho nao subir, a tela continua servindo
  // para trocar de arena e ler os avisos, em vez de ficar em branco.
  (async () => {
    try {
      const modulo = await import('../mapa_canvas.js');
      if (!vivo) return;
      const fabrica = ['criarMapaCanvas', 'criarMapa', 'montarMapa', 'criar', 'default']
        .map((n) => modulo[n]).find((f) => typeof f === 'function');
      if (!fabrica) throw new Error('sem fabrica');
      mapa = fabrica(el('mapa'), {
        // Modo de olhar (o padrao do mapa_canvas): nesta tela um toque errado
        // nao pode virar comando de movimento -- a arena mostrada aqui quase
        // nunca e a arena em que o robo esta.
        aoClicarMarcador: (m) => { if (m) destacar(m.tipo, m.id, true); },
      });
      const vazio = el('mapaVazio');
      if (vazio) vazio.remove();
      if (meta) aplicarMapa();
    } catch (erro) {
      // Sair da tela enquanto o desenho carregava nao e falha: a raiz ja foi
      // esvaziada e nao ha onde escrever recado nenhum.
      const vazio = vivo && el('mapaVazio');
      if (vazio) {
        vazio.textContent =
          'O desenho do mapa nao carregou nesta versao do painel. O resto da tela '
          + 'continua funcionando: da para trocar a arena e ver o que falta nela.';
      }
    }
  })();

  // Os metadados vao junto: o mapa_canvas busca sozinho quando nao recebe, e
  // pedir a mesma arena duas vezes so gasta rede do robo. Ele mesmo enquadra
  // quando a imagem termina de carregar.
  async function aplicarMapa() {
    if (!mapa || !meta || meta.erro) return;
    await chamar(mapa, ['carregarArena', 'definirArena', 'carregar'], escolhida, meta);
    if (!vivo) return;
    camadasDesligadas.forEach(
      (c) => chamar(mapa, ['definirCamada', 'setCamada', 'camada'], c, false));
    desenharCamadas();
  }

  // ---------------------------------------------------------------- arenas
  async function carregarArenas() {
    let dados = {};
    try {
      dados = await app.ler('/api/arenas');
    } catch {
      app.avisar('Nao consegui pedir a lista de arenas ao robo.', false);
      return;
    }
    if (!vivo) return;
    arenas = dados.arenas || [];
    ativa = dados.ativa || '';
    if (!escolhida || !arenas.includes(escolhida)) {
      escolhida = arenas.includes(ativa) ? ativa : (arenas[0] || '');
    }
    desenharListaDeArenas();
    if (escolhida) carregarMeta(escolhida);
    else limparDetalhe('Este robo ainda nao tem nenhuma arena salva. Crie uma em Mapeamento.');
  }

  function desenharListaDeArenas() {
    const lista = el('lista');
    if (!arenas.length) {
      lista.innerHTML = '<p class="msgCartao">Nenhuma arena salva neste robo.</p>';
    } else {
      lista.innerHTML = arenas.map((nome) => `
        <button class="mapas-linha-arena" data-arena="${escapar(nome)}"
                aria-pressed="${nome === escolhida}">
          <span>${escapar(nome)}</span>
          ${nome === ativa ? '<span class="chip">EM USO</span>' : ''}
        </button>`).join('');
      lista.querySelectorAll('[data-arena]').forEach((bt) => {
        bt.onclick = () => {
          escolhida = bt.dataset.arena;
          selecionado = null;
          desenharListaDeArenas();
          carregarMeta(escolhida);
        };
      });
    }
    el('arenaAtiva').innerHTML = ativa
      ? `Arena ativa no robo: <span class="valor">${escapar(ativa)}</span>`
      : 'Arena ativa no robo: <span class="valor">nenhuma escolhida</span>';
    atualizarDisponibilidade();
  }

  async function carregarMeta(nome) {
    el('tituloMapa').textContent = `Mapa da arena ${nome}`;
    el('estadoDaArena').textContent = 'Lendo a arena...';
    let dados;
    try {
      dados = await app.ler(`/api/arenas/${encodeURIComponent(nome)}/meta`);
    } catch {
      dados = {erro: 'Nao consegui ler esta arena no robo.'};
    }
    if (!vivo || nome !== escolhida) return;   // o operador ja trocou de arena
    meta = dados;
    if (meta.erro) { limparDetalhe(meta.erro); return; }
    el('estadoDaArena').textContent = meta.pronta_para_missao
      ? 'Esta arena esta pronta para rodar uma prova.'
      : (meta.motivo_nao_pronta || 'Esta arena ainda tem pendencias.');
    el('estadoDaArena').className = meta.pronta_para_missao ? 'msgCartao' : 'motivoCartao';
    desenharCamadas();
    desenharConteudo();
    desenharResumo();
    desenharAvisos();
    aplicarMapa();
    atualizarDisponibilidade();
  }

  function limparDetalhe(mensagem) {
    meta = null;
    el('estadoDaArena').className = 'motivoCartao';
    el('estadoDaArena').textContent = mensagem;
    el('conteudo').innerHTML = '';
    el('resumo').innerHTML = '';
    el('avisos').innerHTML = '';
    el('camadas').innerHTML = '';
    atualizarDisponibilidade();
  }

  // --------------------------------------------------------------- camadas
  // A lista vem do proprio desenho quando ele ja subiu (ela inclui as camadas
  // ao vivo, que os metadados da arena nao conhecem); antes disso, dos
  // metadados. O que o operador desligou vale para a arena seguinte tambem:
  // religar seis interruptores a cada troca de arena e trabalho a toa.
  function desenharCamadas() {
    const camadas = (mapa && typeof mapa.camadas === 'function' && mapa.camadas())
      || (meta && meta.camadas) || [];
    el('camadas').innerHTML = camadas.map((c) => `
      <label><input type="checkbox" data-camada="${escapar(c.id)}"
        ${camadasDesligadas.has(c.id) ? '' : 'checked'}> ${escapar(c.rotulo)}</label>`).join('');
    el('camadas').querySelectorAll('[data-camada]').forEach((cx) => {
      cx.onchange = () => {
        const id = cx.dataset.camada;
        if (cx.checked) camadasDesligadas.delete(id); else camadasDesligadas.add(id);
        chamar(mapa, ['definirCamada', 'setCamada', 'camada'], id, cx.checked);
      };
    });
  }

  // -------------------------------------------------------------- conteudo
  // Mesma ordem e mesmos rotulos de mapas_module.cpp (listarConteudo): areas,
  // docks e waypoints, com "(sem posicao gravada)" em amarelo. Ponto na origem
  // do mapa nao e detalhe: mandar o robo para ele o faz atravessar a arena ate
  // o canto errado.
  function desenharConteudo() {
    const itens = [];
    (meta.areas || []).forEach((a) => itens.push({
      tipo: 'area', id: a.id, detalhe: a.tipo_rotulo || '', semPose: a.sem_pose,
    }));
    (meta.docks || []).forEach((d) => itens.push({
      tipo: 'dock', id: d.id, detalhe: d.tipo_rotulo || '', semPose: d.sem_pose,
    }));
    (meta.waypoints || []).forEach((w) => itens.push({
      tipo: 'waypoint', id: w.id, detalhe: '', semPose: w.sem_pose,
    }));

    const alvo = el('conteudo');
    if (!itens.length) {
      alvo.innerHTML = '<p class="msgCartao">Esta arena ainda nao tem nenhum ponto marcado.</p>';
      return;
    }
    alvo.innerHTML = itens.map((i) => `
      <button class="mapas-item${i.semPose ? ' sem-pose' : ''}"
              data-tipo="${escapar(i.tipo)}" data-id="${escapar(i.id)}"
              aria-pressed="${!!(selecionado && selecionado.tipo === i.tipo && selecionado.id === i.id)}">
        ${escapar(ROTULO_DO_TIPO[i.tipo])} &nbsp; <b>${escapar(i.id)}</b>${
          i.detalhe ? ' &nbsp;—&nbsp; ' + escapar(i.detalhe) : ''}${
          i.semPose ? ' &nbsp;(sem posicao gravada)' : ''}
      </button>`).join('');
    alvo.querySelectorAll('[data-tipo]').forEach((bt) => {
      bt.onclick = () => destacar(bt.dataset.tipo, bt.dataset.id, false);
    });
  }

  // Tocar no ponto do mapa marca a linha da lista, e vice-versa: sao as duas
  // metades da mesma coisa, e reencontrar o nome numa lista de vinte itens
  // depois de achar o ponto no mapa e trabalho a toa.
  function destacar(tipo, id, veioDoMapa) {
    selecionado = {tipo, id};
    el('conteudo').querySelectorAll('[data-tipo]').forEach((bt) => {
      const meu = bt.dataset.tipo === tipo && bt.dataset.id === id;
      bt.setAttribute('aria-pressed', String(meu));
      if (meu && veioDoMapa) bt.scrollIntoView({block: 'nearest'});
    });
    if (!veioDoMapa) chamar(mapa, ['destacar', 'selecionar'], tipo, id);
  }

  function desenharResumo() {
    const largura = (meta.largura_px || 0) * (meta.resolucao || 0);
    const altura = (meta.altura_px || 0) * (meta.resolucao || 0);
    const comPose = (meta.docks || []).filter((d) => !d.sem_pose).length;
    const linha = (r, v) => `<div><span class="msgCartao">${r}</span><span class="valor">${escapar(v)}</span></div>`;
    el('resumo').innerHTML =
      linha('Tamanho', `${largura.toFixed(1)} x ${altura.toFixed(1)} m`) +
      linha('Detalhe do mapa', `${((meta.resolucao || 0) * 100).toFixed(1)} cm por quadradinho`) +
      linha('Docks', `${(meta.docks || []).length} (${comPose} com posicao)`) +
      linha('Service areas', String((meta.areas || []).length)) +
      linha('Waypoints', String((meta.waypoints || []).length)) +
      linha('Paredes virtuais', meta.tem_keepout ? 'sim' : 'nao');
  }

  function desenharAvisos() {
    const avisos = meta.avisos || [];
    el('avisos').innerHTML = avisos.length
      ? avisos.map((a) => `<p class="motivoCartao">• ${escapar(a)}</p>`).join('')
      : '<p class="msgCartao">Nada a apontar nesta arena.</p>';
  }

  // --------------------------------------------------------------- ajustes
  function desenharAjustes() {
    el('ajustes').innerHTML = AJUSTES.map((a, i) => `
      <div class="mapas-ajuste">
        <button data-ajuste="${i}">${escapar(a.rotulo)}</button>
        <p class="dica">${escapar(a.ajuda)}</p>
      </div>`).join('');
    el('ajustes').querySelectorAll('[data-ajuste]').forEach((bt) => {
      bt.onclick = () => {
        const ajuste = AJUSTES[Number(bt.dataset.ajuste)];
        if (!escolhida) {
          app.avisar('Escolha primeiro a arena que voce quer ajustar.', false);
          return;
        }
        const foi = app.irPara('mapeamento', {
          arena: escolhida, etapa: ajuste.etapa, acao: ajuste.acao, livre: true,
        });
        if (!foi) {
          app.avisar(`Abra Mapeamento no menu principal para ${ajuste.rotulo.toLowerCase()}.`,
                     false);
        }
      };
    });
  }

  // ------------------------------------------------------- disponibilidade
  // Botao cinza sem frase le como interface quebrada; aqui o motivo aparece
  // sempre ao lado (mesma regra da GUI, atualizarDisponibilidade).
  function atualizarDisponibilidade() {
    let motivo = '';
    if (!app.podeComandar()) {
      motivo = 'Este painel foi aberto sem a chave de acesso: da para acompanhar, '
             + 'mas nao para comandar o robo. Peca o link completo a quem ligou o robo.';
    } else if (!escolhida) {
      motivo = 'Escolha uma arena na lista.';
    } else if (escolhida === ativa) {
      motivo = 'O robo ja esta nesta arena.';
    } else if (ocupado) {
      motivo = 'Ha uma missao em andamento: espere ela terminar para trocar de arena.';
    } else if (meta && meta.erro) {
      motivo = meta.erro;
    }
    el('usar').disabled = !!motivo;
    el('motivoUsar').textContent = motivo;
  }

  async function usarArena() {
    if (!escolhida) return;
    // As duas chaves de proposito: a rota de troca de arena esta sendo escrita
    // em paralelo, e mandar so uma seria uma tela que nao troca nada sem dizer
    // por que. O servidor ignora a que nao usa.
    const r = await app.comandar('/api/arena_ativa', {arena: escolhida, nome: escolhida});
    if (!vivo || (r && r.ok === false)) return;
    ativa = escolhida;
    desenharListaDeArenas();
  }

  // ---------------------------------------------------------------- ligacao
  el('usar').onclick = usarArena;
  el('atualizar').onclick = carregarArenas;
  el('enquadrar').onclick = () => chamar(mapa, ['enquadrar', 'ajustar']);
  desenharAjustes();
  carregarArenas();

  return {
    destruir() {
      vivo = false;
      chamar(mapa, ['destruir', 'encerrar', 'parar']);
      mapa = null;
      raiz.innerHTML = '';
    },

    // Chamado a cada quadro do WebSocket (10 Hz). Aqui so entra o que MUDOU:
    // redesenhar a lista dez vezes por segundo num tablet come a bateria e faz
    // o toque falhar bem na hora em que o operador esta com pressa.
    aoEstado(estado) {
      const cartoes = (estado && estado.cartoes) || {};
      const robo = cartoes.robo || {};
      const nomeAtivo = robo.arena_nome || '';
      if (nomeAtivo !== ativa) {
        // A arena tambem muda pela GUI do robo e pelo terminal. Quando isso
        // acontece a lista aqui tem que acompanhar, senao duas telas do mesmo
        // robo passam a discordar sobre onde ele esta.
        ativa = nomeAtivo;
        desenharListaDeArenas();
      }
      const agoraOcupado = !!((estado && estado.capacidades) || {}).missao_ocupada;
      if (agoraOcupado !== ocupado) {
        ocupado = agoraOcupado;
        atualizarDisponibilidade();
      }
      // O robo so aparece desenhado quando a arena escolhida E a arena dele.
      // Desenhar a pose sobre outro mapa poria o robo num lugar que nao existe
      // -- e alguem acreditaria no desenho.
      if (mapa) chamar(mapa, ['definirEstado', 'aoEstado', 'atualizar'],
                       escolhida && escolhida === ativa ? estado : {...estado, pose: null, scan: null});
    },
  };
}
