// Tela Competicao do painel web -- o gemeo de
// caramelo_gui/src/modules/competicao/competicao_module.cpp.
//
// Mesma sequencia da GUI: escolher a prova, dizer como rodar, ver o que ainda
// impede, MONTAR O PLANO sem mover o robo, ler o plano passo a passo e so
// entao comecar. Abortar fica sempre a vista: procurar o botao de parada com o
// robo andando e o pior momento possivel.
//
// Sem mapa nesta tela, igual a GUI (main_window.cpp): aqui o que importa e em
// que passo a missao esta, e o mapa ao lado so disputaria a atencao.
//
// Duas coisas que esta tela NAO promete:
//   - "Abortar" para a missao, nao e botao de emergencia: este robo nao tem um.
//   - o plano mostrado e o plano que roda. Comecar sem plano montado faria o
//     executor planejar de novo, e o que rodaria poderia nao ser o que o
//     operador acabou de ler na tela.

export const tela = {
  id: 'competicao',
  titulo: 'Competicao',
  precisaDoMapa: false,
  montar,
};

// action_kind do caramelo_msgs/MissionStatus (goto | pick | place | home) em
// linguagem de quem opera. Passo com nome tecnico na tela e passo que ninguem
// consegue conferir.
const ACAO = {
  goto: (alvo) => (alvo ? `Ir ate ${alvo}` : 'Ir ate o proximo ponto'),
  pick: (alvo) => (alvo ? `Pegar o objeto em ${alvo}` : 'Pegar o objeto'),
  place: (alvo) => (alvo ? `Deixar o objeto em ${alvo}` : 'Deixar o objeto'),
  home: () => 'Recolher o braco',
};

// stage do MissionStatus. O que nao estiver aqui simplesmente nao aparece: e
// melhor nao dizer nada do que escrever "align" na tela de quem nao sabe ROS.
const SUBESTAGIO = {
  iniciado: 'comecou',
  concluido: 'terminou',
  falhou: 'falhou',
  staging: 'chegando perto',
  align: 'alinhando com a estacao',
  undock: 'saindo da estacao',
  nav: 'indo ate la',
};

const ESTADO_DA_MISSAO = {
  'parado': {rotulo: 'Parada', cor: '#7f93b0'},
  'planejando': {rotulo: 'Montando o plano', cor: '#35c3f0'},
  'pre-flight': {rotulo: 'Conferindo o robo', cor: '#35c3f0'},
  'executando': {rotulo: 'Em andamento', cor: '#27ae60'},
  'concluida': {rotulo: 'Concluida', cor: '#27ae60'},
  'falhou': {rotulo: 'Encerrada por falha', cor: '#eb5757'},
  'abortada': {rotulo: 'Abortada', cor: '#f2994a'},
};

const TERMINOU = new Set(['concluida', 'falhou', 'abortada']);

const escapar = (t) => String(t == null ? '' : t)
  .replace(/[&<>"]/g, (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));

function relogio(segundos) {
  const s = Math.max(0, Math.round(Number(segundos) || 0));
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

// Ponte com o app.js -- mesma da tela Mapas. O roteador e o dono do acesso a
// API, das torradas e da troca de tela; aqui so pedimos essas quatro coisas
// pelo nome mais provavel, e caimos para o fetch cru se nada existir. Uma tela
// que estoura no import derruba o painel inteiro, e isso nao pode acontecer
// justamente na tela da prova.
function ligarContexto(ctx) {
  const c = ctx || {};
  const api = c.api || c;
  const chave = c.chave || new URLSearchParams(location.search).get('t') || '';
  const comChave = (rota) =>
    chave ? rota + (rota.includes('?') ? '&' : '?') + 't=' + encodeURIComponent(chave) : rota;
  const primeira = (alvo, nomes) => nomes.map((n) => alvo && alvo[n])
    .find((f) => typeof f === 'function');

  return {
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

const MOLDE = `
<style>
/* So arranjo. Cor e tipografia vem do tema; o pouco que ha aqui esta dentro de
   :where(), que tem peso ZERO -- se o tema.css definir a mesma classe, o tema
   ganha, e esta tela nao comeca a divergir do resto do painel. */
.tela-competicao { display: flex; flex-direction: column; height: 100%; min-height: 0; }
.tela-competicao .comp-grade {
  display: grid;
  grid-template-columns: minmax(320px, 380px) minmax(0, 1fr);
  gap: 12px; flex: 1 1 auto; min-height: 0;
}
.tela-competicao .comp-col {
  display: flex; flex-direction: column; gap: 14px;
  min-width: 0; min-height: 0; overflow-y: auto; overflow-x: hidden;
  padding: 14px;
  background: var(--painel, #101e35);
  border: 1px solid var(--borda, #1b3457);
  border-radius: 16px;
}
.tela-competicao .comp-rolagem {
  max-height: 240px; overflow-y: auto; overflow-x: hidden;
  display: flex; flex-direction: column; gap: 6px;
}
.tela-competicao .comp-prova {
  width: 100%; text-align: left; font: inherit;
  display: flex; flex-direction: column; gap: 2px;
  padding: 10px 12px; border-radius: 12px;
  border: 1px solid var(--borda, #1b3457);
  background: var(--fundo, #0b1626);
  color: var(--texto, #e6f1ff); cursor: pointer;
}
.tela-competicao .comp-prova:hover { border-color: var(--ciano, #35c3f0); }
.tela-competicao .comp-prova[aria-pressed="true"] {
  background: #103154; border-color: var(--ciano, #35c3f0);
}
.tela-competicao .comp-opcoes { display: flex; flex-direction: column; gap: 10px; }
.tela-competicao .comp-opcoes label { display: flex; gap: 8px; align-items: flex-start; }
.tela-competicao .comp-passos {
  flex: 1 1 auto; min-height: 160px; overflow-y: auto; overflow-x: hidden;
  display: flex; flex-direction: column; gap: 4px;
  padding: 8px; border-radius: 12px;
  background: var(--fundo, #0b1626);
  border: 1px solid var(--borda, #1b3457);
}
.tela-competicao .comp-passo {
  display: flex; gap: 10px; align-items: baseline;
  padding: 7px 8px; border-radius: 8px;
}
.tela-competicao .comp-passo[data-marca="agora"] { background: #103154; }
.tela-competicao .comp-passo .comp-numero {
  min-width: 26px; color: var(--texto-fraco, #7d9cc4); font-weight: 700;
}
.tela-competicao .comp-barra {
  height: 10px; border-radius: 5px; background: #14294a; overflow: hidden;
}
.tela-competicao .comp-barra > div {
  height: 100%; width: 0%; background: var(--ciano, #35c3f0); transition: width .2s linear;
}
.tela-competicao .comp-execucao {
  display: flex; flex-direction: column; gap: 10px;
  padding-top: 12px; border-top: 1px solid var(--borda, #1b3457);
}
.tela-competicao .comp-botoes { display: flex; gap: 10px; flex-wrap: wrap; }
.tela-competicao .comp-botoes button { flex: 1 1 200px; min-height: 52px; font-size: 16px; }
.tela-competicao .comp-item {
  display: flex; gap: 10px; align-items: flex-start;
  padding: 8px 0; border-bottom: 1px solid var(--borda, #1b3457);
}
.tela-competicao .comp-item:last-child { border-bottom: none; }
.tela-competicao .comp-item .comp-texto { flex: 1 1 auto; min-width: 0; }
:where(.tela-competicao) .comp-secao { display: flex; flex-direction: column; gap: 8px; }
:where(.tela-competicao) h2 { font-size: 21px; font-weight: 800; color: #fff; margin: 0; }
:where(.tela-competicao) h3 { font-size: 15px; font-weight: 700; color: #dcecff; margin: 0; }
:where(.tela-competicao) .msgCartao { color: #8fb0d8; font-size: 13px; margin: 0; }
:where(.tela-competicao) .motivoCartao { color: #ffd28f; font-size: 12px; margin: 0; }
:where(.tela-competicao) .dica { color: #7d9cc4; font-size: 12px; margin: 0; }
:where(.tela-competicao) .valor { color: #cfe4fb; font-weight: 700; }
:where(.tela-competicao) button {
  background: #142643; color: #dcecff;
  border: 1px solid #1f3a5f; border-radius: 12px;
  padding: 10px 14px; font: inherit; font-weight: 600; cursor: pointer;
}
:where(.tela-competicao) button:hover:not(:disabled) { border-color: #35c3f0; color: #fff; }
:where(.tela-competicao) button:disabled { color: #3d5a82; border-color: #14294a; cursor: not-allowed; }
:where(.tela-competicao) .acaoPrimaria { background: #ff9a2e; color: #1a0e00; border: none; font-weight: 800; }
:where(.tela-competicao) .acaoPrimaria:hover:not(:disabled) { background: #f07f0e; }
:where(.tela-competicao) .acaoDestrutiva { color: #ff9d9d; border-color: #5c2230; }
:where(.tela-competicao) .acaoDestrutiva:hover:not(:disabled) { background: #3a1620; border-color: #eb5757; color: #fff; }
:where(.tela-competicao) .chip {
  font-size: 11px; font-weight: 800; border-radius: 8px;
  padding: 3px 8px; color: #06121f; background: #7f93b0; white-space: nowrap;
}
@media (max-width: 1024px) {
  .tela-competicao { overflow-y: auto; }
  .tela-competicao .comp-grade { grid-template-columns: minmax(0, 1fr); }
  .tela-competicao .comp-col { overflow: visible; }
  .tela-competicao .comp-rolagem, .tela-competicao .comp-passos { max-height: none; }
}
</style>

<div class="tela-competicao">
  <div class="comp-grade">

    <section class="comp-col" data-col="preparo">
      <div class="comp-secao">
        <h2>Competicao</h2>
        <p class="msgCartao">Rodar uma prova de ponta a ponta, do jeito que ela
          vale ponto: o robo sai do inicio, cumpre a tarefa e volta.</p>
      </div>

      <div class="comp-secao">
        <h3>1. Escolha a prova</h3>
        <div class="comp-rolagem" data-el="provas" aria-label="Provas deste robo"></div>
        <p class="msgCartao" data-el="resumoProva">Escolha uma prova para ver o que ela faz.</p>
      </div>

      <div class="comp-secao">
        <h3>2. Como rodar</h3>
        <div class="comp-opcoes" data-el="opcoes"></div>
      </div>

      <div class="comp-secao">
        <h3>3. Antes de comecar</h3>
        <div data-el="impedimentos"></div>
      </div>
    </section>

    <section class="comp-col" data-col="prova">
      <div class="comp-secao">
        <h3>4. O plano</h3>
        <p class="msgCartao">Montar o plano nao move o robo: ele so calcula e
          mostra o que pretende fazer, passo a passo.</p>
        <div class="comp-botoes">
          <button data-el="montar">Montar o plano</button>
        </div>
        <p class="motivoCartao" data-el="motivoMontar"></p>
      </div>

      <p class="msgCartao" data-el="tituloPassos">Nenhum plano montado ainda.</p>
      <div class="comp-passos" data-el="passos"></div>

      <div class="comp-execucao">
        <div style="display:flex; align-items:center; gap:10px; flex-wrap:wrap;">
          <span class="chip" data-el="chipEstado">PARADA</span>
          <span class="valor" data-el="contadorPassos">—</span>
          <span class="msgCartao" data-el="cronometro"></span>
        </div>
        <div class="comp-barra"><div data-el="barra"></div></div>
        <p class="msgCartao" data-el="mensagemMissao">O robo esta parado.</p>
        <div class="comp-botoes">
          <button class="acaoPrimaria" data-el="comecar">Comecar a prova</button>
          <button class="acaoDestrutiva" data-el="abortar" disabled>Abortar a prova</button>
        </div>
        <p class="motivoCartao" data-el="motivoComecar"></p>
        <p class="dica">Abortar interrompe a missao e para o que o robo estiver
          indo fazer. Nao e botao de emergencia: este robo nao tem um.</p>
      </div>
    </section>

  </div>
</div>`;

// As quatro opcoes da GUI, sem citar nome de programa. "Simular navegacao (sem
// Nav2)" virou "ensaiar sem sair do lugar": e o que a opcao faz para quem opera.
const OPCOES = [
  {
    id: 'home', marcado: true,
    rotulo: 'Recolher o braco antes de comecar',
    ajuda: 'Comeca com o braco na posicao de descanso. Sem isso, o robo pode '
         + 'sair andando com o braco esticado.',
  },
  {
    id: 'refino', marcado: true,
    rotulo: 'Encostar na estacao com ajuda do sensor',
    ajuda: 'O robo corrige a aproximacao final olhando a estacao, em vez de '
         + 'confiar so na posicao gravada.',
  },
  {
    id: 'finish', marcado: true,
    rotulo: 'Ir ao ponto de fim (FINISH) ao terminar',
    ajuda: 'A prova so termina quando o robo chega ao FINISH.',
  },
  {
    id: 'ensaio', marcado: false,
    rotulo: 'Ensaiar sem sair do lugar',
    ajuda: 'O robo finge que anda e executa o resto normalmente. Serve para '
         + 'conferir a prova na bancada; nao vale como prova de verdade.',
  },
];

function montar(raiz, ctx) {
  const app = ligarContexto(ctx);
  raiz.innerHTML = MOLDE;
  const el = (nome) => raiz.querySelector(`[data-el="${nome}"]`);

  let provas = [];
  let prova = null;              // prova escolhida na lista
  let arena = '';                // arena ativa no robo (vem do WebSocket)
  let arenaConferida = '';       // para que ele so leia a arena quando ela muda
  let pendenciasDaArena = '';    // motivo_nao_pronta vindo da propria arena
  let temFinish = null;          // a arena tem o ponto de fim? (null = ainda nao sei)
  let plano = null;              // {linhas, actions_yaml} do ultimo dry-run
  let planoDaProva = '';         // de QUAL prova e o plano que esta na tela
  let ultimaMissao = null;
  let passosVivos = [];          // passos reportados pelo executor
  let ocupado = false;
  let relogioDoPlano = null;
  let ultimoEstado = {};         // ultimo quadro do WebSocket, para recalcular fora dele
  let vivo = true;

  const marcado = (id) => {
    const cx = raiz.querySelector(`[data-opcao="${id}"]`);
    return cx ? cx.checked : OPCOES.find((o) => o.id === id).marcado;
  };

  // ---------------------------------------------------------------- provas
  async function carregarProvas() {
    let dados = {};
    try {
      dados = await app.ler('/api/missoes');
    } catch {
      app.avisar('Nao consegui pedir a lista de provas ao robo.', false);
      return;
    }
    if (!vivo) return;
    provas = dados.missoes || [];
    desenharProvas();
  }

  function desenharProvas() {
    const alvo = el('provas');
    if (!provas.length) {
      alvo.innerHTML = '<p class="motivoCartao">Este robo nao tem nenhuma prova '
        + 'cadastrada. Sem prova cadastrada nao ha o que rodar aqui.</p>';
      return;
    }
    alvo.innerHTML = provas.map((m, i) => `
      <button class="comp-prova" data-prova="${i}"
              aria-pressed="${!!(prova && prova.caminho === m.caminho)}">
        <span class="valor">${escapar(m.rotulo || m.task_id)}</span>
        <span class="dica">${escapar(resumoCurto(m))}</span>
      </button>`).join('');
    alvo.querySelectorAll('[data-prova]').forEach((bt) => {
      bt.onclick = () => escolherProva(provas[Number(bt.dataset.prova)]);
    });
  }

  function resumoCurto(m) {
    const partes = [];
    if (m.duracao_min) partes.push(`${m.duracao_min} min`);
    if (m.objetos) partes.push(`${m.objetos} objeto(s)`);
    if ((m.areas || []).length) partes.push(`estacoes: ${m.areas.join(', ')}`);
    return partes.join('  •  ') || 'sem detalhes cadastrados';
  }

  function escolherProva(m) {
    prova = m;
    // Trocar de prova joga fora o plano: o plano na tela e de OUTRA prova, e
    // deixar ele ali seria oferecer o botao de comecar com o passo a passo
    // errado na frente do operador.
    plano = null;
    planoDaProva = '';
    passosVivos = [];
    desenharProvas();
    el('resumoProva').textContent = prova ? resumoCurto(prova) : '';
    desenharPlano();
    atualizarDisponibilidade();
  }

  // --------------------------------------------------------------- opcoes
  function desenharOpcoes() {
    el('opcoes').innerHTML = OPCOES.map((o) => `
      <label>
        <input type="checkbox" data-opcao="${o.id}" ${o.marcado ? 'checked' : ''}>
        <span><span class="valor">${escapar(o.rotulo)}</span><br>
          <span class="dica">${escapar(o.ajuda)}</span></span>
      </label>`).join('');
    el('opcoes').querySelectorAll('[data-opcao]').forEach((cx) => {
      cx.onchange = () => {
        // Mudar como a prova roda invalida o plano ja montado: o plano foi
        // calculado com as opcoes antigas.
        plano = null;
        planoDaProva = '';
        desenharPlano();
        atualizarDisponibilidade();
      };
    });
  }

  function opcoesDaMissao() {
    return {
      task_yaml: prova ? prova.caminho : '',
      map_name: arena,
      simulate_nav: marcado('ensaio'),
      use_lidar_refine: marcado('refino'),
      finish_dock_id: marcado('finish') ? 'FINISH' : '',
      skip_startup_home: !marcado('home'),
    };
  }

  // ---------------------------------------------------------------- plano
  async function montarPlano() {
    const daProva = prova ? prova.caminho : '';
    plano = null;
    planoDaProva = '';
    passosVivos = [];
    el('tituloPassos').textContent = 'Montando o plano...';
    el('passos').innerHTML = '';
    const r = await app.comandar('/api/missao/plano', opcoesDaMissao());
    if (r && r.ok === false) { el('tituloPassos').textContent = 'Nao consegui montar o plano.'; return; }
    acompanharPlano(daProva);
  }

  // O dry-run responde "ok, estou montando" na hora e o resultado chega depois,
  // por outro caminho. Por isso a tela pergunta pelo plano ate ele ficar pronto
  // -- e desiste com uma frase, em vez de rodar para sempre.
  function acompanharPlano(daProva) {
    if (relogioDoPlano) clearInterval(relogioDoPlano);
    let tentativas = 0;
    relogioDoPlano = setInterval(async () => {
      if (!vivo) { clearInterval(relogioDoPlano); return; }
      tentativas += 1;
      let p = {};
      try {
        p = await app.ler('/api/missoes/plano');
      } catch {
        p = {};
      }
      if (!p || p.estado === 'planejando') {
        if (tentativas > 90) {          // 90 x 800 ms = 1 min e 12 s
          clearInterval(relogioDoPlano);
          relogioDoPlano = null;
          el('tituloPassos').textContent =
            'O robo esta demorando demais para montar o plano desta prova.';
        }
        return;
      }
      clearInterval(relogioDoPlano);
      relogioDoPlano = null;
      if (p.estado === 'pronto') {
        plano = p;
        planoDaProva = daProva;
      } else {
        plano = null;
        planoDaProva = '';
      }
      desenharPlano(p);
      atualizarDisponibilidade();
    }, 800);
  }

  function desenharPlano(cru) {
    const linhas = (plano && plano.linhas) || [];
    if (!linhas.length) {
      const recado = cru && cru.mensagem
        ? cru.mensagem
        : (prova ? 'Nenhum plano montado ainda para esta prova.' : 'Escolha a prova primeiro.');
      el('tituloPassos').textContent = recado;
      el('passos').innerHTML = '';
      return;
    }
    el('tituloPassos').textContent =
      `Plano com ${linhas.length} passo(s). O robo ainda acrescenta sozinho o `
      + 'inicio (recolher o braco) e o fim da prova.';
    el('passos').innerHTML = linhas.map((l, i) => `
      <div class="comp-passo">
        <span class="comp-numero">${i + 1}</span>
        <span>${escapar(String(l).trim())}</span>
      </div>`).join('');
  }

  // -------------------------------------------------------------- execucao
  // Os passos do PLANO e os passos que o executor reporta nao se alinham: o
  // executor acrescenta o inicio e o fim que nao estao no plano. Por isso,
  // quando a prova comeca, a lista e refeita a partir do que ele reporta --
  // nunca indexada pelo plano. Sintoma que isso evita: a linha destacada
  // andando um passo atras do que o robo esta fazendo de verdade.
  function prepararPassos(total) {
    if (!(total > 0) || passosVivos.length === total) return;
    passosVivos = Array.from({length: total}, () => ({texto: '', marca: ''}));
    desenharPassosVivos();
  }

  function desenharPassosVivos() {
    el('passos').innerHTML = passosVivos.map((p, i) => `
      <div class="comp-passo" data-marca="${p.marca || ''}">
        <span class="comp-numero">${i + 1}</span>
        <span>${escapar(p.texto || 'passo do robo')}</span>
        ${p.marca === 'feito' ? '<span class="chip" style="background:#27ae60">FEITO</span>' : ''}
        ${p.marca === 'agora' ? '<span class="chip" style="background:#35c3f0">AGORA</span>' : ''}
        ${p.marca === 'falhou' ? '<span class="chip" style="background:#eb5757;color:#fff">FALHOU</span>' : ''}
      </div>`).join('');
    const atual = el('passos').querySelector('[data-marca="agora"]');
    if (atual) atual.scrollIntoView({block: 'nearest'});
  }

  function aoProgredir(m) {
    const info = ESTADO_DA_MISSAO[m.estado] || {rotulo: m.estado || '—', cor: '#7f93b0'};
    const chip = el('chipEstado');
    chip.textContent = info.rotulo.toUpperCase();
    chip.style.background = info.cor;
    chip.style.color = info.cor === '#eb5757' ? '#fff' : '#06121f';
    el('cronometro').textContent = m.decorrido ? `tempo de prova ${relogio(m.decorrido)}` : '';
    el('mensagemMissao').textContent = m.mensagem || textoDoPasso(m)
      || (m.estado === 'parado' ? 'O robo esta parado.' : 'Aguardando o robo.');

    const antes = ultimaMissao || {};
    const mudouPasso = antes.passo !== m.passo || antes.subestagio !== m.subestagio
      || antes.total !== m.total;

    if (m.estado === 'executando' && m.total > 0) {
      prepararPassos(m.total);
      if (mudouPasso && m.passo >= 0 && m.passo < passosVivos.length) {
        const marca = m.subestagio === 'concluido' ? 'feito'
                    : (m.subestagio === 'falhou' ? 'falhou' : 'agora');
        // Passo que ja passou fica "feito": o executor so reporta o atual, e
        // sem isso a lista voltaria a ficar em branco atras do robo.
        for (let i = 0; i < m.passo; i += 1) {
          if (passosVivos[i].marca !== 'falhou') passosVivos[i].marca = 'feito';
        }
        passosVivos[m.passo] = {texto: textoDoPasso(m), marca};
        desenharPassosVivos();
      }
      // passo -1 quer dizer "nenhum passo em execucao ainda"; escrever
      // "passo 0 de 12" seria contar errado na frente de quem confere.
      el('contadorPassos').textContent = m.passo >= 0
        ? `passo ${Math.min(m.passo + 1, m.total)} de ${m.total}`
        : `${m.total} passos, comecando`;
      const feitos = m.subestagio === 'concluido' ? m.passo + 1 : m.passo;
      el('barra').style.width = `${Math.max(0, Math.min(100, (feitos / m.total) * 100))}%`;
    } else if (TERMINOU.has(m.estado)) {
      el('contadorPassos').textContent = info.rotulo;
      if (m.estado === 'concluida') el('barra').style.width = '100%';
    } else {
      el('contadorPassos').textContent = info.rotulo;
    }
  }

  function textoDoPasso(m) {
    const monta = ACAO[m.tipo];
    const base = monta ? monta(m.alvo) : (m.alvo || '');
    const sub = SUBESTAGIO[m.subestagio];
    return sub && base ? `${base} — ${sub}` : base;
  }

  // -------------------------------------------------- o que ainda impede
  // Cada pendencia vem com a acao que resolve. Lista de problemas sem saida e o
  // que faz o operador ir procurar um teclado no meio da prova.
  function pendencias(estado) {
    const cap = (estado && estado.capacidades) || {};
    const cartoes = (estado && estado.cartoes) || {};
    const robo = cartoes.robo || {};
    const cartao = (cartoes.cartoes || {}).competicao || {};
    const lista = [];

    if (!app.podeComandar()) {
      lista.push({grave: true, texto: 'Este painel foi aberto sem a chave de acesso: '
        + 'da para acompanhar, mas nao para comandar o robo. Peca o link completo a '
        + 'quem ligou o robo.'});
    }
    if (!provas.length) {
      lista.push({grave: true, texto: 'Este robo nao tem nenhuma prova cadastrada.'});
    } else if (!prova) {
      lista.push({grave: true, texto: 'Escolha a prova que o robo vai rodar.'});
    }
    if (cap.missao === false) {
      lista.push({grave: true, texto: 'O sistema que executa as provas nao esta no ar. Reinicie o robo.'});
    }
    if (!robo.arena_existe) {
      lista.push({
        grave: true,
        texto: robo.arena_nome
          ? `A arena "${robo.arena_nome}" nao esta mais salva neste robo.`
          : 'Nenhuma arena escolhida: o robo nao sabe em que lugar vai rodar a prova.',
        acao: {rotulo: 'Escolher a arena', tela: 'mapas'},
      });
    } else if (cartao.estado === 'degradado' && cartao.motivo) {
      // NAO impede de comecar de proposito: se o robo estiver localizado e o
      // aviso for falso, travar a prova aqui custaria a prova inteira. Fica em
      // amarelo, com o caminho para resolver em um toque.
      lista.push({
        grave: false, texto: cartao.motivo,
        acao: {rotulo: 'Marcar onde o robo esta', tela: 'operacao', secao: 'localizacao'},
      });
    }
    if (pendenciasDaArena) {
      lista.push({
        grave: false, texto: pendenciasDaArena,
        acao: {rotulo: 'Abrir Mapas', tela: 'mapas'},
      });
    }
    if (marcado('finish') && temFinish === false) {
      // A opcao manda o robo terminar no FINISH; sem esse ponto na arena o
      // fim da prova falha depois de tudo dar certo, que e o pior lugar
      // possivel para descobrir isso.
      lista.push({
        grave: false,
        texto: 'Esta arena nao tem o ponto de fim (FINISH). Desmarque "ir ao ponto '
             + 'de fim" ou marque o FINISH na arena.',
        acao: {rotulo: 'Abrir Mapas', tela: 'mapas'},
      });
    }
    if (marcado('ensaio')) {
      lista.push({grave: false, texto: 'A prova vai rodar em ensaio: o robo nao sai do lugar.'});
    }
    return lista;
  }

  function desenharPendencias(lista) {
    if (!lista.length) {
      el('impedimentos').innerHTML =
        '<p class="msgCartao">Nada impede a prova de comecar.</p>';
      return;
    }
    el('impedimentos').innerHTML = lista.map((p, i) => `
      <div class="comp-item">
        <span class="chip" style="${p.grave ? 'background:#eb5757;color:#fff' : 'background:#f2994a'}">
          ${p.grave ? 'IMPEDE' : 'ATENCAO'}</span>
        <span class="comp-texto">
          <span class="${p.grave ? 'motivoCartao' : 'msgCartao'}">${escapar(p.texto)}</span>
          ${p.acao ? `<br><button data-pendencia="${i}">${escapar(p.acao.rotulo)}</button>` : ''}
        </span>
      </div>`).join('');
    el('impedimentos').querySelectorAll('[data-pendencia]').forEach((bt) => {
      bt.onclick = () => {
        const acao = lista[Number(bt.dataset.pendencia)].acao;
        if (!app.irPara(acao.tela, {secao: acao.secao})) {
          app.avisar(`Abra ${acao.tela} no menu principal.`, false);
        }
      };
    });
  }

  function atualizarDisponibilidade(estado) {
    const lista = pendencias(estado || ultimoEstado);
    desenharPendencias(lista);
    const grave = lista.find((p) => p.grave);

    let motivoMontar = '';
    if (grave) motivoMontar = grave.texto;
    else if (ocupado) motivoMontar = 'Espere a prova em andamento terminar.';
    el('montar').disabled = !!motivoMontar;
    el('motivoMontar').textContent = motivoMontar;

    let motivoComecar = motivoMontar;
    if (!motivoComecar && (!plano || planoDaProva !== (prova ? prova.caminho : ''))) {
      // O plano e a unica garantia de que o que roda e o que o operador leu.
      // Sem plano, o executor planejaria de novo por conta propria.
      motivoComecar = 'Monte o plano desta prova antes de comecar: assim voce ve o '
        + 'que o robo vai fazer, e e exatamente isso que ele faz.';
    }
    el('comecar').disabled = !!motivoComecar;
    el('motivoComecar').textContent = motivoComecar;
    el('abortar').disabled = !ocupado;
  }

  // ------------------------------------------------------------ arena ativa
  // A prova roda na arena que o robo tem gravada -- a mesma que a tela Mapas
  // escolhe. Um segundo seletor de arena aqui seria o jeito mais rapido de
  // rodar a prova num mapa diferente daquele em que o robo esta localizado.
  async function conferirArena(nome) {
    arenaConferida = nome;
    pendenciasDaArena = '';
    temFinish = null;
    if (!nome) return;
    try {
      const meta = await app.ler(`/api/arenas/${encodeURIComponent(nome)}/meta`);
      if (!vivo || arenaConferida !== nome) return;
      if (!meta.erro && !meta.pronta_para_missao) {
        pendenciasDaArena = meta.motivo_nao_pronta || '';
      }
      if (!meta.erro) {
        temFinish = (meta.docks || []).some((d) => d.papel === 'fim' || d.id === 'FINISH');
      }
    } catch {
      // Arena ilegivel aparece do mesmo jeito pelo cartao do menu; nao vale
      // inventar um segundo recado aqui.
    }
    atualizarDisponibilidade();
  }

  // ---------------------------------------------------------------- ligacao
  el('montar').onclick = montarPlano;
  el('comecar').onclick = async () => {
    const corpo = opcoesDaMissao();
    // Reusa o plano JA MOSTRADO ao operador (o executor devolve o caminho do
    // arquivo do plano). Sem isso, "Comecar" replanejaria e poderia executar
    // algo diferente do que esta na tela dele.
    corpo.actions_yaml = (plano && plano.actions_yaml) || '';
    passosVivos = [];
    await app.comandar('/api/missao/rodar', corpo);
  };
  el('abortar').onclick = () => app.comandar('/api/missao/abortar', {});

  desenharOpcoes();
  carregarProvas().then(() => atualizarDisponibilidade());
  atualizarDisponibilidade();

  return {
    destruir() {
      vivo = false;
      if (relogioDoPlano) clearInterval(relogioDoPlano);
      relogioDoPlano = null;
      // A missao NAO e abortada ao sair da tela: a prova continua rodando com
      // ou sem alguem olhando, e parar o robo por causa de uma troca de tela
      // seria pior do que qualquer coisa que essa troca resolva.
      raiz.innerHTML = '';
    },

    // Chega a 10 Hz. So o que MUDOU e redesenhado: refazer a lista de passos
    // dez vezes por segundo num tablet trava o toque bem na hora da prova.
    aoEstado(estado) {
      ultimoEstado = estado || {};
      const robo = ((estado && estado.cartoes) || {}).robo || {};
      const nome = robo.arena_nome || '';
      if (nome !== arena) {
        arena = nome;
        conferirArena(nome);
      }
      const agoraOcupado = !!((estado && estado.capacidades) || {}).missao_ocupada;
      const m = (estado && estado.missao) || null;
      const mudouMissao = m && (!ultimaMissao
        || ultimaMissao.estado !== m.estado
        || ultimaMissao.passo !== m.passo
        || ultimaMissao.total !== m.total
        || ultimaMissao.subestagio !== m.subestagio
        || ultimaMissao.mensagem !== m.mensagem);
      if (m && (mudouMissao || Math.round(m.decorrido || 0) !== Math.round((ultimaMissao || {}).decorrido || 0))) {
        aoProgredir(m);
        ultimaMissao = m;
      }
      if (agoraOcupado !== ocupado) {
        ocupado = agoraOcupado;
        atualizarDisponibilidade(estado);
      }
    },
  };
}
