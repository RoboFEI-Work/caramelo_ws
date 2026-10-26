"""Chave de acesso do painel: quem pode MUDAR o estado do robô.

O painel escuta em 0.0.0.0 de propósito — numa competição o operador não fica
sentado no robô, ele acompanha de outro computador. Só que a partir do momento
em que o painel ganhou terminal e teleop, "acessível na rede" passou a
significar "shell e volante para qualquer um no mesmo Wi-Fi". Não dá.

A regra é ASSIMÉTRICA de propósito:

  LEITURA  (mapa, pose, saúde, missão em andamento) continua livre. O painel tem
           que abrir sem atrito, inclusive no celular de quem só quer olhar.
  ESCRITA  (mandar navegar, dockar, disparar missão, teleop), o TERMINAL e o
           ESPELHO DA TELA exigem a chave.

A chave viaja no link (`?t=...`). Foi a decisão do operador e ela tem um motivo
concreto: é o único jeito de abrir o painel num tablet sem ninguém digitar
senha. A consequência aceita é que quem tiver o link tem o robô — por isso a
chave é sorteada a cada arranque e impressa no log, e não fica gravada em disco.

Não é autenticação de verdade (não há usuários, não há sessão, não há HTTPS):
é uma tranca contra o vizinho de bancada e contra o clique acidental de quem
abriu o painel por curiosidade. Contra alguém farejando a rede, não é nada.
"""
import ipaddress
import secrets
import socket

# Aceitos nos dois: query `?t=` (o que o link carrega) e cabeçalho, para quem
# chamar a API por script sem sujar a URL.
PARAMETRO = "t"
CABECALHO = "X-Caramelo-Chave"


class ChaveDeAcesso:
    """A chave do arranque atual."""

    def __init__(self, fixa: str = ""):
        fixa = (fixa or "").strip()
        # 24 bytes => 32 caracteres url-safe. Cabe num QR code e ainda são 192
        # bits: ninguém adivinha por tentativa e erro numa rede local.
        self.valor = fixa or secrets.token_urlsafe(24)
        self.fixa = bool(fixa)

    def confere(self, apresentada) -> bool:
        if not apresentada:
            return False
        # compare_digest e não '==': a comparação ingênua para no primeiro
        # caractere diferente e devolve o prefixo certo pelo tempo de resposta.
        return secrets.compare_digest(str(apresentada), self.valor)

    def apresentada_em(self, pedido) -> str:
        """Chave que veio no pedido. Serve para Request E para WebSocket.

        Os dois são objetos do Starlette e os dois expõem query_params e
        headers -- por isso uma função só atende as duas portas de entrada.
        """
        valor = pedido.query_params.get(PARAMETRO)
        if valor:
            return valor
        valor = pedido.headers.get(CABECALHO)
        if valor:
            return valor
        autorizacao = pedido.headers.get("authorization") or ""
        if autorizacao.lower().startswith("bearer "):
            return autorizacao[7:].strip()
        return ""

    def autorizado(self, pedido) -> bool:
        return self.confere(self.apresentada_em(pedido))

    def links(self, host: str, porta: int) -> list:
        """Endereços prontos para colar no navegador, o mais útil primeiro."""
        if host not in ("0.0.0.0", "::", ""):
            enderecos = [host]
        else:
            enderecos = enderecos_do_robo()
        return [f"http://{e}:{porta}/?{PARAMETRO}={self.valor}" for e in enderecos]


def enderecos_do_robo() -> list:
    """IPs em que este computador deve ser alcançável, do mais provável ao menos.

    O IP da rota padrão vem primeiro porque é por ele que o outro computador da
    bancada vai chegar. O localhost fica por último, mas fica: quando o painel
    roda no próprio robô com a tela ligada, é o único que funciona.
    """
    achados = []

    # Socket UDP "conectado" a um IP externo: o kernel escolhe a interface de
    # saída e revela o IP local SEM enviar pacote nenhum. Funciona offline.
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(0.2)
            s.connect(("10.255.255.255", 1))
            achados.append(s.getsockname()[0])
    except OSError:
        pass

    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            achados.append(info[4][0])
    except OSError:
        pass

    saida = []
    for ip in achados:
        try:
            endereco = ipaddress.ip_address(ip)
        except ValueError:
            continue
        if endereco.is_loopback or ip in saida:
            continue
        saida.append(ip)
    saida.append("127.0.0.1")
    return saida


def recado_sem_chave() -> dict:
    """Resposta única para toda tentativa de comandar sem a chave.

    Texto de operador, não de programador: quem cai aqui é alguém que abriu o
    painel por um link encurtado, um favorito antigo ou o histórico do
    navegador, e o que essa pessoa precisa saber é onde conseguir o link certo.
    """
    return {
        "ok": False,
        "mensagem": (
            "Este painel foi aberto sem a chave de acesso. Dá para acompanhar "
            "tudo, mas não dá para comandar o robô nem abrir o terminal. "
            "Peça o link completo a quem ligou o robô: ele aparece na tela do "
            "robô assim que o painel sobe."
        ),
    }
