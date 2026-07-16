# Sincronização de Relógio PC ↔ Raspberry (NTP local)

**Por que:** todos os nós usam `use_sim_time: false` (relógio de parede). Se o relógio da Pi divergir do PC, os timestamps de TF/scan/odom não batem e o Nav2 solta *"message filter dropping / extrapolation into the future"*. O plano mestre exige offset **< 5 ms** (RK-06). Em arena **sem internet**, não dá para depender de NTP público — o PC vira o **servidor de tempo da LAN** e a Pi sincroniza contra ele.

**Arquitetura:** PC roda `chrony` como servidor (serve o próprio relógio mesmo offline, via `local stratum 10`); a Raspberry roda `chrony` como cliente apontando para o PC. Quando houver internet, o PC também sincroniza com a internet e repassa; sem internet, o par PC↔Pi fica coerente entre si (é o que importa para o TF).

> ⚠️ **Requer `sudo` (senha) nas duas máquinas.** Estes passos não foram executados automaticamente — rode os blocos abaixo no terminal de cada máquina.

---

## Passo 1 — IP do PC na rede do robô

No **PC principal do robô** (não numa máquina de desenvolvimento), descubra o IP **na rede que ele compartilha com a Pi**:
```bash
ip -brief addr        # procure a interface UP conectada ao robô (Ethernet, tipicamente)
```
Anote o IP — chame de `IP_DO_PC` nos passos seguintes. (O `192.168.0.x` usado como exemplo abaixo veio de uma máquina de dev em WiFi; troque pelo IP/sub-rede reais do PC do robô.)

> **Recomendação forte:** dê ao PC um **IP fixo** na rede do robô (IP estático ou reserva de DHCP no roteador). Se o IP do PC mudar, a Pi perde o servidor de tempo. Se usarem link Ethernet direto PC↔Pi, defina IPs estáticos nos dois (ex.: PC `192.168.50.1`, Pi `192.168.50.10`) e use esse IP como `IP_DO_PC`.

---

## Passo 2 — PC como servidor NTP

```bash
sudo apt update && sudo apt install -y chrony

# Drop-in de servidor (não edita o chrony.conf principal):
sudo tee /etc/chrony/conf.d/caramelo-ntp-server.conf > /dev/null <<'EOF'
# Servir tempo para a LAN do robo Caramelo.
# Ajuste a sub-rede para a rede real PC<->Pi (ex.: 192.168.0.0/24 no WiFi atual,
# ou 192.168.50.0/24 se usarem link Ethernet direto).
allow 192.168.0.0/24

# "Ilha de tempo": continua servindo o proprio relogio mesmo SEM internet,
# como stratum 10 (pior que qualquer fonte real de internet, melhor que nada).
local stratum 10
EOF

sudo systemctl restart chrony
sudo systemctl enable chrony

# Verificar:
chronyc sources -v      # deve listar as fontes (internet, se houver)
sudo ss -ulpn | grep :123   # chrony escutando na porta 123/UDP (servidor ativo)
```

> Instalar o `chrony` desativa automaticamente o `systemd-timesyncd` (só um cliente NTP por vez).

---

## Passo 3 — Raspberry como cliente do PC

Na Raspberry (com `IP_DO_PC` = o IP anotado no Passo 1):
```bash
sudo apt update && sudo apt install -y chrony

sudo tee /etc/chrony/conf.d/caramelo-ntp-client.conf > /dev/null <<'EOF'
# Sincronizar com o PC principal (servidor de tempo da LAN).
# TROQUE 192.168.0.9 pelo IP real do PC (IP_DO_PC).
server 192.168.0.9 iburst prefer
EOF

sudo systemctl restart chrony
sudo systemctl enable chrony
```

---

## Passo 4 — Verificar o offset (critério do plano: < 5 ms)

Na Raspberry:
```bash
chronyc sources -v      # o PC (IP_DO_PC) deve aparecer com '^*' (fonte selecionada)
chronyc tracking        # olhar "System time" e "Last offset" -> alvo < 5 ms
```
Repita `chronyc tracking` após ~1 min (o iburst converge rápido). Se o offset ficar acima de 5 ms de forma persistente, verifique firewall (porta 123/UDP) e se o `IP_DO_PC` está certo.

---

## Passo 5 — Teste com o grafo ROS 2 rodando

Com o hardware_bringup na Pi e a navegação no PC, confirme que **sumiram** os avisos de TF do tipo *"message filter dropping message ... extrapolation into the future"* / *"timestamp ... is in the future"*. Se ainda aparecerem com o offset < 5 ms, o problema é outro (QoS/latência de rede), não relógio.

---

## Notas

- **Firewall:** se o PC tiver `ufw` ativo, liberar NTP para a sub-rede do robô: `sudo ufw allow from 192.168.0.0/24 to any port 123 proto udp`.
- **Arena sem internet:** o `local stratum 10` garante que o PC serve tempo mesmo sem fonte externa. O par PC↔Pi fica coerente entre si — que é o necessário para o TF do Nav2. O horário "absoluto" pode estar deslocado do mundo real, e tudo bem.
- **Alternativa por hostname:** se o avahi/mDNS resolver o hostname do PC nos dois lados (já usam `raspberrypi.local` para as meshes), dá para usar `server <hostname_do_PC>.local iburst prefer` no lugar do IP — mais robusto a troca de IP, porém depende do mDNS estar de pé.
