# Experimentos de desempenho

Cada experimento testa uma hipótese refutável sobre o gateway. `run_all.sh`
executa todos e grava os resultados brutos. `analyze.py` agrega as
repetições, decide cada hipótese e gera as séries para os gráficos.

```bash
# no contêiner do projeto (WSL2 ou Linux), com o pacote compilado em Release
docker compose run --rm -T ros bash -lc \
  'source /ros2_ws/install/setup.bash && src/semantic_gateway/bench/run_all.sh'
python3 bench/analyze.py bench/results/<execução>     # → summary.md, plots/*.dat
```

Variáveis úteis:

| Variável | Efeito |
|---|---|
| `QUICK=1` | Poucas repetições, só para conferir o pipeline. |
| `ONLY="h1 h3"` | Roda apenas um subconjunto dos experimentos. |
| `CPU=2` | Fixa a *thread* medida num núcleo. |
| `GAZEBO=1` | Inclui o H5, que leva alguns minutos por repetição. |
| `REPS`, `REPS_OL`, `REPS_E2E` | Número de repetições de cada grupo. |

## Hipóteses

| | Hipótese | Fatores | Métrica | Critério de decisão |
|---|---|---|---|---|
| **H1** | A latência de tradução cresce linearmente com o número de campos da mensagem, e o p99 fica abaixo de 100 µs para as mensagens do descritor. | `LaserScan` com 10, 100, 360, 1000 e 4000 feixes; `Imu`; `Temperature`. Sem envio. | Mín., p5, p50, p95, p99, p99,9 e máx. de 20 000 mensagens por repetição, com 30 repetições, cada uma num processo novo. | O ajuste linear do p50 tem R² > 0,95, e o p99 < 100 µs até 360 feixes. |
| **Convergência** | As estatísticas relatadas estabilizam com o número de iterações usado. | Prefixos de 500 até 200 000 iterações de uma mesma série. | p50, p95, p99 e p99,9 em função de n. | A partir de n, o percentil fica a ±5% do valor final. |
| **H2** | Existe uma taxa R\* abaixo da qual a latência fica estável e não há perdas. Acima dela, a fila do tipo *keep-last* 5, como a do `SensorDataQoS`, transborda. | Taxa ofertada em laço aberto, sem envio (K = 0) e com 100 respostas por varredura (K = 100). | Latência medida desde a chegada agendada, perdas e taxa servida. | R\* é a maior taxa com perda < 1%. |
| **H3** | O custo do envio é linear no número K de respostas e se deve ao `sendto` do sistema operacional, não ao gateway. | K = 0, 1, 10, 50, 100 no gateway, contra três bases de comparação sem o gateway: socket cru, socket cru com receptor local no grupo e `sd::Network::send`. | Inclinação do p50 em K, em µs por datagrama. | Linear com R² > 0,95, e o custo por datagrama do gateway fica até 1,5× o do `sendto` sob as mesmas condições de recepção local. |
| **H4** | O caminho de ponta a ponta (publicador ROS → gateway → rede → proxy → assinante ROS) tem p95 abaixo de 1 ms e acrescenta ao salto ROS 2 direto menos que um salto adicional. | Taxa de publicação de `Imu`: 100, 500, 1000, 2000 e 5000 Hz. | `agora − header.stamp` em cada caminho, com o mesmo relógio, e as perdas. | p95 < 1 ms, sobrecarga do p50 < p50 do salto direto, perda < 1%. |
| **H5** | O robô segue os *setpoints* enviados pela rede SmartData, com erro de regime pequeno e atraso de comando de poucos milissegundos. | Sequência de degraus de (v, ω) no Gazebo, num mundo vazio. | Erro em regime, pela média da segunda metade de cada degrau, e atraso entre `/cmd_vel_command` e `/cmd_vel`. | \|erro\| < máx(0,01; 5% do comando). |

As medições de latência isolada usam `steady_clock` e excluem um aquecimento
inicial. Cada configuração roda em processos separados, para que a variação
entre repetições apareça nos intervalos de confiança: a mediana entre
repetições é acompanhada de um IC de 95% por *bootstrap*.

## Linux nativo (notebook Ubuntu)

O WSL2 virtualiza a pilha de rede, que o H3 e o H4 medem, e o escalonador
do *host*, que afeta o pior caso. Por isso as medições são repetidas em
Linux nativo, usando apenas o repositório e o Docker:

```bash
sudo apt install -y git docker.io docker-compose-v2
sudo usermod -aG docker $USER        # depois, sair e entrar de novo na sessão
git clone https://github.com/schasha/tcc.git && cd tcc
QUICK=1 bench/notebook.sh            # teste rápido (poucos minutos)
bench/notebook.sh                    # bateria completa (cerca de 1 h a 1 h 30)
```

Durante a bateria completa:
- deixe o notebook na tomada;
- feche os outros programas;
- não use a máquina.

O resultado fica em `bench/results/<máquina>-<data>/`.
