# Semantic Gateway — ROS 2 → SmartData

Componente C++ para ROS 2 que exporta sensores e atuadores ROS como
SmartData (Fröhlich, 2018): cada tópico vira um conjunto de **transducers**
(uma classe por grandeza física, uma instância por eixo/elemento), cada
instância é um SmartData que segue a dinâmica do paper (modos, interesses,
respostas event-driven ou time-triggered, expiry, comandos) numa rede
UDP multicast com mensagens no formato TSTP.

Material da proposta: `proposta.md`. Mapeamento técnico: `docs/mapping.md`
e `docs/wire_format.md`.

## Arquitetura num parágrafo

`config/sensors.yaml` descreve cada tópico (tipo ROS, decomposição,
unidade, expiry, período, transform). Em build time, `codegen/generate.py`
emite três headers tipados: `parsed_transducers.hpp` (uma classe
Transducer por tópico × grandeza, com acesso direto ao campo),
`parsed_gateway.hpp` (as subscriptions e um `Responsive_SmartData` por
instância: sensores ADVERTISED, atuadores COMMANDED) e
`parsed_proxies.hpp` (`Interested_SmartData` que reconstroem as mensagens
ROS no lado remoto e enviam comandos). O run-time SmartData
(`include/smartdata/`) fornece a rede, o escalonamento periódico, o
relógio (ROS) e a localização (tf2). Em runtime: zero introspecção, zero
string lookup, zero parsing de YAML.

## Pré-requisitos

- Linux (testado em WSL2 Ubuntu 22.04)
- Docker + Docker Compose
- Para o Gazebo: `xhost +local:` ou ambiente WSLg para a GUI (opcional)

Não é necessário instalar ROS 2 no host — tudo roda no container.

## Setup

```bash
docker compose build              # cria a imagem tcc-semantic-gateway:humble
docker compose up -d ros          # sobe o container
docker exec -it tcc-ros bash      # entra
```

A partir daqui todos os comandos rodam em `/ros2_ws` dentro do container.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_gateway --symlink-install \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

CMake re-roda o codegen via `add_custom_command` sempre que
`config/sensors.yaml` ou `codegen/generate.py` mudam.

## Testes

```bash
colcon test --packages-select semantic_gateway --event-handlers console_direct+
```

**36 testes funcionais (gtest), todos verdes:**

| Suite                          | Testes | O que cobre                                                       |
|--------------------------------|--------|-------------------------------------------------------------------|
| `test_si_units`                | 5      | Encoding 32-bit da Fig.1 (METER, accel, ω, VOLT, `to_string`)     |
| `test_smartdata_runtime`       | 15     | Layout TSTP; dinâmica: interesses, event-driven/time-triggered, região, device, revogação, expiry, comandos, fuser |
| `test_gateway_pipeline`        | 8      | Transducers gerados, filtros, conversões, atuador; latência e vazão |
| `test_bus_roundtrip`           | 8      | Ponta a ponta ROS → gateway → rede → proxies → ROS, e comandos até `/cmd_vel` (event-driven e time-triggered) |

Para rodar só as suites funcionais:
`cd build/semantic_gateway && ctest -R "test_(si_units|smartdata_runtime|gateway_pipeline|bus_roundtrip)"`.

Os linters cosméticos (cpplint, copyright, uncrustify, lint_cmake, pep257,
flake8) falham por falta de boilerplate em headers próprios e pelo conteúdo
do submódulo `sdav-ieee1722-smartdada-adapter/`. Sem impacto funcional.

### Métricas medidas (ambiente container, x86_64)

| Métrica (LaserScan, 100 feixes)                       | Meta     | Medido                |
|-------------------------------------------------------|----------|-----------------------|
| Latência da tradução (ROS → transducers → SmartData)  | < 100 µs | **≈ 7,5 µs** p95      |
| Tradução + envio das 100 respostas (interesse ativo)  | —        | **≈ 2–2,6 ms** p95    |
| Vazão da tradução                                     | > 1200 scans/s | **≈ 145 k scans/s** |
| Vazão com envio                                       | > 4200 SmartData/s | **≈ 60–75 k datagramas/s** |
| Fidelidade campo a campo scalar / array / composite   | exata    | passa                 |

O custo dominante é o envio: um datagrama de 78 B por instância (um
`sendto` por feixe).

## Cenário 1 — Synthetic publisher

Publica `LaserScan` (360 feixes, 10 Hz), `Imu` (100 Hz) e `Temperature`
(2 Hz) falsos sobre um TF estático.

```bash
ros2 launch semantic_gateway gateway.launch.py
```

O launch sobe três nós: `demo_publisher`, `semantic_gateway` (transducers
e SmartData ADVERTISED/COMMANDED, espelho em `/smartdata/*`) e `proxy`
(anuncia interesses, republica em `<topic>_recovered` e envia comandos).
Em outro terminal:

```bash
ros2 topic list | grep -E 'smartdata|_recovered|_command'
ros2 topic echo /smartdata/temperature           # espelho local (Kelvin)
ros2 topic echo /temperature_recovered           # ida e volta pela rede SmartData

# atuação: a mensagem vira Commands SmartData, que o gateway aplica em /cmd_vel
ros2 topic pub --once /cmd_vel_command geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

Interesses time-triggered em vez de event-driven:
`ros2 run semantic_gateway proxy_node --ros-args -p period_ms:=100`.
Restringir a uma região: `-p region_x:=0.0 -p region_y:=0.0 -p region_radius:=2.0`.

Para ver a rede diretamente: `tcpdump -i any 'udp port 5000'`.

## Cenário 2 — Gazebo + TurtleBot3

TurtleBot3 burger num mundo Gazebo, navegando em círculo. Sensores reais
simulados pelo modelo SDF do Gazebo, TF dinâmico.

```bash
export TURTLEBOT3_MODEL=burger

# Headless (recomendado em WSL)
ros2 launch semantic_gateway gazebo_gateway.launch.py \
    gui:=false \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world

# Com GUI (requer X11/WSLg)
ros2 launch semantic_gateway gazebo_gateway.launch.py \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```

A configuração default cobre tanto `/imu` (TurtleBot3) quanto `/imu/data`
(publisher sintético), então o mesmo gateway serve ambos os cenários.

## Adicionar um sensor

Editar `config/sensors.yaml` (seção `sensors:`; atuadores em `actuators:`):

```yaml
  - topic: /pressure
    type: sensor_msgs/msg/FluidPressure
    decomposition: scalar
    value_field: fluid_pressure
    unit: pascal
    expiry_ms: 500
```

E rebuildar (`colcon build --packages-select semantic_gateway`). O codegen
gera `Pressure_Transducer` (unidade Pa), o gateway o exporta como SmartData
e o `proxy` passa a republicar `/pressure_recovered`.

## Receber SmartData de outros nós

O `proxy_node` também publica SmartData que **não** vêm deste gateway
(outro gateway, um nó SDAV/EPOS): basta declarar na seção `imports:` o que
ele deve pedir na rede e em que mensagem ROS publicar.

```yaml
imports:
  - topic: /lab/temperature
    type: sensor_msgs/msg/Temperature
    decomposition: scalar
    unit: kelvin
    device: any                 # todos os produtores de kelvin (ou um id)
    value_field: temperature
    value_transform: { kind: linear, a: 1.0, b: 273.15 }
    frame_id: "sd_{device}"     # identifica o produtor
```

Opções (região própria, período, `stamp: receive`, `composite` com um
dispositivo por campo, unidade por código cru): `docs/mapping.md` §5.6.

Detalhes completos da DSL: `docs/mapping.md`. Para acrescentar uma
unidade nova, editar `UNIT_TO_SYMBOL` em `codegen/generate.py` e adicionar
a constante em `include/si_units.hpp`.

## Rede SmartData

1. **Rede SmartData** — UDP multicast `224.1.1.1:5000`, mensagens
   Interest / Response / Command com o layout TSTP (`docs/wire_format.md`).
   Um SmartData ADVERTISED só responde quando há interesse por ele; o
   consumidor escolhe unidade, região, expiry e período.
2. **Espelho ROS** — tópico `/smartdata/<topic>` do tipo
   `semantic_gateway/msg/SmartData`, com toda atualização local
   (instrumentação: `ros2 topic echo`, rosbag). Desliga com
   `enable_ros_mirror:=false`.

O `proxy_node` é o lado remoto: proxies `Interested_SmartData` que
reconstroem as mensagens ROS e transformam `<topic>_command` em Commands.

## Estrutura do código

```
tcc/
├── package.xml                          # nome do pacote = semantic_gateway
├── CMakeLists.txt
├── README.md
├── proposta.md                          # proposta de TCC
├── smartdata.pdf / smartdata-extra.md   # paper de referência
├── docker/                              # Dockerfile + entrypoint
├── docker-compose.yml
├── docs/
│   ├── mapping.md                       # modelo de transducers, dinâmica, DSL
│   └── wire_format.md                   # mensagens Interest/Response/Command
├── msg/SmartData.msg                    # tipo do espelho ROS
├── config/sensors.yaml                  # tabela de transducers (build-time)
├── codegen/generate.py                  # gera transducers, gateway e proxies
├── include/
│   ├── si_units.hpp                     # encoding dimensional 32-bit
│   └── smartdata/
│       ├── wire.hpp                     # mensagens de rede (layout TSTP)
│       ├── network.hpp                  # rede multicast + scheduler
│       ├── transducer.hpp               # Transducer (UNIT, TYPE, active)
│       ├── smartdata.hpp                # Responsive_ / Interested_SmartData
│       └── ros.hpp                      # Source/Sink: tópicos ROS ↔ transducers
├── src/
│   ├── smartdata/network.cpp            # run-time: sockets e timer
│   ├── gateway_node.cpp                 # sensores/atuadores ROS → SmartData
│   ├── proxy_node.cpp                   # proxies remotos → ROS
│   └── demo_publisher_node.cpp          # sensores sintéticos
├── launch/
│   ├── gateway.launch.py                # cenário sintético
│   └── gazebo_gateway.launch.py         # cenário Gazebo + TB3
└── test/
    ├── test_si_units.cpp                # encoding Fig.1
    ├── test_smartdata_runtime.cpp       # dinâmica SmartData sem ROS
    ├── test_gateway_pipeline.cpp        # transducers gerados + desempenho
    └── test_bus_roundtrip.cpp           # ponta a ponta pela rede SmartData
```

## Limpeza

```bash
exit                            # sair do container
docker compose stop ros         # parar
docker compose down -v          # remover container + volumes
```
