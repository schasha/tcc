# Semantic Gateway — ROS 2 → SmartData

Componente C++ para ROS 2 que traduz mensagens de sensores em SmartData
(Fröhlich, 2018), com decomposição declarada em YAML e código tipado
gerado em build time. Os pacotes SmartData são publicados (a) em tópicos
DDS para inspeção, e (b) no barramento UDP multicast nativo da
SmartData, onde transducers externos podem consumir diretamente.

Material da proposta: `proposta.md`. Mapeamento técnico: `docs/mapping.md`
e `docs/wire_format.md`.

## Arquitetura num parágrafo

`config/sensors.yaml` descreve cada sensor (tipo ROS, decomposição,
unidade, expiry, transform). Em build time, `codegen/generate.py` emite
quatro headers C++ tipados: `parsed_decompose.hpp` (`decompose_<name>` por
sensor), `parsed_dispatch.hpp` (`register_all` cria `Subscription<T>`),
`parsed_recompose.hpp` (estado + `feed_<name>` para o lado inverso) e
`parsed_sd_dispatch.hpp` (roteia SD por `(unit, dev_base range)`). Em
runtime: zero introspecção, zero string lookup, zero parsing de YAML.

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

**14 testes funcionais (gtest), todos verdes:**

| Suite                          | Testes | O que cobre                                                       |
|--------------------------------|--------|-------------------------------------------------------------------|
| `test_si_units`                | 5      | Encoding 32-bit da Fig.1 (METER, accel, ω, VOLT, `to_string`)     |
| `test_gateway_pipeline`        | 3      | Fan-out (ROS → SmartData), latência codegen p95, throughput       |
| `test_validator`               | 3      | Fidelidade direta `decompose_<name>` ↔ `feed_<name>` + MSE LaserScan |
| `test_bus_roundtrip`           | 3      | End-to-end ROS → gateway → UDP multicast → sd_to_ros → ROS        |

Os linters cosméticos (cpplint, copyright, uncrustify, lint_cmake, pep257,
flake8) falham por falta de boilerplate em headers próprios e pelo conteúdo
do submódulo `sdav-ieee1722-smartdada-adapter/`. Sem impacto funcional.

### Métricas medidas (ambiente container, x86_64)

| Métrica                                         | Meta da proposta | Medido          |
|-------------------------------------------------|------------------|-----------------|
| Latência de marshaling (LaserScan 100 feixes)   | < 150 µs         | **0.8 µs** p95  |
| Throughput sustentado (`decompose_scan`)        | —                | **1.2 M scans/s** = 122 M SmartData/s |
| MSE da reconstrução LaserScan                   | → 0              | **0.0**         |
| Fidelidade campo-a-campo scalar / array / composite | exata        | passa           |

## Cenário 1 — Synthetic publisher

Publica `LaserScan` (360 feixes, 10 Hz), `Imu` (100 Hz) e `Temperature`
(2 Hz) falsos sobre um TF estático.

```bash
ros2 launch semantic_gateway gateway.launch.py
```

O launch sobe três nós: `demo_publisher`, `semantic_gateway` (decompose,
broadcast no bus e mirror em `/smartdata/*`) e `sd_to_ros` (lê o bus e
republica em `<topic>_recovered`). Em outro terminal:

```bash
ros2 topic list | grep -E 'smartdata|_recovered'
ros2 topic echo /smartdata/temperature           # mirror DDS
ros2 topic echo /temperature_recovered           # ida e volta pelo bus
```

Para sniffar o bus binário diretamente: `tcpdump -i any 'udp port 5000'`.

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

Editar `config/sensors.yaml`:

```yaml
  - topic: /pressure
    type: sensor_msgs/msg/FluidPressure
    decomposition: scalar
    value_field: fluid_pressure
    unit: pascal
    expiry_ms: 500
```

E rebuildar (`colcon build --packages-select semantic_gateway`). O codegen
roda; o tópico `/smartdata/pressure` aparece no próximo `ros2 launch`.

Detalhes completos da DSL: `docs/mapping.md`. Para acrescentar uma
unidade nova, editar `UNIT_TO_SYMBOL` em `codegen/generate.py` e adicionar
a constante em `include/si_units.hpp`.

## Transporte do SmartData

O gateway emite cada SmartData em duas vias:

1. **Barramento nativo SmartData** — UDP multicast `224.1.1.1:5000`,
   payload de 72 bytes little-endian. Layout em `docs/wire_format.md`. É
   o caminho que transducers externos compatíveis com SmartData escutam
   diretamente.
2. **Mirror DDS** — tópico ROS `/smartdata/<sensor>` do tipo
   `semantic_gateway/msg/SmartData`. Útil para `ros2 topic echo`,
   gravação em rosbag e integração com outros nós ROS. Pode ser desligado
   com `enable_ros_mirror:=false`.

O `sd_to_ros_node` faz o caminho inverso a partir do bus binário,
servindo como o "consumidor DDS genérico" descrito na proposta §3.3.

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
│   ├── mapping.md                       # DSL + unidades + dispatch + validator
│   └── wire_format.md                   # layout do barramento UDP (72 B)
├── msg/SmartData.msg                    # IDL do tipo ROS/DDS
├── config/sensors.yaml                  # tabela de tradução (build-time)
├── codegen/generate.py                  # gera 4 headers tipados
├── include/
│   ├── si_units.hpp                     # encoding dimensional 32-bit
│   ├── sd_codec.hpp                     # make_sd(): factory de envelope
│   ├── sd_wire.hpp                      # struct packed 72 B + (to|from)_wire
│   └── sd_bus.hpp                       # API do bus UDP multicast
├── src/
│   ├── sd_bus.cpp                       # implementação do bus
│   ├── gateway_node.cpp                 # ROS → SmartData (codegen)
│   ├── sd_to_ros_node.cpp               # SmartData → ROS (codegen)
│   └── demo_publisher_node.cpp          # sensores sintéticos
├── launch/
│   ├── gateway.launch.py                # cenário sintético
│   └── gazebo_gateway.launch.py         # cenário Gazebo + TB3
└── test/
    ├── test_si_units.cpp                # encoding Fig.1
    ├── test_gateway_pipeline.cpp        # fan-out + latência + throughput
    ├── test_validator.cpp               # fidelidade + MSE (proposta §3.3)
    └── test_bus_roundtrip.cpp           # end-to-end pelo bus UDP
```

## Limpeza

```bash
exit                            # sair do container
docker compose stop ros         # parar
docker compose down -v          # remover container + volumes
```
