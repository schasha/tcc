# Semantic Gateway — ROS 2 → SmartData

Componente C++ que traduz mensagens de sensores ROS 2 para o formato
SmartData (Fröhlich, 2018) sobre DDS, com decomposição configurável por
YAML.

## Pré-requisitos

- Linux (testado em WSL2 Ubuntu 22.04)
- Docker + Docker Compose
- Para o Gazebo: `xhost +local:` ou ambiente WSLg para a GUI (opcional)

Não é necessário instalar ROS 2 no host — tudo roda no container.

## Setup

```bash
# 1. Construir a imagem
docker compose build

# 2. Subir o container
docker compose up -d ros

# 3. Entrar no container
docker exec -it tcc-ros bash
```

A partir daqui, todos os comandos rodam dentro do container, em `/ros2_ws`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_gateway --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Testes

```bash
colcon test --packages-select semantic_gateway --event-handlers console_direct+
```

Resultado esperado: 20 gtests funcionais passam (5 SI units, 4 config
loader, 3 introspection, 4 decomposer, 2 latência, 2 pipeline).

Os linters cosméticos (cpplint, copyright, etc.) podem falhar — não afetam
funcionalidade.

## Cenário 1 — Synthetic publisher

Publica `LaserScan`, `Imu` e `Temperature` falsos em um TF estático.

```bash
ros2 launch semantic_gateway gateway.launch.py
```

Logs esperados:
```
[semantic_gateway] registered sensor: topic=/scan ... out=/smartdata/scan
[semantic_gateway] registered sensor: topic=/imu/data ... out=/smartdata/imu/data
[semantic_gateway] registered sensor: topic=/temperature ... out=/smartdata/temperature
[smartdata_validator] watching: /scan ⇄ /smartdata/scan
[smartdata_validator]  /scan  obs=N matched=N expired=0 unmatched=0 MSE=0 ...
```

Em outro terminal, inspecionar SmartData:
```bash
ros2 topic list | grep smartdata
ros2 topic echo /smartdata/temperature
```

## Cenário 2 — Gazebo + TurtleBot3

TurtleBot3 burger num mundo com obstáculos, girando em círculo. Valida o
gateway com TF dinâmico e sensores reais simulados.

```bash
export TURTLEBOT3_MODEL=burger

# Headless (recomendado, mais leve)
ros2 launch semantic_gateway gazebo_gateway.launch.py \
    gui:=false \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world

# Com GUI (requer X11/WSLg)
ros2 launch semantic_gateway gazebo_gateway.launch.py \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```

Resultado típico do validator
```
 /scan  obs=42000  matched=42000  expired=0  MSE=0  spatial_RMSE=2.5 mm
 /imu   obs=40000  matched=40000  expired=0  MSE=0  spatial_RMSE=1.2 mm
```

## Adicionar um sensor sem recompilar

Editar `config/sensors.yaml` e adicionar entrada nova:

```yaml
  - topic: /pressure
    type: sensor_msgs/msg/FluidPressure
    decomposition: scalar
    value_field: fluid_pressure
    unit: pascal
    expiry_ms: 500
```

Relançar o gateway. O tópico `/smartdata/pressure` aparece automaticamente.

Detalhes da DSL: `docs/idl_mapping.md`.

## Estrutura do código

```
tcc/                                 ← raiz do projeto + pacote ROS
├── package.xml                      ← define o nome do pacote (semantic_gateway)
├── CMakeLists.txt
├── README.md
├── proposta.md                      ← proposta de TCC
├── docker/                          ← Dockerfile + entrypoint
├── docker-compose.yml
├── docs/                            ← idl_mapping.md, performance_report.md
├── msg/
│   └── SmartData.msg                ← formato do dado (paper Fig. 1)
├── config/
│   ├── sensors.yaml                 ← tabela de tradução (a "DSL")
│   └── gazebo_sensors.yaml          ← config específica do TB3 + Gazebo
├── include/
│   ├── si_units.hpp                 ← 32-bit dimensional SI encoding
│   ├── spec.hpp                     ← structs C++ da config parseada
│   ├── config_loader.hpp            ← YAML → spec
│   ├── field_access.hpp             ← introspecção runtime de mensagens ROS
│   └── decomposer.hpp               ← spec + msg → vector<SmartData>
├── src/
│   ├── config_loader.cpp
│   ├── field_access.cpp
│   ├── gateway_node.cpp             ← nó principal (ROS → SmartData)
│   ├── validator_node.cpp           ← valida fidelidade
│   └── demo_publisher_node.cpp      ← sensores sintéticos
├── launch/
│   ├── gateway.launch.py            ← cenário sintético
│   └── gazebo_gateway.launch.py     ← cenário Gazebo + TB3
└── test/                            ← 20 gtests cobrindo as 5 camadas
```

## Limpeza

```bash
# Sair do container
exit

# Parar o container
docker compose stop ros

# Remover container + volumes (build cache)
docker compose down -v
```
