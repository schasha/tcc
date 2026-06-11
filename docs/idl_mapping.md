# Mapeamento ROS 2 → SmartData

Este documento especifica formalmente como qualquer mensagem ROS 2 de
sensor é convertida em uma ou mais instâncias `SmartData` publicadas sobre DDS.

A referência é:

> Fröhlich, A. A. (2018) "SmartData: an IoT-ready API for sensor networks",

---

## 1. Estratégia: motor genérico dirigido por DSL

O gateway não tem mapeamento hardcoded por tipo de sensor. Em vez disso,
um arquivo YAML (`config/sensors.yaml`) descreve cada sensor a ser traduzido,
e o motor em runtime:

1. carrega o YAML em structs C++ (`spec.hpp`);
2. cria uma `rclcpp::GenericSubscription` por sensor — recebe bytes em vez de
   tipo C++ resolvido em compile-time;
3. deserializa os bytes via `rosidl_typesupport_introspection_cpp` para um
   buffer tipado de acordo com a string `type` do YAML;
4. lê os campos do buffer **por nome pontuado** (`header.stamp`,
   `linear_acceleration.x`, `ranges`) usando a tabela de membros da
   introspecção;
5. aplica a decomposição configurada e publica as SmartData resultantes.

Adicionar um sensor novo ao gateway é uma operação de configuração

---

## 2. Layout do pacote SmartData (IDL)

Definição em `msg/SmartData.msg`; após `colcon build` fica disponível como
`semantic_gateway::msg::SmartData` em C++.

```idl
uint32  unit            //  32-bit dimensional SI/digital encoding
float64 value           //  Native Float64 (N=3)
float64 origin_x        //  Location where data was produced (meters)
float64 origin_y
float64 origin_z
uint32  origin_device   //  'd' — disambiguates same-Unit transducers at (x,y,z,t)
uint64  origin_time     //  µs since UNIX epoch
uint64  expiry          //  µs relative to origin_time
uint8[16] mac           //  Poly1305-AES
```

---

## 3. Codificação dimensional da `unit` (Figura 1 do paper)

```
bit 31  30..29  28..27  26..24  23..21  20..18  17..15  14..12  11..9  8..6   5..3   2..0
    T   N       M       sr      rad     m       kg      s       A      K      mol    cd
```

| Campo | Bits | Significado                                                              |
|-------|------|--------------------------------------------------------------------------|
| T     |  1   | 0 = Digital · 1 = SI Physical Quantity                                   |
| N     |  2   | Tipo numérico: 0=Int32 · 1=Int64 · 2=Float32 · 3=Float64                 |
| M     |  2   | Modificador: 0=direct · 1=1/U · 2=log(U) · 3=1/log(U)                    |
| sr…cd |  3×9 | Expoente da dimensão SI correspondente (offset +4, faixa −4…+3)          |

Constantes prontas em `si_units.hpp` (via `constexpr`): `METER`, `KILOGRAM`,
`SECOND`, `AMPERE`, `KELVIN`, `MOLE`, `CANDELA`, `RADIAN`,
`METER_PER_SECOND`, `METER_PER_SEC_SQ`, `RADIAN_PER_SECOND`, `NEWTON`,
`PASCAL`, `VOLT`. 
Para adicionar nova unidade adicione uma linha de `encode_si(...)`.

---

## 4. DSL de mapeamento (`config/sensors.yaml`)

### 4.1 Esquema geral

```yaml
defaults:
  target_frame: <world frame para TF lookup>     # default: map

sensors:
  - topic:         </input/ros/topic>            # obrigatório
    type:          <pkg/msg/Type>                # obrigatório
    decomposition: scalar | array | composite    # obrigatório
    stamp_field:   <dotted.path>                 # default: header.stamp
    frame_field:   <dotted.path>                 # default: header.frame_id
    expiry_ms:     <int>                         # validade em ms
    output_topic:  </smartdata/...>              # default: /smartdata + topic
    # Campos específicos da decomposição (ver §4.2-4.4)
```

### 4.2 Decomposição `scalar` — 1 SmartData por mensagem ROS

```yaml
decomposition: scalar
value_field:   <dotted.path>                     # campo escalar (double/float/int)
unit:          <unit_name>                       # ver tabela §5
value_transform:                                 # opcional
  kind: linear
  a:    1.0                                      # value' = a*value + b
  b:    273.15
```

Exemplo: `Temperature` em °C → SmartData em Kelvin.

### 4.3 Decomposição `array` — N SmartData por mensagem ROS

```yaml
decomposition: array
array_field: <dotted.path>                       # vetor de escalares
unit:        <unit_name>                         # mesmo unit para todos os elementos
filter:                                          # opcional
  kind: none | finite | finite_within
  min_field: <dotted.path>                       # apenas para finite_within
  max_field: <dotted.path>
```

Cada elemento `i` do vetor vira uma SmartData com `value = array[i]` e
`origin_device = i`. Elementos rejeitados pelo filtro **não** geram
SmartData (mas mantêm o `i` original — o consumer pode reconstruir a
geometria correta).

### 4.4 Decomposição `composite` — vários grupos com unidades distintas

```yaml
decomposition: composite
groups:
  - unit: <unit_a>
    fields: [<dotted.path>, <dotted.path>, ...]   # cada campo vira uma SmartData
  - unit: <unit_b>
    fields: [...]
```

---

## 5. Nomes de unidade reconhecidos

Implementados em `config_loader.cpp::unit_table()`. Cada nome resolve para
a constante correspondente de `si_units.hpp`:

| Nome YAML         | Equivalente             | Encoding                              |
|-------------------|-------------------------|----------------------------------------|
| `meter` / `m`     | METER                   | só `m=+1`                              |
| `kilogram` / `kg` | KILOGRAM                | só `kg=+1`                             |
| `second` / `s`    | SECOND                  | só `s=+1`                              |
| `kelvin` / `K`    | KELVIN                  | só `K=+1`                              |
| `ampere` / `A`    | AMPERE                  | só `A=+1`                              |
| `mole` / `mol`    | MOLE                    | só `mol=+1`                            |
| `candela` / `cd`  | CANDELA                 | só `cd=+1`                             |
| `radian` / `rad`  | RADIAN                  | só `rad=+1`                            |
| `meter_per_second`/`m_per_s` | METER_PER_SECOND | `m=+1, s=-1`                       |
| `meter_per_sec_sq`/`m_per_s2`| METER_PER_SEC_SQ | `m=+1, s=-2`                       |
| `rad_per_sec`/`radian_per_second` | RADIAN_PER_SECOND | `rad=+1, s=-1`             |
| `newton` / `N`    | NEWTON                  | `m=+1, kg=+1, s=-2`                    |
| `pascal` / `Pa`   | PASCAL                  | `m=-1, kg=+1, s=-2`                    |
| `volt` / `V`      | VOLT                    | `m=+2, kg=+1, s=-3, A=-1`              |

YAML com nome desconhecido falha com `std::runtime_error` no `load_config_file`
durante a inicialização do gateway 

---

## 6. Mapeamento dos sensores da validação

A proposta cita LaserScan, Imu e Temperature como sensores representativos para teste.

### LaserScan
```yaml
- topic: /scan
  type:  sensor_msgs/msg/LaserScan
  decomposition: array
  array_field: ranges
  unit:        meter
  filter: { kind: finite_within, min_field: range_min, max_field: range_max }
  expiry_ms: 100
```
Resultado: N SmartData (uma por feixe), origin = posição do sensor lida do
TF, `dev = i`, valor = distância. Feixes `inf`/`NaN`/fora-de-faixa filtrados.

### Imu
```yaml
- topic: /imu/data
  type:  sensor_msgs/msg/Imu
  decomposition: composite
  groups:
    - unit: meter_per_sec_sq
      fields: [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]
    - unit: rad_per_sec
      fields: [angular_velocity.x, angular_velocity.y, angular_velocity.z]
  expiry_ms: 10
```
Resultado: 6 SmartData (3 m·s⁻² + 3 rad·s⁻¹) em mesmo `(x,y,z,t)`.
Orientação (quaternion) intencionalmente fora — adimensional, fora do mundo
SI da Fig. 1 do paper.

### Temperature
```yaml
- topic: /temperature
  type:  sensor_msgs/msg/Temperature
  decomposition: scalar
  value_field: temperature
  value_transform: { kind: linear, a: 1.0, b: 273.15 }
  unit: kelvin
  expiry_ms: 500
```
Resultado: 1 SmartData em Kelvin (conversão °C → K aplicada antes do publish).

---

## 7. Demonstração de extensibilidade

Para acrescentar sensor de pressão

```yaml
- topic: /pressure
  type:  sensor_msgs/msg/FluidPressure
  decomposition: scalar
  value_field: fluid_pressure
  unit: pascal
  expiry_ms: 500
```

---

## 8. Tópicos DDS publicados

Cada SensorSpec produz um tópico de saída cujo default é `/smartdata` + tópico
de entrada. Pode ser sobrescrito com `output_topic:` no YAML.

| Tópico ROS in     | Tópico SmartData out                                        |
|-------------------|-------------------------------------------------------------|
| `/scan`           | `/smartdata/scan`                                           |
| `/imu/data`       | `/smartdata/imu/data`                                       |
| `/temperature`    | `/smartdata/temperature`                                    |

O consumer demultiplexa por `unit` para distinguir as diferentes
quantidades dentro de um mesmo tópico composto (e.g., `/smartdata/imu/data`
carrega tanto `m·s⁻²` quanto `rad·s⁻¹`).

---

## 9. Mapa do código

| Camada                  | Arquivo(s)                            | Responsabilidade                                 |
|-------------------------|---------------------------------------|--------------------------------------------------|
| Contrato do dado        | `msg/SmartData.msg`                   | layout do pacote de rede (Fig. 1 do paper)       |
| Vocabulário SI          | `include/si_units.hpp`                | encoding dimensional 32-bit + constantes         |
| Spec parseada           | `include/spec.hpp`                    | structs C++ representando o YAML                 |
| Parser de config        | `include/config_loader.hpp` + `.cpp`  | YAML → spec, tabela de unidades, validação       |
| Introspecção            | `include/field_access.hpp` + `.cpp`   | `MessageView`: leitura de campos ROS por nome    |
| Decomposição            | `include/decomposer.hpp`              | spec + MessageView → vector\<SmartData\>          |
| Nó gateway              | `src/gateway_node.cpp`                | orquestração: TF + introspecção + publish        |
| Nó validator            | `src/validator_node.cpp`              | verificação de fidelidade em runtime             |
| Publisher sintético     | `src/demo_publisher_node.cpp`         | mensagens ROS falsas para testes                 |
| Config padrão           | `config/sensors.yaml`                 | sensores do cenário sintético                    |
| Config Gazebo           | `config/gazebo_sensors.yaml`          | sensores do TurtleBot3                           |
| Launch                  | `launch/gateway.launch.py`            | cenário sintético                                |
| Launch Gazebo           | `launch/gazebo_gateway.launch.py`     | cenário TB3 + Gazebo                             |
| Testes                  | `test/test_smartdata_marshaling.cpp`  | unit: SI, config, introspecção, decompose, latência |
| Testes integração       | `test/test_gateway_pipeline.cpp`      | pipeline ROS → SmartData fim-a-fim               |
