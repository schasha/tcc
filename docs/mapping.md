# Mapeamento semântico ROS 2 → SmartData

Este documento é a referência única do **mapeamento semântico**: como cada
campo de uma mensagem ROS vira parte de uma SmartData. O layout binário do
transporte está em `docs/wire_format.md`; a estrutura ROS/DDS espelhada
está em `msg/SmartData.msg`. O paper de referência é Fröhlich (2018).

---

## 1. Codegen build-time

A descrição declarativa dos sensores vive em `config/sensors.yaml`. Em
build time, `codegen/generate.py` lê o YAML e emite quatro headers tipados
em `build/generated/`:

| Header                       | Conteúdo                                                                  |
|------------------------------|---------------------------------------------------------------------------|
| `parsed_decompose.hpp`       | `decompose_<topic>(msg, ox, oy, oz, out)` por sensor                      |
| `parsed_dispatch.hpp`        | `register_all(node, lookup, on_sd)` cria `Subscription<T>` por sensor     |
| `parsed_recompose.hpp`       | `State_<topic>`, `feed_<topic>`, `flush_<topic>` para reconstrução tipada |
| `parsed_sd_dispatch.hpp`     | `Consumers` + `dispatch_sd(c, sd)` roteia SD por `(unit, dev_base range)` |

Não há introspecção em runtime, nem parsing de YAML em runtime, nem string
lookup de campo. Adicionar sensor = editar `sensors.yaml` + `colcon build`.

---

## 2. Codificação dimensional da `unit` (Fig. 1 do paper)

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

Constantes em `include/si_units.hpp`: `METER`, `KILOGRAM`, `SECOND`,
`AMPERE`, `KELVIN`, `MOLE`, `CANDELA`, `RADIAN`, `METER_PER_SECOND`,
`METER_PER_SEC_SQ`, `RADIAN_PER_SECOND`, `NEWTON`, `PASCAL`, `VOLT`.

Adicionar nova unidade: linha em `si_units.hpp` + entrada na tabela
`UNIT_TO_SYMBOL` de `codegen/generate.py`.

---

## 3. DSL do `sensors.yaml`

### 3.1 Esquema geral

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
```

### 3.2 `scalar` — 1 SmartData por mensagem

```yaml
decomposition: scalar
value_field:   <dotted.path>
unit:          <unit_name>
value_transform:                                 # opcional
  kind: linear
  a:    1.0                                      # value' = a*value + b
  b:    273.15
```

Exemplo: `Temperature` em °C → SmartData em Kelvin (a=1, b=273.15). A
reconstrução aplica a inversa `(v − b)/a`.

### 3.3 `array` — N SmartData por mensagem

```yaml
decomposition: array
array_field: <dotted.path>
unit:        <unit_name>
filter:                                          # opcional
  kind: none | finite | finite_within
  min_field: <dotted.path>                       # só para finite_within
  max_field: <dotted.path>
```

Cada índice `i` do vetor vira `SmartData(value=array[i], dev=dev_base+i)`.
Elementos rejeitados pelo filtro não geram SmartData — o consumidor vê o
índice como buraco (valor zero após o resize).

### 3.4 `composite` — vários grupos com unidades distintas

```yaml
decomposition: composite
groups:
  - unit: <unit_a>
    fields: [<dotted.path>, ...]   # cada campo vira uma SmartData
  - unit: <unit_b>
    fields: [...]
```

Os dispositivos são alocados em ordem dentro do `dev_base` do sensor.

---

## 4. Nomes de unidade reconhecidos

| Nome YAML                          | Constante              | Dimensão                              |
|------------------------------------|------------------------|---------------------------------------|
| `meter` / `m`                      | METER                  | `m=+1`                                |
| `kilogram` / `kg`                  | KILOGRAM               | `kg=+1`                               |
| `second` / `s`                     | SECOND                 | `s=+1`                                |
| `kelvin` / `K`                     | KELVIN                 | `K=+1`                                |
| `ampere` / `A`                     | AMPERE                 | `A=+1`                                |
| `mole` / `mol`                     | MOLE                   | `mol=+1`                              |
| `candela` / `cd`                   | CANDELA                | `cd=+1`                               |
| `radian` / `rad`                   | RADIAN                 | `rad=+1`                              |
| `meter_per_second` / `m_per_s`     | METER_PER_SECOND       | `m=+1, s=-1`                          |
| `meter_per_sec_sq` / `m_per_s2`    | METER_PER_SEC_SQ       | `m=+1, s=-2`                          |
| `rad_per_sec` / `radian_per_second`| RADIAN_PER_SECOND      | `rad=+1, s=-1`                        |
| `newton` / `N`                     | NEWTON                 | `m=+1, kg=+1, s=-2`                   |
| `pascal` / `Pa`                    | PASCAL                 | `m=-1, kg=+1, s=-2`                   |
| `volt` / `V`                       | VOLT                   | `m=+2, kg=+1, s=-3, A=-1`             |

Nome desconhecido faz o codegen falhar com mensagem clara durante o build.

---

## 5. Discriminação de sensor no bus único — `(unit, dev_base range)`

O bus UDP multicast carrega SmartData de todos os sensores no mesmo grupo
(`224.1.1.1:5000`). Para o consumidor saber a qual sensor cada SD pertence,
o codegen aloca um intervalo de `origin_device` por sensor, sequencialmente:

| Posição no YAML | dev_base       | dev_range  |
|-----------------|----------------|------------|
| sensor 0        | 0x00000        | 0x10000    |
| sensor 1        | 0x10000        | 0x10000    |
| sensor i        | i × 0x10000    | 0x10000    |

Cada sensor tem 16 bits de dispositivos (65535 máx). Para `array` isso
limita feixes; para `composite` limita campos. Aumentar = mudar
`DEV_RANGE_PER_SENSOR` em `codegen/generate.py`.

O dispatcher (`parsed_sd_dispatch.hpp`) roteia com
`(unit guard, dev em [base, base+range))`.

---

## 6. Sensores na configuração default (`config/sensors.yaml`)

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
N SmartData (uma por feixe). Feixes `inf`/`NaN`/fora-de-faixa filtrados.

### Imu (`/imu/data` e `/imu`)
```yaml
- topic: /imu/data    # ou /imu para TurtleBot3
  type:  sensor_msgs/msg/Imu
  decomposition: composite
  groups:
    - unit: meter_per_sec_sq
      fields: [linear_acceleration.x, .y, .z]
    - unit: rad_per_sec
      fields: [angular_velocity.x, .y, .z]
  expiry_ms: 10
```
6 SmartData (3 m·s⁻² + 3 rad·s⁻¹). Quaternion fica de fora — é
adimensional, fora do mundo SI da Fig. 1.

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
1 SmartData em Kelvin (conversão °C → K aplicada no decompose; reconstrução
aplica a inversa).

---

## 7. Validador como reconstrução (proposta §3.3)

A fidelidade do gateway é definida operacionalmente: dada uma mensagem ROS
`m`, o gateway produz um burst `S = decompose(m)`. Fidelidade vale quando
`recompose(S) ≡ campos_relevantes(m)`.

Dois testes ancoram isso:

- `test/test_validator.cpp` — exercício direto do par codegen, sem
  rede/DDS. Inclui asserção explícita de **MSE → 0** para o LaserScan
  (proposta §3.3) e `EXPECT_NEAR` por campo para scalar e composite.
- `test/test_bus_roundtrip.cpp` — atravessa o caminho completo
  `ROS → gateway → UDP multicast → sd_to_ros → ROS`, validando que o
  transporte preserva semântica bit a bit.

Campos não-SI (`range_min`, `angle_increment`, quaternions, `frame_id`,
covariâncias etc.) **não** atravessam a SmartData por design — são
metadados de framing, não quantidades físicas. A reconstrução é exatamente
do conjunto de campos que o spec elegeu como semânticos.

---

## 8. Mapa do código relevante a este documento

| Camada                  | Arquivo(s)                                       |
|-------------------------|--------------------------------------------------|
| Vocabulário SI          | `include/si_units.hpp`                           |
| DSL parseada            | `config/sensors.yaml`                            |
| Codegen                 | `codegen/generate.py`                            |
| Decompose por sensor    | `build/generated/parsed_decompose.hpp` (gerado)  |
| Dispatch ROS→bus        | `build/generated/parsed_dispatch.hpp` (gerado)   |
| Recompose por sensor    | `build/generated/parsed_recompose.hpp` (gerado)  |
| Dispatch bus→ROS        | `build/generated/parsed_sd_dispatch.hpp` (gerado)|
| Factory envelope        | `include/sd_codec.hpp`                           |
| Wire format binário     | `docs/wire_format.md` + `include/sd_wire.hpp`    |
| IDL ROS/DDS (mirror)    | `msg/SmartData.msg`                              |
