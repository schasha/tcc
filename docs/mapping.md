# Mapeamento semântico ROS 2 → SmartData

Este documento é a referência do **mapeamento**: como um tópico ROS vira
transducers SmartData, e como esses transducers se comportam na rede segundo
a dinâmica de Fröhlich (2018). O formato binário das mensagens está em
`docs/wire_format.md`.

---

## 1. Modelo: tópicos ROS como transducers

No paper, um SmartData é ligado a um **Transducer**, uma classe que declara
a grandeza física que mede ou controla (`UNIT`, Fig. 1), se é sensor e/ou
atuador (`TYPE`) e se avisa sozinha quando há dado novo (`active`, o caso
"por interrupção"). Várias instâncias da mesma classe num nó são
distinguidas pelo `dev`.

O gateway aplica isso aos tópicos ROS:

| Conceito do paper                     | No gateway                                                        |
|---------------------------------------|-------------------------------------------------------------------|
| Transducer (uma classe por grandeza)  | uma classe por (tópico × grandeza): `/imu/data` → aceleração e velocidade angular |
| instâncias distinguidas por `dev`     | eixos x/y/z, feixes do LiDAR: `dev = DEV_BASE + índice`           |
| transducer ativo (interrupção)        | a chegada da mensagem ROS: a `Subscription` notifica os transducers |
| `sense()` / `actuate()`               | leitura direta e tipada do campo / escrita do campo no setpoint   |
| SmartData local (1º construtor)       | `Responsive_SmartData<T>(dev, expiry, mode, period)` por instância |
| SmartData remoto (2º construtor)      | `Interested_SmartData<UNIT>(region, expiry, period, mode, device, fuser)` |
| Origin(x, y, z, t)                    | `header.stamp` + tf2 (`frame_id` → `target_frame`)                 |
| suporte de run-time (rede, tempo, local.) | `sd::Network` (UDP multicast), relógio ROS, tf2               |

Como o sensor ROS não está no nó que roda o gateway, cada transducer
carrega a origem da sua amostra, em vez de usar a posição do nó.

---

## 2. Dinâmica SmartData implementada (`include/smartdata/`)

| Comportamento (paper §2)                                            | Onde                                    |
|---------------------------------------------------------------------|-----------------------------------------|
| **Modos** PRIVATE / ADVERTISED / COMMANDED                          | `Responsive_SmartData`, `mode`          |
| ADVERTISED anuncia-se na rede e **só responde a quem tem interesse** | ADVERTISE no construtor; responde só com bindings |
| **Interest** = unidade + região espaço-temporal + expiry + period (+ device) | `Interested_SmartData` → mensagem INTEREST |
| period = 0 → **event-driven**: resposta a cada atualização          | `transducer_updated()`                  |
| period > 0 → **time-triggered**: resposta a cada período; transducers passivos são amostrados | `tick()` no `Scheduler` |
| períodos diferentes → amostragem no **MDC** dos períodos            | `reschedule_locked()`                   |
| a resposta leva o **menor expiry** entre o próprio e os dos interesses | `response_expiry_locked()`           |
| **expiry < period é inválido**                                      | `check_timing()` / interesse ignorado   |
| ler um SmartData **expirado** dispara uma atualização (passivos)    | `Responsive_SmartData::value()`         |
| `wait()` bloqueia até a próxima atualização                         | `wait()` nas duas classes               |
| escrever num SmartData = novo **setpoint**; remoto → **Command** aos COMMANDED da região | `operator=(double)` |
| **fuser**: com função de fusão, toda resposta passa por ela          | `Interested_SmartData`, parâmetro `fuser` |
| revogar um interesse                                                | REVOKE no destrutor do proxy            |

Adaptações à rede UDP (sem a confiabilidade do TSTP):

- **interesses são estado "soft"**: o proxy reanuncia o interesse a cada
  `interest_refresh` (1 s) e o produtor descarta um binding não renovado
  por `interest_lease` (3,5 s). Isso cobre nós que entram depois e pacotes
  perdidos;
- um SmartData que ainda **não tem amostra** não responde (nem aos ticks
  time-triggered);
- a origem é marcada com `location_confidence`: 100 com tf exata, 50 com
  a última tf conhecida, 0 sem tf.

Não implementado: supressão de respostas no modo SINGLE (no paper, um nó
deixa de responder se já ouviu outro responder), segurança (MAC/assinatura)
e o roteamento multi-hop do TSTP. O segmento é um único domínio multicast.

---

## 3. Codegen build-time

`codegen/generate.py` lê `config/sensors.yaml` e emite em `build/generated/`:

| Header                   | Conteúdo                                                                 |
|--------------------------|--------------------------------------------------------------------------|
| `parsed_transducers.hpp` | uma classe Transducer por (tópico × grandeza): `UNIT`, `TYPE`, `active`, `DEV_BASE`, `DEVICES`, `to_si`/`to_ros`; sensores observam o `sd::Source<Msg>` do tópico, atuadores escrevem no `sd::Sink<Msg>` |
| `parsed_gateway.hpp`     | `class Gateway`: subscriptions, sources/sinks e um `Responsive_SmartData` por instância (sensores ADVERTISED, atuadores COMMANDED) |
| `parsed_proxies.hpp`     | `class Proxies`: um `Interested_SmartData` por classe de sensor (todas as instâncias, modo ALL) que reconstrói a mensagem ROS; um por instância de atuador, que transforma `<topic>_command` em Commands |

O acesso aos campos é literal (`msg.linear_acceleration.x`): não há
introspecção nem parsing de YAML em runtime. Adicionar um tópico = editar o
YAML e recompilar; erros de campo viram erros de compilação.

---

## 4. Codificação dimensional da `unit` (Fig. 1 do paper)

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

Todas as unidades do gateway usam N = Float64. Constantes em
`include/si_units.hpp`; nova unidade = constante lá + entrada em
`UNIT_TO_SYMBOL` de `codegen/generate.py`.

---

## 5. DSL do `sensors.yaml`

### 5.1 Esquema geral

```yaml
defaults:
  target_frame: map

sensors:
  - topic:         </topic>                      # obrigatório
    type:          <pkg/msg/Type>                # obrigatório
    decomposition: scalar | array | composite    # obrigatório
    stamp_field:   header.stamp                  # default
    frame_field:   header.frame_id               # default
    expiry_ms:     <ms>                          # validade; 0 = não expira
    period_ms:     <ms>                          # 0 (default) = transducer ativo
    mode:          advertised | private          # default advertised
    output_topic:  </smartdata/...>              # espelho ROS (default /smartdata + topic)

actuators:
  - topic:         </topic>
    type:          <pkg/msg/Type>
    decomposition: scalar | composite
    frame:         <tf frame do atuador>         # usado para a região dos comandos
    expiry_ms:     <ms>
    coalesce_ms:   2                             # ver 5.5
```

### 5.2 `scalar` — uma grandeza, uma instância

```yaml
decomposition: scalar
value_field:   temperature
unit:          kelvin
value_transform: { kind: linear, a: 1.0, b: 273.15 }   # SI = a·ROS + b
```

`to_si` aplica a transformação na entrada; `to_ros`, a inversa
`(v − b)/a`, na reconstrução e nos comandos.

### 5.3 `array` — uma grandeza, uma instância por elemento

```yaml
decomposition: array
array_field: ranges
unit:        meter
filter: { kind: finite_within, min_field: range_min, max_field: range_max }
```

As instâncias são criadas quando a primeira mensagem revela o tamanho do
vetor. Um elemento inválido (filtro) **não atualiza** sua instância: o valor
anterior continua com sua origem antiga e simplesmente expira.

### 5.4 `composite` — uma classe de transducer por grupo

```yaml
decomposition: composite
groups:
  - name: linear_acceleration       # vira ImuDataLinearAcceleration_Transducer
    unit: meter_per_sec_sq
    fields: [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]
  - name: angular_velocity
    unit: rad_per_sec
    fields: [angular_velocity.x, angular_velocity.y, angular_velocity.z]
```

Cada grupo é uma classe; cada campo, uma instância (`dev = DEV_BASE + i`).
Cada grupo aceita seu próprio `value_transform`.

### 5.5 Atuadores

Um atuador é COMMANDED: um Command vindo da rede chama `actuate(v)`, que
escreve `to_ros(v)` no seu campo do setpoint e publica a mensagem no tópico.
Como cada campo é um SmartData próprio, escrever a mensagem inteira
remotamente gera um Command por campo; atualizações dentro de
`coalesce_ms` são publicadas como **um** setpoint, em vez de uma rajada de
mensagens parcialmente atualizadas. `coalesce_ms: 0` publica a cada campo.

### 5.6 Imports — SmartData de outros nós

`sensors:` e `actuators:` descrevem o que **este** gateway exporta. A seção
`imports:` descreve SmartData produzidos **em outro lugar** (outro gateway,
um nó SDAV/EPOS, qualquer produtor com o layout de `docs/wire_format.md`)
que o `proxy_node` deve pedir e publicar em ROS. O gateway não muda: um
import não tem transducer local nem bloco de dispositivos — os `device` são
os do produtor.

```yaml
imports:
  # qualquer produtor de kelvin; uma mensagem por resposta
  - topic: /lab/temperature            # tópico ROS publicado pelo proxy
    type: sensor_msgs/msg/Temperature
    decomposition: scalar
    unit: kelvin                       # nome da seção 6 ou código de 32 bits (0x...)
    device: any                        # padrão; ou um id específico
    value_field: temperature
    value_transform: { kind: linear, a: 1.0, b: 273.15 }   # ROS → SI, como em sensors
    frame_id: "sd_{device}"            # opcional; {device} vira o id do produtor
    region: { x: 3.0, y: 1.0, z: 0.0, radius: 5.0 }       # opcional (padrão: região do nó)
    period_ms: 1000                    # opcional (padrão: parâmetro period_ms do nó)
    expiry_ms: 0                       # padrão 0
    stamp: origin                      # origin (padrão) | receive
    exclude_local: true                # padrão

  # vários dispositivos de um nó estrangeiro reconstruídos numa mensagem
  - topic: /lab/accel
    type: geometry_msgs/msg/Vector3
    decomposition: composite
    stamp_field: null                  # mensagem sem header
    groups:
      - name: accel
        unit: meter_per_sec_sq
        fields:
          - { field: x, device: 10 }   # composite: dispositivo explícito por campo
          - { field: y, device: 11 }
          - { field: z, device: 12 }
```

| Chave           | Significado                                                                 |
|-----------------|-----------------------------------------------------------------------------|
| `decomposition` | `scalar` ou `composite` (sem `array`: o tamanho de um vetor estrangeiro não é conhecido) |
| `unit`          | a do produtor, exatamente; um código cru aceita unidades sem nome na tabela da seção 6 |
| `device`        | `any` = um interesse com `ANY_DEVICE`, todos os produtores daquela unidade; um id = só ele |
| `frame_id`      | escrito no `header.frame_id`; com `{device}`, distingue produtores de um import `any` |
| `region`        | esfera (metros, no frame do produtor) só para este import                   |
| `stamp`         | `origin` = tempo de origem da amostra; `receive` = relógio do proxy na chegada (produtor com outra base de tempo) |
| `stamp_field`   | `header.stamp` (padrão) ou `null` para mensagens sem header (`std_msgs/Float64`, `Vector3`) |
| `exclude_local` | um import `any` ignora os dispositivos dos transducers deste pacote com a mesma unidade |

Um import `scalar` com `device: any` publica uma mensagem por resposta,
deduplicando por produtor (cada `device` tem a sua última origem). Um
`composite` reconstrói a mensagem como na seção 8: os campos com a mesma
origem temporal são uma amostra, então o produtor precisa dar o mesmo `t` a
todos os campos de um instante.

O codegen recusa: tópico já usado (inclusive `_recovered`/`_command`/espelho),
`device` dentro do bloco de um transducer local da mesma unidade, `frame_id`
sem header, `composite` sem `device` por campo, e `array`.

---

## 6. Nomes de unidade reconhecidos

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

---

## 7. Numeração de dispositivos

O codegen dá a cada **classe de transducer** um bloco disjunto de 16 bits:

| Classe (ordem no YAML)          | DEV_BASE   | instâncias            |
|---------------------------------|------------|-----------------------|
| `Scan_Transducer`               | 0x00000    | 1 por feixe (≤ 65536) |
| `ImuDataLinearAcceleration_…`   | 0x10000    | x, y, z               |
| `ImuDataAngularVelocity_…`      | 0x20000    | x, y, z               |
| classe k                        | k × 0x10000|                       |

Assim duas classes com a mesma unidade no mesmo nó (`/imu` e `/imu/data`)
nunca compartilham um `dev`, que é exatamente o papel do `dev` no paper.
Os proxies usam o bloco para saber a qual mensagem cada resposta pertence.

---

## 8. Reconstrução no lado remoto

`Proxies` agrupa as respostas pela origem temporal (`header.stamp`):

- **scalar**: publica a cada amostra nova;
- **composite**: publica quando todos os campos da amostra chegaram (ou
  quando chega uma amostra mais nova);
- **array**: publica quando começa uma varredura nova, ou no `drain()`
  periódico se a varredura parou de receber feixes.

Como os interesses usam `ANY_DEVICE`, um proxy recebe também as respostas
de outras classes (e de outros nós) com a mesma unidade — por exemplo, os
setpoints de `/cmd_vel` (rad/s) chegam ao proxy do IMU. Cada `on_<topic>`
descarta os dispositivos que não são seus **antes** de mexer na amostra em
construção.

Amostras com origem já publicada são ignoradas, então interesses
time-triggered (que repetem a amostra atual a cada período) não geram
mensagens duplicadas.

Campos fora do SI (`range_min`, `angle_increment`, quaternions, `frame_id`,
covariâncias…) **não** atravessam a SmartData: a reconstrução é exata sobre
os campos declarados no YAML.

---

## 9. Testes

| Suite                    | O que cobre                                                              |
|--------------------------|--------------------------------------------------------------------------|
| `test_si_units`          | codificação 32-bit da Fig. 1                                             |
| `test_smartdata_runtime` | layout das mensagens; dinâmica: silêncio sem interesse, event-driven, time-triggered, região, device, revogação/lease, menor expiry, expiry < period, leitura expirada, `wait()`, Commands só a COMMANDED, fuser |
| `test_gateway_pipeline`  | transducers gerados (unidades, blocos de dev, filtros, conversões, atuador) e latência/vazão da tradução |
| `test_bus_roundtrip`     | ponta a ponta ROS → gateway → rede → proxies → ROS, e `<topic>_command` → Commands → `/cmd_vel`, com interesses event-driven e time-triggered; setpoints de mesma unidade não vazam no IMU recuperado |
| `test_imports`           | `imports:` (gerado de `test/imports_test.yaml`): produtores estrangeiros por `device: any` separados por `frame_id`, transducer local não importado, dispositivo específico em mensagem sem header, composite, região própria e `stamp: receive` |

---

## 10. Mapa do código

| Camada                         | Arquivo(s)                                        |
|--------------------------------|---------------------------------------------------|
| Vocabulário SI                 | `include/si_units.hpp`                            |
| Mensagens de rede              | `include/smartdata/wire.hpp`                      |
| Rede + scheduler (run-time)    | `include/smartdata/network.hpp`, `src/smartdata/network.cpp` |
| Transducer                     | `include/smartdata/transducer.hpp`                |
| SmartData local e remoto       | `include/smartdata/smartdata.hpp`                 |
| Ponte tópicos ROS ↔ transducers| `include/smartdata/ros.hpp`                       |
| DSL / codegen                  | `config/sensors.yaml`, `codegen/generate.py`      |
| Gerados                        | `build/generated/parsed_{transducers,gateway,proxies}.hpp` |
| Nós                            | `src/gateway_node.cpp`, `src/proxy_node.cpp`      |
| Espelho ROS (instrumentação)   | `msg/SmartData.msg`                               |
