# Formato das mensagens SmartData na rede

O paper deixa a codificação ao protocolo de comunicação ("the actual
encoding of SmartData attributes in a network packet is protocol-dependent",
Fröhlich 2018). O gateway segue o layout das mensagens TSTP da
implementação do LISHA (SDAV, `smartdata.h`: `_Header`, `Interest`,
`Response`, `Command`) na escala de 32 bits em centímetros, de modo que os
offsets de cada campo coincidem com os daquela implementação. Definição em
`include/smartdata/wire.hpp`; offsets verificados em
`test/test_smartdata_runtime.cpp` (`Wire.LayoutMatchesTstpMessages`).

Todos os campos estão na ordem de bytes do host (little-endian em x86_64 e
arm64), sem padding (`#pragma pack(1)`).

## Transporte

- grupo multicast `224.1.1.1`, porta UDP `5000` (parâmetros `bus_group`,
  `bus_port`);
- TTL 1 (um segmento de rede), loopback ligado: produtores e consumidores
  podem dividir o host ou o processo;
- uma mensagem por datagrama, entrega best-effort.

## Cabeçalho comum (60 B)

| Offset | Tam. | Campo                 | Conteúdo                                                        |
|--------|------|-----------------------|-----------------------------------------------------------------|
| 0      | 1    | `config`              | escala [7:6] = 3 (32 bits, cm) · time request [5] · tipo [4:3] · versão [2:0] = 4 |
| 1      | 1    | `mode`                | modo [1:0] · operação [3:2] · subtipo [7:4]                     |
| 2      | 1    | `misc`                | incerteza do valor (respostas)                                  |
| 3      | 1    | `location_confidence` | 0–100 %: 100 tf exata, 50 última tf conhecida, 0 desconhecida   |
| 4      | 12   | `origin.x/y/z`        | int32, cm, no `target_frame` do gateway                         |
| 16     | 8    | `origin.t`            | int64, µs (relógio ROS; `header.stamp` da amostra)              |
| 24     | 4    | `unit`                | unidade SI de 32 bits (Fig. 1, `si_units.hpp`)                  |
| 28     | 4    | `device`              | instância do transducer (`DEV_BASE + índice`); `0xFFFFFFFF` = qualquer, em interesses/comandos |
| 32     | 8    | `signature`           | 0 (sem camada de segurança)                                     |
| 40     | 20   | `last_hop.x/y/z/t`    | = origem (segmento de um salto)                                 |

Tipos (`config[4:3]`): 0 INTEREST, 1 RESPONSE, 2 COMMAND, 3 CONTROL (não usado).

Modos (`mode`):

| Mensagem      | modo [1:0]                         | operação [3:2]                         |
|---------------|------------------------------------|----------------------------------------|
| INTEREST      | 0 SINGLE · 1 ALL                   | 0 ANNOUNCE · 1 REVOKE                  |
| RESPONSE      | 0 PRIVATE · 1 ADVERTISED · 3 COMMANDED | 0 ADVERTISE · 1 CONCEAL · 2 RESPOND |
| COMMAND       | 0 SINGLE · 1 ALL                   | —                                      |

## Interest (104 B)

| Offset | Tam. | Campo        |                                                    |
|--------|------|--------------|----------------------------------------------------|
| 0      | 60   | cabeçalho    | `unit`, `device` pedidos; origem = quem pede       |
| 60     | 12   | região x/y/z | centro, int32 cm                                   |
| 72     | 4    | raio         | uint32 cm                                          |
| 76     | 16   | t0, t1       | int64 µs (intervalo de interesse)                  |
| 92     | 8    | expiry       | int64 µs, relativo                                 |
| 100    | 4    | period       | int32 µs; 0 = event-driven                         |

## Response (70 B + valor)

| Offset | Tam. | Campo      |                                                         |
|--------|------|------------|---------------------------------------------------------|
| 0      | 60   | cabeçalho  | origem e incerteza da amostra                           |
| 60     | 8    | expiry     | o menor entre o do SmartData e os dos interesses        |
| 68     | 2    | fragment   | 0 (valores escalares nunca fragmentam)                  |
| 70     | 8    | valor      | IEEE-754 binary64 (unit N = Float64)                    |

RESPOND leva o valor (78 B). ADVERTISE/CONCEAL anunciam o SmartData sem
valor (70 B).

## Command (84 B + valor)

| Offset | Tam. | Campo      |                                                         |
|--------|------|------------|---------------------------------------------------------|
| 0      | 60   | cabeçalho  | origem = centro da região e t0                          |
| 60     | 4    | raio       | uint32 cm                                               |
| 64     | 8    | t1         | int64 µs                                                |
| 72     | 8    | expiry     | int64 µs                                                |
| 80     | 4    | period     | int32 µs                                                |
| 84     | 8    | valor      | novo setpoint, binary64                                 |

## O que não está incluído

- `signature`/MAC e mensagens CONTROL de segurança e sincronização de
  relógio do TSTP: fora do escopo;
- roteamento multi-hop (o `last_hop` só repete a origem);
- valores digitais e fragmentação: o gateway só emite grandezas SI escalares.
