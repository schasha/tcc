#!/usr/bin/env python3
"""analyze.py — aggregates a bench/run_all.sh results directory.

For every configuration the repetitions are summarized by the median of each
statistic and a 95% bootstrap confidence interval of that median. Each
hypothesis of bench/README.md gets a verdict, written to <dir>/summary.md, and
the series behind the figures are written to <dir>/plots/*.dat (whitespace
separated, ready for pgfplots).

Usage: python3 bench/analyze.py bench/results/<run>
"""

import csv
import math
import os
import random
import sys
from collections import defaultdict

CRITERION_US = 100.0   # acceptance criterion of the translation (p99 here)
ROS_HOP_P95_US = 1000.0
random.seed(1)


# ── helpers ──────────────────────────────────────────────────────────────────

def read(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return float("nan")
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def boot_ci(xs, stat=median, b=2000):
    """95% percentile-bootstrap CI of `stat` over the repetitions."""
    if len(xs) < 2:
        v = stat(xs)
        return v, v
    reps = sorted(stat([random.choice(xs) for _ in xs]) for _ in range(b))
    return reps[int(0.025 * b)], reps[int(0.975 * b) - 1]


def group(rows, keys):
    g = defaultdict(list)
    for r in rows:
        g[tuple(r[k] for k in keys)].append(r)
    return g


def col(rows, name):
    return [float(r[name]) for r in rows]


def fmt(v, d=2):
    return "–" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{d}f}"


def ci_str(rows, name, d=2):
    xs = col(rows, name)
    lo, hi = boot_ci(xs)
    return f"{fmt(median(xs), d)} [{fmt(lo, d)}, {fmt(hi, d)}]"


def linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    b = sxy / sxx
    a = my - b * mx
    ss_res = sum((y - a - b * x) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    return a, b, 1 - ss_res / ss_tot if ss_tot else 1.0


def write_dat(path, header, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(" ".join(header) + "\n")
        for r in rows:
            f.write(" ".join(fmt(v, 4) if isinstance(v, float) else str(v) for v in r) + "\n")


def load_raw(path):
    with open(path) as f:
        return [float(l) for l in f if l.strip()]


def cdf_points(xs, n=200):
    xs = sorted(xs)
    out = []
    for i in range(n + 1):
        q = i / n
        out.append((xs[min(len(xs) - 1, int(q * len(xs)))], q))
    return out


# ── hypotheses ───────────────────────────────────────────────────────────────

def h1(d, out):
    rows = read(os.path.join(d, "h1.csv"))
    if not rows:
        return
    out.append("## H1 — latência de tradução × tamanho da mensagem\n")
    out.append("Hipótese: a latência cresce linearmente com o número de campos e o p99 "
               f"fica abaixo de {CRITERION_US:.0f} µs para as mensagens do descritor.\n")
    out.append("| mensagem | campos | reps | p50 (µs) | p95 (µs) | p99 (µs) | p99,9 (µs) | máx (µs) |")
    out.append("|---|---:|---:|---|---|---|---|---|")
    g = group(rows, ["msg", "fields"])
    series = []
    for (msg, fields), rs in sorted(g.items(), key=lambda kv: (kv[0][0], int(kv[0][1]))):
        out.append(f"| {msg} | {fields} | {len(rs)} | {ci_str(rs, 'p50')} | {ci_str(rs, 'p95')} | "
                   f"{ci_str(rs, 'p99')} | {ci_str(rs, 'p999')} | {ci_str(rs, 'max')} |")
        if msg == "scan":
            series.append((int(fields), median(col(rs, "p50")), median(col(rs, "p95")),
                           median(col(rs, "p99")), median(col(rs, "max"))))
    series.sort()
    write_dat(os.path.join(d, "plots", "h1_scan.dat"), ["fields", "p50", "p95", "p99", "max"], series)
    scan = [(int(r["fields"]), float(r["p50"])) for r in rows if r["msg"] == "scan"]
    a, b, r2 = linfit([x for x, _ in scan], [y for _, y in scan])
    out.append(f"\nAjuste linear (scan, p50 de cada repetição): latência ≈ {a:.2f} µs + "
               f"{b * 1000:.1f} ns × campos, R² = {r2:.4f}.")
    for name in ("p99", "max"):
        lim = [f for f, *_ in series if median(col(g[('scan', str(f))], name)) < CRITERION_US]
        out.append(f"Maior LaserScan com {name} < {CRITERION_US:.0f} µs: "
                   f"{max(lim) if lim else 'nenhum'} feixes.")
    ok = r2 > 0.95 and all(median(col(rs, "p99")) < CRITERION_US for (m, f), rs in g.items()
                           if m != "scan" or int(f) <= 360)
    out.append(f"\n**Veredito H1:** {'confirmada' if ok else 'refutada'} "
               "(linearidade R² > 0,95 e p99 < critério até 360 feixes, IMU e temperatura).\n")
    for name in sorted(os.listdir(os.path.join(d, "raw"))):
        if name.startswith("h1_") and name.endswith(".txt"):
            write_dat(os.path.join(d, "plots", name.replace(".txt", "_cdf.dat")), ["us", "q"],
                      cdf_points(load_raw(os.path.join(d, "raw", name))))


def conv(d, out):
    path = os.path.join(d, "raw", "conv_scan_100.txt")
    if not os.path.exists(path):
        return
    xs = load_raw(path)
    out.append("## Convergência das estatísticas\n")
    out.append("Percentis calculados sobre prefixos crescentes da mesma série "
               "(LaserScan de 100 feixes, sem envio).\n")
    out.append("| iterações | p50 | p95 | p99 | p99,9 |")
    out.append("|---:|---:|---:|---:|---:|")
    pts = []
    n = 500
    while n <= len(xs):
        s = sorted(xs[:n])
        q = lambda p: s[min(n - 1, int(p * n))]
        pts.append((n, q(0.5), q(0.95), q(0.99), q(0.999)))
        n *= 2
    if pts[-1][0] != len(xs):
        s = sorted(xs)
        n = len(xs)
        q = lambda p: s[min(n - 1, int(p * n))]
        pts.append((n, q(0.5), q(0.95), q(0.99), q(0.999)))
    for p in pts:
        out.append(f"| {p[0]} | {fmt(p[1])} | {fmt(p[2])} | {fmt(p[3])} | {fmt(p[4])} |")
    write_dat(os.path.join(d, "plots", "convergence.dat"), ["n", "p50", "p95", "p99", "p999"], pts)
    final = pts[-1]
    stable = {name: next((p[0] for p in pts if all(abs(q[i] - final[i]) <= 0.05 * final[i]
                                                  for q in pts if q[0] >= p[0])), None)
              for i, name in ((1, "p50"), (2, "p95"), (3, "p99"), (4, "p99,9"))}
    out.append("\nMenor prefixo a partir do qual o percentil fica a ±5% do valor final: " +
               ", ".join(f"{k} = {v}" for k, v in stable.items()) + ".\n")


def h3(d, out):
    gw = read(os.path.join(d, "h3_gateway.csv"))
    st = read(os.path.join(d, "h3_sendto.csv"))
    if not gw:
        return
    out.append("## H3 — custo do envio\n")
    out.append("Hipótese: o custo com envio é linear no número K de respostas e o custo por "
               "datagrama é o de uma chamada `sendto`, não do gateway.\n")
    out.append("| configuração | K | p50 (µs) | p95 (µs) | p99 (µs) | máx (µs) |")
    out.append("|---|---:|---|---|---|---|")
    fits = {}
    g = group(gw, ["k"])
    series = []
    for (k,), rs in sorted(g.items(), key=lambda kv: int(kv[0][0])):
        out.append(f"| gateway (tradução + envio) | {k} | {ci_str(rs, 'p50')} | {ci_str(rs, 'p95')} | "
                   f"{ci_str(rs, 'p99')} | {ci_str(rs, 'max')} |")
        series.append((int(k), median(col(rs, "p50")), median(col(rs, "p95"))))
    write_dat(os.path.join(d, "plots", "h3_gateway.dat"), ["k", "p50", "p95"], series)
    a, b, r2 = linfit([float(r["k"]) for r in gw], [float(r["p50"]) for r in gw])
    fits["gateway"] = (a, b, r2)
    for (via,), rows in sorted(group(st, ["via"]).items()):
        series = []
        for (k,), rs in sorted(group(rows, ["k"]).items(), key=lambda kv: int(kv[0][0])):
            out.append(f"| sendto {via} | {k} | {ci_str(rs, 'p50')} | {ci_str(rs, 'p95')} | "
                       f"{ci_str(rs, 'p99')} | {ci_str(rs, 'max')} |")
            series.append((int(k), median(col(rs, "p50")), median(col(rs, "p95"))))
        write_dat(os.path.join(d, "plots", f"h3_sendto_{via.replace('+', '_')}.dat"), ["k", "p50", "p95"], series)
        fits[via] = linfit([float(r["k"]) for r in rows], [float(r["p50"]) for r in rows])
    out.append("\nCusto marginal por datagrama (inclinação do p50 em K):\n")
    for name, (a, b, r2) in fits.items():
        out.append(f"- {name}: {b:.2f} µs/datagrama (intercepto {a:.2f} µs, R² = {r2:.3f})")
    if "raw" in fits:
        gb, rb = fits["gateway"][1], fits["raw"][1]
        lb = fits.get("raw+listener", (0, float("nan"), 0))[1]
        out.append(f"\nO gateway paga {gb / rb:.1f}× o `sendto` sem receptor local e "
                   f"{gb / lb:.1f}× o `sendto` com um receptor local no mesmo grupo.")
        ok = fits["gateway"][2] > 0.95 and gb <= 1.5 * lb
        out.append(f"\n**Veredito H3:** {'confirmada' if ok else 'refutada'} "
                   "(linear com R² > 0,95 e custo por datagrama até 1,5× o do `sendto` "
                   "nas mesmas condições de recepção local).\n")


def h2(d, out):
    rows = read(os.path.join(d, "h2.csv"))
    if not rows:
        return
    out.append("## H2 — carga em laço aberto e ponto de saturação\n")
    out.append("Hipótese: a latência fica estável até uma taxa R\\*; acima dela, a fila de "
               "profundidade 5 (como o SensorDataQoS) transborda e mensagens se perdem.\n")
    for (k,), rs_k in sorted(group(rows, ["k"]).items()):
        out.append(f"\n**K = {k}** (respostas enviadas por varredura)\n")
        out.append("| taxa ofertada (/s) | servida (/s) | perda | p50 (µs) | p99 (µs) | máx (µs) |")
        out.append("|---:|---:|---:|---|---|---|")
        series = []
        knee = None
        for (rate,), rs in sorted(group(rs_k, ["rate"]).items(), key=lambda kv: float(kv[0][0])):
            loss = median(col(rs, "loss_ratio"))
            served = median(col(rs, "served_rate"))
            out.append(f"| {float(rate):.0f} | {served:.0f} | {loss * 100:.2f}% | {ci_str(rs, 'p50')} | "
                       f"{ci_str(rs, 'p99')} | {ci_str(rs, 'max')} |")
            series.append((float(rate), served, loss, median(col(rs, "p50")), median(col(rs, "p99"))))
            if loss < 0.01:
                knee = float(rate)
        write_dat(os.path.join(d, "plots", f"h2_k{k}.dat"), ["rate", "served", "loss", "p50", "p99"], series)
        out.append(f"\nMaior taxa avaliada com perda < 1%: {knee:.0f} varreduras/s "
                   f"({knee * 100:.0f} SmartData/s)." if knee else "\nPerda ≥ 1% já na menor taxa.")
    out.append("\n**Veredito H2:** ver as tabelas; R\\* é a maior taxa com perda < 1% e p99 estável.\n")


def h4(d, out):
    rows = read(os.path.join(d, "h4.csv"))
    if not rows:
        return
    out.append("## H4 — latência de ponta a ponta\n")
    out.append("Hipótese: o caminho publicador → gateway → rede → proxy → assinante tem p95 "
               f"abaixo de {ROS_HOP_P95_US:.0f} µs e acrescenta ao salto ROS 2 direto menos que "
               "um salto ROS 2 adicional.\n")
    out.append("| taxa (Hz) | caminho | perda | p50 (µs) | p95 (µs) | p99 (µs) | máx (µs) |")
    out.append("|---:|---|---:|---|---|---|---|")
    series = defaultdict(list)
    verdict = True
    for (rate, path), rs in sorted(group(rows, ["rate", "path"]).items(), key=lambda kv: (float(kv[0][0]), kv[0][1])):
        loss = median(col(rs, "loss_ratio"))
        out.append(f"| {float(rate):.0f} | {path} | {loss * 100:.2f}% | {ci_str(rs, 'p50', 1)} | "
                   f"{ci_str(rs, 'p95', 1)} | {ci_str(rs, 'p99', 1)} | {ci_str(rs, 'max', 1)} |")
        series[path].append((float(rate), loss, median(col(rs, "p50")), median(col(rs, "p95")),
                             median(col(rs, "p99"))))
    for path, s in series.items():
        write_dat(os.path.join(d, "plots", f"h4_{path}.dat"), ["rate", "loss", "p50", "p95", "p99"], s)
    if "direct" in series and "gateway" in series:
        out.append("\nSobrecarga do caminho pelo gateway (p50 gateway − p50 direto) e razão com o salto direto:\n")
        for (rate, _, d50, d95, _), (_, gl, g50, g95, _) in zip(series["direct"], series["gateway"]):
            out.append(f"- {rate:.0f} Hz: +{g50 - d50:.1f} µs no p50 ({g50 / d50:.2f}× o salto direto), "
                       f"p95 {g95:.1f} µs")
            verdict &= g95 < ROS_HOP_P95_US and (g50 - d50) < d50 and gl < 0.01
    out.append(f"\n**Veredito H4:** {'confirmada' if verdict else 'refutada em ao menos uma taxa'}.\n")


def h5(d, out):
    raw = os.path.join(d, "raw")
    if not os.path.isdir(raw):
        return
    reps = sorted(n[:-9] for n in os.listdir(raw) if n.startswith("h5_") and n.endswith("_odom.csv"))
    if not reps:
        return
    out.append("## H5 — seguimento de setpoints enviados pela rede SmartData\n")
    out.append("Hipótese: para cada degrau, o erro em regime é pequeno (< 5% do valor comandado "
               "ou < 0,01 em módulo) e o atraso do comando pela rede é de poucos milissegundos.\n")
    out.append("| rep | degrau | v cmd | w cmd | erro v regime | erro w regime | atraso do comando (ms) |")
    out.append("|---|---:|---:|---:|---:|---:|---:|")
    ok = True
    for rep in reps:
        odom = read(os.path.join(raw, rep + "_odom.csv"))
        cmds = {r["step"]: float(r["delay_us"]) / 1000 for r in read(os.path.join(raw, rep + "_cmd.csv"))}
        for (step,), rs in sorted(group(odom, ["step"]).items(), key=lambda kv: int(kv[0][0])):
            ts = col(rs, "t")
            t_mid = min(ts) + 0.5 * (max(ts) - min(ts))
            tail = [r for r in rs if float(r["t"]) >= t_mid]
            v, w = float(rs[0]["cmd_v"]), float(rs[0]["cmd_w"])
            ev = sum(float(r["odom_v"]) for r in tail) / len(tail) - v
            ew = sum(float(r["odom_w"]) for r in tail) / len(tail) - w
            delay = cmds.get(step)
            out.append(f"| {rep[3:]} | {step} | {v:.2f} | {w:.2f} | {ev:+.4f} | {ew:+.4f} | "
                       f"{fmt(delay)} |")
            ok &= abs(ev) <= max(0.01, 0.05 * abs(v)) and abs(ew) <= max(0.01, 0.05 * abs(w))
        write_dat(os.path.join(d, "plots", f"{rep}_odom.dat"), ["t", "cmd_v", "cmd_w", "odom_v", "odom_w"],
                  [(float(r["t"]), float(r["cmd_v"]), float(r["cmd_w"]), float(r["odom_v"]), float(r["odom_w"]))
                   for r in odom])
    out.append(f"\n**Veredito H5:** {'confirmada' if ok else 'refutada em ao menos um degrau'}.\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    d = sys.argv[1]
    out = ["# Resultados dos experimentos\n"]
    env = os.path.join(d, "env.txt")
    if os.path.exists(env):
        out.append("```\n" + open(env).read().strip() + "\n```\n")
    out.append("Valores: mediana entre repetições [IC 95% por bootstrap da mediana].\n")
    for f in (h1, conv, h3, h2, h4, h5):
        f(d, out)
    with open(os.path.join(d, "summary.md"), "w") as f:
        f.write("\n".join(out) + "\n")
    print("\n".join(out))


if __name__ == "__main__":
    main()
