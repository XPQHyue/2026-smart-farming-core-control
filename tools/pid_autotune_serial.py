#!/usr/bin/env python3
"""Serial PID tuning utility for STM32 protocol {channel:value}.

Features:
1) Send mode: send one fixed 3-parameter group in one shot.
2) Auto mode: heuristic coordinate-descent tuning based on serial feedback.

Notes:
- Firmware parser uses integer values; sender rounds to int.
- Default scoring assumes CSV feedback where value is column 0 and target is column 3.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial. Install with: pip install pyserial", file=sys.stderr)
    raise


@dataclass
class TuneConfig:
    channels: Tuple[int, int, int]
    initial: List[float]
    steps: List[float]
    min_values: List[float]
    max_values: List[float]
    max_rounds: int
    step_decay: float
    settle_sec: float
    measure_sec: float
    score_value_index: int
    score_target_index: int
    patience_rounds: int
    min_improve: float
    export_best_path: str


class SerialPidClient:
    def __init__(self, port: str, baud: int, timeout: float = 0.05) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    @staticmethod
    def _packet(channel: int, value: float) -> bytes:
        iv = int(round(value))
        return f"{{{channel}:{iv}}}".encode("ascii")

    def send_group(self, channels: Sequence[int], values: Sequence[float], gap_ms: int = 10) -> str:
        if len(channels) != 3 or len(values) != 3:
            raise ValueError("channels and values must both have length 3")

        payload = b"".join(self._packet(ch, v) for ch, v in zip(channels, values))
        self.ser.write(payload)
        self.ser.flush()
        if gap_ms > 0:
            time.sleep(gap_ms / 1000.0)
        return payload.decode("ascii")

    def read_lines(self, duration_sec: float) -> List[str]:
        end_t = time.time() + duration_sec
        buf = bytearray()
        lines: List[str] = []

        while time.time() < end_t:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                continue
            buf.extend(data)

            while b"\n" in buf:
                one, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                line = one.decode("utf-8", errors="ignore").strip()
                if line:
                    lines.append(line)

        return lines


def parse_csv_numbers(line: str) -> List[float]:
    out: List[float] = []
    for part in line.split(","):
        p = part.strip()
        if not p:
            continue
        try:
            out.append(float(p))
        except ValueError:
            return []
    return out


def calc_score(lines: Sequence[str], value_index: int, target_index: int) -> Tuple[float, dict]:
    errors: List[float] = []
    values: List[float] = []

    for line in lines:
        nums = parse_csv_numbers(line)
        if not nums:
            continue
        if value_index >= len(nums) or target_index >= len(nums):
            continue
        v = nums[value_index]
        t = nums[target_index]
        e = t - v
        errors.append(e)
        values.append(v)

    if not errors:
        return float("inf"), {"valid": 0}

    abs_e = [abs(x) for x in errors]
    mae = sum(abs_e) / len(abs_e)
    rms = math.sqrt(sum(x * x for x in errors) / len(errors))
    stdev = statistics.pstdev(values) if len(values) > 1 else 0.0

    zero_cross = 0
    for i in range(1, len(errors)):
        if errors[i - 1] == 0:
            continue
        if errors[i - 1] * errors[i] < 0:
            zero_cross += 1

    score = mae + 0.5 * rms + 0.2 * stdev + 0.05 * zero_cross
    detail = {
        "valid": len(errors),
        "mae": mae,
        "rms": rms,
        "stdev": stdev,
        "zero_cross": zero_cross,
        "score": score,
    }
    return score, detail


def clamp3(values: Sequence[float], mins: Sequence[float], maxs: Sequence[float]) -> List[float]:
    return [max(mi, min(ma, v)) for v, mi, ma in zip(values, mins, maxs)]


def evaluate_once(client: SerialPidClient, cfg: TuneConfig, params: Sequence[float], verbose: bool = True) -> Tuple[float, dict]:
    pkt = client.send_group(cfg.channels, params)
    if verbose:
        print(f"TX: {pkt}")

    if cfg.settle_sec > 0:
        time.sleep(cfg.settle_sec)

    lines = client.read_lines(cfg.measure_sec)
    score, detail = calc_score(lines, cfg.score_value_index, cfg.score_target_index)

    # Structured metric line for GUI parsing.
    print(
        "METRIC,"
        f"score={detail.get('score', float('inf')):.6f},"
        f"mae={detail.get('mae', float('inf')):.6f},"
        f"rms={detail.get('rms', float('inf')):.6f},"
        f"std={detail.get('stdev', float('inf')):.6f},"
        f"zc={detail.get('zero_cross', 0)},"
        f"valid={detail.get('valid', 0)},"
        f"p0={params[0]:.6f},"
        f"p1={params[1]:.6f},"
        f"p2={params[2]:.6f}"
    )

    if verbose:
        print(
            "Score: "
            f"score={detail.get('score', float('inf')):.4f}, "
            f"valid={detail.get('valid', 0)}, "
            f"mae={detail.get('mae', float('inf')):.4f}, "
            f"rms={detail.get('rms', float('inf')):.4f}, "
            f"std={detail.get('stdev', float('inf')):.4f}, "
            f"zc={detail.get('zero_cross', 0)}"
        )

    return score, detail


def auto_tune(client: SerialPidClient, cfg: TuneConfig) -> Tuple[List[float], float]:
    current = clamp3(cfg.initial, cfg.min_values, cfg.max_values)
    steps = list(cfg.steps)

    print("\n=== Auto tune start ===")
    print(f"Init params: {current}")
    best_score, _ = evaluate_once(client, cfg, current)
    best_params = current[:]
    no_improve_rounds = 0

    for round_idx in range(1, cfg.max_rounds + 1):
        improved = False
        print(f"\n--- Round {round_idx} ---")

        for i in range(3):
            if steps[i] <= 0:
                continue

            candidates = []
            plus = current[:]
            plus[i] += steps[i]
            plus = clamp3(plus, cfg.min_values, cfg.max_values)
            candidates.append((f"p{i}+", plus))

            minus = current[:]
            minus[i] -= steps[i]
            minus = clamp3(minus, cfg.min_values, cfg.max_values)
            candidates.append((f"p{i}-", minus))

            local_best = ("keep", current, best_score)
            tested_scores: list[tuple[str, float]] = []
            for tag, c in candidates:
                s, _ = evaluate_once(client, cfg, c, verbose=False)
                tested_scores.append((tag, s))
                if (local_best[2] - s) >= cfg.min_improve:
                    local_best = (tag, c, s)

            print(
                f"Param p{i} tested: "
                + ", ".join([f"{t}={sc:.4f}" for t, sc in tested_scores])
            )

            if local_best[0] != "keep":
                current = list(local_best[1])
                best_score = local_best[2]
                best_params = current[:]
                improved = True
                print(f"Accept {local_best[0]}: {current}, score={best_score:.4f}")
            else:
                steps[i] *= cfg.step_decay
                print(f"No gain, shrink step p{i}: {steps[i]:.4f}")

        if all(s < 1e-6 for s in steps):
            print("Step size converged. Stop early.")
            break

        if improved:
            no_improve_rounds = 0
        else:
            no_improve_rounds += 1
            if cfg.patience_rounds > 0 and no_improve_rounds >= cfg.patience_rounds:
                print(
                    "Early stop: "
                    f"no significant improvement for {no_improve_rounds} round(s), "
                    f"threshold={cfg.min_improve:.6f}"
                )
                break

        if not improved:
            steps = [s * cfg.step_decay for s in steps]
            print(f"No improvement this round. Global step shrink: {steps}")

    pkt = client.send_group(cfg.channels, best_params)
    print(f"\nBest params sent: {pkt}")
    print(f"Best score: {best_score:.4f}")

    if cfg.export_best_path:
        out = Path(cfg.export_best_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        text = (
            "# PID Auto Tune Result\n"
            f"channels={cfg.channels[0]},{cfg.channels[1]},{cfg.channels[2]}\n"
            f"best_p0={best_params[0]:.6f}\n"
            f"best_p1={best_params[1]:.6f}\n"
            f"best_p2={best_params[2]:.6f}\n"
            f"best_score={best_score:.6f}\n"
        )
        out.write_text(text, encoding="utf-8")
        print(f"BEST_SAVED,{out.as_posix()}")

    return best_params, best_score


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Serial PID sender and heuristic auto tuner")
    p.add_argument("--port", required=True, help="COM port, e.g. COM8")
    p.add_argument("--baud", type=int, default=9600, help="Baudrate, default 9600")

    sub = p.add_subparsers(dest="mode", required=True)

    p_send = sub.add_parser("send", help="Send one 3-parameter group")
    p_send.add_argument("--channels", default="0,1,2", help="Channels, e.g. 0,1,2")
    p_send.add_argument("--values", required=True, help="Values, e.g. 100,120,80")

    p_auto = sub.add_parser("auto", help="Heuristic auto tune")
    p_auto.add_argument("--channels", default="0,1,2", help="Three parameter channels")
    p_auto.add_argument("--init", required=True, help="Init values, e.g. 100,100,100")
    p_auto.add_argument("--step", default="20,20,20", help="Init step, e.g. 20,10,10")
    p_auto.add_argument("--min", dest="minv", default="0,0,0", help="Min bound")
    p_auto.add_argument("--max", dest="maxv", default="1000,1000,1000", help="Max bound")
    p_auto.add_argument("--rounds", type=int, default=10, help="Max rounds")
    p_auto.add_argument("--decay", type=float, default=0.5, help="Step decay factor")
    p_auto.add_argument("--settle", type=float, default=0.8, help="Settle seconds")
    p_auto.add_argument("--measure", type=float, default=2.0, help="Measure window seconds")
    p_auto.add_argument("--value-index", type=int, default=0, help="CSV value column index")
    p_auto.add_argument("--target-index", type=int, default=3, help="CSV target column index")
    p_auto.add_argument("--patience", type=int, default=3, help="Early-stop patience rounds")
    p_auto.add_argument("--min-improve", type=float, default=0.01, help="Minimum meaningful improvement")
    p_auto.add_argument("--export-best", default="", help="Optional output text file for best params")
    p_auto.add_argument(
        "--print-metric",
        action="store_true",
        help="Compatibility flag for GUI; metrics are already printed by default",
    )

    return p


def parse_triplet(raw: str, cast=float) -> List[float]:
    items = [x.strip() for x in raw.split(",") if x.strip()]
    if len(items) != 3:
        raise ValueError(f"Need exactly 3 values, got: {raw}")
    return [cast(x) for x in items]


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        client = SerialPidClient(args.port, args.baud)
    except Exception as exc:
        print(f"Open serial failed: {exc}", file=sys.stderr)
        return 2

    try:
        if args.mode == "send":
            channels = [int(x) for x in parse_triplet(args.channels, int)]
            values = parse_triplet(args.values, float)
            pkt = client.send_group(channels, values)
            print(f"Sent: {pkt}")
            return 0

        channels = tuple(int(x) for x in parse_triplet(args.channels, int))
        cfg = TuneConfig(
            channels=channels,
            initial=parse_triplet(args.init, float),
            steps=parse_triplet(args.step, float),
            min_values=parse_triplet(args.minv, float),
            max_values=parse_triplet(args.maxv, float),
            max_rounds=args.rounds,
            step_decay=args.decay,
            settle_sec=args.settle,
            measure_sec=args.measure,
            score_value_index=args.value_index,
            score_target_index=args.target_index,
            patience_rounds=args.patience,
            min_improve=args.min_improve,
            export_best_path=args.export_best,
        )
        auto_tune(client, cfg)
        return 0

    except Exception as exc:
        print(f"Run failed: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
