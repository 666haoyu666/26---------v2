#!/usr/bin/env python3
"""直线循迹闭环仿真：七路二值探头、IMU、后轴换算与差速电机。"""

import argparse
import csv
import json
import math
import re
from collections import deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class FirmwareParams:
    ctrl_period_s: float
    motor_period_s: float
    speed_mm_s: float
    wheel_track_mm: float
    sensor_to_axle_mm: float
    sensor_pos_mm: Tuple[float, ...]
    lpf_alpha: float
    lost_hold_mm: float
    k_err: float
    k_yaw: float


@dataclass(frozen=True)
class MotorModel:
    tau_s: float
    delay_s: float
    left_gain: float
    right_gain: float
    max_speed_mm_s: Optional[float]


@dataclass(frozen=True)
class SimCase:
    name: str
    init_y_mm: float
    init_yaw_deg: float
    sensor_centered: bool
    port_sign: float
    motor: MotorModel
    duration_s: float = 12.0
    line_width_mm: float = 20.0


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def macro_number(text: str, name: str) -> float:
    pattern = (
        r"^\s*#define\s+" + re.escape(name) +
        r"\s+\(?\s*([-+]?\d+(?:\.\d+)?)\s*[fFuUlL]*\s*\)?"
    )
    match = re.search(pattern, text, flags=re.MULTILINE)
    if match is None:
        raise ValueError("找不到固件宏: " + name)
    return float(match.group(1))


def parse_firmware(root: Path) -> FirmwareParams:
    ctrl_path = root / "01_Service/service/service_track_ctrl/service_track_ctrl.c"
    ctrl_hdr = root / "01_Service/service/service_track_ctrl/service_track_ctrl.h"
    motor_cfg = root / "02_Config/board_motor_config.h"
    sensor_cfg = root / "02_Config/board_line_sensor_config.h"
    freertos = root / "Core/Src/freertos.c"

    ctrl_text = read_text(ctrl_path)
    hdr_text = read_text(ctrl_hdr)
    motor_text = read_text(motor_cfg)
    sensor_text = read_text(sensor_cfg)
    freertos_text = read_text(freertos)

    pos_match = re.search(
        r"#define\s+BOARD_LSENSOR_POSITIONS_MM\s*\\\s*\{([^}]+)\}",
        sensor_text,
        flags=re.MULTILINE,
    )
    if pos_match is None:
        raise ValueError("找不到七路探头坐标")
    positions = tuple(
        float(item.strip().rstrip("fF"))
        for item in pos_match.group(1).split(",")
    )

    speed_match = re.search(
        r"track_ctrl_set_mode\(\s*TRACK_CTRL_MODE_TRACK_DIR\s*,"
        r"\s*[-+]?\d+(?:\.\d+)?[fF]?\s*,\s*([-+]?\d+)\s*\)",
        freertos_text,
        flags=re.MULTILINE,
    )
    if speed_match is None:
        raise ValueError("找不到直线巡线目标速度")

    return FirmwareParams(
        ctrl_period_s=macro_number(hdr_text, "SERVER_CTRL_PERIOD_MS") / 1000.0,
        motor_period_s=macro_number(motor_text, "BOARD_MOTOR_TICK_S"),
        speed_mm_s=float(speed_match.group(1)),
        wheel_track_mm=macro_number(ctrl_text, "TRACK_WHEEL_DIST_MM"),
        sensor_to_axle_mm=macro_number(ctrl_text,
                                       "TRACK_SENSOR_TO_WHEEL_MM"),
        sensor_pos_mm=positions,
        lpf_alpha=macro_number(ctrl_text, "TRACK_ERR_LPF_ALPHA"),
        lost_hold_mm=macro_number(ctrl_text, "TRACK_ERR_SOS_MM"),
        k_err=macro_number(ctrl_text, "TRACK_K1_ERR"),
        k_yaw=macro_number(ctrl_text, "TRACK_K2_YAW"),
    )


def clamp(value: float, limit: Optional[float]) -> Tuple[float, bool]:
    if limit is None:
        return value, False
    clipped = max(-limit, min(limit, value))
    return clipped, clipped != value


def line_sensor(
    rear_y_mm: float,
    yaw_rad: float,
    positions_mm: Sequence[float],
    sensor_to_axle_mm: float,
    line_width_mm: float,
) -> Tuple[str, float, int, float]:
    front_y = rear_y_mm + sensor_to_axle_mm * math.sin(yaw_rad)
    hits = []
    for pos in positions_mm:
        probe_y = front_y + pos * math.cos(yaw_rad)
        hits.append(abs(probe_y) <= line_width_mm * 0.5)

    hit_count = sum(1 for hit in hits if hit)
    run_count = 0
    in_run = False
    for hit in hits:
        if hit and not in_run:
            run_count += 1
        in_run = hit

    bitmap = sum((1 << idx) for idx, hit in enumerate(hits) if hit)
    if hit_count == 0:
        return "no_line", 0.0, bitmap, front_y
    if run_count >= 2 or hit_count == len(positions_mm):
        return "ambiguous", 0.0, bitmap, front_y

    raw_offset = sum(
        positions_mm[idx] for idx, hit in enumerate(hits) if hit
    ) / float(hit_count)
    return "valid", raw_offset, bitmap, front_y


def calc_metrics(rows: List[Dict[str, float]], case: SimCase) -> Dict[str, object]:
    rear = [row["rear_y_mm"] for row in rows]
    yaw = [row["yaw_deg"] for row in rows]
    times = [row["time_s"] for row in rows]
    settle_s = None
    settle_x = None
    suffix_y = 0.0
    suffix_yaw = 0.0
    for idx in range(len(rows) - 1, -1, -1):
        suffix_y = max(suffix_y, abs(rear[idx]))
        suffix_yaw = max(suffix_yaw, abs(yaw[idx]))
        if times[idx] >= 0.5 and suffix_y <= 5.0 and suffix_yaw <= 2.0:
            settle_s = times[idx]
            settle_x = rows[idx]["x_mm"]

    tail = [row for row in rows if row["time_s"] >= case.duration_s - 2.0]
    rms_tail = math.sqrt(sum(row["rear_y_mm"] ** 2 for row in tail) /
                         max(1, len(tail)))
    wheel_err = sum(
        (row["left_cmd_mm_s"] - row["left_act_mm_s"]) ** 2 +
        (row["right_cmd_mm_s"] - row["right_act_mm_s"]) ** 2
        for row in rows
    ) / max(1, 2 * len(rows))

    init_sign = 1.0 if rows[0]["rear_y_mm"] >= 0.0 else -1.0
    opposite = [max(0.0, -init_sign * value) for value in rear]
    lost_count = sum(1 for row in rows if row["sensor_valid"] == 0.0)
    sat_count = sum(1 for row in rows if row["motor_saturated"] != 0.0)

    return {
        "settling_time_s": None if settle_s is None else round(settle_s, 3),
        "settling_distance_mm": None if settle_x is None else round(settle_x, 1),
        "final_rear_error_mm": round(rear[-1], 3),
        "tail_rms_error_mm": round(rms_tail, 3),
        "max_abs_rear_error_mm": round(max(abs(value) for value in rear), 3),
        "overshoot_mm": round(max(opposite), 3),
        "max_abs_yaw_deg": round(max(abs(value) for value in yaw), 3),
        "max_abs_w_cmd_deg_s": round(
            max(abs(row["w_cmd_deg_s"]) for row in rows), 3),
        "max_abs_wheel_cmd_mm_s": round(max(
            max(abs(row["left_cmd_mm_s"]), abs(row["right_cmd_mm_s"]))
            for row in rows
        ), 3),
        "wheel_tracking_rms_mm_s": round(math.sqrt(wheel_err), 3),
        "sensor_invalid_percent": round(100.0 * lost_count / len(rows), 3),
        "motor_saturation_percent": round(100.0 * sat_count / len(rows), 3),
    }


def simulate(params: FirmwareParams, case: SimCase) -> Tuple[List[Dict[str, float]],
                                                               Dict[str, object]]:
    dt = min(0.001, params.ctrl_period_s / 10.0,
             params.motor_period_s / 10.0)
    ctrl_steps = max(1, int(round(params.ctrl_period_s / dt)))
    delay_steps = max(0, int(round(case.motor.delay_s / dt)))
    left_delay = deque([0.0] * delay_steps)
    right_delay = deque([0.0] * delay_steps)

    yaw_rad = math.radians(case.init_yaw_deg)
    rear_y = case.init_y_mm
    if case.sensor_centered:
        rear_y = -params.sensor_to_axle_mm * math.sin(yaw_rad)

    x_mm = 0.0
    left_act = 0.0
    right_act = 0.0
    left_cmd = 0.0
    right_cmd = 0.0
    w_cmd_deg = 0.0
    raw_offset = 0.0
    filtered_err = 0.0
    rear_est = 0.0
    sensor_state = "no_line"
    bitmap = 0
    front_y = 0.0
    saturated = False
    rows: List[Dict[str, float]] = []

    total_steps = int(round(case.duration_s / dt))
    for step in range(total_steps + 1):
        time_s = step * dt
        if step % ctrl_steps == 0:
            sensor_state, raw_offset, bitmap, front_y = line_sensor(
                rear_y,
                yaw_rad,
                params.sensor_pos_mm,
                params.sensor_to_axle_mm,
                case.line_width_mm,
            )
            if sensor_state == "valid":
                body_err = case.port_sign * raw_offset
                filtered_err = (
                    params.lpf_alpha * body_err +
                    (1.0 - params.lpf_alpha) * filtered_err
                )
                track_err = filtered_err
            elif sensor_state == "no_line":
                track_err = filtered_err if abs(filtered_err) > \
                    params.lost_hold_mm else 0.0
            else:
                track_err = filtered_err

            yaw_err_deg = -math.degrees(yaw_rad)
            rear_est = (
                track_err + params.sensor_to_axle_mm *
                math.sin(math.radians(yaw_err_deg))
            )
            w_cmd_deg = (
                params.k_yaw * yaw_err_deg - params.k_err * rear_est
            )
            diff_mm_s = (
                math.radians(w_cmd_deg) * params.wheel_track_mm * 0.5
            )
            left_cmd = params.speed_mm_s + diff_mm_s
            right_cmd = params.speed_mm_s - diff_mm_s

            rows.append({
                "time_s": time_s,
                "x_mm": x_mm,
                "rear_y_mm": rear_y,
                "front_y_mm": front_y,
                "yaw_deg": math.degrees(yaw_rad),
                "raw_offset_mm": raw_offset,
                "filtered_track_err_mm": filtered_err,
                "rear_est_mm": rear_est,
                "w_cmd_deg_s": w_cmd_deg,
                "left_cmd_mm_s": left_cmd,
                "right_cmd_mm_s": right_cmd,
                "left_act_mm_s": left_act,
                "right_act_mm_s": right_act,
                "sensor_bitmap": float(bitmap),
                "sensor_valid": 1.0 if sensor_state == "valid" else 0.0,
                "motor_saturated": 1.0 if saturated else 0.0,
            })

        if step == total_steps:
            break

        delayed_left = left_cmd
        delayed_right = right_cmd
        if delay_steps > 0:
            left_delay.append(left_cmd)
            right_delay.append(right_cmd)
            delayed_left = left_delay.popleft()
            delayed_right = right_delay.popleft()

        left_target, left_sat = clamp(
            delayed_left * case.motor.left_gain,
            case.motor.max_speed_mm_s,
        )
        right_target, right_sat = clamp(
            delayed_right * case.motor.right_gain,
            case.motor.max_speed_mm_s,
        )
        saturated = left_sat or right_sat
        if case.motor.tau_s <= 0.0:
            left_act = left_target
            right_act = right_target
        else:
            alpha = min(1.0, dt / case.motor.tau_s)
            left_act += alpha * (left_target - left_act)
            right_act += alpha * (right_target - right_act)

        v_mm_s = 0.5 * (left_act + right_act)
        w_rad_s = (left_act - right_act) / params.wheel_track_mm
        x_mm += v_mm_s * math.cos(yaw_rad) * dt
        rear_y += v_mm_s * math.sin(yaw_rad) * dt
        yaw_rad += w_rad_s * dt

    metrics = calc_metrics(rows, case)
    return rows, metrics


def suite(params: FirmwareParams) -> Tuple[Dict[str, List[Dict[str, float]]],
                                            Dict[str, object]]:
    ideal = MotorModel(0.0, 0.0, 1.0, 1.0, None)
    nominal = MotorModel(0.06, 0.01, 1.0, 1.0, 500.0)
    degraded = MotorModel(0.18, 0.04, 0.92, 1.0, 350.0)
    cases = [
        SimCase("contract_ideal_offset", 30.0, 0.0, False, -1.0, ideal),
        SimCase("contract_nominal_offset", 30.0, 0.0, False, -1.0, nominal),
        SimCase("contract_degraded_offset", 30.0, 0.0, False, -1.0, degraded),
        SimCase("contract_nominal_slanted", 0.0, 10.0, True, -1.0, nominal),
        SimCase("as_coded_nominal_offset", 30.0, 0.0, False, 1.0, nominal),
    ]

    runs: Dict[str, List[Dict[str, float]]] = {}
    report: Dict[str, object] = {
        "firmware_parameters": asdict(params),
        "assumptions": {
            "line_width_mm": 20.0,
            "contract_port_sign": -1.0,
            "as_coded_port_sign": 1.0,
            "nominal_motor": asdict(nominal),
            "degraded_motor": asdict(degraded),
        },
        "cases": {},
        "gain_candidates": {},
    }
    for case in cases:
        rows, metrics = simulate(params, case)
        runs[case.name] = rows
        report["cases"][case.name] = {
            "setup": asdict(case),
            "metrics": metrics,
        }

    for k_err in (0.25, 0.30, 0.35, 0.40):
        trial = replace(params, k_err=k_err)
        offset_case = SimCase("gain_offset", 30.0, 0.0, False,
                              -1.0, nominal)
        slanted_case = SimCase("gain_slanted", 0.0, 10.0, True,
                               -1.0, nominal)
        _, offset_metrics = simulate(trial, offset_case)
        _, slanted_metrics = simulate(trial, slanted_case)
        report["gain_candidates"]["K1_{:.2f}_K2_{:.2f}".format(
            k_err, params.k_yaw
        )] = {
            "offset_30mm": offset_metrics,
            "sensor_centered_yaw_10deg": slanted_metrics,
        }
    return runs, report


def write_csv(path: Path, runs: Dict[str, List[Dict[str, float]]]) -> None:
    fieldnames = ["case"] + list(next(iter(runs.values()))[0].keys())
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for name, rows in runs.items():
            for row in rows:
                writer.writerow({"case": name, **row})


def svg_polyline(
    rows: Sequence[Dict[str, float]],
    x_key: str,
    y_key: str,
    bounds: Tuple[float, float, float, float],
    box: Tuple[float, float, float, float],
) -> str:
    x_min, x_max, y_min, y_max = bounds
    left, top, width, height = box
    x_span = max(1e-9, x_max - x_min)
    y_span = max(1e-9, y_max - y_min)
    points = []
    for row in rows:
        px = left + (row[x_key] - x_min) / x_span * width
        py = top + height - (row[y_key] - y_min) / y_span * height
        points.append("{:.2f},{:.2f}".format(px, py))
    return " ".join(points)


def write_svg(path: Path, runs: Dict[str, List[Dict[str, float]]]) -> None:
    width = 1200
    height = 760
    margin = 70
    gap = 65
    panel_w = (width - margin * 2 - gap) / 2
    panel_h = (height - margin * 2 - gap) / 2
    colors = {
        "contract_ideal_offset": "#1f77b4",
        "contract_nominal_offset": "#2ca02c",
        "contract_degraded_offset": "#ff7f0e",
        "contract_nominal_slanted": "#9467bd",
        "as_coded_nominal_offset": "#d62728",
    }
    labels = {
        "contract_ideal_offset": "契约/理想电机",
        "contract_nominal_offset": "契约/标称电机",
        "contract_degraded_offset": "契约/劣化电机",
        "contract_nominal_slanted": "探头居中/车身斜10°",
        "as_coded_nominal_offset": "当前透传符号",
    }
    panels = [
        ("直线轨迹", "x_mm", "rear_y_mm", "行驶距离 x (mm)",
         "后轴偏差 e (mm)"),
        ("后轴误差响应", "time_s", "rear_y_mm", "时间 (s)",
         "后轴偏差 e (mm)"),
        ("航向响应", "time_s", "yaw_deg", "时间 (s)",
         "航向偏差 (deg)"),
        ("目标角速度", "time_s", "w_cmd_deg_s", "时间 (s)",
         "角速度命令 (deg/s)"),
    ]

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
        'viewBox="0 0 {} {}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Segoe UI,Microsoft YaHei,sans-serif;fill:#222}'
        '.axis{stroke:#555;stroke-width:1}.grid{stroke:#ddd;stroke-width:1}'
        '.curve{fill:none;stroke-width:2}</style>',
    ]
    for panel_idx, panel in enumerate(panels):
        row_idx = panel_idx // 2
        col_idx = panel_idx % 2
        left = margin + col_idx * (panel_w + gap)
        top = margin + row_idx * (panel_h + gap)
        title, x_key, y_key, x_label, y_label = panel

        all_rows = [row for rows in runs.values() for row in rows]
        x_values = [row[x_key] for row in all_rows]
        y_values = [row[y_key] for row in all_rows]
        x_min, x_max = min(x_values), max(x_values)
        y_min, y_max = min(y_values), max(y_values)
        y_pad = max(2.0, (y_max - y_min) * 0.08)
        y_min -= y_pad
        y_max += y_pad
        bounds = (x_min, x_max, y_min, y_max)
        box = (left, top, panel_w, panel_h)

        parts.append('<text x="{:.1f}" y="{:.1f}" font-size="16" '
                     'font-weight="600">{}</text>'.format(left, top - 22, title))
        for tick in range(5):
            px = left + panel_w * tick / 4.0
            py = top + panel_h * tick / 4.0
            parts.append('<line class="grid" x1="{0:.1f}" y1="{1:.1f}" '
                         'x2="{0:.1f}" y2="{2:.1f}"/>'.format(
                             px, top, top + panel_h))
            parts.append('<line class="grid" x1="{0:.1f}" y1="{1:.1f}" '
                         'x2="{2:.1f}" y2="{1:.1f}"/>'.format(
                             left, py, left + panel_w))
        parts.append('<line class="axis" x1="{0}" y1="{1}" x2="{2}" '
                     'y2="{1}"/>'.format(left, top + panel_h, left + panel_w))
        parts.append('<line class="axis" x1="{0}" y1="{1}" x2="{0}" '
                     'y2="{2}"/>'.format(left, top, top + panel_h))
        parts.append('<text x="{:.1f}" y="{:.1f}" text-anchor="middle" '
                     'font-size="12">{}</text>'.format(
                         left + panel_w / 2.0, top + panel_h + 35, x_label))
        parts.append('<text x="{:.1f}" y="{:.1f}" text-anchor="middle" '
                     'font-size="12" transform="rotate(-90 {:.1f} {:.1f})">'
                     '{}</text>'.format(
                         left - 45, top + panel_h / 2.0,
                         left - 45, top + panel_h / 2.0, y_label))
        for name, rows in runs.items():
            points = svg_polyline(rows, x_key, y_key, bounds, box)
            parts.append('<polyline class="curve" stroke="{}" points="{}"/>'
                         .format(colors[name], points))

    legend_y = 24
    legend_x = 70
    for name in runs:
        parts.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{1}" '
                     'stroke="{3}" stroke-width="3"/>'.format(
                         legend_x, legend_y, legend_x + 24, colors[name]))
        parts.append('<text x="{}" y="{}" font-size="12">{}</text>'.format(
            legend_x + 30, legend_y + 4, labels[name]))
        legend_x += 205
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="直线巡线闭环仿真")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent / "output",
        help="输出目录",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    params = parse_firmware(root)
    runs, report = suite(params)
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "straight_line_report.json"
    csv_path = args.output / "straight_line_timeseries.csv"
    svg_path = args.output / "straight_line_response.svg"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_csv(csv_path, runs)
    write_svg(svg_path, runs)

    print("参数来源: 固件当前宏定义")
    print("控制周期: {:.1f} ms, 轮距: {:.1f} mm, 探头到后轴: {:.1f} mm".format(
        params.ctrl_period_s * 1000.0,
        params.wheel_track_mm,
        params.sensor_to_axle_mm,
    ))
    print("目标速度: {:.1f} mm/s, K1: {:.3f}, K2: {:.3f}".format(
        params.speed_mm_s, params.k_err, params.k_yaw))
    for name, case_info in report["cases"].items():
        metrics = case_info["metrics"]
        print(
            "{}: settle={} s, final={} mm, rms={} mm, lost={} %".format(
                name,
                metrics["settling_time_s"],
                metrics["final_rear_error_mm"],
                metrics["tail_rms_error_mm"],
                metrics["sensor_invalid_percent"],
            )
        )
    print("报告: " + str(report_path))
    print("曲线: " + str(svg_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
