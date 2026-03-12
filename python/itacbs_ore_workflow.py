#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml


Coord = Tuple[int, int]  # (row, col), same convention as ITACBS x,y
OCCUPIED_MAP_CHARS = {"@", "T", "O", "W"}


@dataclass
class AgentState:
    name: str
    pos: Coord
    is_droppingoff: bool
    past_path_cost: int
    current_hold_ore: int
    capacity: int
    current_target: Optional[Coord] = None


@dataclass
class ScenarioState:
    scenario_path: Optional[Path]
    base_dir: Path
    map_spec: Any
    grid: List[List[int]]  # 0 free, 1 blocked
    ore_points: List[Coord]
    ore_amounts: List[int]
    dropoff_points: List[Coord]
    agents: List[AgentState]


def _normalize_coord(node: Any) -> Coord:
    if not isinstance(node, Sequence) or len(node) != 2:
        raise ValueError(f"Invalid coordinate node: {node}")
    return int(node[0]), int(node[1])


def _sort_coords(coords: Sequence[Coord]) -> List[Coord]:
    return sorted(coords, key=lambda p: (p[0], p[1]))


def _load_map_from_file(map_path: Path) -> List[List[int]]:
    if not map_path.exists():
        raise FileNotFoundError(f"Map file not found: {map_path}")
    with map_path.open("r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f]
    if len(lines) < 5:
        raise ValueError(f"Invalid .map format: {map_path}")
    height_line = lines[1].strip()
    width_line = lines[2].strip()
    if not height_line.startswith("height ") or not width_line.startswith("width "):
        raise ValueError(f"Invalid .map header in {map_path}")
    height = int(height_line.split()[1])
    width = int(width_line.split()[1])
    raw = lines[4:]
    if len(raw) != height:
        raise ValueError(f"Map height mismatch in {map_path}: header={height}, rows={len(raw)}")
    grid = [[0 for _ in range(width)] for _ in range(height)]
    for r, line in enumerate(raw):
        if len(line) != width:
            raise ValueError(f"Map width mismatch in {map_path} at row {r}")
        for c, ch in enumerate(line):
            grid[r][c] = 1 if ch in OCCUPIED_MAP_CHARS else 0
    return grid


def _load_map_from_spec(map_spec: Any, base_dir: Path) -> List[List[int]]:
    if isinstance(map_spec, dict):
        dims = map_spec.get("dimensions")
        if not isinstance(dims, Sequence) or len(dims) != 2:
            raise ValueError("Inline map requires dimensions: [rows, cols]")
        rows, cols = int(dims[0]), int(dims[1])
        grid = [[0 for _ in range(cols)] for _ in range(rows)]
        for ob in map_spec.get("obstacles", []):
            x, y = _normalize_coord(ob)
            if 0 <= x < rows and 0 <= y < cols:
                grid[x][y] = 1
        return grid
    if isinstance(map_spec, str):
        map_path = Path(map_spec)
        if not map_path.is_absolute():
            map_path = base_dir / map_path
        return _load_map_from_file(map_path)
    raise ValueError("mapinfo.map must be string or inline map object")


def load_scenario(path: Path) -> ScenarioState:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("Scenario YAML must be a map/object")
    mapinfo = data.get("mapinfo", data)
    if "map" not in mapinfo:
        raise ValueError("Scenario missing mapinfo.map")

    ore_points = [_normalize_coord(p) for p in mapinfo.get("potentialGoals", [])]
    ore_amounts = [int(v) for v in mapinfo.get("potentialGoalsOre", [])]
    if len(ore_amounts) < len(ore_points):
        ore_amounts.extend([0] * (len(ore_points) - len(ore_amounts)))
    elif len(ore_amounts) > len(ore_points):
        ore_amounts = ore_amounts[: len(ore_points)]
    ore_amounts = [max(0, x) for x in ore_amounts]

    dropoff_points = [_normalize_coord(p) for p in mapinfo.get("potentialDropoffGoals", [])]
    agents_yaml = data.get("agents", [])
    agents: List[AgentState] = []
    for i, node in enumerate(agents_yaml):
        start = _normalize_coord(node.get("start", [0, 0]))
        current_target: Optional[Coord] = None
        if node.get("currentTarget") is not None:
            current_target = _normalize_coord(node.get("currentTarget"))
        agents.append(
            AgentState(
                name=str(node.get("name", f"agent{i}")),
                pos=start,
                is_droppingoff=bool(node.get("isDroppingoff", False)),
                past_path_cost=int(node.get("pastPathCost", 0)),
                current_hold_ore=max(0, int(node.get("currentHoldOre", 0))),
                capacity=max(0, int(node.get("capacity", 0))),
                current_target=current_target,
            )
        )

    base_dir = path.parent.resolve()
    grid = _load_map_from_spec(mapinfo["map"], base_dir)
    return ScenarioState(
        scenario_path=path.resolve(),
        base_dir=base_dir,
        map_spec=copy.deepcopy(mapinfo["map"]),
        grid=grid,
        ore_points=ore_points,
        ore_amounts=ore_amounts,
        dropoff_points=dropoff_points,
        agents=agents,
    )


def _scenario_to_yaml_dict(state: ScenarioState) -> Dict[str, Any]:
    ore_indices = list(range(len(state.ore_points)))
    drop_indices = list(range(len(state.dropoff_points)))
    agents_yaml: List[Dict[str, Any]] = []
    for a in state.agents:
        node: Dict[str, Any] = {
            "name": a.name,
            "potentialGoals": ore_indices,
            "start": [a.pos[0], a.pos[1]],
            "isDroppingoff": bool(a.is_droppingoff),
            "pastPathCost": int(a.past_path_cost),
            "currentHoldOre": int(a.current_hold_ore),
            "capacity": int(a.capacity),
            "potentialDropoffGoals": drop_indices,
        }
        if a.current_target is not None:
            node["currentTarget"] = [a.current_target[0], a.current_target[1]]
        agents_yaml.append(node)

    return {
        "mapinfo": {
            "map": copy.deepcopy(state.map_spec),
            "potentialGoals": [[x, y] for (x, y) in state.ore_points],
            "potentialGoalsOre": [int(v) for v in state.ore_amounts],
            "potentialDropoffGoals": [[x, y] for (x, y) in state.dropoff_points],
        },
        "agents": agents_yaml,
    }


def save_scenario(path: Path, state: ScenarioState) -> None:
    out = _scenario_to_yaml_dict(state)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, sort_keys=False)


def _parse_solver_output(
    out_path: Path, fallback_positions: List[Coord]
) -> Tuple[List[List[Coord]], Any, int, bool]:
    with out_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid solver output YAML: {out_path}")
    schedule = data.get("schedule", {})
    if schedule is None:
        schedule = {}
    has_schedule = bool(schedule)
    paths: List[List[Coord]] = []
    for i, fallback in enumerate(fallback_positions):
        key = f"agent{i}"
        seq = schedule.get(key, [])
        if not seq:
            paths.append([fallback])
            continue
        path: List[Coord] = []
        for node in seq:
            x = int(node["x"])
            y = int(node["y"])
            path.append((x, y))
        if not path:
            path = [fallback]
        paths.append(path)
    stats = data.get("statistics", {})
    solver_cost = stats.get("cost")
    team_size = int(stats.get("teamSize", 0))
    return paths, solver_cost, team_size, has_schedule


def _coord_to_list(pos: Optional[Coord]) -> Optional[List[int]]:
    if pos is None:
        return None
    return [int(pos[0]), int(pos[1])]


def _write_round_assignment_file(
    path: Path,
    rid: int,
    sim_time: int,
    solver_cost: Any,
    team_size: int,
    has_schedule: bool,
    agents: Sequence[AgentState],
    paths: Sequence[List[Coord]],
    planned_targets: Sequence[Optional[Coord]],
    candidate_events: Sequence[Dict[str, Any]],
) -> None:
    event_by_agent: Dict[int, Dict[str, Any]] = {}
    for e in candidate_events:
        aid = int(e.get("agent", -1))
        if aid < 0:
            continue
        prev = event_by_agent.get(aid)
        if prev is None or int(e.get("time", 10**9)) < int(prev.get("time", 10**9)):
            event_by_agent[aid] = e

    assignments: List[Dict[str, Any]] = []
    for i, agent in enumerate(agents):
        path_i = paths[i] if i < len(paths) else []
        goal = planned_targets[i] if i < len(planned_targets) else None
        evt = event_by_agent.get(i)
        assignments.append(
            {
                "agent": int(i),
                "name": agent.name,
                "mode": "dropoff" if agent.is_droppingoff else "pickup",
                "start": [int(agent.pos[0]), int(agent.pos[1])],
                "goal": _coord_to_list(goal),
                "event_type": evt.get("kind") if evt is not None else None,
                "event_time": int(evt.get("time", 0)) if evt is not None else None,
                "event_pos": _coord_to_list(evt.get("pos")) if evt is not None else None,
                "path_nodes": int(len(path_i)),
                "path_end": _coord_to_list(path_i[-1]) if path_i else None,
            }
        )

    payload = {
        "round": int(rid),
        "time_start": int(sim_time),
        "has_schedule": bool(has_schedule),
        "team_size": int(team_size),
        "solver_cost": solver_cost,
        "field_guide_en": {
            "mode": "Agent mission mode in this round: pickup or dropoff.",
            "goal": "Assigned target for this round (based on first actionable event).",
            "event_type": "Type of first actionable event on the path.",
            "event_time": "Steps from round start to the first actionable event.",
            "event_pos": "Grid location where the first actionable event occurs.",
            "path_nodes": "Number of nodes in the solver path sequence.",
            "path_end": "Last node in solver path; may be beyond executed steps in this round.",
        },
        "assignments": assignments,
    }
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, sort_keys=False)


def _first_event_time_for_agent(
    agent: AgentState,
    path: List[Coord],
    ore_index: Dict[Coord, int],
    ore_amounts: Sequence[int],
    dropoff_set: set[Coord],
) -> Tuple[Optional[int], Optional[str], Optional[Coord]]:
    if not path:
        return None, None, None
    # Handle immediate event on current cell.
    pos0 = path[0]
    if agent.is_droppingoff:
        if pos0 in dropoff_set:
            return 0, "dropoff", pos0
    else:
        idx0 = ore_index.get(pos0)
        if idx0 is not None and ore_amounts[idx0] > 0:
            return 0, "pickup", pos0
    if len(path) <= 1:
        return None, None, None
    for t in range(1, len(path)):
        pos = path[t]
        if agent.is_droppingoff:
            if pos in dropoff_set:
                return t, "dropoff", pos
        else:
            idx = ore_index.get(pos)
            if idx is not None and ore_amounts[idx] > 0:
                return t, "pickup", pos
    return None, None, None


def _first_collision_time(paths: List[List[Coord]]) -> Optional[int]:
    if not paths:
        return None
    max_t = max((len(p) for p in paths), default=0)
    if max_t <= 0:
        return None

    def at(path: List[Coord], t: int) -> Coord:
        return path[min(t, len(path) - 1)]

    for t in range(max_t):
        # Vertex collision
        seen: Dict[Coord, int] = {}
        for i, path in enumerate(paths):
            pos = at(path, t)
            prev = seen.get(pos)
            if prev is not None and prev != i:
                return t
            seen[pos] = i

        # Edge swap collision
        for i in range(len(paths)):
            for j in range(i + 1, len(paths)):
                a0 = at(paths[i], t)
                b0 = at(paths[j], t)
                a1 = at(paths[i], t + 1)
                b1 = at(paths[j], t + 1)
                if a0 == b1 and a1 == b0:
                    return t
    return None


def _run_solver(binary: Path, input_yaml: Path, output_yaml: Path, timeout_s: int) -> str:
    cmd = [str(binary), "-i", str(input_yaml), "-o", str(output_yaml)]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    output = (proc.stdout or "") + (proc.stderr or "")
    if proc.returncode != 0:
        raise RuntimeError(f"ITACBS failed (code {proc.returncode}).\n{output}")
    if not output_yaml.exists():
        raise RuntimeError(
            f"ITACBS finished without writing output file: {output_yaml}\n"
            f"Input: {input_yaml}\n{output}"
        )
    return output


def run_event_simulation(
    scenario: ScenarioState,
    itacbs_binary: Path,
    max_rounds: int,
    max_steps: int,
    work_dir: Path,
    keep_round_files: bool,
    timeout_s: int,
    verbose: bool,
) -> Dict[str, Any]:
    if not itacbs_binary.exists():
        raise FileNotFoundError(f"ITACBS binary not found: {itacbs_binary}")
    state = copy.deepcopy(scenario)
    if isinstance(state.map_spec, str):
        map_path = Path(state.map_spec)
        if not map_path.is_absolute():
            state.map_spec = str((state.base_dir / map_path).resolve())
    work_dir.mkdir(parents=True, exist_ok=True)

    trajectories: Dict[str, List[List[int]]] = {}
    for i, a in enumerate(state.agents):
        trajectories[str(i)] = [[a.pos[0], a.pos[1]]]

    rounds: List[Dict[str, Any]] = []
    ore_checkpoints: List[Dict[str, Any]] = [{"time": 0, "ore": [int(v) for v in state.ore_amounts]}]
    sim_time = 0
    total_delivered_ore = 0
    stop_reason = "max_rounds"

    for rid in range(max_rounds):
        ore_left = sum(state.ore_amounts)
        if ore_left <= 0 and all(a.current_hold_ore <= 0 for a in state.agents):
            stop_reason = "all_ore_finished"
            break
        if sim_time >= max_steps:
            stop_reason = "max_steps"
            break

        in_file = work_dir / f"round_{rid:04d}_input.yaml"
        out_file = work_dir / f"round_{rid:04d}_output.yaml"
        assign_file = work_dir / f"round_{rid:04d}_assignment.yaml"
        save_scenario(in_file, state)

        if verbose:
            print(
                f"[round {rid}] t={sim_time} ore_left={sum(state.ore_amounts)} "
                f"loaded={sum(1 for a in state.agents if a.is_droppingoff)}"
            )

        solver_log = _run_solver(itacbs_binary, in_file, out_file, timeout_s=timeout_s)
        fallback_positions = [a.pos for a in state.agents]
        paths, solver_cost, team_size, has_schedule = _parse_solver_output(out_file, fallback_positions)
        if not has_schedule or team_size <= 0:
            _write_round_assignment_file(
                path=assign_file,
                rid=rid,
                sim_time=sim_time,
                solver_cost=solver_cost,
                team_size=team_size,
                has_schedule=has_schedule,
                agents=state.agents,
                paths=paths,
                planned_targets=[None] * len(state.agents),
                candidate_events=[],
            )
            stop_reason = "solver_no_schedule"
            rounds.append(
                {
                    "round": rid,
                    "time_start": sim_time,
                    "time_end": sim_time,
                    "delta": 0,
                    "solver_cost": solver_cost,
                    "events": [],
                    "ore_after": [int(v) for v in state.ore_amounts],
                    "solver_log": solver_log[-2000:],
                }
            )
            break

        ore_index = {p: i for i, p in enumerate(state.ore_points)}
        dropoff_set = set(state.dropoff_points)
        candidate_events: List[Dict[str, Any]] = []
        planned_targets: List[Optional[Coord]] = [None] * len(state.agents)
        for i, (agent, path) in enumerate(zip(state.agents, paths)):
            t, kind, pos = _first_event_time_for_agent(agent, path, ore_index, state.ore_amounts, dropoff_set)
            if pos is not None:
                planned_targets[i] = pos
            if t is None:
                continue
            candidate_events.append({"agent": i, "time": t, "kind": kind, "pos": pos})
        for i, agent in enumerate(state.agents):
            agent.current_target = planned_targets[i]
        _write_round_assignment_file(
            path=assign_file,
            rid=rid,
            sim_time=sim_time,
            solver_cost=solver_cost,
            team_size=team_size,
            has_schedule=has_schedule,
            agents=state.agents,
            paths=paths,
            planned_targets=planned_targets,
            candidate_events=candidate_events,
        )

        if not candidate_events:
            stop_reason = "no_future_event"
            rounds.append(
                {
                    "round": rid,
                    "time_start": sim_time,
                    "time_end": sim_time,
                    "delta": 0,
                    "solver_cost": solver_cost,
                    "events": [],
                    "ore_after": [int(v) for v in state.ore_amounts],
                    "solver_log": solver_log[-2000:],
                }
            )
            break

        delta = min(e["time"] for e in candidate_events)
        if sim_time + delta > max_steps:
            delta = max_steps - sim_time
        collision_t = _first_collision_time(paths)
        if collision_t is not None and collision_t <= delta:
            safe_delta = collision_t - 1
            if safe_delta < 0:
                stop_reason = "solver_conflicting_schedule"
                rounds.append(
                    {
                        "round": rid,
                        "time_start": sim_time,
                        "time_end": sim_time,
                        "delta": 0,
                        "solver_cost": solver_cost,
                        "events": [],
                        "ore_after": [int(v) for v in state.ore_amounts],
                        "solver_log": solver_log[-2000:],
                    }
                )
                break
            delta = safe_delta
        if delta < 0:
            stop_reason = "zero_delta"
            break
        if delta == 0 and not any(e["time"] == 0 for e in candidate_events):
            stop_reason = "solver_conflicting_schedule"
            rounds.append(
                {
                    "round": rid,
                    "time_start": sim_time,
                    "time_end": sim_time,
                    "delta": 0,
                    "solver_cost": solver_cost,
                    "events": [],
                    "ore_after": [int(v) for v in state.ore_amounts],
                    "solver_log": solver_log[-2000:],
                }
            )
            break

        for i, (agent, path) in enumerate(zip(state.agents, paths)):
            for step in range(1, delta + 1):
                idx = min(step, len(path) - 1)
                p = path[idx]
                trajectories[str(i)].append([p[0], p[1]])
            end_idx = min(delta, len(path) - 1)
            agent.pos = path[end_idx]
            agent.past_path_cost += delta

        round_events: List[Dict[str, Any]] = []
        for e in candidate_events:
            if e["time"] != delta:
                continue
            i = int(e["agent"])
            agent = state.agents[i]
            pos = e["pos"]
            if e["kind"] == "pickup":
                ore_idx = ore_index[pos]
                available = max(0, state.ore_amounts[ore_idx])
                free_cap = max(0, agent.capacity - agent.current_hold_ore)
                take = min(available, free_cap)
                if take > 0:
                    state.ore_amounts[ore_idx] -= take
                    agent.current_hold_ore += take
                    agent.is_droppingoff = True
                agent.current_target = None
                round_events.append(
                    {
                        "agent": i,
                        "type": "pickup",
                        "pos": [pos[0], pos[1]],
                        "amount": int(take),
                        "ore_remaining_at_point": int(state.ore_amounts[ore_idx]),
                    }
                )
            elif e["kind"] == "dropoff":
                delivered = max(0, agent.current_hold_ore)
                total_delivered_ore += delivered
                agent.current_hold_ore = 0
                agent.is_droppingoff = False
                # Start a new trip after delivery; reset historical path cost.
                agent.past_path_cost = 0
                agent.current_target = None
                round_events.append(
                    {
                        "agent": i,
                        "type": "dropoff",
                        "pos": [pos[0], pos[1]],
                        "amount": int(delivered),
                    }
                )

        sim_time += delta
        ore_checkpoints.append({"time": sim_time, "ore": [int(v) for v in state.ore_amounts]})
        rounds.append(
            {
                "round": rid,
                "time_start": sim_time - delta,
                "time_end": sim_time,
                "delta": int(delta),
                "solver_cost": solver_cost,
                "events": round_events,
                "ore_after": [int(v) for v in state.ore_amounts],
                "solver_log": solver_log[-2000:],
            }
        )

        if not keep_round_files:
            try:
                in_file.unlink(missing_ok=True)
                out_file.unlink(missing_ok=True)
            except OSError:
                pass

    result = {
        "source_scenario": str(scenario.scenario_path) if scenario.scenario_path else None,
        "itacbs_binary": str(itacbs_binary),
        "final_time": int(sim_time),
        "stop_reason": stop_reason,
        "total_delivered_ore": int(total_delivered_ore),
        "ore_points": [[p[0], p[1]] for p in state.ore_points],
        "dropoff_points": [[p[0], p[1]] for p in state.dropoff_points],
        "ore_checkpoints": ore_checkpoints,
        "map_spec": copy.deepcopy(state.map_spec),
        "rounds": rounds,
        "agents": {
            str(i): {
                "name": a.name,
                "trajectory": trajectories[str(i)],
                "final": {
                    "pos": [a.pos[0], a.pos[1]],
                    "isDroppingoff": bool(a.is_droppingoff),
                    "pastPathCost": int(a.past_path_cost),
                    "currentHoldOre": int(a.current_hold_ore),
                    "capacity": int(a.capacity),
                    "currentTarget": [a.current_target[0], a.current_target[1]] if a.current_target is not None else None,
                },
            }
            for i, a in enumerate(state.agents)
        },
    }
    return result


def _ore_at_time(checkpoints: List[Dict[str, Any]], t: int) -> List[int]:
    if not checkpoints:
        return []
    best = checkpoints[0]["ore"]
    for cp in checkpoints:
        if int(cp["time"]) <= t:
            best = cp["ore"]
        else:
            break
    return [int(x) for x in best]


def replay_with_tk(sim_output_path: Path) -> None:
    import tkinter as tk

    with sim_output_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    map_spec = data.get("map_spec")
    if map_spec is None:
        raise ValueError("Simulation output missing map_spec")
    base_dir = sim_output_path.parent
    grid = _load_map_from_spec(map_spec, base_dir)
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0

    ore_points = [tuple(p) for p in data.get("ore_points", [])]
    drop_points = {tuple(p) for p in data.get("dropoff_points", [])}
    checkpoints = data.get("ore_checkpoints", [])
    agents = data.get("agents", {})
    trajs = {int(k): [tuple(p) for p in v.get("trajectory", [])] for k, v in agents.items()}
    max_t = max((len(tr) for tr in trajs.values()), default=1) - 1

    cell = 26
    margin = 16
    width = margin * 2 + cols * cell
    height = margin * 2 + rows * cell

    root = tk.Tk()
    root.title("ITACBS Ore Replay")
    canvas = tk.Canvas(root, width=width, height=height, bg="#f8f8f8")
    canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    ctl = tk.Frame(root)
    ctl.pack(side=tk.BOTTOM, fill=tk.X)
    t_var = tk.IntVar(value=0)
    playing = {"v": False}
    after_id = {"v": None}

    def grid_to_canvas(r: int, c: int) -> Tuple[int, int, int, int]:
        x0 = margin + c * cell
        y0 = margin + r * cell
        return x0, y0, x0 + cell, y0 + cell

    def draw():
        canvas.delete("all")
        t = t_var.get()
        ore_now = _ore_at_time(checkpoints, t)
        ore_dict = {ore_points[i]: ore_now[i] if i < len(ore_now) else 0 for i in range(len(ore_points))}

        for r in range(rows):
            for c in range(cols):
                x0, y0, x1, y1 = grid_to_canvas(r, c)
                fill = "#2f2f2f" if grid[r][c] else "#ffffff"
                canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#c8c8c8")

        for (r, c) in drop_points:
            x0, y0, x1, y1 = grid_to_canvas(r, c)
            canvas.create_rectangle(x0 + 4, y0 + 4, x1 - 4, y1 - 4, fill="#41b883", outline="")

        for (r, c), q in ore_dict.items():
            x0, y0, x1, y1 = grid_to_canvas(r, c)
            canvas.create_oval(x0 + 5, y0 + 5, x1 - 5, y1 - 5, fill="#f39c12", outline="")
            canvas.create_text((x0 + x1) // 2, (y0 + y1) // 2, text=str(q), fill="black")

        for aid in sorted(trajs.keys()):
            tr = trajs[aid]
            if not tr:
                continue
            idx = min(t, len(tr) - 1)
            r, c = tr[idx]
            x0, y0, x1, y1 = grid_to_canvas(r, c)
            canvas.create_oval(x0 + 6, y0 + 6, x1 - 6, y1 - 6, fill="#3498db", outline="")
            canvas.create_text((x0 + x1) // 2, y0 + 3, text=str(aid), fill="#1f3a5f", anchor="n")

        label.config(text=f"t={t}  delivered={data.get('total_delivered_ore', 0)}")

    def tick():
        if not playing["v"]:
            return
        t = t_var.get()
        if t >= max_t:
            playing["v"] = False
            return
        t_var.set(t + 1)
        draw()
        after_id["v"] = root.after(150, tick)

    def on_play():
        playing["v"] = True
        tick()

    def on_pause():
        playing["v"] = False
        if after_id["v"] is not None:
            root.after_cancel(after_id["v"])
            after_id["v"] = None

    def on_step(d: int):
        on_pause()
        t = max(0, min(max_t, t_var.get() + d))
        t_var.set(t)
        draw()

    tk.Button(ctl, text="Play", command=on_play).pack(side=tk.LEFT)
    tk.Button(ctl, text="Pause", command=on_pause).pack(side=tk.LEFT)
    tk.Button(ctl, text="<", command=lambda: on_step(-1)).pack(side=tk.LEFT)
    tk.Button(ctl, text=">", command=lambda: on_step(1)).pack(side=tk.LEFT)
    scale = tk.Scale(ctl, from_=0, to=max_t, orient=tk.HORIZONTAL, variable=t_var, command=lambda _v: draw())
    scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
    label = tk.Label(ctl, text="")
    label.pack(side=tk.RIGHT)

    draw()
    root.mainloop()


class OreScenarioEditor:
    def __init__(self, root, scenario_path: Path):
        import tkinter as tk
        from tkinter import filedialog, messagebox, simpledialog

        self.tk = tk
        self.filedialog = filedialog
        self.messagebox = messagebox
        self.simpledialog = simpledialog

        self.root = root
        self.root.title("ITACBS Ore Scenario Editor")
        self.scenario_path = scenario_path.resolve()
        self.state = load_scenario(self.scenario_path)

        self.cell = 28
        self.margin = 16
        self.tool = tk.StringVar(value="ore")
        self.default_capacity = tk.IntVar(value=5)
        self.default_past = tk.IntVar(value=0)
        self.default_hold = tk.IntVar(value=0)
        self.default_drop = tk.IntVar(value=0)

        self._build_ui()
        self._draw()

    def _build_ui(self):
        tk = self.tk
        left = tk.Frame(self.root)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = tk.Frame(self.root)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        tk.Label(left, text=f"Scenario:\n{self.scenario_path.name}", justify="left").pack(anchor="w")
        tk.Button(left, text="Load", command=self._load).pack(fill=tk.X)
        tk.Button(left, text="Save", command=self._save).pack(fill=tk.X)
        tk.Button(left, text="Save As", command=self._save_as).pack(fill=tk.X)

        tk.Label(left, text="Tool").pack(anchor="w", pady=(8, 0))
        for v in ["ore", "dropoff", "agent", "erase"]:
            tk.Radiobutton(left, text=v, variable=self.tool, value=v).pack(anchor="w")

        tk.Label(left, text="New Agent Defaults").pack(anchor="w", pady=(8, 0))
        tk.Label(left, text="capacity").pack(anchor="w")
        tk.Entry(left, textvariable=self.default_capacity, width=8).pack(anchor="w")
        tk.Label(left, text="pastPathCost").pack(anchor="w")
        tk.Entry(left, textvariable=self.default_past, width=8).pack(anchor="w")
        tk.Label(left, text="currentHoldOre").pack(anchor="w")
        tk.Entry(left, textvariable=self.default_hold, width=8).pack(anchor="w")
        tk.Checkbutton(left, text="isDroppingoff", variable=self.default_drop).pack(anchor="w")

        self.info = tk.Label(left, text="", justify="left")
        self.info.pack(anchor="w", pady=(8, 0))

        self.canvas = tk.Canvas(right, bg="#f4f4f4")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<Motion>", self._on_hover)

    def _grid_to_canvas(self, r: int, c: int) -> Tuple[int, int, int, int]:
        x0 = self.margin + c * self.cell
        y0 = self.margin + r * self.cell
        return x0, y0, x0 + self.cell, y0 + self.cell

    def _canvas_to_grid(self, event) -> Coord:
        c = int((event.x - self.margin) / self.cell)
        r = int((event.y - self.margin) / self.cell)
        return r, c

    def _draw(self):
        rows = len(self.state.grid)
        cols = len(self.state.grid[0]) if rows > 0 else 0
        self.canvas.delete("all")
        ore_dict = {self.state.ore_points[i]: self.state.ore_amounts[i] for i in range(len(self.state.ore_points))}
        drop_set = set(self.state.dropoff_points)
        agent_pos = {a.pos: (idx, a) for idx, a in enumerate(self.state.agents)}

        for r in range(rows):
            for c in range(cols):
                x0, y0, x1, y1 = self._grid_to_canvas(r, c)
                fill = "#2d2d2d" if self.state.grid[r][c] else "#ffffff"
                self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="#c7c7c7")

        for (r, c), q in ore_dict.items():
            x0, y0, x1, y1 = self._grid_to_canvas(r, c)
            self.canvas.create_oval(x0 + 5, y0 + 5, x1 - 5, y1 - 5, fill="#f39c12", outline="")
            self.canvas.create_text((x0 + x1) // 2, (y0 + y1) // 2, text=str(q), fill="#111111")

        for (r, c) in drop_set:
            x0, y0, x1, y1 = self._grid_to_canvas(r, c)
            self.canvas.create_rectangle(x0 + 4, y0 + 4, x1 - 4, y1 - 4, fill="#3cb371", outline="")

        for idx, a in enumerate(self.state.agents):
            r, c = a.pos
            x0, y0, x1, y1 = self._grid_to_canvas(r, c)
            fill = "#8e44ad" if a.is_droppingoff else "#3498db"
            self.canvas.create_oval(x0 + 6, y0 + 6, x1 - 6, y1 - 6, fill=fill, outline="")
            self.canvas.create_text((x0 + x1) // 2, y0 + 2, text=str(idx), anchor="n", fill="#1f2d3d")

        self.info.config(
            text=(
                f"ore_points={len(self.state.ore_points)}  dropoffs={len(self.state.dropoff_points)}\n"
                f"agents={len(self.state.agents)}"
            )
        )

    def _on_hover(self, event):
        rows = len(self.state.grid)
        cols = len(self.state.grid[0]) if rows > 0 else 0
        r, c = self._canvas_to_grid(event)
        if r < 0 or c < 0 or r >= rows or c >= cols:
            return
        ore_dict = {self.state.ore_points[i]: self.state.ore_amounts[i] for i in range(len(self.state.ore_points))}
        msg = f"cell=({r},{c})"
        if (r, c) in ore_dict:
            msg += f" ore={ore_dict[(r, c)]}"
        for idx, a in enumerate(self.state.agents):
            if a.pos == (r, c):
                msg += (
                    f" | agent{idx}: drop={a.is_droppingoff}, hold={a.current_hold_ore}, "
                    f"past={a.past_path_cost}, cap={a.capacity}"
                )
                break
        self.root.title(f"ITACBS Ore Scenario Editor - {msg}")

    def _on_click(self, event):
        rows = len(self.state.grid)
        cols = len(self.state.grid[0]) if rows > 0 else 0
        r, c = self._canvas_to_grid(event)
        if r < 0 or c < 0 or r >= rows or c >= cols:
            return
        if self.state.grid[r][c] == 1:
            return

        tool = self.tool.get()
        ore_dict = {self.state.ore_points[i]: i for i in range(len(self.state.ore_points))}
        drop_set = set(self.state.dropoff_points)

        if tool == "ore":
            current = 0
            if (r, c) in ore_dict:
                current = self.state.ore_amounts[ore_dict[(r, c)]]
            q = self.simpledialog.askinteger("Ore quantity", f"Set ore at ({r},{c})", initialvalue=current, minvalue=0)
            if q is None:
                return
            if (r, c) in ore_dict:
                idx = ore_dict[(r, c)]
                if q <= 0:
                    self.state.ore_points.pop(idx)
                    self.state.ore_amounts.pop(idx)
                else:
                    self.state.ore_amounts[idx] = int(q)
            elif q > 0:
                self.state.ore_points.append((r, c))
                self.state.ore_amounts.append(int(q))

        elif tool == "dropoff":
            if (r, c) in drop_set:
                self.state.dropoff_points = [p for p in self.state.dropoff_points if p != (r, c)]
            else:
                self.state.dropoff_points.append((r, c))

        elif tool == "agent":
            for a in self.state.agents:
                if a.pos == (r, c):
                    return
            idx = len(self.state.agents)
            self.state.agents.append(
                AgentState(
                    name=f"agent{idx}",
                    pos=(r, c),
                    is_droppingoff=bool(self.default_drop.get()),
                    past_path_cost=max(0, int(self.default_past.get())),
                    current_hold_ore=max(0, int(self.default_hold.get())),
                    capacity=max(0, int(self.default_capacity.get())),
                )
            )

        elif tool == "erase":
            if (r, c) in ore_dict:
                idx = ore_dict[(r, c)]
                self.state.ore_points.pop(idx)
                self.state.ore_amounts.pop(idx)
            self.state.dropoff_points = [p for p in self.state.dropoff_points if p != (r, c)]
            self.state.agents = [a for a in self.state.agents if a.pos != (r, c)]

        self._draw()

    def _normalize_for_save(self):
        paired = list(zip(self.state.ore_points, self.state.ore_amounts))
        paired.sort(key=lambda t: (t[0][0], t[0][1]))
        self.state.ore_points = [p for p, _ in paired]
        self.state.ore_amounts = [max(0, int(v)) for _, v in paired]
        self.state.dropoff_points = _sort_coords(self.state.dropoff_points)

    def _load(self):
        path = self.filedialog.askopenfilename(
            title="Open scenario YAML",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self.scenario_path = Path(path).resolve()
            self.state = load_scenario(self.scenario_path)
            self._draw()
        except Exception as e:
            self.messagebox.showerror("Load failed", str(e))

    def _save(self):
        try:
            self._normalize_for_save()
            save_scenario(self.scenario_path, self.state)
            self.messagebox.showinfo("Saved", f"Saved to {self.scenario_path}")
        except Exception as e:
            self.messagebox.showerror("Save failed", str(e))

    def _save_as(self):
        path = self.filedialog.asksaveasfilename(
            title="Save scenario as",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self._normalize_for_save()
            self.scenario_path = Path(path).resolve()
            save_scenario(self.scenario_path, self.state)
            self.messagebox.showinfo("Saved", f"Saved to {self.scenario_path}")
        except Exception as e:
            self.messagebox.showerror("Save failed", str(e))


def run_editor(scenario_path: Path) -> None:
    import tkinter as tk

    root = tk.Tk()
    OreScenarioEditor(root, scenario_path)
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="ITACBS event-driven ore workflow tools")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_sim = sub.add_parser("simulate", help="Run event-driven simulation with repeated ITACBS calls")
    p_sim.add_argument("--input", required=True, help="Input LTAPF YAML")
    p_sim.add_argument("--binary", default="build/ITACBS", help="Path to ITACBS binary")
    p_sim.add_argument("--output", default="outputs/itacbs_event_sim.json", help="Simulation output JSON")
    p_sim.add_argument("--work-dir", default="outputs/itacbs_rounds", help="Round YAML temp/output directory")
    p_sim.add_argument("--max-rounds", type=int, default=100, help="Max replanning rounds")
    p_sim.add_argument("--max-steps", type=int, default=2000, help="Max simulated timesteps")
    p_sim.add_argument("--timeout", type=int, default=120, help="Timeout per ITACBS call (seconds)")
    p_sim.add_argument("--keep-round-files", action="store_true", help="Keep per-round input/output YAML files")
    p_sim.add_argument("--verbose", action="store_true", help="Print per-round summary")

    p_replay = sub.add_parser("replay", help="Replay a simulation output JSON")
    p_replay.add_argument("--input", required=True, help="Simulation output JSON from simulate")

    p_edit = sub.add_parser("edit", help="Visual editor for ore/dropoff/agents in LTAPF YAML")
    p_edit.add_argument("--input", required=True, help="Scenario YAML to edit")

    args = parser.parse_args()

    if args.cmd == "simulate":
        scenario_path = Path(args.input).resolve()
        binary = Path(args.binary).resolve()
        output_json = Path(args.output).resolve()
        work_dir = Path(args.work_dir).resolve()

        scenario = load_scenario(scenario_path)
        result = run_event_simulation(
            scenario=scenario,
            itacbs_binary=binary,
            max_rounds=max(1, int(args.max_rounds)),
            max_steps=max(1, int(args.max_steps)),
            work_dir=work_dir,
            keep_round_files=bool(args.keep_round_files),
            timeout_s=max(1, int(args.timeout)),
            verbose=bool(args.verbose),
        )
        output_json.parent.mkdir(parents=True, exist_ok=True)
        with output_json.open("w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        print(
            f"Simulation done. time={result['final_time']} stop={result['stop_reason']} "
            f"delivered={result['total_delivered_ore']} -> {output_json}"
        )

    elif args.cmd == "replay":
        replay_with_tk(Path(args.input).resolve())

    elif args.cmd == "edit":
        run_editor(Path(args.input).resolve())


if __name__ == "__main__":
    main()
