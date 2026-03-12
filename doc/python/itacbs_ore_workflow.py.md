# Documentation: `ITA-CBS2/python/itacbs_ore_workflow.py`

- Source File: `ITA-CBS2/python/itacbs_ore_workflow.py`
- Language: `Python`
- Generated: `2026-03-05`

## Purpose
- Standalone event-driven ore workflow: scenario editing, iterative planning simulation, and replay.

## Top-Level Classes

- `AgentState`
- `ScenarioState`
- `OreScenarioEditor` methods: `__init__`, `_build_ui`, `_grid_to_canvas`, `_canvas_to_grid`, `_draw`, `_on_hover`, `_on_click`, `_normalize_for_save`, `_load`, `_save`, `_save_as`

## Top-Level Functions
- `_normalize_coord`
- `_sort_coords`
- `_load_map_from_file`
- `_load_map_from_spec`
- `load_scenario`
- `_scenario_to_yaml_dict`
- `save_scenario`
- `_parse_solver_output`
- `_first_event_time_for_agent`
- `_run_solver`
- `run_event_simulation`
- `_ore_at_time`
- `replay_with_tk`
- `run_editor`
- `main`

## Imports
- `__future__`
- `argparse`
- `copy`
- `dataclasses`
- `json`
- `pathlib`
- `subprocess`
- `typing`
- `yaml`

## CLI Flags
- `--binary`
- `--input`
- `--keep-round-files`
- `--max-rounds`
- `--max-steps`
- `--output`
- `--timeout`
- `--verbose`
- `--work-dir`

## Entrypoint
- `if __name__ == "__main__"` block is present.

## Reading Notes
- This document is generated from direct static reading of the source file (symbols/imports/includes) plus path-based role inference.
- For behavior-level validation, run the corresponding executable/tests.
