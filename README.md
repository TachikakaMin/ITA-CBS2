# ITA-CBS

map_data: [data_download](https://drive.google.com/file/d/1NOI4AxlLeqFZKxPTLKaU5x70LJUBswk-/view?usp=sharing)

ITA-CBS new code
Build:

```bash
mkdir build
cd build
cmake ..
cmake --build . --target ITACBS
cmake --build . --target CBSTA
```

Run:
```bash
./ITACBS -i ../map_file/debug_data.yaml -o ../outputs/output.yaml
```

ITACBS Input (LTAPF style):
```bash
./ITACBS -i ../map_file/debug_data_LTAPF.yaml -o ../outputs/output_LTAPF.yaml
```

`main_ITACBS.cpp` now supports LTAPF-style `mapinfo` and reads these per-agent fields:
- `isDroppingoff` (bool)
- `pastPathCost` (int)
- `currentHoldOre` (int)
- `capacity` (int)

And global ore info:
- `mapinfo.potentialGoalsOre` -> mapped as `idx_to_ore` (goal index -> remaining ore)

Hungarian scoring uses:
- If agent goes to collect ore (`isDroppingoff == false`):
  `x = min(remaining_ore_at_goal, agent_capacity)`
  `score = x / (path_cost + agent_past_cost)`
- If agent goes to dropoff (`isDroppingoff == true`):
  `score = agent_current_hold_ore / (path_cost + agent_past_cost)`

Implementation detail:
- Hungarian internally solves minimization, so we minimize `-score` to maximize `score`.
- Internal search keeps this signed value, but final reported `cost` is converted to positive throughput.

Input note:
- `agents[*].potentialGoals` is goal index list.
- `agents[*].potentialDropoffGoals` supports both index list and coordinate list.
- `mapinfo.map` supports either inline map or external `.map` filename.

Output note:
- ITACBS YAML includes `task_assignment` with per-agent `mode`, `goal_idx`, `goal` coordinates, and `ore`.
- A `task_assignment_field_guide` map is emitted as inline documentation for those fields.

Event-Driven Ore Workflow (Python, standalone from LAMAPF-P code):
```bash
# 1) simulate: repeatedly call ITACBS; each pickup/dropoff event triggers replanning
python python/itacbs_ore_workflow.py simulate \
  --input map_file/debug_data_LTAPF.yaml \
  --binary build/ITACBS \
  --output outputs/itacbs_event_sim.json \
  --work-dir outputs/itacbs_rounds \
  --max-rounds 100 --max-steps 2000 --verbose

# 2) replay simulation output (shows agents + ore remaining over time)
python python/itacbs_ore_workflow.py replay --input outputs/itacbs_event_sim.json

# 3) visual edit scenario (ore quantity / ore points / dropoff / agents)
#    save will regenerate each agent's potentialGoals/potentialDropoffGoals as "all points"
python python/itacbs_ore_workflow.py edit --input map_file/debug_data_LTAPF.yaml
```

Generate test case:

```bash
# please check generate_data_for_exp1.py/generate_data_for_exp2.py
# there are some config in it

# in root dir:
python python/generate_data_for_exp2.py --map_path map_file/Boston_0_256.map --output_dir map_file/Paper_boston_256_256_060 --common_ratio 0.6
```

Test:

```bash
# TYMPath and CBSTAPath are just paths to binary file, no difference
# in root dir:
python python/compareTYM2TA.py --TYMPath cmake-build-debug/CBSTA_remake --CBSTAPath cmake-build-debug/ITACBS_remake --map_dir map_file/Paper_boston_256_256_060 --time 15 --seed 0
```

Reproduce Paper Results:

```bash
./run_test/single_exp1.sh build/ITACBS_remake 1 exp1_ITACBS.txt
./run_test/single_exp1.sh build/CBSTA_remake 1 exp1_CBSTA.txt
./run_test/single_exp2.sh build/ITACBS_remake 1 exp2_ITACBS.txt
./run_test/single_exp2.sh build/CBSTA_remake 1 exp2_CBSTA.txt
```

# ITA-ECBS

map_data: [data_download](https://drive.google.com/file/d/1qSGpVkGmnsI23DeuGRqj7CNO0hKfHXB-/view?usp=sharing)

ITA-ECBS
Build:

```bash
mkdir build
cd build
cmake ..
cmake --build . --target ITA_ECBS_v0
cmake --build . --target ITA_ECBS
cmake --build . --target ECBSTA
```

Run:
```bash
./ITA_ECBS_v0 -i ../map_file/debug_data.yaml -o ../outputs/output.yaml
```

# Visualize:

Old:
```bash
# don't use large map(larger than 100*100), it will be very slow and agents will be very small.
# in root dir:
python python/visualize.py [your testcase yaml file] [your program output yaml]
```

New:
You should clone my PlanViz and use ITA-CBS branch:
```bash
python python/PlanViz/script/plan_viz.py --map  map_file/Boston_0_256.map --plan_path_type2 outputs/output.yaml --grid --aid --ca --tid --ppm 2
```


Thank [@MinakoOikawa](https://twitter.com/minakooikawa) for providing the correct dynamic hungarian implementation.


# Papers

```
@INPROCEEDINGS{10416794,
  author={Tang, Yimin and Ren, Zhongqiang and Li, Jiaoyang and Sycara, Katia},
  booktitle={2023 International Symposium on Multi-Robot and Multi-Agent Systems (MRS)}, 
  title={Solving Multi-Agent Target Assignment and Path Finding with a Single Constraint Tree}, 
  year={2023},
  volume={},
  number={},
  pages={8-14},
  keywords={Scalability;Search problems;Planning;Faces;Multi-agent systems},
  doi={10.1109/MRS60187.2023.10416794}}
```

```
@inproceedings{tang2024ita,
  title={ITA-ECBS: A Bounded-Suboptimal Algorithm for Combined Target-Assignment and Path-Finding Problem},
  author={Tang, Yimin and Koenig, Sven and Li, Jiaoyang},
  booktitle={Proceedings of the International Symposium on Combinatorial Search},
  volume={17},
  pages={134--142},
  year={2024}
}
```
