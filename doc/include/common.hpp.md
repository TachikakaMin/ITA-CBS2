# Documentation: `ITA-CBS2/include/common.hpp`

- Source File: `ITA-CBS2/include/common.hpp`
- Language: `C++`
- Generated: `2026-03-05`

## Purpose
- Shared utilities, map/YAML I/O, path and conflict data types used across solver variants.

## Classes
- `Timer`
- `ScopedTimer`
- `BoolMatrix3D`
- `Edge`

## Structs
- `State`
- `Location`
- `hash`
- `hash`
- `Conflict`
- `VertexConstraint`
- `hash`
- `EdgeConstraint`
- `hash`
- `Constraints`
- `PathEntry`
- `PathEntryCompare`
- `PathEntryCompare2`
- `tuple_hash`
- `tuple_equal`

## Methods (Scoped)
- None detected.

## Functions
- `BoolMatrix3D`
- `Edge`
- `EdgeConstraint`
- `Location`
- `PathEntry`
- `ScopedTimer`
- `State`
- `Timer`
- `VertexConstraint`
- `add`
- `check_ans_valid`
- `createConstraintsFromConflict`
- `elapsedSeconds`
- `equalExceptTime`
- `get`
- `getState`
- `get_block_map`
- `high_focal_score_v2`
- `high_focal_score_v3`
- `high_focal_score_v4`
- `overlap`
- `read_map_file`
- `reset`
- `set`
- `stateValid`
- `stop`
- `transitionValid`
- `~ScopedTimer`

## Includes
- `tuple`
- `list`
- `vector`
- `set`
- `ctime`
- `fstream`
- `iostream`
- `iomanip`
- `algorithm`
- `cmath`
- `chrono`
- `string`
- `filesystem`
- `unordered_set`
- `queue`
- `boost/heap/pairing_heap.hpp`
- `boost/unordered_set.hpp`
- `boost/unordered_map.hpp`
- `boost/functional/hash.hpp`
- `boost/heap/d_ary_heap.hpp`
- `boost/shared_ptr.hpp`
- `boost/heap/priority_queue.hpp`
- `boost/range/algorithm/reverse.hpp`
- `boost/utility.hpp`
- `boost/container/vector.hpp`
- `boost/dynamic_bitset.hpp`
- `boost/thread.hpp`
- `boost/random.hpp`

## Reading Notes
- This document is generated from direct static reading of the source file (symbols/imports/includes) plus path-based role inference.
- For behavior-level validation, run the corresponding executable/tests.
