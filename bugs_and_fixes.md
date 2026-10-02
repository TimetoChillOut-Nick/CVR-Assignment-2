# Bugs and Fixes

## 1. Escape manoeuvre aborts as soon as sensors clear
**File:** `group_project_controller.py` (safety wrapper in `main()` from first major groupe merge)

- **Expected:** When an obstacle triggers recovery, the robot backs up (or drives forward) then turns away before resuming the mission.
- **Actual:** Recovery only ran while a sensor read above `PS_TRIGGER`. Once the robot backed up enough for the reading to drop, it skipped the turn and went straight back to the missionhitting the same obstacle.
- **Solved:** Changed the wrapper condition to `if recovery_state is not None or is_unsafe(prox):` so a started manoeuvre always runs to completion and resets `recovery_state` to `None`.

## 2. Station order does two laps of the arena (attempted improvement)
**File:** `group_project_controller.py` (NAVIGATION planning), `project_utils.py` (`QUADRANT_PAIRS`)

- **Expected:** The bot visits stations in an efficient order.
- **Actual:** Always picking the closest station meant it did a lap of the interior stations, then a lap of the boundary stations. A full run took 3:50.
- **Attempted:** Stations are paired by quadrant (S1/S2, S3/S4, S5/S6, S7/S8). After clearing a station the bot goes to its partner next, then uses A* to pick the closest station to start the next quadrant.
- **Result:** 4:12, slower than the 3:50 baseline. Rolled back to closest-first.

Code that was tried:

```python
#project_utils.py
#Stations paired by quadrant so the bot clears one quadrant before moving on
QUADRANT_PAIRS = {
    "S1": "S2", "S2": "S1",
    "S3": "S4", "S4": "S3",
    "S5": "S6", "S6": "S5",
    "S7": "S8", "S8": "S7",
}
```

```python
#group_project_controller.py, NAVIGATION planning
#Go to the quadrant partner of the last cleared station first
partner = QUADRANT_PAIRS.get(last_cleared)
quadrant = [s for s in remaining_stations if s["id"] == partner]
current_station, path, scores = find_closest_station(GRID, start_rc, quadrant)

#Quadrant done or partner unreachable, fall back to closest station
if current_station is None:
    current_station, path, scores = find_closest_station(GRID, start_rc, remaining_stations)
```

```python
#group_project_controller.py, SEARCH when a station is dropped
last_cleared = current_station["id"]
```

## 3. Drive speed below what the e-puck can do (attempted improvement)
**File:** `group_project_controller.py` (`MAX_SPEED`, `MOVE_SPEED`)

- **Expected:** The bot drives straight sections close to its top speed.
- **Actual:** `MOVE_SPEED` was 5.0 rad/s, while the e-puck max is 6.28. `MAX_SPEED` was 10, above the motor limit, so the clip in `set_speed` never did anything.
- **Attempted:** `MAX_SPEED = 6.28` to match the motor, `MOVE_SPEED = 6.0`. `TURN_SPEED` left at 4.0 to avoid overshooting turns.
- **Result:** 3:43, down from 3:50 (about 3%). Kept. Straight-line driving is not where most of the time goes.

## 4. 360° search spin can match the wrong station
**File:** `group_project_controller.py` (`search_step`, SEARCH state), `project_utils.py` (`station_coordinates`)

- **Expected:** A match during SEARCH means the target is at the station being checked.
- **Actual:** The robot spun a full 360° at each station, so the camera also saw neighbouring stations and the B1–B5 distractors. A match mid-spin could come from a different station, and the robot would stop at the wrong one.
- **Solved:** The robot turns to the station's `observe_yaw` from the config, then holds still for 5 s (`SEARCH_HOLD_S`). The camera is only checked during the hold. After an obstacle escape mid-search, it turns back to the station and restarts the hold.
- **Result:** Works in Webots. Full tour down to 3:34 from 3:43.

## 5. Controller crashes on start after the SIFT vision refactor
**File:** `group_project_controller.py` (imports, SEARCH state), `vision_helper.py`

- **Expected:** Controller starts and uses the new vision code.
- **Actual:** `ImportError: cannot import name 'detect_target'`. The vision refactor (commit `f38cee7`) replaced `detect_target` with `build_references`, `score_frame_all`, `target_vs_others` and `StationJudge`, but the controller still called the old function. The follow-up commit `a0dd5e8` also left a bare `from pathlib`, a syntax error.
- **Solved:** The pathlib import was fixed to `from pathlib import Path`. The controller loads all references once and scores each frame during the hold. It confirms FOUND once `StationJudge` has enough frames and the target clearly beats the other images, and only drops a station after the full 5 s hold. Station evidence is printed at each decision.
- **Result:** To be tested.
