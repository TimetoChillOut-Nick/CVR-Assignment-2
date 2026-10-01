# Bugs and Fixes

## 1. Escape manoeuvre aborts as soon as sensors clear
**File:** `group_project_controller.py` (safety wrapper in `main()` from first major groupe merge)

- **Expected:** When an obstacle triggers recovery, the robot backs up (or drives forward) then turns away before resuming the mission.
- **Actual:** Recovery only ran while a sensor read above `PS_TRIGGER`. Once the robot backed up enough for the reading to drop, it skipped the turn and went straight back to the missionhitting the same obstacle.
- **Solved:** Changed the wrapper condition to `if recovery_state is not None or is_unsafe(prox):` so a started manoeuvre always runs to completion and resets `recovery_state` to `None`.
