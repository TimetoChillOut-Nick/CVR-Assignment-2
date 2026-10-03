"""Run logging, prints a mission summary at the end of the run."""

#Mission limits
TIME_LIMIT_S = 240.0
POSITION_TOLERANCE_M = 0.20


def new_run_log():
    return {"travel": 0.0, "search": 0.0, "stations": [], "escapes": 0, "final_error": None, "printed": False}


#Add one tick of time to travelling or searching
def add_time(run_log, state, seconds):
    if state in ("NAVIGATION", "RETURN"):
        run_log["travel"] += seconds
    elif state == "SEARCH":
        run_log["search"] += seconds


#Record a finished station search and its best scores
def add_station(run_log, station_id, best_target, best_other, other_name, found):
    run_log["stations"].append((station_id, best_target, best_other, other_name, found))


def format_time(seconds):
    return f"{int(seconds // 60)}:{seconds % 60:05.2f}"


#Prints once, final_error is None if the target was never found
def print_summary(run_log, total_time):
    if run_log["printed"]:
        return
    run_log["printed"] = True

    error = run_log["final_error"]
    if error is None:
        reason = "target not found"
    elif error > POSITION_TOLERANCE_M:
        reason = "stopped outside 0.20 m"
    elif total_time > TIME_LIMIT_S:
        reason = "over the 4:00 limit"
    else:
        reason = None

    print("=" * 40)
    print("RUN SUMMARY")
    print(f"  Result:         {'SUCCESS' if reason is None else 'FAIL (' + reason + ')'}")
    print(f"  Total time:     {format_time(total_time)} (limit 4:00)")
    print(f"  Time travelled: {format_time(run_log['travel'])}")
    print(f"  Time searched:  {format_time(run_log['search'])}")
    print(f"  Safety escapes: {run_log['escapes']}")
    if error is not None:
        print(f"  Final distance: {error:.3f} m (limit {POSITION_TOLERANCE_M} m)")
    print(f"  Stations visited ({len(run_log['stations'])}):")
    for station_id, best_target, best_other, other_name, found in run_log["stations"]:
        result = "FOUND  " if found else "dropped"
        print(f"    {station_id}  {result}  best target {best_target}, best other {best_other} ({other_name})")
    print("=" * 40)


#Quick test to see if the summary prints correctly in a vacuum
if __name__ == "__main__":
    log = new_run_log()
    for _ in range(100):
        add_time(log, "NAVIGATION", 0.032)
    for _ in range(50):
        add_time(log, "SEARCH", 0.032)
    add_station(log, "S8", 3, 7, "backpack", False)
    add_station(log, "S1", 14, 4, "camera", True)
    log["final_error"] = 0.023
    print_summary(log, 154.3)

    #Second call should print nothing
    print_summary(log, 200.0)
    assert abs(log["travel"] - 3.2) < 1e-9 and abs(log["search"] - 1.6) < 1e-9
    print("run log dummy test passed")
