"""
Safety layer, if a proximity sensor is too close the controller pauses
the state machine and backs off then turns away from the obstacle.

e-puck sensor layout (clockwise from front-right):
  ps0 front-right   ps1 right        ps2 right-rear
  ps3 rear-right    ps4 rear-left    ps5 left-rear
  ps6 left          ps7 front-left
"""

FRONT_IDX = (0, 1, 6, 7)
REAR_IDX = (2, 3, 4, 5)
RIGHT_IDX = (0, 1, 2)
LEFT_IDX = (7, 6, 5)

#Background noise sits between 59 and 75
PS_TRIGGER = 85.0

#Seconds spent moving clear, turning, and max turn time
RECOVERY_MOVE_S = 0.5
RECOVERY_TURN_S = 0.4
RECOVERY_TURN_MAX_S = 1.2


def is_unsafe(prox):
    return any(v > PS_TRIGGER for v in prox)


#Move away from the closer end and turn away from the closer side
def start_recovery(prox):
    front = max(prox[i] for i in FRONT_IDX)
    rear = max(prox[i] for i in REAR_IDX)
    right = max(prox[i] for i in RIGHT_IDX)
    left = max(prox[i] for i in LEFT_IDX)
    escape = "BACKUP" if front >= rear else "FORWARD"
    turn_left = right >= left
    return {"phase": escape, "timer": 0, "turn_left": turn_left}


#One tick of the escape, returns (done, left_speed, right_speed)
def recovery_step(rec_state, prox, timestep, move_speed, turn_speed):
    move_steps = max(1, int(RECOVERY_MOVE_S * 1000 / timestep))
    turn_steps = max(1, int(RECOVERY_TURN_S * 1000 / timestep))
    turn_steps_max = max(turn_steps, int(RECOVERY_TURN_MAX_S * 1000 / timestep))

    if rec_state["phase"] in ("BACKUP", "FORWARD"):
        speed = -move_speed if rec_state["phase"] == "BACKUP" else move_speed
        rec_state["timer"] += 1
        if rec_state["timer"] >= move_steps or not is_unsafe(prox):
            rec_state["phase"] = "TURN"
            rec_state["timer"] = 0
        return False, speed, speed

    if rec_state["phase"] == "TURN":
        if rec_state["turn_left"]:
            left, right = -turn_speed, turn_speed
        else:
            left, right = turn_speed, -turn_speed
        rec_state["timer"] += 1
        #Finish once clear, or after max turn time so it can't spin forever
        if rec_state["timer"] >= turn_steps and not is_unsafe(prox):
            return True, 0.0, 0.0
        if rec_state["timer"] >= turn_steps_max:
            return True, 0.0, 0.0
        return False, left, right

    return True, 0.0, 0.0


#Quick test to see if the functions actually work in a vacuum
if __name__ == "__main__":
    clear = [65.0] * 8
    front_hit = [200.0, 65, 65, 65, 65, 65, 65, 65]
    rear_hit = [65, 65, 65, 200.0, 65, 65, 65, 65]

    assert not is_unsafe(clear), "background noise should be safe"
    assert is_unsafe(front_hit), "front obstacle should be unsafe"

    rec = start_recovery(front_hit)
    assert rec["phase"] == "BACKUP" and rec["turn_left"], "front-right hit should back up then turn left"
    rec = start_recovery(rear_hit)
    assert rec["phase"] == "FORWARD", "rear hit should drive forward"

    #Sensors stay blocked the whole time, escape must still finish
    rec = start_recovery(front_hit)
    done, ticks = False, 0
    while not done:
        done, left, right = recovery_step(rec, front_hit, 32, 5.0, 4.0)
        ticks += 1
        assert ticks < 1000, "recovery never finished"
    print("recovery finished in", ticks, "ticks")

    print("safety helper dummy test passed")
