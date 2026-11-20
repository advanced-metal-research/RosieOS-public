"""Authored weld boundaries; no hardware access or inferred output assignment."""
import math


def problems(nodes):
    errors, active, speed, legs = [], None, None, 0
    for node in nodes:
        if not node.get("enabled", True):
            continue
        label = node.get("label") or node.get("id")
        transition = node.get("weld_transition", "none")
        for key in ("weld_start_delay_s", "weld_stop_delay_s"):
            value = node.get(key, 0)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 30:
                errors.append(f"{label}: {key} must be 0–30 seconds")
        if transition not in ("none", "start", "stop"):
            errors.append(f"{label}: unknown weld action")
        if active is not None:
            if node.get("op") != "move" or node.get("motion") not in ("linear", "circular"):
                errors.append(f"{label}: a weld span permits only MoveL/MoveC; stop the weld before I/O, waits or joint moves")
            else:
                value = node.get("speed_mm_s")
                if type(value) not in (int,float) or not math.isfinite(value) or value<=0 or node.get("constant_tcp_speed",True) is not True:
                    errors.append(f"{label}: welding requires positive TCP speed and constant-speed path timing")
                elif speed is None:
                    speed = value
                elif abs(speed-value)>1e-6:
                    errors.append(f"{label}: keep one TCP speed within this weld span or stop and start a new weld")
                legs += 1
        if transition=="start":
            if node.get("op")!="move" or active is not None:
                errors.append(f"{label}: weld start must be an arrival waypoint outside an existing weld")
            active, speed, legs = label, None, 0
        elif transition=="stop":
            if active is None or legs==0:
                errors.append(f"{label}: weld stop needs a start and at least one weld move")
            active=None
    if active is not None:
        errors.append(f"{active}: weld start has no matching stop")
    return errors


def assigned_output(doc):
    """Resolve the operator's channel without changing native hardware policy.

    A general output (e.g. a test lamp) needs no torch qualification. An actual
    torch still does. An explicit empty selection never falls back to O5.
    """
    from .io_schedule import validate_outputs
    table = validate_outputs(doc)
    if "weld_output" in doc:
        name = doc["weld_output"]
    else:
        torches = [name for name, spec in table.items() if spec["class"] == "torch"]
        name = torches[0] if len(torches) == 1 else None
    if not isinstance(name, str) or name not in table:
        raise ValueError("Weld output is unassigned or unavailable. Select an output in Cell I/O > Weld setup, save and replan; Trace switches nothing")
    if table[name]["class"] == "torch" and doc.get("torch_qualified") is not True:
        raise ValueError(f"Weld output {name} is an unqualified torch. Complete its hardware qualification before welding; Trace switches nothing")
    return name


def spans(nodes):
    """Start-index -> following Cartesian destinations through Weld stop."""
    errors = problems(nodes)
    if errors:
        raise ValueError("; ".join(errors))
    result, begin = {}, None
    for i, node in enumerate(nodes):
        if node.get("weld_transition") == "start": begin = i + 1
        elif node.get("weld_transition") == "stop":
            result[begin] = nodes[begin:i+1]
            begin = None
    return result
