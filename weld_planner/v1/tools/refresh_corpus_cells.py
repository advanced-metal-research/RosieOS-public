"""Re-bundle the corpus .weldplan requests against the current robot description.

A plan request freezes the robot it was authored against -- the planning
context (docs/PLANNING-CONTEXT.md) and the cell descriptor parsed from it --
and the corpus was frozen before `26e61b0` gave the arm its real joint bounds:
the bundles carried J4/J5 at the ±6.3 rad round guesses, so a plan made from
them emitted joints today's cell refuses (docs/OLP-EXPORT.md records how that
was found). This rewrites each bundle with a context compiled fresh by the OLP
from the description, preserving the program, tooling, STEP bytes, request id
and timestamp -- the diff is the robot and nothing else.

Run it whenever the description changes:

    pixi run python tools/refresh_corpus_cells.py
"""

from __future__ import annotations

import sys
from pathlib import Path

V1 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(V1 / "python"))

from olp_context import compiled_context
from weldplan import plan_request
from weldplan.urdf import descriptor_of

MODEL = "rosie_1400_v3"
CORPUS = sorted((V1 / "data" / "motion").glob("*.weldplan"))


def main() -> int:
    context = compiled_context(MODEL)
    fresh = descriptor_of(context)
    changed = 0
    for path in CORPUS:
        opened = plan_request.read_request(path.read_bytes())
        manifest = opened["manifest"]
        # The program pins the robot it was packed for, and the packer
        # refuses to overrule a real pin -- which is what this tool is for.
        # Reset it to the placeholder a fresh producer writes; the packer
        # then resolves it to the description packed beside it. A program
        # from before the robot pin (a `cell` asset ref) is carried over.
        program = {key: value for key, value in opened["program"].items() if key != "cell"}
        program["robot"] = {"model_id": MODEL, "robot_description_sha256": "sha256:" + "0" * 64}
        data = plan_request.write_request(
            request_id=str(manifest["request_id"]),
            program=program,
            cell=fresh,
            step_bytes=opened["step_bytes"],
            step_filename=Path(manifest["source"]["path"]).name,
            tooling=opened["tooling"],
            label=manifest.get("label"),
            created_utc=str(manifest["created_utc"]),
            context=context,
        )
        if data == path.read_bytes():
            print(f"{path.name}: already current")
            continue
        path.write_bytes(data)
        changed += 1
        print(f"{path.name}: re-bundled against {MODEL} {fresh['robot_cell']['robot_description_sha256'][:19]}...")
    print(f"{changed} of {len(CORPUS)} bundles rewritten")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
