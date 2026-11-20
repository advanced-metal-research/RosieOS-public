#!/usr/bin/env python
"""What every planning MODULE is, so the planner can hold them all the same way.

    PlannerModule          one step of planning: one options in, one result out
    PlannerModuleOptions   what it is ASKED for. Settings only
    PlannerModuleResult    what it PRODUCED, and what that cost
    PlannerContext         the plumbing it needs and does not choose

A module and the planner are deliberately different things. A module owns one
step and knows nothing about the others -- not what ran before it, not whether
anything runs after. The PLANNER (`planner_main.py`) owns the order, holds one
`PlannerModuleOptions` and one `PlannerModuleResult` per module on
`MotionPlanningOptions` and `MotionPlanningResult`, and is the only thing that
knows there are three. Those two do NOT inherit from these: they hold modules,
they are not one.

`name` is also the key the module's span appears under in `timings_ms`, so the
planner never spells a timing key itself.

Nothing here imports anything of ours except the request it is handed: every
module's dataclasses import this file, so it has to be importable from all of
them and cheap.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar


@dataclass
class PlannerModuleOptions:
    """A module's settings. One object per module, and it takes exactly one."""


@dataclass
class PlannerModuleResult:
    """A module's output, with the cost of producing it.

    `timings_ms` is keyword-only so a subclass can declare required fields after
    it -- dataclass inheritance puts base fields first, and a defaulted one would
    otherwise forbid every field below.
    """

    timings_ms: dict[str, float] = field(default_factory=dict, kw_only=True)

    def as_json(self) -> dict[str, Any]:
        """Overridden per module. The base carries only what every module has."""
        return {"timings_ms": dict(self.timings_ms)}


@dataclass
class PlannerContext:
    """What a module needs and does not choose: which models to ask, which card.

    Separate from its options because these are not decisions about the plan. The
    planner builds them once for the whole request and hands the same ones to
    every module, which is what stops two modules screening against different
    worlds.
    """

    # The work cell, assembled once. `None` only in a module test that has no
    # cell to model.
    cell: Any = None
    planner_collision_model: Any = None
    verifier_collision_model: Any = None
    device: Any = None
    # `None` means the module's own default. Not typed against `collision.model`
    # here: that module imports a result, which imports this file.
    mesh_root: Path | None = None
    log: bool = True
    # Called with a short stage name when one starts -- "weldseam",
    # "optimize" -- so an operator watching a minutes-long plan sees where it
    # is. `None` when nobody is watching; never let it gate work.
    progress: Any = None


class PlannerModule(ABC):
    """One step of planning. Subclass per module; the planner holds a list.

    `upstream` is whatever the module before it produced -- `None` for the first.
    A module may not reach for anything else: if it needs a value, it is either
    in `upstream`, in `options`, or in `context`, and if it is in none of them
    the module is being asked to know the order it sits in.
    """

    #: The timing key, and the module's name wherever a reader sees one.
    name: ClassVar[str]
    #: The pair this module speaks in.
    Options: ClassVar[type[PlannerModuleOptions]]
    Result: ClassVar[type[PlannerModuleResult]]

    @abstractmethod
    def plan(self, request: Any, upstream: Any,
             options: PlannerModuleOptions,
             context: PlannerContext) -> PlannerModuleResult:
        """Run this module. One options in, one result out."""
