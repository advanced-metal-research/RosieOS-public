# Retired machine regression fixtures

`bench-ninemotor-a6ec.json` and its derivation note are frozen historical
inputs retained for compiler, coordinate-identity, gearing and migration tests.
They were removed from `config/machines` on 2026-09-22 and are not deployment
configurations. Their original annotations and bytes are preserved as evidence;
they do not prescribe the current robot or bench settings.

Deployable robot and bench configurations obtain robot settings from the
referenced robot definition. Do not copy this retired fixture back into the
machine catalogue or ship it in a runtime package.

Tests exercising current robot-bound compilation explicitly remove archived cell
travel and evidence-mode overrides in memory before compilation. Raw fixture
bytes remain available for provenance checks; the retired inputs themselves are
no longer valid deployable machine configurations.
