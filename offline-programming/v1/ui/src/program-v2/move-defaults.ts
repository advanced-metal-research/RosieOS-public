/**
 * Speeds a row gets when nobody chose one.
 *
 * A recorded waypoint stores its speed in the row (`speed_mm_s`), so changing
 * these never changes a saved program: rows recorded at 25 mm/s keep 25 mm/s
 * until someone edits them. Joint velocity/acceleration limits and the
 * planner's exact rate judge still bound every move; a line is flown at the
 * row's speed or slower, never faster.
 */

// The owner's MoveL range is 100-250 mm/s (2026-10-11). At the old 25 mm/s,
// a Rosie 1430 arm's `test1` (four 0.42-0.70 m lines and a MoveJ) planned a 98.5 s
// trajectory; at 250 mm/s every line is under 3 s on that arm's limits
// (weld_planner tests/motion_planner/test_taught_plan_cost.py). The measured
// TCP stayed within 1.2 mm of a line flown at 400 mm/s on the same arm (2026-10-10).
export const RECORDED_MOVE_SPEED_MM_S = 250;

// A Home in a program with no weld is planned as a MoveJ to the reset pose at
// this TCP bound (weld_planner taught.HOME_AS_WAYPOINT, which a planner test
// holds equal to this number). With welds, Home is a searched free-space move
// with no TCP speed of its own.
export const HOME_WAYPOINT_SPEED_MM_S = 250;
