// The example parts the UI offers under Import STEP, staged from where they
// already live in the repository so no STEP is committed twice. Plain JS so
// both the asset stage and a test can import it.
export const STEP_EXAMPLES = [
  {
    id: "dog_box",
    source: "cadquery/v1/step/dog_box.step",
    label: "Dog box",
    description: "A plate lapped onto a block: the bundled example, closed fillet round the plate.",
  },
  {
    id: "pipe_bracket",
    source: "weld_planner/v1/data/fixtures/pipe_bracket.step",
    label: "Pipe bracket",
    description: "A column on a base plate with two gussets: all tee joints, with stretches no torch can reach.",
  },
  {
    id: "bracket_a",
    source: "weld_planner/v1/data/fixtures/bracket_a.step",
    label: "Bracket A",
    description: "Sheet bracket, single.",
  },
  {
    id: "bracket_b",
    source: "weld_planner/v1/data/fixtures/bracket_b.step",
    label: "Bracket B",
    description: "Sheet bracket, single.",
  },
  {
    id: "hitch",
    source: "weld_planner/v1/data/fixtures/hitch.step",
    label: "Hitch",
    description: "Tube-and-plate hitch.",
  },
  {
    id: "bracket_a_2x",
    source: "weld_planner/v1/data/fixtures/bracket_a_2x.step",
    label: "Bracket A ×2",
    description: "Bracket A at twice the scale: the weld planner's motion corpus part.",
  },
  {
    id: "bracket_b_2x",
    source: "weld_planner/v1/data/fixtures/bracket_b_2x.step",
    label: "Bracket B ×2",
    description: "Bracket B at twice the scale: the weld planner's motion corpus part.",
  },
  {
    id: "hitch_2x",
    source: "weld_planner/v1/data/fixtures/hitch_2x.step",
    label: "Hitch ×2",
    description: "The hitch at twice the scale: the weld planner's motion corpus part.",
  },
];
