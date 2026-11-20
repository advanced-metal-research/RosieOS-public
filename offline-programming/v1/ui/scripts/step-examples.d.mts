export interface StepExampleSource {
  id: string;
  /** Repository-relative path the asset stage copies from. */
  source: string;
  label: string;
  description: string;
}
export const STEP_EXAMPLES: readonly StepExampleSource[];
