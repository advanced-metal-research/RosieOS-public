/** Commit the inspector name on both normal change and Enter, like the title editor. */
export function bindProgramNameInput(input: HTMLInputElement, commit: (name: string) => void): void {
  input.addEventListener("change", () => commit(input.value));
  input.addEventListener("keydown", event => {
    if (event.key !== "Enter" || event.isComposing) return;
    event.preventDefault();
    commit(input.value);
  });
}
