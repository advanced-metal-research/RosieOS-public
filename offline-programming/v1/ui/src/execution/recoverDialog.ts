import type { RecoverConfirmation } from "./recoverToZero.ts";

/** The browser's question before All joints to 0° or Run program arms or
 *  moves the robot: the action by name, every joint's angle → target and
 *  travel, and the sweep warning (`recoverToZero.ts` and `runProgram.ts`
 *  write the words; the Deck shows the same ones).
 *  Cancel has the focus, so Enter or Escape never arms; only a click on
 *  the confirm button resolves true. */
export function confirmRecoverDialog(confirmation: RecoverConfirmation, host: HTMLElement = document.body): Promise<boolean> {
  const dialog = document.createElement("dialog");
  dialog.className = "new-program-dialog recover-dialog";
  dialog.setAttribute("aria-labelledby", "recover-dialog-title");
  const form = document.createElement("form");
  form.method = "dialog";
  form.className = "new-program-form";
  const heading = document.createElement("div");
  heading.className = "new-program-dialog-heading";
  const titles = document.createElement("div");
  const eyebrow = document.createElement("p");
  eyebrow.className = "eyebrow";
  eyebrow.textContent = confirmation.eyebrow ?? "ALL JOINTS TO 0°";
  const title = document.createElement("h2");
  title.id = "recover-dialog-title";
  title.textContent = confirmation.title;
  titles.append(eyebrow, title);
  heading.append(titles);
  const intro = document.createElement("p");
  intro.className = "new-program-dialog-copy";
  intro.textContent = confirmation.intro;
  const table = document.createElement("table");
  table.className = "recover-dialog-joints";
  table.innerHTML = "<thead><tr><th>Joint</th><th>Now</th><th>Target</th><th>Travel</th></tr></thead>";
  const body = document.createElement("tbody");
  for (const joint of confirmation.joints) {
    const row = document.createElement("tr");
    row.classList.toggle("is-still", !joint.moves);
    for (const text of [joint.id, joint.from, joint.to, joint.delta]) {
      const cell = document.createElement("td");
      cell.textContent = text;
      row.append(cell);
    }
    body.append(row);
  }
  table.append(body);
  const warning = document.createElement("p");
  warning.className = "recover-dialog-warning";
  warning.setAttribute("role", "alert");
  warning.textContent = confirmation.warning;
  const actions = document.createElement("div");
  actions.className = "new-program-dialog-actions";
  const cancel = document.createElement("button");
  cancel.type = "button";
  cancel.className = "button secondary";
  cancel.textContent = confirmation.cancel;
  const confirm = document.createElement("button");
  confirm.type = "button";
  confirm.className = "button primary recover-dialog-confirm";
  confirm.textContent = confirmation.confirm;
  actions.append(cancel, confirm);
  form.append(heading, intro, table, warning, actions);
  dialog.append(form);
  host.append(dialog);
  return new Promise(resolve => {
    let answered = false;
    const finish = (yes: boolean) => {
      if (answered) return;
      answered = true;
      if (dialog.open) dialog.close();
      dialog.remove();
      resolve(yes);
    };
    cancel.addEventListener("click", () => finish(false));
    confirm.addEventListener("click", () => finish(true));
    dialog.addEventListener("cancel", event => { event.preventDefault(); finish(false); });
    dialog.addEventListener("close", () => finish(false));
    form.addEventListener("submit", event => { event.preventDefault(); finish(false); });
    dialog.showModal();
    cancel.focus();
  });
}
