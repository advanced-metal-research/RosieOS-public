import type { DenseCellInput, DenseCellProbe, DenseCellSpec } from "../execution/denseClient.ts";
import type { DiscoveredRow } from "../targets/discoveredCells.ts";
import { Dialog } from "../ui/dialog.ts";
import { element } from "../ui/widget.ts";

/**
 * The Cells dialog presents the Picker's commissioned catalogue, selected
 * cell and authenticated settings. Every action delegates to that authority;
 * the form cannot commission a listener or replace its identity pins.
 */
export interface RobotDialogCallbacks {
  onClose: () => void;
  /** A cell was chosen ("" for none). */
  onSelect: (id: string) => void;
  onRefresh: () => void;
  onRemove: (id: string) => void;
  onAdd: (input: DenseCellInput) => Promise<boolean>;
  onPull: () => void;
  /** SEARCH in Search local network; `force` asks the server for a new search. */
  onNetworkSearch?: (force: boolean) => void;
  /** The Search local network section opened or closed: live refresh runs while open. */
  onNetworkSectionOpen?: (open: boolean) => void;
}

export interface RobotDialogDiscoveryState {
  rows: readonly DiscoveredRow[];
  status: string;
  searching: boolean;
}

export interface RobotDialogCellsState {
  cells: readonly DenseCellSpec[];
  probes: readonly DenseCellProbe[] | null;
  activeId: string;
  /** Native status is fresh for the verified selected cell. */
  daemonReachable: boolean;
  selecting: boolean;
  status: string;
}

export interface RobotDialogSettingsState {
  /** The program's cell settings, described; null when none. */
  held: string | null;
  heldCell: string | null;
  /** What the connected cell reports now, when it differs from held. */
  warning: string | null;
  canPull: boolean;
  pullReason: string;
}

// Unverified form policy; the server verifies every value against commissioning.
export const ADD_CELL_FIELDS: ReadonlyArray<{ key: keyof DenseCellInput; label: string; placeholder: string; required?: boolean }> = [
  { key: "id", label: "Cell id", placeholder: "example-arm-1420", required: true },
  { key: "host", label: "Host", placeholder: "127.0.0.1", required: true },
  { key: "address", label: "Remote listener", placeholder: "https://127.0.0.1:8443", required: true },
  { key: "credentialRef", label: "PKI material reference", placeholder: "Commissioned server catalogue reference", required: true },
];

export class RobotDialog extends Dialog {
  readonly #callbacks: RobotDialogCallbacks;
  readonly #cellsSection: HTMLDivElement;
  readonly #list: HTMLDivElement;
  readonly #status: HTMLParagraphElement;
  readonly #refresh: HTMLButtonElement;
  readonly #disconnect: HTMLButtonElement;
  readonly #settingsSection: HTMLDivElement;
  readonly #held: HTMLParagraphElement;
  readonly #warning: HTMLParagraphElement;
  readonly #pull: HTMLButtonElement;
  readonly #inputs = new Map<keyof DenseCellInput, HTMLInputElement>();
  readonly #addError: HTMLParagraphElement;
  readonly #add: HTMLButtonElement;
  readonly #addDetails: HTMLDetailsElement | null;
  readonly #prefillNote: HTMLParagraphElement;
  readonly #networkSection: HTMLDivElement;
  readonly #networkDetails: HTMLDetailsElement | null;
  readonly #networkList: HTMLDivElement;
  readonly #networkStatus: HTMLParagraphElement;
  readonly #networkSearch: HTMLButtonElement;

  constructor(callbacks: RobotDialogCallbacks) {
    super({ title: "Cells", className: "robot-dialog", onClose: callbacks.onClose });
    this.#callbacks = callbacks;

    this.#cellsSection = this.addSection({ title: "Cells" });
    this.#list = element("div", "robot-cell-list");
    this.#list.setAttribute("role", "radiogroup");
    this.#list.setAttribute("aria-label", "Cells");
    this.#status = element("p", "robot-dialog-status", "");
    const tools = element("div", "robot-dialog-tools");
    this.#refresh = element("button", "mini-button", "REFRESH");
    this.#refresh.type = "button";
    this.#refresh.title = "Describe every cell and verify its native identity";
    this.listen(this.#refresh, "click", () => callbacks.onRefresh());
    this.#disconnect = element("button", "mini-button", "DISCONNECT");
    this.#disconnect.type = "button";
    this.#disconnect.title = "Leave the cell: the server stops any run under its lease and releases it";
    this.listen(this.#disconnect, "click", () => callbacks.onSelect(""));
    tools.append(this.#refresh, this.#disconnect);
    this.#cellsSection.append(this.#list, this.#status, tools);

    this.#settingsSection = this.addSection({ title: "Cell settings" });
    this.#held = element("p", "robot-dialog-held", "");
    this.#warning = element("p", "inspector-note is-warn", "");
    this.#warning.hidden = true;
    this.#pull = element("button", "mini-button robot-dialog-pull", "PULL CELL SETTINGS");
    this.#pull.type = "button";
    this.listen(this.#pull, "click", () => callbacks.onPull());
    this.#settingsSection.append(this.#held, this.#warning, this.#pull);

    // Cells other OLP servers advertise on this server's network. A row only
    // fills the Add cell form, from this server's commissioning; the
    // operator confirms with ADD TO REGISTRY, and nothing connects or arms.
    this.#networkSection = this.addSection({ title: "Search local network", collapsible: true, open: false });
    this.#networkDetails = detailsOf(this.#networkSection);
    this.#networkList = element("div", "robot-discovery-list");
    this.#networkList.setAttribute("aria-label", "Cells found on the network");
    this.#networkStatus = element("p", "robot-dialog-status", "");
    this.#networkSearch = element("button", "mini-button robot-discovery-search", "SEARCH");
    this.#networkSearch.type = "button";
    this.#networkSearch.title = "Ask the network for OLP servers now";
    this.listen(this.#networkSearch, "click", () => callbacks.onNetworkSearch?.(true));
    if (this.#networkDetails) {
      const details = this.#networkDetails;
      this.listen(details, "toggle", () => callbacks.onNetworkSectionOpen?.(details.open));
    }
    const networkTools = element("div", "robot-dialog-tools");
    networkTools.append(this.#networkSearch);
    this.#networkSection.append(
      element("p", "robot-add-help", "OLP servers that advertise on this server's network, with the cells they present. What they say is unverified: Add only fills the form from this server's own commissioning, and Connect still verifies the cell over mutual TLS."),
      this.#networkList, this.#networkStatus, networkTools,
    );

    const addSection = this.addSection({ title: "Add cell", collapsible: true, open: false });
    this.#addDetails = detailsOf(addSection);
    const form = element("div", "robot-add-form");
    for (const field of ADD_CELL_FIELDS) {
      const label = element("label", "robot-add-field");
      label.append(element("span", "robot-add-label", field.label));
      const input = document.createElement("input");
      input.type = "text";
      input.placeholder = field.placeholder;
      input.autocomplete = "off";
      input.spellcheck = false;
      input.dataset.cellField = field.key;
      if (field.required) input.required = true;
      this.listen(input, "input", () => this.#syncAdd());
      label.append(input);
      form.append(label);
      this.#inputs.set(field.key, input);
    }
    this.#addError = element("p", "inspector-note is-warn robot-add-error", "");
    this.#addError.hidden = true;
    this.#add = element("button", "mini-button robot-add-submit", "ADD TO REGISTRY");
    this.#add.type = "button";
    this.#add.disabled = true;
    this.listen(this.#add, "click", () => void this.#submitAdd());
    this.#prefillNote = element("p", "inspector-note is-warn robot-add-prefill", "");
    this.#prefillNote.hidden = true;
    addSection.append(
      element("p", "robot-add-help", "Registers an existing commissioned cell binding. Endpoints, PKI references, model and configuration pins cannot be replaced here."),
      this.#prefillNote, form, this.#addError, this.#add,
    );
  }

  /** Open Search local network (from the Cells menu); its toggle event starts live refresh. */
  openNetworkSection(): void {
    if (this.#networkDetails) this.#networkDetails.open = true;
  }

  /** The discovered cells, with this server's view of each. */
  renderDiscovery(state: RobotDialogDiscoveryState): void {
    this.#networkList.replaceChildren();
    for (const row of state.rows) this.#networkList.append(this.#discoveryRow(row));
    if (state.rows.length === 0) this.#networkList.append(element("p", "inspector-empty", state.searching ? "Searching…" : "No cells found on the network."));
    this.#networkStatus.textContent = state.status;
    this.#networkSearch.disabled = state.searching;
    const flagged = state.rows.filter((row) => row.compatibility === "incompatible").length;
    this.setSectionSummary(this.#networkSection, state.searching && state.rows.length === 0 ? "searching…"
      : `${state.rows.length} found${flagged ? ` · ${flagged} incompatible` : ""}`);
  }

  /**
   * Fill the Add cell form from a discovered row and open it. Nothing is
   * submitted: the operator checks the values and presses ADD TO REGISTRY.
   */
  prefillFromDiscovery(row: DiscoveredRow): boolean {
    if (!row.addable || !row.prefill) return false;
    for (const field of ADD_CELL_FIELDS) {
      const input = this.#inputs.get(field.key);
      if (input) input.value = row.prefill[field.key];
    }
    if (this.#addDetails) this.#addDetails.open = true;
    const warnings = row.badges.filter((badge) => badge.kind === "bad").map((badge) => badge.text.toLowerCase());
    this.#prefillNote.textContent = `Filled from ${row.where}'s advertisement of ${row.cellId}, with this server's commissioned listener for it. `
      + `Check the values, then press ADD TO REGISTRY to confirm. The server accepts only a commissioned binding; Connect verifies the cell over mutual TLS.`
      + (warnings.length ? ` Flagged: ${warnings.join(", ")}.` : "")
      + (row.addReason ? ` ${row.addReason}.` : "");
    this.#prefillNote.hidden = false;
    this.setAddError(null);
    this.#syncAdd();
    this.#add.focus();
    return true;
  }

  #discoveryRow(row: DiscoveredRow): HTMLElement {
    const host = element("div", "robot-discovery-row");
    host.dataset.discovered = row.key;
    host.dataset.compatibility = row.compatibility;
    const text = element("div", "robot-cell-text");
    text.append(element("span", "robot-cell-title", row.title), element("span", "robot-cell-subtitle", row.subtitle));
    const badges = element("div", "robot-discovery-badges");
    for (const badge of row.badges) {
      const chip = element("span", `robot-discovery-badge is-${badge.kind}`, badge.text);
      chip.title = badge.title;
      badges.append(chip);
    }
    text.append(badges);
    const add = element("button", "mini-button robot-discovery-add", "ADD…");
    add.type = "button";
    add.disabled = !row.addable;
    add.title = row.addable ? "Fill the Add cell form with this cell's commissioned binding; you confirm there" : row.addReason;
    add.setAttribute("aria-label", `Add ${row.cellId}`);
    this.listen(add, "click", () => { this.prefillFromDiscovery(row); });
    host.append(text, add);
    return host;
  }

  /** The registry as a list, the selected cell marked. */
  renderCells(state: RobotDialogCellsState): void {
    this.#list.replaceChildren();
    const probeOf = new Map((state.probes ?? []).map((probe) => [probe.id, probe]));
    const none = this.#cellRow({
      id: "",
      title: "None (offline)",
      subtitle: "Browser preview only; nothing is driven",
      selected: state.activeId === "",
      reach: "none",
      source: "",
      removable: false,
      disabled: state.selecting,
    });
    this.#list.append(none);
    for (const cell of state.cells) {
      const probe = probeOf.get(cell.id) ?? null;
      const selected = cell.id === state.activeId;
      const reach = selected
        ? (state.daemonReachable ? "up" : "down")
        : probe === null ? "unknown" : probe.reachable ? "up" : "down";
      const subtitleParts = [cell.address || cell.host, cell.modelId, probe?.backend ?? cell.backend];
      this.#list.append(this.#cellRow({
        id: cell.id,
        title: cell.label || cell.id,
        subtitle: subtitleParts.filter(Boolean).join(" · "),
        selected,
        reach,
        source: cell.source,
        removable: false,
        disabled: state.selecting,
        error: probe?.error ?? null,
      }));
    }
    if (state.cells.length === 0) {
      this.#list.append(element("p", "inspector-empty", "No commissioned cells configured. Configure the server cell catalogue first."));
    }
    this.#status.textContent = state.status;
    this.#disconnect.disabled = state.activeId === "" || state.selecting;
    this.#refresh.disabled = state.selecting;
    this.setSectionSummary(this.#cellsSection, state.activeId
      ? `${state.activeId}${state.daemonReachable ? "" : " · unreachable"}`
      : `${state.cells.length} registered · none selected`);
  }

  renderSettings(state: RobotDialogSettingsState): void {
    this.#held.textContent = state.held
      ? `From ${state.heldCell}: ${state.held}`
      : "The program holds no cell settings yet.";
    this.#warning.textContent = state.warning ?? "";
    this.#warning.hidden = state.warning === null;
    this.#pull.disabled = !state.canPull;
    this.#pull.title = state.canPull
      ? "Copy the connected cell's robot model and axis map into this program's Workspace"
      : state.pullReason;
    this.setSectionSummary(this.#settingsSection, state.heldCell ? `from ${state.heldCell}` : "none");
  }

  setAddError(text: string | null): void {
    this.#addError.textContent = text ?? "";
    this.#addError.hidden = text === null;
  }

  /** What the Add cell form holds. */
  addInput(): DenseCellInput {
    const value = (key: keyof DenseCellInput): string => this.#inputs.get(key)?.value.trim() ?? "";
    return {
      id: value("id"),
      address: value("address"),
      host: value("host"),
      credentialRef: value("credentialRef"),
    };
  }

  clearAddForm(): void {
    for (const input of this.#inputs.values()) input.value = "";
    this.#prefillNote.hidden = true;
    this.setAddError(null);
    this.#syncAdd();
  }

  #syncAdd(): void {
    const input = this.addInput();
    this.#add.disabled = !(input.id && input.address && input.host && input.credentialRef);
  }

  async #submitAdd(): Promise<void> {
    const input = this.addInput();
    const problem = validateCellInput(input);
    if (problem) {
      this.setAddError(problem);
      return;
    }
    this.setAddError(null);
    this.#add.disabled = true;
    const added = await this.#callbacks.onAdd(input);
    if (added) this.clearAddForm();
    else this.#syncAdd();
  }

  #cellRow(row: {
    id: string;
    title: string;
    subtitle: string;
    selected: boolean;
    reach: "up" | "down" | "unknown" | "none";
    source: string;
    removable: boolean;
    disabled: boolean;
    error?: string | null;
  }): HTMLElement {
    const host = element("div", "robot-cell");
    host.dataset.cell = row.id;
    host.dataset.reach = row.reach;
    host.classList.toggle("is-selected", row.selected);
    const pick = element("button", "robot-cell-pick");
    pick.type = "button";
    pick.setAttribute("role", "radio");
    pick.setAttribute("aria-checked", String(row.selected));
    pick.disabled = row.disabled;
    const dot = element("span", "robot-cell-dot");
    dot.title = row.reach === "up" ? "native identity verified"
      : row.reach === "down" ? `native description refused${row.error ? `: ${row.error}` : ""}`
        : row.reach === "unknown" ? "not described yet" : "";
    const text = element("span", "robot-cell-text");
    text.append(element("span", "robot-cell-title", row.title), element("span", "robot-cell-subtitle", row.subtitle));
    pick.append(dot, text);
    if (row.source) pick.append(element("span", "robot-cell-source", row.source));
    this.listen(pick, "click", () => {
      if (!row.selected) this.#callbacks.onSelect(row.id);
    });
    host.append(pick);
    if (row.removable) {
      const remove = element("button", "robot-cell-remove", "×");
      remove.type = "button";
      remove.title = `Remove ${row.id} from the registry`;
      remove.setAttribute("aria-label", `Remove ${row.id}`);
      remove.disabled = row.disabled;
      this.listen(remove, "click", () => this.#callbacks.onRemove(row.id));
      host.append(remove);
    }
    return host;
  }
}

/** The <details> a collapsible dialog section lives in. */
function detailsOf(section: HTMLElement): HTMLDetailsElement | null {
  const parent = section.parentElement;
  return parent?.tagName === "DETAILS" ? parent as HTMLDetailsElement : null;
}

/** The checks the server makes, made here first so the form says what is
 *  wrong before a round trip. */
export function validateCellInput(input: DenseCellInput): string | null {
  if (!/^[A-Za-z0-9][A-Za-z0-9._-]{0,62}$/.test(input.id)) {
    return "Cell id: 1-63 letters, digits, '.', '_' or '-', starting with a letter or digit.";
  }
  try {
    const address = new URL(input.address);
    if (address.protocol !== "https:" || address.hostname !== input.host || !address.port || address.username || address.password || address.pathname !== "/" || address.search || address.hash) {
      return "Remote listener must be the commissioned HTTPS host and port.";
    }
  } catch { return "Remote listener must be a commissioned HTTPS address."; }
  if (!input.credentialRef) return "A commissioned PKI material reference is required.";
  return null;
}
