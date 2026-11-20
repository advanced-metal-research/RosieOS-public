import { CLIENT_ID_HEADER, CONTROL_REFUSAL } from "./controlNotice.ts";

/**
 * Names this tab to the OLP server and hears its control refusals, for every
 * API client at once: the app's clients bind `fetch` when they are built,
 * so this wraps it before any is (main.ts, first statement).
 *
 * The id is random, one per tab, kept in sessionStorage so a reload is the
 * same controller (control.go: a reload must not make the operator "another
 * controller" to their own armed robot). Only same-origin /api/ requests carry
 * it. A 409 `in_use_by_other_controller` is passed to `onRefusal` with its
 * body; the caller still gets the response unchanged.
 */
export const CLIENT_ID_STORAGE_KEY = "olp.control.client-id";

export function tabClientId(storage: Pick<Storage, "getItem" | "setItem"> | null, random: () => string = randomId): string {
  try {
    const stored = storage?.getItem(CLIENT_ID_STORAGE_KEY) ?? "";
    if (/^[A-Za-z0-9._:-]{8,64}$/.test(stored)) return stored;
  } catch {
    // Storage blocked: a fresh id per load.
  }
  const id = `tab-${random()}`;
  try {
    storage?.setItem(CLIENT_ID_STORAGE_KEY, id);
  } catch {
    // Per-load id only.
  }
  return id;
}

function randomId(): string {
  const bytes = new Uint8Array(8);
  crypto.getRandomValues(bytes);
  return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");
}

export interface ControlFetchOptions {
  clientId: string;
  /** The page's origin: requests elsewhere are left alone. */
  origin: string;
  onRefusal: (body: unknown) => void;
}

/** `fetch` that names the tab and reports control refusals. */
export function controlFetch(inner: typeof fetch, options: ControlFetchOptions): typeof fetch {
  return async (input: RequestInfo | URL, init?: RequestInit): Promise<Response> => {
    const raw = typeof input === "string" ? input : input instanceof URL ? input.href : input.url;
    let url: URL;
    try {
      url = new URL(raw, options.origin);
    } catch {
      return inner(input, init);
    }
    if (url.origin !== new URL(options.origin).origin || !url.pathname.startsWith("/api/")) return inner(input, init);
    const headers = new Headers(init?.headers ?? (typeof input === "object" && "headers" in input ? input.headers : undefined));
    headers.set(CLIENT_ID_HEADER, options.clientId);
    const response = await inner(input, { ...init, headers });
    if (response.status === 409) {
      void response.clone().json().then((body: unknown) => {
        if (body && typeof body === "object" && (body as Record<string, unknown>).error === CONTROL_REFUSAL) options.onRefusal(body);
      }, () => undefined);
    }
    return response;
  };
}

/** Wrap the page's `fetch` once. */
export function installControlFetch(scope: { fetch: typeof fetch; location: { origin: string } }, clientId: string, onRefusal: (body: unknown) => void): void {
  scope.fetch = controlFetch(scope.fetch.bind(scope), { clientId, origin: scope.location.origin, onRefusal });
}
