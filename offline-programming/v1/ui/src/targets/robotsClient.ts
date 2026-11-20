/**
 * The robots this server holds descriptions for.
 *
 * One GET, answered from `robot_description/<model>/` — the only place that
 * directory is read (`docs/ROBOT-DESCRIPTION-CONTRACT.md`). The browser asks
 * once at boot and keeps the answer in `domain/robots.ts`; it does not resolve
 * a path, guess an id, or carry a list of its own.
 */

import { parseRobotCatalogue, type RobotDescriptorSummary } from "../domain/robots.ts";

export const ROBOTS_PATH = "/api/offline-programming/v1/robots";

export type FetchLike = (input: string, init?: RequestInit) => Promise<Response>;

/** What the server has. Throws with the server's own words when it cannot
 *  say -- a browser that invented a robot here would author against a
 *  description nobody has. */
export async function fetchRobotCatalogue(
  fetchImpl: FetchLike = fetch,
  path: string = ROBOTS_PATH,
): Promise<RobotDescriptorSummary[]> {
  const response = await fetchImpl(path);
  if (!response.ok) {
    let detail = `${response.status}`;
    try {
      const failure = (await response.json()) as { error?: string; code?: string };
      detail = failure.error ?? failure.code ?? detail;
    } catch {
      /* the status alone, then */
    }
    throw new Error(detail);
  }
  return parseRobotCatalogue(await response.json());
}
