// Where rosie finds its own build products.
import path from "node:path";
import { fileURLToPath } from "node:url";

/** cli/v1, from dist/src/cli. */
export function cliRoot(): string {
  return path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../../..");
}
