import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import test from "node:test";
import {
  parseRobotV4Catalog,
  parseRobotV4Program,
  reviseRobotV4Program,
} from "../contracts/robot-v4-program.ts";
import {
  PROGRAMS_PATH,
  ProgramCatalogClient,
  ProgramCatalogConflictError,
} from "./catalog-client.ts";

const fixtures = resolve(
  import.meta.dirname,
  "../../../../../steamdeck/real/v4/contracts/programs/fixtures",
);

test("program catalog client uses fixed same-origin list/get/history routes", async () => {
  const catalog = parseRobotV4Catalog(JSON.parse(
    await readFile(resolve(fixtures, "complete-catalog.json"), "utf8"),
  ));
  const calls: string[] = [];
  const client = new ProgramCatalogClient({
    fetch: async (input, init) => {
      const path = String(input);
      calls.push(`${init?.method ?? "GET"} ${path}`);
      if (path.endsWith("/revisions")) {
        return jsonResponse({
          schema: "robot.v4.program-revisions.v1",
          owner: "motion-server",
          program_id: catalog.programs[0]!.program_id,
          catalog_revision: catalog.catalog_revision,
          revisions: catalog.programs,
        });
      }
      if (path === PROGRAMS_PATH) return jsonResponse(catalog);
      return jsonResponse({
        schema: "robot.v4.program-entry.v1",
        owner: "motion-server",
        catalog_revision: catalog.catalog_revision,
        program: catalog.programs[0],
      });
    },
  });

  assert.equal((await client.list()).catalog_revision, 11);
  const id = catalog.programs[0]!.program_id;
  assert.equal((await client.get(id, 2)).program.program_id, id);
  assert.equal((await client.revisions(id)).revisions.length, 1);
  assert.deepEqual(calls, [
    `GET ${PROGRAMS_PATH}`,
    `GET ${PROGRAMS_PATH}/${id}?revision=2`,
    `GET ${PROGRAMS_PATH}/${id}/revisions`,
  ]);
});

test("program catalog save carries exact base/catalog identities and stable idempotency", async () => {
  const source = parseRobotV4Program(JSON.parse(
    await readFile(resolve(fixtures, "complete-program.json"), "utf8"),
  ));
  const program = await reviseRobotV4Program(source, "offline_programming", (draft) => {
    draft.program_name = "Offline edit";
  });
  let request: { path: string; init?: RequestInit } | undefined;
  const client = new ProgramCatalogClient({
    fetch: async (input, init) => {
      request = { path: String(input), init };
      const body = JSON.parse(String(init?.body)) as {
        catalog_revision: number;
        program: typeof program;
      };
      return jsonResponse({
        ok: true,
        catalog: {
          schema: "robot.v4.program_catalog.v1",
          owner: "motion-server",
          catalog_revision: 12,
          programs: [body.program],
        },
      }, 202);
    },
  });
  const saved = await client.save(program, 11);
  assert.equal(saved.catalog_revision, 12);
  assert.equal(request?.path, `${PROGRAMS_PATH}/${program.program_id}`);
  assert.equal(request?.init?.method, "PUT");
  const headers = new Headers(request?.init?.headers);
  assert.match(headers.get("Idempotency-Key") ?? "", new RegExp(
    `^save:${program.program_id}:${program.program_revision}:sha256:`,
  ));
  const body = JSON.parse(String(request?.init?.body)) as Record<string, unknown>;
  assert.equal(body.catalog_revision, 11);
  assert.deepEqual(body.program, program);
  assert.equal(program.base_program_revision, source.program_revision);
});

test("program catalog exposes typed current revisions on optimistic conflict", async () => {
  const client = new ProgramCatalogClient({
    fetch: async () => jsonResponse({
      ok: false,
      code: "catalog_conflict",
      error: "program was edited on SteamDeck",
      catalog_revision: 14,
      program_id: "shared-program",
      current_program_revision: 6,
    }, 409),
  });
  await assert.rejects(
    () => client.list(),
    (error: unknown) => {
      assert.ok(error instanceof ProgramCatalogConflictError);
      assert.equal(error.catalogRevision, 14);
      assert.equal(error.programId, "shared-program");
      assert.equal(error.currentProgramRevision, 6);
      return true;
    },
  );
});

function jsonResponse(value: unknown, status = 200): Response {
  return new Response(JSON.stringify(value), {
    status,
    headers: { "Content-Type": "application/json" },
  });
}
