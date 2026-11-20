/** Preloaded into every test process by `scripts/test.mjs`: the catalogue the
 *  app fetches at boot, so a test that is about placement or rows is not
 *  first a test about an empty catalogue. */
import { seedRobotCatalogue } from "./robot-catalogue.ts";

seedRobotCatalogue();
