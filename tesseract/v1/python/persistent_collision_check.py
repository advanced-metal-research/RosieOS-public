#!/usr/bin/env python
"""Warm, bounded cache wrapper for the canonical Robot V4 collision worker.

The underlying collision implementation remains collision_check.py. This
entry point only retains exact per-waypoint results inside the persistent,
loopback-only Offline Programming process so unchanged samples are not sent
through Bullet again after every edit or repeated Plan action.
"""

from __future__ import annotations

from collections import OrderedDict
import copy
import sys
import time

import collision_check as canonical

MAX_CACHED_WAYPOINTS = 50_000


class CachedCollisionWorld(canonical.CollisionWorld):
    def __init__(self, request: dict):
        super().__init__(request)
        self.contact_cache: OrderedDict[
            tuple[float, ...], list[dict]
        ] = OrderedDict()

    @staticmethod
    def cache_key(waypoint: dict, contact_distance_m: float) -> tuple[float, ...]:
        return (
            contact_distance_m,
            *(float(value) for value in waypoint["positions_rad"]),
        )

    def check(self, request: dict) -> dict:
        started_at = time.time()
        contact_distance_m = float(request.get("contact_distance_m", 0.0))
        waypoints = request.get("waypoints", [])
        missing: list[dict] = []
        cache_hits = 0
        for waypoint in waypoints:
            key = self.cache_key(waypoint, contact_distance_m)
            if key in self.contact_cache:
                cache_hits += 1
                self.contact_cache.move_to_end(key)
            else:
                missing.append(waypoint)

        if missing:
            miss_request = dict(request)
            miss_request["waypoints"] = missing
            miss_result = super().check(miss_request)
            contacts_by_waypoint: dict[int, list[dict]] = {}
            for contact in miss_result["contacts"]:
                contacts_by_waypoint.setdefault(
                    int(contact["waypoint_index"]), []).append(contact)
            for waypoint in missing:
                key = self.cache_key(waypoint, contact_distance_m)
                cached_contacts: list[dict] = []
                for contact in contacts_by_waypoint.get(
                        int(waypoint.get("waypoint_index", -1)), []):
                    retained = copy.deepcopy(contact)
                    retained.pop("waypoint_index", None)
                    retained.pop("time_s", None)
                    cached_contacts.append(retained)
                self.contact_cache[key] = cached_contacts
                if len(self.contact_cache) > MAX_CACHED_WAYPOINTS:
                    self.contact_cache.popitem(last=False)

        contacts: list[dict] = []
        for waypoint in waypoints:
            key = self.cache_key(waypoint, contact_distance_m)
            for retained in self.contact_cache.get(key, []):
                contact = copy.deepcopy(retained)
                contact["waypoint_index"] = int(
                    waypoint.get("waypoint_index", -1))
                contact["time_s"] = float(waypoint.get("time_s", 0.0))
                contacts.append(contact)

        return {
            "schema": canonical.RESULT_SCHEMA,
            "ok": True,
            "checker": self.manager.getName(),
            "tesseract_nanobind_version": self.library_version,
            "collision_geometry_source": "urdf_visual_and_collision_meshes",
            "checked_waypoint_count": len(waypoints),
            "contact_count": len(contacts),
            "cache_hit_waypoint_count": cache_hits,
            "acm_adjacent_pairs": [
                f"{link_a}|{link_b}" for link_a, link_b in self.acm_pairs
            ],
            "contacts": contacts,
            "check_duration_s": time.time() - started_at,
        }


canonical.CollisionWorld = CachedCollisionWorld


if __name__ == "__main__":
    sys.exit(canonical.main())
