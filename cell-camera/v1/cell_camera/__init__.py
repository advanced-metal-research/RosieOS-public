"""Cell camera: an on-demand, newest-frame JPEG view of one camera on a cell host.

The OLP server relays this service to its clients (offline-programming/v1
cell_camera.go), so a Steam Deck pendant or a browser can look at the cell.
It has no robot or cabinet interface. Started from the earlier
amr-camera on the first camera host (2026-10-07); the HTTP contract is unchanged.
"""
