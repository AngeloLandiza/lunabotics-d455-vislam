#!/usr/bin/env bash
# Export a saved map as a textured mesh (OBJ + MTL + JPG texture) and a point cloud (PLY).
# Works on any map from this repo, physical or sim: colour scans from
# run/physical/reconstruct.sh come out in colour, the other modes in greyscale IR.
#
#   run/export_mesh.sh                               ~/.ros/d455_reconstruct.db
#   run/export_mesh.sh ~/.ros/lunabotics_sim.db      any other map
#   run/export_mesh.sh ~/.ros/x.db --max_range 3     extra args go to rtabmap-export
#                                                    (rtabmap-export --help lists them)
#
# Output: ~/.ros/reconstructions/<map name>/. Open it in MeshLab, Blender or CloudCompare.
set -eo pipefail
db="${1:-$HOME/.ros/d455_reconstruct.db}"
shift || true
[ -f "$db" ] || { echo "[export_mesh] no map at $db" >&2; exit 1; }
name="$(basename "$db" .db)"
out="$HOME/.ros/reconstructions/$name"
mkdir -p "$out"

source /opt/ros/humble/setup.bash
echo "[export_mesh] exporting $db ..."
if ! rtabmap-export --cloud --mesh --texture --output_dir "$out" --output "$name" "$@" "$db" \
     | grep -v "Failed to find match for field"; then   # harmless PCL noise
  rmdir "$out" 2>/dev/null || true
  echo "[export_mesh] export failed: is the map empty (camera not streaming, scan too short)?" >&2
  exit 1
fi
echo "[export_mesh] saved to $out:"
ls -1 "$out"
