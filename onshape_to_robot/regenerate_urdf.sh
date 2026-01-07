#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARM_DESCRIPTION_PACKAGE_FILEPATH="$SCRIPT_DIR/../src/arm_description"

URDF_FILEPATH="$ARM_DESCRIPTION_PACKAGE_FILEPATH/urdf/arm.urdf"
ASSETS_FILEPATH="$ARM_DESCRIPTION_PACKAGE_FILEPATH/assets"

rm "$URDF_FILEPATH"
rm -rf "$ASSETS_FILEPATH"

onshape-to-robot "$SCRIPT_DIR/onshape_to_robot_config.json"

mv "$SCRIPT_DIR/arm.urdf" "$URDF_FILEPATH"
mv "$SCRIPT_DIR/assets" "$ASSETS_FILEPATH"

sed -i 's|package://|package://arm_description/|g' "$URDF_FILEPATH"
