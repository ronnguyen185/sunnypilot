# QML UI Integration for VinFast Lateral Offset

## Overview
The system uses the QML-based UI (`SettingsWindowSP`), not the raylib Python UI. To add the VinFast lateral offset control, you need to modify the QML source files.

## Parameter Details
- **Parameter Name**: `VinFastLateralOffset`
- **Type**: INT
- **Default Value**: `2` (Left 2)
- **Range**: 0-6
- **Values**:
  - `0` = Center (no offset)
  - `1` = Left 1 (-0.0001)
  - `2` = Left 2 (-0.00025) - default
  - `3` = Left 3 (-0.0005)
  - `4` = Right 1 (0.0001)
  - `5` = Right 2 (0.00025)
  - `6` = Right 3 (0.0005)

## Implementation Steps

### 1. Find the Steering Panel QML File
The QML UI has a Steering panel (mentioned in translations as `SettingsWindowSP`). Locate the QML file that implements the Steering settings panel.

### 2. Add the Control
In the Steering panel QML file, add a control similar to other parameter controls. Here's an example structure:

```qml
// In Steering panel QML file
Item {
    // ... existing controls ...

    // VinFast Lateral Offset Control
    // Only show if car brand is VinFast
    visible: CarParams.brand === "vinfast"

    Label {
        text: "Lateral Position"
        // ... styling ...
    }

    Text {
        text: "Adjust the lateral position of the vehicle within the lane. " +
              "Shift Left moves the vehicle left, Shift Right moves it right. " +
              "Only applies when vehicle speed is below 60 km/h."
        // ... styling ...
    }

    // Button group or ComboBox for selection
    Row {
        // 7 buttons: Center, Left 1, Left 2, Left 3, Right 1, Right 2, Right 3
        Button { text: "Center"; onClicked: params.put("VinFastLateralOffset", 0) }
        Button { text: "Left 1"; onClicked: params.put("VinFastLateralOffset", 1) }
        Button { text: "Left 2"; onClicked: params.put("VinFastLateralOffset", 2) }
        Button { text: "Left 3"; onClicked: params.put("VinFastLateralOffset", 3) }
        Button { text: "Right 1"; onClicked: params.put("VinFastLateralOffset", 4) }
        Button { text: "Right 2"; onClicked: params.put("VinFastLateralOffset", 5) }
        Button { text: "Right 3"; onClicked: params.put("VinFastLateralOffset", 6) }
    }
}
```

### 3. Read Current Value
Make sure to read the current parameter value to highlight the selected option:

```qml
property int currentOffset: parseInt(params.get("VinFastLateralOffset", "2"))
```

### 4. Recompile the UI
After modifying the QML files, rebuild the UI binary:
```bash
# Rebuild the UI (exact command depends on your build system)
scons -j$(nproc) selfdrive/ui/ui
```

## Alternative: Add to Toggles Panel
If you prefer to add it to the Toggles panel instead of Steering panel, follow the same pattern but add it to the Toggles panel QML file.

## Notes
- The parameter is already defined in `common/params_keys.h`
- The control logic is already implemented in `sunnypilot/selfdrive/controls/controlsd_ext.py`
- The parameter is read every 0.1 seconds, so changes take effect immediately
- The offset only applies when vehicle speed is below 60 km/h (enforced in `controlsd.py`)

