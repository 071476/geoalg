# geoalg in Unity: IK demo

A robot arm in Unity that follows a target, moved by geoalg's inverse kinematics.
It is the same engine that runs in the browser (WASM), here called from C# through
the C bridge (`geoalg-ffi`).

Tested on **Unity 6.3 LTS (Windows)**.

## What it does

- Builds geoalg's demo arm (3 joints, links of length 1).
- Creates a pink sphere: the **target**.
- Every frame, geoalg's IK computes the joint angles to reach the target, and the
  arm is drawn with spheres and cylinders from geoalg's forward kinematics.
- The tip turns **green** when it reaches the target and **red** when it is out of reach.

## Files

| File | What it is |
|---|---|
| `GeoalgRobot.cs` | C# wrapper for the geoalg robot API (create, FK, IK) and axis conversions |
| `GeoalgIKDemo.cs` | The demo: a `MonoBehaviour` you add to an empty GameObject |

## Steps

### 1. Build the native library

From the root of this repository:

```
cargo build -p geoalg-ffi --release
```

This creates `target/release/geoalg_ffi.dll` (Windows).

### 2. Copy the files into your Unity project

| From | To |
|---|---|
| `target/release/geoalg_ffi.dll` | `Assets/Plugins/` |
| `examples/unity/GeoalgRobot.cs` | `Assets/Scripts/` |
| `examples/unity/GeoalgIKDemo.cs` | `Assets/Scripts/` |

Close Unity before copying the DLL (see the notes below).

### 3. Set up the scene

1. Open the project in Unity and check the **Console**: there must be no red errors.
2. **GameObject > Create Empty**, rename it to `Geoalg`, and set its position to `0, 0, 0`.
3. With `Geoalg` selected: **Add Component > Geoalg IK Demo**. Add it **only once**.

### 4. Play

1. Press **Play**. The Console shows `geoalg listo: 3 articulaciones` once.
2. Go to the **Scene** tab and select **Geoalg Target** in the Hierarchy.
3. Move it with the **Move tool (W)**, or change its **Position** in the Inspector.
4. The arm follows the target. Try `4, 1, 0` to see the tip turn red (out of reach).

## Notes

- **Axes.** geoalg always uses the right-hand rule; Unity uses Y up and the left-hand
  rule. The demo arm is described with Z up (like ROS), so the demo converts points
  with `GeoalgRobot.FromZUp` / `ToZUp`. For robots described with Y up (like the
  studio examples), use `FromYUp` / `ToYUp`.
- **The DLL stays locked.** Once Unity loads `geoalg_ffi.dll`, Windows keeps it locked
  until Unity closes. Close Unity before replacing it with a new build.
- **One component only.** If `geoalg listo` appears twice in the Console, the script was
  added twice: there are two arms on top of each other, and only one follows the target.

## Troubleshooting

| Problem | Fix |
|---|---|
| `DllNotFoundException: geoalg_ffi` | Check that the DLL is in `Assets/Plugins/`, then restart Unity |
| The arm does not move | Make sure Play is on and Pause is off, and that you move **Geoalg Target** |
| Two pink spheres | The script was added twice: remove one component |
| You cannot see the arm | In the Scene view, select `Geoalg` and press **F** |
