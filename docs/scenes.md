# Scenes

A scene is the 3D model of the space your robot works in. Every session is recorded against one
version of one scene, and Session Replay draws the robot's path, its events and its camera and
lidar records inside that model. You do not need Unity to make a scene: any glTF model works, and
the SDK ships a tool that exports a Gazebo world.

## Scene id and scene version

The node records into the scene that two parameters name:

- `scene_id`: the scene's UUID, which Cognitive3D assigns when you first upload the scene.
  Configure refuses a value that is not a UUID, so a placeholder left in the file stops the node.
- `scene_version`: the version number. The first upload is version 1, and every update of the
  scene creates the next version.

```yaml
/**:
  ros__parameters:
    scene_id: "<your-scene-uuid>"
    scene_version: "<your-scene-version>"
```

`scene_version` is a string of digits, so quote it: `scene_version: "3"`. An unquoted `3` is a YAML
integer and the node does not start, and a value that is not a whole number of 1 or more refuses
configure. [Configuration](configuration.md) has the parameter reference.

Use the version number, not the version id. The Upload Web App's result lists `sceneId`,
`versionNumber` and `versionId`; `scene_version` takes `versionNumber`, and `versionId` is a
different number. With `C3D_ORG_API_KEY` set, `doctor` says when it is given the version id; see
[Confirm the scene belongs to your project](#confirm-the-scene-belongs-to-your-project).

A session stays with the scene version it was recorded against. When you upload a new version of
the scene, set `scene_version` to it; sessions recorded before stay on the earlier version.

## Get a scene

Use whichever of these fits:

- **Your project already has the scene**, uploaded earlier or from another Cognitive3D SDK. Take its
  id and version number from the Cognitive3D dashboard.
- **You have a 3D model of the site.** Export it as glTF with a separate `.bin` buffer (not `.glb`),
  then [prepare the files](#prepare-the-files) and [upload them](#upload-the-scene).
- **Your robot runs in Gazebo.** [Export the world](#export-a-gazebo-world), check the result, and
  upload it.

## Export a Gazebo world

`export_world_gltf.sh` turns an SDF world into glTF with Gazebo's own exporter and assimp. It needs
Gazebo Garden or later: it runs `gz sim`, reads `GZ_SIM_RESOURCE_PATH`, and loads the exporter by
its Gazebo Sim plugin name. It is tested with Gazebo Harmonic, the release paired with Jazzy.
Gazebo Classic and Gazebo Fortress are not supported; see
[On Gazebo Classic or Fortress](#on-gazebo-classic-or-fortress).

Run it where Gazebo already loads your world, such as the machine or container you run the
simulation in, with `gz` on the `PATH` and assimp installed:

```bash
sudo apt install assimp-utils
ros2 run cognitive3d_ros export_world_gltf.sh my_world /tmp/scene
```

The first argument is a world name, found as `<name>.sdf` under `GZ_SIM_RESOURCE_PATH` and
`AMENT_PREFIX_PATH`, or the path to an `.sdf` file. The tool writes `scene.gltf` and `scene.bin`
into the output directory, with every texture the world uses copied beside them, ready to upload.
It:

1. Runs a copy of the world headless for 20 iterations, with Gazebo's COLLADA world exporter added.
2. Repairs the COLLADA up axis. Gazebo's exporter declares no up axis, COLLADA then defaults to
   Y-up, and Gazebo is Z-up, so an unrepaired export lies on its side with no error from any
   tool. `collada_zup_repair.py` adds the declaration, and copies the file unchanged when one is
   already there.
3. Converts the result to glTF with assimp, as `scene.gltf` and its buffer `scene.bin`.
4. Copies each texture beside `scene.gltf` and points the glTF at it by its bare file name, which
   is all a scene upload keeps. It stops with an error when two textures share a file name, since
   one would replace the other on the platform.
5. Checks the result with `gltf_bbox.py`, and reports its orientation.

It stops with an error when Gazebo could not resolve some of the world's geometry, and it prints
every warning Gazebo logged. Read them: Gazebo's wording varies between versions, so a model that
failed to load can be missing from the export without the error firing.

The orientation verdict comes last:

| verdict | meaning |
| -- | -- |
| `consistent with UPRIGHT` | Y, glTF's up axis, is the shortest extent, as it is for a building or a floor plan |
| `SOMETHING TO LOOK AT` | Y is not the shortest extent. Either the up-axis repair did not take, or the world is taller than it is wide. Open the file and look |
| `INCONCLUSIVE` | The extents are too close to each other for the check to say anything. Check by eye after upload |

The tool exits with status 0 when the export is fit to upload. Status 1 is a real problem that
`gltf_bbox.py`'s report names, such as a texture the world references that exists nowhere: the
scene would upload and render without it.

A third argument of `glb` writes one self-contained `<name>.glb` file instead. The platform does
not accept a `.glb` scene, so keep the default for anything you upload.

To repair a COLLADA file you exported from Gazebo another way, run the repair on its own:

```bash
ros2 run cognitive3d_ros collada_zup_repair.py world.dae world_zup.dae
```

### On Gazebo Classic or Fortress

A Humble robot often simulates in Gazebo Classic (`gazebo`) or Gazebo Fortress (`ign gazebo`),
and the tool runs with neither. Run the export on a Jazzy machine or container instead, which
needs no ROS graph and none of the robot's packages:

1. Install Gazebo Harmonic and assimp, and then the Jazzy packages as [Install](install.md)
   describes:

   ```bash
   sudo apt install ros-jazzy-ros-gz assimp-utils
   ```

2. Copy the world file there, with the directories of every model it includes, and put those
   directories on `GZ_SIM_RESOURCE_PATH`. Harmonic resolves `model://` URIs through it, where
   Classic uses `GAZEBO_MODEL_PATH`.
3. Run `export_world_gltf.sh` with the path to the world file.

Harmonic may not load a Classic world exactly as written. The tool stops with an error when some
of the world's geometry does not resolve, and prints every warning Gazebo logged: read them, and
look at the result before you upload it. When the world will not load, export a 3D model of the
site another way, and [prepare the files](#prepare-the-files) yourself.

## Prepare the files

A scene upload is a `scene.gltf` and its `scene.bin`, plus any textures it references. The upload
stores the two files under exactly those names and does not edit the glTF, so the glTF's
`buffers[].uri` must already read `scene.bin`. A glTF that still references `my_world.bin` uploads
without an error and the scene is empty in Session Replay.

An export from `export_world_gltf.sh` already follows these rules; go straight to the check below.
For a glTF from another exporter, rename the files and point the glTF at the new buffer name:

```bash
cd /tmp/scene
mv my_world.bin scene.bin
mv my_world.gltf scene.gltf
python3 - <<'EOF'
import json
with open("scene.gltf") as f:
    spec = json.load(f)
for buffer in spec["buffers"]:
    buffer["uri"] = "scene.bin"
with open("scene.gltf", "w") as f:
    json.dump(spec, f)
EOF
```

This assumes one buffer. Two more rules apply to every scene:

- **Every primitive needs a material.** A primitive with vertex colors and no material renders
  solid pink, with no error anywhere. A flat `baseColorFactor` is enough.
- **Texture references are bare file names**, uploaded beside the `.gltf`. The upload keeps file
  names only, so a reference such as `textures/wall.png` resolves on your disk and is missing on
  the platform, and the model renders untextured.

Check the result before you upload it:

```bash
ros2 run cognitive3d_ros gltf_bbox.py scene.gltf
```

It exits with status 0 when the files are fit to upload. It reports `INCOMPLETE` for a referenced
file that is missing or shorter than the glTF declares, and `WILL UPLOAD EMPTY` for a reference the
viewer cannot fetch. The upload accepts both kinds of file without an error. It does not check
materials. Given no file, it prints its usage and exits with status 2.

## Upload the scene

Upload the scene with the Upload Web App at <https://upload.cognitive3d.com>, which also comes as a
desktop app. Sign in with your developer key, choose **New Scene**, or **Update Scene** to add a
version to an existing scene, and upload `scene.gltf`, `scene.bin` and the textures. The result
lists the `sceneId` and `versionNumber` to put in your parameter file. The dashboard documentation's
[Scene Uploads](https://docs.cognitive3d.com/dashboard/upload-webapp-scenes/) page describes the app
and its file requirements.

## Confirm the scene belongs to your project

> [!WARNING]
> The platform accepts a session whose scene id belongs to another project. It creates an empty
> scene with that id in your application key's project and records the session there, so Session
> Replay draws the robot in an empty world. Nothing at run time reports it. Check the scene's
> project before your first session.

The Upload Web App has no project picker. The developer key you enter decides the project, and
the app reuses the last developer key entered in that browser unless you enter another.

`doctor` checks this for you. With `C3D_ORG_API_KEY` set, its `scene/project` check reads the scene
from the platform, and fails when it has no version numbered `scene_version`, saying so when the
value you gave is the version id. With `C3D_PROJECT_ID` set as well, `doctor` checks that your
application key belongs to that project, and `scene/project` also fails when the scene belongs to
another project, or when no scene with this id is visible to your organization: the id does not
exist, or belongs to another organization. Without `C3D_PROJECT_ID`, it names the scene's project
and does not compare the project. When the platform does not answer, the check is a note; without
an organization key it does not run. See [doctor and verification](doctor-and-verification.md).

To check by hand, open the project your application key belongs to in the Cognitive3D dashboard and
find the scene in it. Or ask the API, with your organization key. This asks the API host your
environment names (production unless `C3D_API_HOST` is set), picks the header format from the key
without printing it, and prints the scene's project and version numbers:

```bash
(
  case "$C3D_ORG_API_KEY" in
    orgkey-*) auth="$C3D_ORG_API_KEY" ;;
    *) auth="APIKEY:ORGANIZATION $C3D_ORG_API_KEY" ;;
  esac
  curl -fsS -H "Authorization: $auth" \
    "https://${C3D_API_HOST:-api.cognitive3d.com}/v0/scenes/<your-scene-uuid>"
) | python3 -c 'import json, sys; s = json.load(sys.stdin); print("projectId:", s["projectId"]); print("versionNumber:", [v["versionNumber"] for v in s["versions"]])'
```

`projectId` must be your project's id, and `versionNumber` must include your `scene_version`. The
API answers 403 when no scene with this id is visible to your organization. An organization key
that begins with `orgkey-` goes in the header as it is; an older key without that prefix goes as
`APIKEY:ORGANIZATION <key>`.

## Choose a pose convention

`pose_convention` says how the robot's ROS poses map onto the scene's axes. It has no default, and
configure refuses a node without it. For a scene exported with `export_world_gltf.sh`, or by any
ROS-to-glTF exporter that maps `gltf.xyz = (ros.x, ros.z, -ros.y)`, use `gltf_authored`. For a
scene authored in Unity, use `unity`. The two differ by a 90 degree turn, and the platform accepts a
session recorded under the wrong one and draws its path off the floor plan.
[Coordinates](coordinates.md) explains how to choose and how to check.

## The robot as a Dynamic Object

The robot's Dynamic Object is off by default (`robot_dynamic_object: false`). Session Replay draws
the robot from the pose timeline, so a default session registers no object and needs no robot
mesh. Turn it on when you want Session Replay to draw your robot's own model, or when a tool reads
the Dynamic Object stream.

With `robot_dynamic_object: true`, a robot mesh must be uploaded to the scene version the session
records against, and four more parameters apply:

| parameter | default | what |
| -- | -- | -- |
| `robot_mesh` | `"robot_placeholder"` | The uploaded mesh's name |
| `robot_object_id` | `"robot"` | The object id the session records the robot under |
| `robot_display_name` | `"Robot"` | The object's name in the dashboard |
| `mesh_yaw_offset_deg` | `0.0` | A turn about the vertical axis, applied to the mesh |

[Configuration](configuration.md) has the full reference, and [Coordinates](coordinates.md) covers
how the robot's rotation is drawn under each pose convention.

### Prepare the robot mesh

A robot mesh is a glTF named after the mesh, its buffer, and optionally a thumbnail:

- `<mesh>.gltf`, where `<mesh>` is the value of `robot_mesh`. Session Replay fetches the model as
  `objects/<mesh>/<mesh>.gltf`, so a glTF with any other name renders nothing.
- Its buffer, `<mesh>.bin`, named in the glTF's `buffers[].uri` by that bare file name. Unlike a
  scene, an object keeps the file names you upload. The Upload Web App requires the two names to
  match; a scripted upload accepts any bare file name for the buffer.
- Optionally `cvr_object_thumbnail.png`, which the dashboard shows in object lists.

The platform does not accept a `.glb` mesh. Check the files with the object rules:

```bash
ros2 run cognitive3d_ros gltf_bbox.py --object robot_placeholder.gltf
```

Choose a model whose front and side look different. On a rotationally symmetric model, such as a
disc or a cylinder, a robot drawn 90 degrees wrong looks right.

`mesh_yaw_offset_deg` corrects a mesh whose forward axis is not the robot frame's: `90` turns the
mesh's +X onto the direction of travel, and `-90` the other way. It is yaw only. When the robot's
URDF places its visual mesh with a roll or pitch as well, bake that rotation into the mesh before
you upload it. The mesh is already in glTF axes, so express the rotation in glTF axes too: a
rotation `R` written in ROS axes becomes `M R M⁻¹`, where `M` is the ROS-to-glTF axis map your
exporter used. Applied directly, it turns the mesh about the wrong axis and still looks plausible.

### Upload the robot mesh

Objects belong to a scene version, and a new scene version starts with none. Upload the robot mesh
again after every new version of the scene.

The Upload Web App can upload the robot mesh; the dashboard documentation's
[Object Uploads](https://docs.cognitive3d.com/dashboard/upload-webapp-objects/) page describes how.
The app assigns the object its own id. Set `robot_object_id` to that id: `doctor`'s
`object id/registered` check lists the ids your project has. With a different id, Session Replay
still draws the robot, but the uploaded object's initial position, rotation, scale and thumbnail do
not apply, and analytics that key on the object id report nothing for it.

To script the upload instead, make two requests with your developer key, to the data host your
environment names (production unless `C3D_DATA_HOST` is set).

> [!WARNING]
> A Dynamic Object upload is two requests. The first creates the object; the second stores its
> mesh files, and its path ends in the mesh name, not the object id. Both answer with an empty 2xx
> response whether or not anything was stored. With only the first done, sessions record normally
> and Session Replay draws no robot.

The first request creates the object, with the id and mesh name your parameters use:

```bash
curl -X POST \
  -H "Authorization: APIKEY:DEVELOPER $DEVELOPER_KEY" \
  -H "Content-Type: application/json" \
  -d '{"objects":[{"id":"robot","mesh":"robot_placeholder","name":"Robot","scaleCustom":[1,1,1],"initialPosition":[0,0,0],"initialRotation":[0,0,0,1]}]}' \
  "https://${C3D_DATA_HOST:-data.cognitive3d.com}/v0/objects/<your-scene-uuid>?version=<your-scene-version>"
```

The second stores the files, one `file` part each, under the mesh name:

```bash
curl -X POST \
  -H "Authorization: APIKEY:DEVELOPER $DEVELOPER_KEY" \
  -F "file=@robot_placeholder.gltf" \
  -F "file=@robot_placeholder.bin" \
  "https://${C3D_DATA_HOST:-data.cognitive3d.com}/v0/objects/<your-scene-uuid>/robot_placeholder?version=<your-scene-version>"
```

Neither response says whether the object is complete. A `.glb`, or a file that is not a mesh at
all, gets the same answer as a good upload. Check with `doctor`, below.

### Mesh files are permanent per mesh name

Once files are stored under a mesh name, they are never replaced. Uploading under the same name
again answers 2xx and keeps serving the original files, and deleting and recreating the object does
not release the name. While you are still changing the model, upload each change under a new mesh
name (`robot_v2`, `robot_v3`, and so on) and set `robot_mesh` to match.

A recorded session keeps asking for the mesh name it was recorded with. To see a corrected mesh,
upload it under a new name and record a new session.

### Check before you drive

With `C3D_ORG_API_KEY` and `C3D_PROJECT_ID` set, `doctor` checks the Dynamic Object against the
platform: `mesh/exists` looks for the mesh in your project, `object id/registered` looks for the
object id, and `mesh/files` reads the object back from your scene version with its files. Run it
after every upload. With `robot_dynamic_object` off, the three are reported together as
`robot dynamic object (NOT CHECKED)`. [doctor and verification](doctor-and-verification.md)
describes each check.
