# Robot cat face for ROS 2

## Presenter integration (v0.2.0)

No publisher code changes or topic remaps are needed. Additional subscriptions:

| Topic | Type | Behavior |
| --- | --- | --- |
| `/tts/say` | `std_msgs/msg/String` | Queue the supplied text; auto-select Japanese for kana/kanji, English otherwise |
| `/presenter_abort` | `std_msgs/msg/Empty` | Cancel synthesis, clear queue, stop playback |
| `/arm_health_state` | `std_msgs/msg/String` | `FAULT` announces arm fault; `HEALTHY` rearms |
| `/presenter_state` | `std_msgs/msg/String` | `ARM_FAULT` also announces arm fault, deduplicated against the health topic |

The warning is **"The robot arm is faulty."** in English by default, even if
Japanese is selected manually. It interrupts current speech and replaces the
pending queue. Repeated FAULT/ARM_FAULT messages do not repeat the warning.
Only a HEALTHY health message rearms it; RECOVERING/VERIFYING/DIAGNOSTIC and
presenter IDLE do not. Presenter text is rejected while the fault is latched.
Manual controls and explicit `/robot_cat/say/*` remain usable for the operator.

An abort received while a fault is latched preserves the fault warning, so a
coordinator abort cannot erase it. Manual `/stop` always cancels everything,
including the warning. The warning is attempted once per fault episode; errors
appear on `/robot_cat/status` and in the terminal, with no automatic retry loop.
`arm:fault_warning_queued` means the warning was scheduled, not that audio completed.

Health subscriptions are best-effort/volatile to accept live updates from
reliable or best-effort publishers. Start this subscriber before the health
publisher, or ensure health is periodically republished: a state published only
before subscriber startup is not replayed. String states must be plain values
(case/whitespace ignored), not JSON objects. No missing-health timeout is added;
the existing coordinator remains responsible for movement/arm health policy.

### Run now, using only English

Stop the old face and controls. Replace the package directory in your workspace
with the extracted `robot_cat_ros` directory, then rebuild:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_test
colcon build --symlink-install --packages-select robot_cat_ros
source install/setup.bash
```

Use your actual workspace name and installed ROS distribution if different.
In each terminal, source ROS and that workspace's `install/setup.bash`.

Terminal 1:

```bash
ros2 run robot_cat_ros face
```

Terminal 2:

```bash
ros2 run robot_cat_ros controls --ros-args \
  -p english_binary:="$HOME/venvs/robot-piper/bin/piper" \
  -p presenter_text_override:="Hello everyone. Welcome to our robot demonstration."
```

The optional override substitutes that fixed English line for each nonempty
`/tts/say` message. It is NOT translation. This lets the unchanged Japanese
presenter drive your English-only test. Without the override, Japanese presenter
text requires a working Japanese engine and reports an error if unavailable.
When Japanese works, omit `presenter_text_override` to speak the original text.
No Japanese engine is loaded just by starting the node or announcing a fault.

### Additional startup parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `presenter_language` | `auto` | `auto`, `en`, `ja`, or `current` (manual selection) |
| `presenter_text_override` | empty | Optional fixed replacement for presenter text |
| `fault_language` | `en` | Engine used for fault warning (`en` or `ja`) |
| `fault_text` | `The robot arm is faulty.` | Warning text (1-1000 characters) |

Auto detection is a simple script check, not translation or full language
identification. Manual language commands affect `/robot_cat/say` and typed text;
they affect `/tts/say` only with `presenter_language:=current`.

### Test without moving the robot

In another sourced terminal:

```bash
ros2 topic pub --once /tts/say std_msgs/msg/String "{data: 'Hello from the presenter'}"
ros2 topic pub --once /presenter_abort std_msgs/msg/Empty '{}'
```

To simulate health without sending fake FAULT/HEALTHY to real robot controllers,
start terminal 2 with these extra remaps:

```bash
-r /arm_health_state:=/tts_test/arm_health_state \
-r /presenter_state:=/tts_test/presenter_state
```

Then publish on the isolated test topics:

```bash
ros2 topic pub --once /tts_test/arm_health_state std_msgs/msg/String "{data: 'FAULT'}"
ros2 topic pub --once /tts_test/presenter_state std_msgs/msg/String "{data: 'ARM_FAULT'}"
ros2 topic pub --once /tts_test/arm_health_state std_msgs/msg/String "{data: 'HEALTHY'}"
ros2 topic pub --once /tts_test/arm_health_state std_msgs/msg/String "{data: 'FAULT'}"
```

Expect one warning from the first two messages combined, and another after
HEALTHY then FAULT. Remove test remaps for normal integration. The package never
publishes arm/head/base commands and does not change the presenter's sequence.
Speech completion is not fed back into the coordinator: it will continue
according to its existing code.

## Original setup and reference

Offline English (Piper Lessac) and Japanese (Piper Plus Tsukuyomi-chan),
with a happy idle face, blinking, a volume-driven w mouth, and fullscreen.
No ears or nose. The ESP demo is unrelated to this package.

Targets ROS 2 Humble on Ubuntu 22.04 and Jazzy on Ubuntu 24.04 using
ament_python. ROS 2 itself must already be installed and sourced.
This package was syntax-checked and unit-tested without ROS; a real DDS,
display, audio-device and installed-model integration test is still needed.

## Install and build

Extract the archive so you have `~/ros2_ws/src/robot_cat_ros/package.xml`.
From a normal Ubuntu desktop terminal (not sudo):

```bash
sudo apt install python3-tk python3-pygame python3-venv python3-colcon-common-extensions curl
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_cat_ros
source install/setup.bash
```

Use `jazzy` in the source command if that is your installed distribution.
Run colcon with the system Python, not a Piper virtual environment.

## English engine and model

Keep your existing working Piper installation and Lessac files if available.
The `english_binary` parameter is the absolute path to its `piper` executable,
not to Python. Otherwise make an isolated environment:

```bash
python3 -m venv ~/venvs/robot-piper
~/venvs/robot-piper/bin/pip install 'piper-tts>=1.3,<2'
mkdir -p ~/robot_voices
curl -fL https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/en_US-lessac-medium.onnx -o ~/robot_voices/en_US-lessac-medium.onnx
curl -fL https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json -o ~/robot_voices/en_US-lessac-medium.onnx.json
```

## Japanese engine and model (separate installation)

Use a Linux x64 **C++** build of Piper Plus compatible with your Ubuntu/glibc.
The v1.13.0 asset is `piper-linux-x64.tar.gz`, not the filename previously shown
in this guide. Its downloaded binary required GLIBC_2.38 on your test machine;
use a compatible machine/build, not a manual system-glibc replacement.
Find its real executable path after extraction and supply `japanese_binary`.
The default below is only a configurable path, not a guaranteed archive layout.
This package's Japanese
adapter targets that CLI, not the Python or Rust variants.

Download the model and its matching config together:

```bash
curl -fL https://huggingface.co/ayousanz/piper-plus-tsukuyomi-chan/resolve/main/tsukuyomi-chan-6lang-fp16.onnx -o ~/robot_voices/tsukuyomi-chan-6lang-fp16.onnx
curl -fL https://huggingface.co/ayousanz/piper-plus-tsukuyomi-chan/resolve/main/config.json -o ~/robot_voices/tsukuyomi-config.json
~/piper-plus/bin/piper-plus --model ~/robot_voices/tsukuyomi-chan-6lang-fp16.onnx --config ~/robot_voices/tsukuyomi-config.json --language ja --length-scale 1.5 --text 'こんにちは！' -f /tmp/japanese-test.wav
```

Complete any required Japanese dictionary setup reported by the release before
disconnecting the network. Test again offline. The ROS adapter never requests
model downloads, uses explicit local files, and does not enable CUDA. Engine
dependencies/dictionaries must already be installed. The model card is at
https://huggingface.co/ayousanz/piper-plus-tsukuyomi-chan (samples and terms).

## Two-terminal operation

Source ROS and `~/ros2_ws/install/setup.bash` in both terminals.

Terminal 1 — face and audio playback only:

```bash
ros2 run robot_cat_ros face
```

Terminal 2 — all controls, speech worker, and ROS subscriptions:

```bash
ros2 run robot_cat_ros controls --ros-args \
  -p english_binary:="$HOME/venvs/robot-piper/bin/piper"
```

Type text to speak it. Commands are case-insensitive:

```text
/english
Hello, I am ready.
/japanese
こんにちは！準備ができました。
/happy
/Fullscreen
/blink
/stop
/quit
```

Also supported: `/neutral`, `/sleepy`, `/curious`, `/surprised`, `/idle`,
`/help`, `/say text`. `/stop` cancels active synthesis/playback and clears the
pending speech queue. `/quit` exits terminal 2; the face keeps idling.
Expression commands here do not accept duration arguments.

For autonomous operation, replace `controls` with `speech` using the same
parameters. **Run only one of these two executables at a time.** Do not use
the legacy `face controls` mode alongside them.

## ROS interface

All messages are `std_msgs/msg/String`. Subscribers use reliable, volatile,
keep-last depth 10 QoS (no historical latched speech replay).

| Topic | Direction | Data |
| --- | --- | --- |
| `/robot_cat/say` | subscribe | Plain text using current language |
| `/robot_cat/say/en` | subscribe | Plain text explicitly in English |
| `/robot_cat/say/ja` | subscribe | Plain text explicitly in Japanese |
| `/robot_cat/language` | subscribe | `en`, `ja`, `english`, `japanese` |
| `/robot_cat/command` | subscribe | `happy`, `fullscreen`, `stop`, etc. |
| `/robot_cat/status` | publish | `synthesizing:en`, `speaking:ja`, `idle`, `error: ...` |

```bash
ros2 topic pub --once /robot_cat/say/en std_msgs/msg/String "{data: 'Hello from ROS 2'}"
ros2 topic pub --once /robot_cat/say/ja std_msgs/msg/String "{data: 'こんにちは、ロボットです。'}"
ros2 topic pub --once /robot_cat/language std_msgs/msg/String "{data: 'ja'}"
ros2 topic pub --once /robot_cat/command std_msgs/msg/String "{data: 'fullscreen'}"
ros2 topic echo /robot_cat/status
```

Use explicit `/say/en` and `/say/ja` topics when language and text arrive from
automated publishers: separate topics have no cross-topic ordering guarantee.
Plain speech is never interpreted as a command, even if it starts with `/`.

Subscribe to your NLP node's existing output topic by remapping:

```bash
ros2 run robot_cat_ros controls --ros-args \
  -p english_binary:="$HOME/venvs/robot-piper/bin/piper" \
  -r /robot_cat/say:=/nlp/response
```

The producer must publish a String containing the actual text to speak.
For JSON/custom message types, extract the desired field in the producer or
an adapter; this node does not guess which fields should be spoken.

Example inside another Python ROS node:

```python
from std_msgs.msg import String
publisher = node.create_publisher(String, '/robot_cat/say/ja', 10)
# In a callback after DDS discovery:
publisher.publish(String(data='目的地に到着しました。'))
```

## Parameters and resource behavior

Startup ROS parameters:

| Parameter | Default |
| --- | --- |
| `language` | `en` |
| `face_port` | `8765` |
| `queue_size` | `4` pending utterances, plus one active |
| `synthesis_timeout` | `60.0` seconds |
| `english_binary` | `piper` on PATH |
| `english_model` | `~/robot_voices/en_US-lessac-medium.onnx` |
| `english_config` | same path plus `.json` |
| `english_length_scale` | `1.0` |
| `japanese_binary` | `~/piper-plus/bin/piper-plus` |
| `japanese_model` | `~/robot_voices/tsukuyomi-chan-6lang-fp16.onnx` |
| `japanese_config` | `~/robot_voices/tsukuyomi-config.json` |
| `japanese_length_scale` | `1.5` |

Use startup `-p` overrides for these; runtime parameter edits are not applied.
Use the language topic/terminal commands to switch while running.
For a different face port, pass `face --port 8766` and `-p face_port:=8766`.
The face and speech node must run on the same desktop machine; remote ROS
publishers may send text normally using your configured ROS networking.

Only one synthesis child process runs at a time. It exits after each utterance,
releasing its model memory before playback; expect a model-loading delay each
time. New messages queue without interrupting the current utterance. Each
message snapshots its language at arrival. The newest is rejected if full.
Maximum text length is 1000 characters. Newlines are collapsed for CLI input.
Missing models produce visible errors; there is no silent system-voice fallback.

OMP/OpenBLAS threads default to 2 for child processes, but this is **not** a
hard CPU limit and ONNX may use its own thread pools. For testing on a busy
robot, you can run terminal 2 with `nice -n 10 ros2 run ...`; priority is not
a resource reservation. Measure with your other models running.

## Validation and troubleshooting

```bash
cd ~/ros2_ws/src/robot_cat_ros
PYTHONPATH=. python3 test/test_queue.py
PYTHONPATH=. python3 -m unittest discover -s test -p 'test_presenter.py' -v
```

Local tests cover language snapshotting, ordered playback, bounded queues,
stale-audio cancellation, and UTF-8 subprocess transport using fake engines.
They do not validate voice quality, real model CLI compatibility, DDS, or audio
hardware. After installing, test each explicit language topic, switch via the
terminal, issue `/stop` during synthesis/playback, and check `/Fullscreen`.

If there is no face: use a desktop session with DISPLAY set and Tk installed.
If there is no speech: inspect `/robot_cat/status` and terminal errors; verify
both engine executables with `--help`, model paths, and desktop audio output.
Start the face first. A disconnected face rejects speech; failed text is not
retried automatically. First-use dictionary setup belongs to Piper Plus.

References:
- https://github.com/OHF-Voice/piper1-gpl/blob/main/docs/CLI.md
- https://github.com/ayutaz/piper-plus/blob/dev/docs/guides/development/cli-usage.md
- https://github.com/ros2/examples/tree/humble/rclpy/topics/minimal_subscriber

Models and engines are not bundled; their respective licenses apply.
