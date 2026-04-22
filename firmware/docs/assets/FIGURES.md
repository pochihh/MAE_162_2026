# Figures Needed for Firmware Documentation

Each figure below is described with its filename, destination document, purpose,
and exact content to draw. Create each figure and save it in this folder
(`firmware/docs/assets/`) using the filename listed.

---

## Figure 1 — System Topology

**Filename:** `system_topology.png`
**Used in:** `firmware/README.md` (first section, before setup instructions)
**Purpose:** Give a first-time reader a mental model of the whole system before
any technical content.

**Content to draw:**

Two main hardware boxes side by side:

Left box — **Arduino Mega 2560**
- Label: "Arduino Mega 2560"
- Sub-items inside the box:
  - DC Motors ×4
  - Stepper Motors ×4
  - Servo Driver (PCA9685 via I2C)
  - IMU (ICM-20948 via I2C)
  - Encoders ×4
  - Voltage Monitoring (ADC)
  - User I/O (Buttons, LEDs, NeoPixel)

Right box — **Raspberry Pi 5**
- Label: "Raspberry Pi 5"
- Sub-items inside the box:
  - ROS 2 Bridge Node
  - RPLidar Driver Node
  - Vision Node (YOLO / Camera)

Above the RPi box, stacked:
- **robot.py** — "Robot API layer"
- **student main.py** — "Student code"

Connections:
- A thick horizontal arrow between Arduino and RPi labeled:
  "UART Serial2 · 200 kbps · TLV protocol"
- An arrow from RPi up to robot.py labeled: "ROS 2 topics"
- An arrow from robot.py up to student main.py labeled: "Python API"
- On the Arduino side: small arrows from the box to hardware labels:
  - "I2C" pointing to PCA9685 and IMU together
  - "PWM / DIR" pointing to DC Motors
  - "STEP / DIR" pointing to Steppers
  - "INT / PCINT" pointing to Encoders

**Style notes:** Keep it clean and block-diagram style. No perspective or 3D.
Horizontal layout. The UART link is the most important connection to emphasize.

---

## Figure 2 — Execution Layer Stack

**Filename:** `execution_layers.png`
**Used in:** `firmware/docs/architecture.md` (Section 1, "What Runs Where")
**Purpose:** Show the three execution tiers and what lives in each one.

**Content to draw:**

Three horizontal stacked bands, top to bottom:

**Top band — Hard Real-Time ISR Layer** (red/orange tint)
- Label: "Hard Real-Time ISR Layer — interrupts enabled, no I2C, no UART, no EEPROM"
- Three boxes inside:
  - "Timer1 OVF · 800 Hz" → "Motor latch / apply (1 slot per tick)"
  - "Timer3 OVF · 10 kHz" → "Stepper pulse generation"
  - "INT / PCINT · edge-driven" → "Encoder counting"

**Middle band — Fast Cooperative Lane** (yellow/amber tint)
- Label: "Fast Cooperative Lane — runs every loop() pass, work-available only"
- Four boxes inside (left to right):
  - "drainUart()" — "RX bytes → TLV decoder"
  - "drainTx()" — "TX queue → Serial2"
  - "fastMotorCompute()" — "Trigger pending motor compute"
  - "fastStatusChunk()" — "Emit status report chunk"

**Bottom band — Periodic Soft Task Layer** (green tint)
- Label: "Periodic Soft Task Layer — millis-based, at most one task per loop() pass"
- Seven boxes inside (left to right), each showing task name and rate:
  - "taskUART · 100 Hz"
  - "taskMotorFeedback · 200 Hz"
  - "taskMotors · event-driven"
  - "taskSafety · 100 Hz"
  - "taskSensors · 100 Hz" (note: "I2C here only")
  - "taskUserIO · 20 Hz"
  - "taskI2CRetry · 1 Hz · priority 7"

A vertical label on the right side spanning all three bands:
"loop() top → bottom order"

**Style notes:** Use distinct background tints for each band. All three bands
are the same width. Keep task boxes uniform in size.

---

## Figure 3 — DC Motor Pipeline Timeline

**Filename:** `dc_motor_pipeline.png`
**Used in:** `firmware/docs/motion_control.md` (Section 1, after the slot model text)
**Purpose:** Show the one-slot pipeline lag and how ISR and loop interleave.

**Content to draw:**

A horizontal timeline showing two full 5ms rounds (8 Timer1 ticks total).

**Row 1 — Timer1 ISR ticks:**
Eight equally spaced tick marks labeled:
`t=0` `t=1.25ms` `t=2.5ms` `t=3.75ms` `t=5ms` `t=6.25ms` `t=7.5ms` `t=8.75ms`

Each tick labeled with its motor slot:
- t=0: Slot 0 (Motor 1)
- t=1.25ms: Slot 1 (Motor 2)
- t=2.5ms: Slot 2 (Motor 3)
- t=3.75ms: Slot 3 (Motor 4)
- t=5ms: Slot 0 (Motor 1) — next round
- etc.

**Row 2 — What the ISR does at each tick:**
Small annotation below each tick:
- Slot 0: "Apply M1 staged output / Latch M1 encoder"
- (same pattern for slots 1–3)

**Row 3 — loop() compute windows:**
Shaded rectangles between ISR ticks showing:
- "loop computes M1" (in the gap after slot 0)
- "loop computes M2" (in the gap after slot 1)
- etc.

**Row 4 — Pipeline lag annotation:**
A bracket spanning from "Latch M1 encoder at t=0" to "Apply M1 output at t=5ms"
labeled: "one full round lag (5ms) between feedback latch and output apply"

**Style notes:** Timeline flows left to right. Use color to distinguish ISR
events (red tick marks) from loop compute windows (blue shaded rectangles).
Keep it readable at ~900px wide.

---

## Figure 4 — Firmware State Machine

**Filename:** `state_machine.png`
**Used in:** `firmware/docs/communication_and_state.md` (Section 7, "State Machine Ownership")
**Purpose:** Replace the bullet-point transition list with a visual that is
immediately useful when debugging unexpected firmware behavior.

**Content to draw:**

Standard state diagram with five states as rounded rectangles:

States:
- **INIT** (gray)
- **IDLE** (blue)
- **RUNNING** (green)
- **ERROR** (orange)
- **ESTOP** (red)

Transitions (directed arrows with labels):

- INIT → IDLE: "boot completed"
- IDLE → RUNNING: "START command"
- RUNNING → IDLE: "STOP command"
- RUNNING → ERROR: "safety fault (battery / heartbeat timeout)"
- ERROR → IDLE: "RESET command"
- ESTOP → IDLE: "RESET command"
- any state → ESTOP: "ESTOP trigger" (draw as one arrow from a dot above all states, or as individual arrows)

Annotations inside each state box:
- IDLE: "actuators disabled"
- RUNNING: "actuators enabled (if battery present)"
- ERROR: "actuators disabled, motors stopped"
- ESTOP: "all outputs zeroed immediately"

**Style notes:** Use color fills matching the severity: gray / blue / green /
orange / red. Keep transitions as clean curved arrows. The "any → ESTOP"
transition can be shown as a red dashed arrow from each state.

---

## Figure 5 — I2C Bus Map

**Filename:** `i2c_bus_map.png`
**Used in:** `firmware/docs/sensors_and_i2c.md` (Section 1, "Shared Wire Bus Ownership")
**Purpose:** Show what is on the shared Wire bus, which tasks access which
device, and the "no I2C in ISR" constraint.

**Content to draw:**

A horizontal I2C bus line in the center of the figure.

**On the bus (boxes hanging below the line):**
- "ICM-20948 IMU · 0x69" — connected to bus
- "PCA9685 Servo Driver · 0x40 · 100 kHz" — connected to bus

**Above the bus (task access arrows pointing down to the bus):**
- "taskSensors() · 100 Hz" → arrow to ICM-20948
- "processDeferred() (UART task)" → arrow to PCA9685
- "taskI2CRetry() · 1 Hz · init only" → arrow to PCA9685

**Blocked zone (red zone to the side):**
A red box labeled "ISR Context — Wire FORBIDDEN"
containing:
- "ISR(TIMER1_OVF_vect)"
- "ISR(TIMER3_OVF_vect)"
- "encoder INT / PCINT"
With a red X crossing an arrow that would point toward the bus.

**Style notes:** Keep it simple. The bus is a horizontal line. Devices hang
below. Tasks access from above. The forbidden ISR zone is clearly red and
separated. Left-to-right layout.

---

## Figure 6 — TLV Communication Flow

**Filename:** `tlv_flow.png`
**Used in:** `firmware/docs/communication_and_state.md` (Section 1–2, UART/TLV overview)
**Purpose:** Show the RX and TX paths through MessageCenter so a maintainer
knows where to look when debugging communication issues.

**Content to draw:**

Two swim-lane rows, clearly separated:

**RX Path (top row):**
Left to right:
1. Box: "Raspberry Pi" — sends bytes
2. Arrow: "Serial2 hardware RX buffer"
3. Box: "drainUart()\n(fast lane, every loop)"
4. Arrow: "raw bytes"
5. Box: "TLV decoder\n(tlvcodec)"
6. Arrow: "decoded frame"
7. Box: "routeMessage()\n(refreshes heartbeat)"
8. Two arrows branching:
   - Down to: "Subsystem handlers\n(immediate, short)"
   - Down to: "Deferred queue\n(servo writes, mag cal)"

**TX Path (bottom row):**
Left to right:
1. Box: "sendTelemetry()\n(taskUART, 100 Hz)"
2. Arrow: "TLV encode"
3. Box: "txStorage_\n(shared TX buffer)"
4. Arrow: "drainTx()\n(fast lane)"
5. Box: "Serial2 hardware TX buffer"
6. Arrow: bytes
7. Box: "Raspberry Pi" — receives

**Vertical divider line** between the two rows with label "MessageCenter owns both paths"

**Style notes:** Swim-lane style. RX path flows left-to-right on top.
TX path flows left-to-right on bottom. Use consistent box shapes.
The deferred queue branch in RX is the most important detail to show clearly.

---

## Notes for All Figures

- Save as `.png` at a minimum of 1200px wide (or 900px for figures 4 and 5).
- Use a white background.
- Font: any clean sans-serif (e.g., Inter, Helvetica, or system default).
- Once created, drop the files in this folder (`firmware/docs/assets/`) and
  let me know — I will add the `![](assets/filename.png)` references into
  the correct location in each document.
