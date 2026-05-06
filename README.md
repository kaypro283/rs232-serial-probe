# Serial Probe

`serial_probe.py` is a Windows terminal utility for identifying workable serial settings through an inline serial buffer. It was built to diagnose a **Consolink Corporation Microspooler (Model SS16)** that was upgraded from **16K to 64K** memory, where switch positions and real behavior did not always line up cleanly with old notes or assumptions.

In plain terms: this tool sends known probe data into one COM port, reads what comes out of another COM port, and scores how closely the output matches the input. It then ranks likely settings and writes a persistent report/log so you can compare runs.

---

## Why this program exists

I wrote this for bench troubleshooting where "try a few settings and hope" was too slow and too ambiguous. The original target was a Consolink microspooler path:

- Input side: `COM1` (probe transmit)
- Output side: `COM5` (probe receive)
- Physical chain: `COM1 -> buffer input -> buffer output -> COM5`

For this hardware, one practical assumption during discovery is to set both switch banks the same way and test from there.

Also important: the program applies serial settings directly when it opens each port (baud, bits, parity, stop, flow). It does **not** trust Device Manager defaults as test truth.

---

## Other practical uses beyond the Consolink SS16

Even though the script was created for a specific microspooler scenario, it is broadly useful anywhere serial behavior is uncertain:

- **Unknown legacy device recovery**: find plausible framing/flow combinations when documentation is incomplete.
- **Post-repair validation**: confirm that a repaired cable path or interface converter still transfers cleanly.
- **Handshake behavior checks**: separate plain transfer compatibility from actual hold/release and buffer-full backpressure behavior.
- **Service documentation**: generate repeatable evidence (report + debug log) for future techs.
- **Lab qualification**: compare multiple switch states or adapters without manual retesting.

If a device speaks RS-232-style serial and you can loop through it with known TX/RX ports, this workflow is usually applicable.

---

## Requirements

- Windows with Python 3
- Access to the two COM ports in your test path
- `pyserial` installed (if not already):

```powershell
pip install pyserial
```

- No other software actively holding those COM ports open (terminal emulator, driver UI, vendor tool, etc.)

---

## Run

Start the interactive menu:

```powershell
python serial_probe.py
```

Show help:

```powershell
python serial_probe.py --help
```

The UI is intentionally old-terminal style (80x25 friendly), with paged screens and compact operator text. ANSI green is used when supported (PyCharm is treated as color-capable). Set `NO_COLOR=1` if you prefer plain text output.

---

## Quick start (recommended first pass)

1. Launch `python serial_probe.py`
2. In the menu, review `5 CURRENT SETTINGS`
3. Keep defaults (`COM1` input, `COM5` output, both fixed at `38400`) unless you know you need different ports/bauds
4. Default scan range is `75` through `115200`
5. Choose `1 START SCAN`
6. Run `AUTOMATED DISCOVERY`
7. Use the generated report to decide whether to validate further or adjust switch state

During any active scan/test, `Ctrl+C` opens an operator-break menu so you can resume, end and report, return to menu, or quit after writing partial results.

---

## Start Scan workflows

From `1 START SCAN`:

```text
1 AUTOMATED DISCOVERY
2 KNOWN-BAUD DEVICE TEST
3 PHASE 0 BAUD LIVENESS ONLY
4 RETURN TO MAIN MENU
```

### 1) Automated Discovery

This is the default path for unknown settings:

- Runs Phase 0 baud liveness first
- Runs staged frame sweeps
- Falls back to full matrix only when needed
- Optionally validates top match afterward (based on menu setting)

### 2) Known-Baud Device Test

Use this when input/output baud is already known.

This mode runs targeted checks for:

- clean ASCII frame transfer
- 8-bit challenge behavior
- raw byte fidelity classes
- ETX/ACK behavior
- flow transfer validation across all 16 in/out flow combinations
- output hold/release proof (when applicable)
- input buffer-full stress behavior after proven output hold

Report sections intentionally separate these into `FLOW MATRIX`, `OUTPUT HOLD`, and `INPUT FULL` because they prove different things.

### 3) Phase 0 Baud Liveness Only

A fast gate for "is anything alive at these baud pairs?" using fixed baseline framing/flow rules. This mode does not perform full ranking.

---

## Phase 0 liveness details

Phase 0 uses fixed baseline serial settings:

- 8 data bits
- even parity
- 1 stop bit
- flow off
- compact structural payload
- fixed timing/count rules
- old-output clearing enabled

A pair is marked `ALIVE` only when structure checks pass and output quality is acceptable (no stale contamination/serial error, limited tolerated extras).

---

## Matrix size and baud order

With default baud list:

```text
75, 110, 134, 150, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200
```

Full same-frame matrix:

```text
15 baud x 2 data-bit choices x 5 parity choices x 2 stop choices x 4 flow modes = 1200 combinations
```

Scan order is highest selected baud to lowest (default: `115200` down to `75`).

---

## Performance notes

Default settings are tuned for practical bench use:

- `512` quick-scan bytes per setting
- `1` test per setting
- `2.0s` post-send output wait
- top-match verify enabled (`8K` payload)
- flow tests enabled
- quick stale clear cap `DEFAULT_MAX_DRAIN_BYTES` (`128 KiB`, `131072` bytes; enough for a 64K buffer plus margin)
- known-baud purge stages use longer calculated drain limits

One pass is usually enough to identify likely settings. For confidence, rerun with larger payloads or more repeats around top candidates.

---

## Live output behavior

During scans, the console shows:

- current setting banner (e.g., `9600 8N1 FLOW=NONE`)
- per-test progress
- bytes read and pre-test stale-clear count
- result class (`PASS`, `GOOD`, `PARTIAL`, `FAIL`, `STALE`, `ERROR`)
- numeric score (`0` to `100`)
- running scan-time estimate

Example:

```text
22:10:02 *              SETTING 1/480  38400 8N1 FLOW=NONE            *
22:10:02 [0001/0480 38400 8N1 FLOW=NONE] TEST 1/1: SEND 512 BYTES
22:10:04 [0001/0480 38400 8N1 FLOW=NONE] TEST 1/1: RESULT FAIL SCORE=0.00
SCAN TIME 0001/0480: ELAPSED=2S AVG=2S/SET LEFT=15M58S FINISH=22:26:02
```

---

## Stale output handling

If buffered old data is still dumping, a score can be misleading. The tool tries to clear old output before tests and waits for the line to go quiet.

- Quick per-test stale handling is controlled in `TIMING / PER-TEST STALE`
- The default quick stale-clear cap is `DEFAULT_MAX_DRAIN_BYTES` (`128 KiB`, `131072` bytes)
- Known-baud and specialized workflows use longer calculated purge limits
- If output does not settle, the test is marked `RESULT STALE`

For the upgraded 64K SS16 scenario, default quick clear (`DEFAULT_MAX_DRAIN_BYTES`, `128 KiB` / `131072` bytes) is typically sufficient. If much more arrives continuously, you are usually seeing repeat/noise/wrong framing rather than normal stale residue.

---

## Reports and logs

Every run appends to fixed files:

- `serial_probe_report.txt`
- `serial_probe_debug.log`

The files are intentionally append-only across restarts so you keep historical evidence unless you manually rotate/clear them.

Report output includes:

- ranked top results
- recommended setting (when evidence is strong enough)
- tie reporting for effectively equal top candidates
- explicit no-working-setting outcomes
- phase summaries, validation notes, and interpretation guidance

---

## Safety

Before testing, disconnect live equipment that could print, actuate, move, or queue real jobs. Probe only through the intended buffer path. Confirm electrical compatibility and cabling for your RS-232 hardware.

---

## Troubleshooting

### No data

- Verify physical direction (`COM1` into buffer input, `COM5` from buffer output)
- Close applications that may be holding either COM port
- Confirm COM numbering in Device Manager

### Garbled data

- Baud may be close, but framing is likely wrong
- Compare top rows across bits/parity/stop/flow, not baud alone

### Flow-control confusion

- Compare `none`, `xon/xoff`, `rts/cts`, `dsr/dtr`
- Use known-baud testing and read `INPUT FULL` if you need evidence of actual backpressure behavior

### Persistent stale

- Clear/reset the physical buffer
- Let buffered output finish
- Increase quick clear time if needed for regular scan mode
- Remember known-baud purge stages already apply longer drain logic
