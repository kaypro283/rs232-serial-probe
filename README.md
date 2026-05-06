# Serial Probe

`serial_probe.py` is a Windows terminal tool for discovering serial switch settings for a serial-to-serial printer buffer. It sends known ASCII probe data into one COM port, reads the output from another COM port, scores the match, and ranks the likely settings.

The intended setup is:

- Input side: `COM1`, where the tool transmits test data.
- Output side: `COM5`, where the tool reads data from the buffer.
- Physical path: `COM1 >> buffer input >> buffer output >> COM5`.
- Switch-mapping assumption: set both buffer switch banks the same way for a scan.

The program sets the COM port baud/data/parity/stop/flow options itself when it opens the ports. Device Manager defaults are not used as fixed test settings.

## Run

Start the interactive terminal menu:

```powershell
python serial_probe.py
```

Usage screen:

```powershell
python serial_probe.py --help
```

The first screen is the command menu. Use `5 CURRENT SETTINGS` to view ports, each port's configured baud, the last scan baud range, test message size, repeat count, timing, old-output clearing, Phase 0 liveness settings, fixed report files, and available test workflows. `1 START SCAN` opens the discovery workflow menu.

The terminal UI is written for an 80-column by 25-line early terminal style. Long operator screens pause with `PRESS ENTER FOR MORE, Q TO STOP:`. Screens use terse uppercase operator text and bright green text when the console supports ANSI color. PyCharm runs are treated as color-capable. Set `NO_COLOR=1` before running if you want plain console text.

Status screens, setting-change notices, and the final report use `*` borders to match the style of terminal reports from early printer and communications utilities.

Suggested first run for this COM1-to-COM5 setup:

```powershell
python serial_probe.py
```

Use the default fixed settings (`COM1` input and `COM5` output at `38400` baud each) or set ports and fixed bauds with `2 SET COM PORTS / BAUD`, then select `1 START SCAN`. Discovery and Phase 0 ask for the baud range inside that start-scan workflow.

After a scan finishes or is interrupted by the operator, the program stays in the terminal UI and asks whether to run the same settings again, return to the main menu, or quit.

During a running scan or validation pass, press `Ctrl+C` for the `OPERATOR BREAK` menu. The menu can resume the same test, end the test and write a partial report, return to the main menu after writing the report, or quit after writing the report.

## Start Scan Workflow

Select `1 START SCAN` from the main menu to choose one of the scan workflows:

```text
1 AUTOMATED DISCOVERY
2 KNOWN-BAUD DEVICE TEST
3 PHASE 0 BAUD LIVENESS ONLY
4 RETURN TO MAIN MENU
```

`AUTOMATED DISCOVERY` starts with Phase 0 baud liveness, then runs staged input/output frame sweeps and a full matrix fallback only when needed. It can validate the top match afterward, depending on `3 SCAN / VALIDATE SETUP`.

After choosing `AUTOMATED DISCOVERY` or `PHASE 0 BAUD LIVENESS ONLY`, the workflow asks for the baud range for that run.

`KNOWN-BAUD DEVICE TEST` is for any serial device or switch mode where you already know the input and output baud. It uses the fixed input/output bauds configured in `2 SET COM PORTS / BAUD`, then runs targeted ASCII frame checks, an 8-bit challenge, raw byte behavior probes, ETX/ACK probing, and flow validation checks.

When known-baud testing finds a clean follow-up frame, flow validation sweeps the 16 input/output flow-control combinations as transfer checks. If the follow-up frame is the same on both sides, it also runs the output-side hold/release handshake proof. When an output hold is proven, the buffer-full stress test holds output, sends a larger input payload, watches for input-side XOFF/CTS/DSR throttling, then releases output and verifies the drain. The report keeps these separate as `FLOW MATRIX`, `OUTPUT HOLD`, and `INPUT FULL` because transfer compatibility, output pause/resume, and input buffer-full backpressure are different evidence.

The raw byte behavior phase runs several payload classes after a likely frame is found: CR-only, LF-only, CR/LF, printer-control bytes with TAB/FF/ESC, printable ASCII `0x20..0x7E`, 7-bit controls excluding XON/XOFF, and 7-bit controls including XON/XOFF. Its report compares bytes exactly and records sent/read counts, sent and received hashes, first mismatch offset, and missing/extra byte counts. If the XON/XOFF-free control sweep is exact but the full control sweep changes, the report calls out that XON/XOFF control bytes affected the raw path.

Known-baud device reports use `FOLLOW-UP FRAME` only for a clean ASCII or clean 8-bit target. If no clean ASCII transfer is observed, follow-up behavior, ETX/ACK, and flow checks are skipped or marked diagnostic so a weak fallback row is not mistaken for a recommended setting.

If the known-baud device test cannot find a clean ASCII transfer for the selected known bauds, the result is `NO WORKING SERIAL SETTING FOUND` for that device/switch state and baud pair. The next step is to check the selected bauds, cabling direction, switch state, and buffer clear/reset state, or run automated discovery instead of assuming the known bauds are correct.

`PHASE 0 BAUD LIVENESS ONLY` only tests whether selected input/output baud pairs show a basic signal. It does not use the scan/validate message-size settings.

## Phase 0 Baud Liveness Sweep

Phase 0 tests selected baud pairs using fixed baseline settings:

- `8` data bits, even parity, `1` stop bit.
- Flow control off.
- Compact structural liveness payload.
- Fixed internal count and timing.
- Old-output clearing on.

Phase 0 is a boolean gate, not a ranking. A baud pair is marked `ALIVE` only when the received bytes contain valid checked probe structure, no serial error, no stale output, and only limited extra bytes.

If no baud pair is alive, the program explains the condition. For narrow baud ranges it can offer a same-baud frame fallback so the operator is not silently returned to the main menu.

## How Many Combinations?

With the default printer-buffer baud range, the complete same-frame matrix is:

```text
7 baud rates x 2 data-bit choices x 5 parity choices x 2 stop-bit choices x 4 flow-control choices = 560 combinations
```

The default baud list is:

```text
300, 1200, 2400, 4800, 9600, 19200, 38400
```

The scan tries the fastest selected baud rate first, then works downward. With the default range, it starts at `38400` and ends at `300`. The range is selected inside `1 START SCAN` for automated discovery and Phase 0 runs. Automated discovery starts with Phase 0 and staged frame sweeps; it runs the larger full matrix only when the staged checks do not find a strong pair.

## Speed

The default menu settings are tuned for a practical scan:

- `512` quick-scan bytes per setting.
- `1` test per setting.
- Automated discovery uses Phase 0 and staged frame sweeps before any full matrix fallback.
- Output wait after send is `2.0` seconds by default.
- `Ask on top match` is off by default. If enabled, a `PASS` result pauses the scan and asks whether to continue looking for possible ties.
- `Top-match verify` is on by default. It retests the top-score setting or settings with an 8K payload. The menu can turn this off or change the size.
- `Flow tests` are on by default. Known-baud flow transfer still uses a short 1K payload, while `BUFFER-FULL STRESS` uses 128K by default to test input-side backpressure after output hold is proven.
- Quick per-test old-output clearing stops after `131072` bytes by default, which is enough for a 64K buffer plus margin.
- Known-baud purge stages use calculated long drain limits instead of the quick per-test clear time.
- No early stop.

Three repeated tests are not required for the first scan. One test is enough to rank settings. If you want more certainty afterward, rerun with a larger test message or more tests around the suspected setting range.

Lower baud rates are still physically slower because the serial line can only move a limited number of bytes per second.

The menu estimates scan time in plain terms:

- `Time sending data`: time spent sending the test messages.
- `Time waiting`: short pauses while the tool waits for the ports or buffer.
- `Estimated total`: rough total scan time.

## Live Output

The console shows:

- A `SETTING` banner for each new setting.
- The active setting, such as `9600 8N1 FLOW=NONE`.
- Test number.
- Write progress.
- Bytes received.
- Old bytes cleared before sending.
- Result indicator: `PASS`, `GOOD`, `PARTIAL`, `FAIL`, `STALE`, or `ERROR`.
- Score from `0` to `100`.
- A `SCAN TIME` line with elapsed time, average time per setting, remaining time, and approximate finish clock.
- If `Ask on top match` is enabled, a `TOP MATCH FOUND` prompt after a `PASS` result.

Example:

```text
22:10:02 *              SETTING 1/480  38400 8N1 FLOW=NONE            *
22:10:02 [0001/0480 38400 8N1 FLOW=NONE] TEST 1/1: SEND 512 BYTES
22:10:04 [0001/0480 38400 8N1 FLOW=NONE] TEST 1/1: RESULT FAIL SCORE=0.00
SCAN TIME 0001/0480: ELAPSED=2S AVG=2S/SET LEFT=15M58S FINISH=22:26:02
```

## Stale Output

If the buffer is already dumping old data, the scan cannot reliably score the current test. The tool clears old output before each test and waits for the output side to go quiet.

The `TIMING / PER-TEST STALE` menu controls the quick stale-data check that happens before individual tests. Its `MAX QUICK CLEAR TIME BEFORE TEST` setting is not meant to empty a full low-baud buffer.

When the output baud and frame are known, the tool uses calculated long purge limits instead. Known-baud device tests, flow validation, dual validation, and post-Phase-0 frame scans use those known-baud purge paths.

If the output does not go quiet, the test is marked:

```text
RESULT STALE
```

That means the physical buffer should be cleared, reset, or allowed to finish dumping old content before the probe result can be trusted.

## Reports

At the end, the tool prints:

- Ranked table of top observed results.
- Recommended switch setting with baud rate, data bits, parity, stop bits, and flow control, but only when the result is strong enough.
- A `MULTIPLE TOP SETTINGS FOUND` report when two or more strong results are effectively tied.
- A clear `NO WORKING SETTING FOUND` report when the current buffer switch setup fails every test.
- Scan duration.
- Number of settings tested.
- Result counts.
- Best match and interpretation.
- Exact-byte, line-integrity, printable-ASCII, missing-byte, and extra-byte metrics.

It also writes:

- One fixed text report, `serial_probe_report.txt`.
- One fixed debug log, `serial_probe_debug.log`.

The report paths are not menu-configurable. Report and debug-log writes always append, including across program restarts, so prior session evidence is preserved unless you archive or clear files yourself.

Each test workflow writes a compact run block with the switch/jumper note, selected workflow, phase summary, top results or liveness rows, validation results when run, and interpretation notes. Phase 0-only runs and early no-baud-pair exits write a text report block even though no later frame scan ran.

## Safety Notes

Disconnect real printers or equipment that could print, move, actuate, or store jobs. Use the buffer only. Confirm voltage levels and cabling are appropriate for RS-232 serial hardware.

## Troubleshooting

No data:

- Confirm `COM1` is connected to the buffer input and `COM5` to the buffer output.
- Close terminal programs, printer drivers, or vendor tools using the COM ports.
- Check Device Manager for the actual COM numbers.

Gibberish data:

- The baud rate may be close but framing may be wrong.
- Compare the top rows by baud, data bits, parity, stop bits, and flow control.

Flow-control problems:

- Compare `none`, `xon/xoff`, `rts/cts`, and `dsr/dtr`; those are the flow-control modes in the scan.
- For input-side buffer-full handshake mapping, use `KNOWN-BAUD DEVICE TEST` and check the `INPUT FULL` result. A clean transfer matrix alone does not prove buffer-full backpressure.

Stale data:

- Clear/reset the physical buffer.
- Let it finish dumping old data.
- Increase the quick per-test clear time from the menu if you want normal scan tests to wait longer before marking a setting `STALE`.
- Known-baud purge stages already use calculated long limits for the selected output baud/frame.
- For a 64K buffer, the default quick max clear value of `131072` bytes should usually be enough. If more than that keeps arriving, the output is probably repeating, noisy, or not really stale buffer contents.
