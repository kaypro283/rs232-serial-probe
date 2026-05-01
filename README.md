# Serial Probe

`serial_probe.py` is a Windows terminal tool for discovering serial switch settings for a serial-to-serial printer buffer. It sends known ASCII probe data into one COM port, reads the output from another COM port, scores the match, and ranks the likely settings.

The intended setup is:

- Input side: `COM1`, where the tool transmits test data.
- Output side: `COM5`, where the tool reads data from the buffer.

## Run

Start the interactive terminal menu:

```powershell
python serial_probe.py
```

Usage screen:

```powershell
python serial_probe.py --help
```

The first screen is the command menu. Use `9 CURRENT SETTINGS` to view ports, baud range, number of settings to test, test message size, repeat count, timing, old-output clearing, Phase 0 liveness settings, baud focus rules, report files, and estimated scan time. It also shows that scan mode is asked at scan start and that blank means `AUTO`. Use `11 MEMORY TEST` after you have a likely switch setting. The normal full scan still tests every selected combination unless quick exploratory findings are accepted for phase 2.

The terminal UI is written for an 80-column early terminal style. Screens use terse uppercase operator text and bright green text when the console supports ANSI color. PyCharm runs are treated as color-capable. Set `NO_COLOR=1` before running if you want plain console text.

Status screens, setting-change notices, and the final report use `*` borders to match the style of terminal reports from early printer and communications utilities.

Suggested first run for this COM1-to-COM5 setup:

```powershell
python serial_probe.py
```

Use the default settings, then select `1. Start scan`.

## Scan Mode

When a scan starts, the tool asks:

```text
SCAN MODE: AUTO OR MANUAL [AUTO]:
```

Accepted answers are `A`, `AUTO`, `M`, and `MANUAL`. Press Enter for `AUTO`.

`AUTO` is the default. It runs Phase 0 baud liveness, then quick exploratory mode, and accepts phase-2 full analysis from the quick findings without asking the two follow-up yes/no questions:

```text
AUTO MODE: EXPLORATORY=YES PHASE2=YES
```

`MANUAL` preserves the old operator flow. It asks whether to run quick exploratory mode and, when quick findings are usable, later asks whether to use them for phase 2.

## Quick Exploratory Mode

In `MANUAL` mode, the tool asks:

```text
RUN QUICK EXPLORATORY MODE FIRST? (Y/N) [N]:
```

Answer `N` or press Enter to run the normal full scan over every selected setting.

Answer `Y` to run the quick pipeline first. The pipeline starts with Phase 0, then runs the normal quick exploratory pass against the bauds Phase 0 marked alive. The quick pass uses fixed internal settings:

- `160` byte probe payload, or the generator minimum if that is ever larger.
- `1` test per setting.
- `0.4` second output quiet wait after sending.
- `20` ms port-open pause.
- Old-output clearing on, with `0.1` seconds quiet, `0.5` seconds limit, and `32768` max clear bytes.

These exploratory payload, timing, repeat, and old-output clearing settings are not configurable from the menu. Menu settings for test size, timing, repeat count, and clearing apply to the full scan only.

After the quick pass, the tool prints a concise ranked summary with top observed candidates, confidence notes, stale/no-data/error counts, and whether the findings are strong enough to form a shortlist. If the findings are usable, it asks:

```text
USE THESE FINDINGS TO NARROW FULL ANALYSIS? (Y/N) [N]:
```

Answer `Y` to run the full scan only against the exploratory shortlist. Only the candidate list is narrowed; the full scan still uses the normal menu-configured payload, timing, repeat count, report settings, and validation settings. To avoid dropping meaningful flow-control differences, the shortlist keeps all flow-control modes for each promising baud/data/parity/stop-bit frame found by the quick pass.

Answer `N` or press Enter to ignore the quick findings and run the normal full scan over every selected setting.

If quick exploratory mode finds no usable signal, produces only low-confidence results, or is dominated by stale/no-data/error outcomes, the tool does not offer narrowing and falls back to the normal full scan automatically.

## Phase 0 Baud Liveness Sweep

When quick exploratory mode runs, the tool first tests each selected baud once using fixed baseline settings:

- `8` data bits, no parity, `1` stop bit.
- Flow control off.
- Compact structural liveness payload.
- `0.25` second output quiet wait after sending.
- `10` ms port-open pause.
- Old-output clearing on, with `0.05` seconds quiet, `0.25` seconds limit, and `32768` max clear bytes.

Phase 0 is a boolean gate, not a ranking. A baud is marked `ALIVE` only when the received bytes contain a valid checksummed probe line, at least one expected probe marker, a score of `90.0` or higher, no serial error, no stale output, and only limited extra bytes. Random noise or old backlog bytes are not enough.

If one or more bauds are alive, quick exploratory tests only candidates at those bauds. If zero bauds are alive, the tool prints an explicit fallback and quick exploratory uses all selected bauds, preserving the older behavior. The full scan still tests all selected settings unless the normal quick shortlist is accepted for phase 2.

## Baud Focus

Quick exploratory mode has a cautious baud-focus rule. It looks for a strong clean pattern at one baud across multiple framing and flow-control variants. When the configured confidence gates pass, it stays at that baud, finishes the remaining quick tests there, and may defer the other bauds from the quick pass.

Default gates:

- Baud focus enabled: `YES`.
- Strong-hit score: `90.0` or higher.
- Lead over next best baud: `20.0` score points or higher.
- Minimum strong results at the baud: `3`.
- Minimum early samples per baud: `8`.

Stale output, partial writes, or driver errors before a clean hit pattern do not permanently block focus; they are treated as noise until a baud proves itself. Once focus is engaged, any stale output, partial write, driver error, or confidence drop cancels focus and returns to the normal full baud sweep. The full brute-force scan still exists: choose `MANUAL`, answer `N` to quick mode, or decline quick narrowing; the tool then tests every selected baud/data/parity/stop/flow combination. If `AUTO` accepts a narrowed phase 2, the report records that choice.

Use menu command `12 BAUD FOCUS` to change the focus thresholds. Use `9 CURRENT SETTINGS` to view the active values.

Example operator lines:

```text
SCAN MODE: AUTO OR MANUAL [AUTO]:
AUTO MODE: EXPLORATORY=YES PHASE2=YES
BAUD FOCUS ENGAGED: 38400 SCORE=100.0 GAP=50.0 GOOD=4 SAMPLES=8
OTHER BAUDS DEFERRED BY CONFIDENCE RULE
```

If confidence falls after focus starts, the tool returns to the broad scan:

```text
BAUD FOCUS CANCELED: CONFIDENCE DROPPED
RETURNING TO FULL BAUD SWEEP
```

Prompt-path checks:

```text
Scan mode MANUAL, quick prompt N:
  Skip quick mode. Full scan runs all selected settings.

Scan mode MANUAL, quick prompt Y, phase-2 prompt N:
  Quick summary is shown. Full scan still runs all selected settings.

Scan mode MANUAL, quick prompt Y, phase-2 prompt Y:
  Quick summary is shown. Full scan runs the narrowed candidate list using full-scan settings.

Scan mode AUTO:
  Phase 0 and quick mode run. Phase 2 uses quick findings if they are usable; otherwise full scan runs all selected settings.

Scan mode MANUAL, quick prompt Y, no usable quick findings:
  Quick summary explains the fallback. Full scan runs all selected settings.
```

## How Many Combinations?

With the default printer-buffer baud range, the scan tests:

```text
10 baud rates x 2 data-bit choices x 5 parity choices x 2 stop-bit choices x 4 flow-control choices = 800 combinations
```

The default baud list is:

```text
110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400
```

The scan tries the fastest selected baud rate first, then works downward. With the default range, it starts at `38400` and ends at `110`. A normal full scan still tests every data-bit, parity, stop-bit, and flow-control combination. Quick exploratory may test fewer combinations when Phase 0 finds a smaller alive baud set.

## Speed

The default menu settings are tuned for a practical scan:

- `180` bytes per setting.
- `1` test per setting.
- Every selected serial setting is tested in a normal full scan. AUTO may shorten the exploratory pass with Phase 0 and may shorten the phase-2 candidate list only when confidence gates pass.
- Output wait after send is `2.0` seconds by default.
- `Ask on top match` is off by default. If enabled, a `PASS` result pauses the scan and asks whether to continue looking for possible ties.
- `Auto validate top matches after scan` is on by default. It retests the top-score setting or settings with an 8K payload, then uses a 16K payload if a tie remains. The menu can turn this off or change the sizes.
- Old-output clearing stops after `32768` bytes by default, which is enough for a 16K buffer plus margin.
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
22:10:02 *              SETTING 1/800  38400 8N1 FLOW=NONE            *
22:10:02 [0001/0800 38400 8N1 FLOW=NONE] TEST 1/1: SEND 180 BYTES
22:10:04 [0001/0800 38400 8N1 FLOW=NONE] TEST 1/1: RESULT FAIL SCORE=0.00
SCAN TIME 0001/0800: ELAPSED=2S AVG=2S/SET LEFT=26M38S FINISH=22:36:42
```

## Stale Output

If the buffer is already dumping old data, the scan cannot reliably score the current test. The tool clears old output before each test and waits for the output side to go quiet.

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

- Full JSON report.
- CSV summary.
- Debug log.

The report paths are configured from the menu.

The scan start screen, final summary note, debug log, JSON metadata, and CSV rows indicate the selected scan mode, whether Phase 0 and quick exploratory mode ran, which bauds Phase 0 marked alive, whether Phase 0 fell back to all bauds, whether the full analysis candidate list was narrowed, whether baud focus engaged, which baud was focused, whether other bauds were deferred, and why focus was engaged or released.

## Memory Test

The memory test is a separate menu command: `11 MEMORY TEST`.

Run it after the scan has found a likely switch setting. Enter the recommended baud rate, data bits, parity, stop bits, and flow control from the final scan report.

The memory test can check `16K`, `32K`, and `64K` transfers. It reports:

- The serial setting used.
- The largest clean transfer.
- Bytes sent and read.
- Old bytes cleared before the test.
- Bytes that appeared before the buffer was released.
- Missing and extra bytes.

For a real buffer-size check, use `Hold output, then release` if the printer buffer has an `OFF LINE`, `HOLD`, or `PAUSE` control. The program sends the test message while the output is held, then asks you to release the buffer so it can read the stored data.

If the buffer cannot be held, use `Read while sending`. That checks whether a large transfer passes cleanly, but it does not prove the installed RAM size because the buffer may be passing data straight through instead of storing it.

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

Stale data:

- Clear/reset the physical buffer.
- Let it finish dumping old data.
- Increase the old-output clearing time from the menu if you want the tool to wait longer before marking a setting `STALE`.
- For a 16K buffer, the default max clear value of `32768` bytes should usually be enough. If more than that keeps arriving, the output is probably repeating, noisy, or not really stale buffer contents.
