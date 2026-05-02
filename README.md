# Serial Probe

`serial_probe.py` is a Windows terminal tool for discovering serial switch settings for a serial-to-serial printer buffer. It sends known ASCII probe data into one COM port, reads the output from another COM port, scores the match, and ranks the likely settings.

The intended setup is:

- Input side: `COM1`, where the tool transmits test data.
- Output side: `COM5`, where the tool reads data from the buffer.
- Physical path: `COM1 -> buffer input -> buffer output -> COM5`.
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

The first screen is the command menu. Use `S CURRENT SETTINGS` to view ports, baud range, number of settings to test, test message size, repeat count, timing, old-output clearing, Phase 0 liveness settings, report files, and estimated scan time. It also shows that scan type is asked at scan start and that blank means `FULL`. The normal full scan tests every selected combination unless you explicitly choose quick exploratory narrowing.

The terminal UI is written for an 80-column by 25-line early terminal style. Long operator screens pause with `PRESS ENTER FOR MORE, Q TO STOP:`. Screens use terse uppercase operator text and bright green text when the console supports ANSI color. PyCharm runs are treated as color-capable. Set `NO_COLOR=1` before running if you want plain console text.

Status screens, setting-change notices, and the final report use `*` borders to match the style of terminal reports from early printer and communications utilities.

Suggested first run for this COM1-to-COM5 setup:

```powershell
python serial_probe.py
```

Use the default settings, then select `1. Start scan`.

After a scan finishes or is interrupted by the operator, the program stays in the terminal UI and asks whether to run the same settings again, return to the main menu, or quit.

During a running scan or validation pass, press `Ctrl+C` for the `OPERATOR BREAK` menu. The menu can resume the same test, end the test and write a partial report, return to the main menu after writing the report, or quit after writing the report.

## Scan Type

When a scan starts, the tool asks:

```text
SCAN TYPE: FULL OR QUICK [FULL]:
```

Accepted answers are `F`, `FULL`, `M`, `MANUAL`, `Q`, and `QUICK`. Old `A` and `AUTO` inputs are still accepted as aliases for `QUICK`. Press Enter for `FULL`.

`FULL` is the default and is the safest mode for switch mapping. It asks whether to run quick exploratory mode, and pressing Enter answers `N`, so the normal path is a complete scan over every selected setting.

`QUICK` runs Phase 0 baud liveness, then quick exploratory mode, and accepts phase-2 full analysis from the quick findings without asking the two follow-up yes/no questions:

```text
QUICK SCAN: EXPLORATORY=YES PHASE2=YES
```

`MANUAL` is accepted as an alias for `FULL`.

## Quick Exploratory Mode

In `FULL` mode, the tool asks:

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

- `8` data bits, mark parity, `1` stop bit.
- Flow control off.
- Compact structural liveness payload.
- `0.25` second output quiet wait after sending.
- `10` ms port-open pause.
- Old-output clearing on, with `0.05` seconds quiet, `0.25` seconds limit, and `32768` max clear bytes.

Phase 0 is a boolean gate, not a ranking. A baud is marked `ALIVE` only when the received bytes contain a valid checksummed probe line, at least one expected probe marker, a score of `90.0` or higher, no serial error, no stale output, and only limited extra bytes. Random noise or old backlog bytes are not enough.

If one or more bauds are alive, quick exploratory tests only candidates at those bauds. If zero bauds are alive, the tool prints an explicit fallback and quick exploratory uses all selected bauds, preserving the older behavior. The full scan still tests all selected settings unless the normal quick shortlist is accepted for phase 2.

Prompt-path checks:

```text
Scan type FULL, quick prompt N:
  Skip quick mode. Full scan runs all selected settings.

Scan type FULL, quick prompt Y, phase-2 prompt N:
  Quick summary is shown. Full scan still runs all selected settings.

Scan type FULL, quick prompt Y, phase-2 prompt Y:
  Quick summary is shown. Full scan runs the narrowed candidate list using full-scan settings.

Scan type QUICK:
  Phase 0 and quick mode run. Phase 2 uses quick findings if they are usable; otherwise full scan runs all selected settings.

Scan type FULL, quick prompt Y, no usable quick findings:
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
- Every selected serial setting is tested in a normal full scan. QUICK may shorten the exploratory pass with Phase 0 and may shorten the phase-2 candidate list only when confidence gates pass.
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

- One append-only text report, `serial_probe_report.txt` by default.
- One append-only debug log, `serial_probe_debug.log` by default.

The report paths are configured from the menu.

Each scan appends a compact run block with the switch/jumper note, selected scan type, phase summary, top results, validation results when run, and interpretation notes.

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
