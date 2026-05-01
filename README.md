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

The first screen is the command menu. Use `9 CURRENT SETTINGS` to view ports, baud range, number of settings to test, test message size, repeat count, timing, old-output clearing, report files, and estimated scan time. Use `11 MEMORY TEST` after you have a likely switch setting. The scan tests every selected combination.

The terminal UI is written for an 80-column early terminal style. Screens use terse uppercase operator text and bright green text when the console supports ANSI color. PyCharm runs are treated as color-capable. Set `NO_COLOR=1` before running if you want plain console text.

Status screens, setting-change notices, and the final report use `*` borders to match the style of terminal reports from early printer and communications utilities.

Suggested first run for this COM1-to-COM5 setup:

```powershell
python serial_probe.py
```

Use the default settings, then select `1. Start scan`.

## How Many Combinations?

With the default printer-buffer baud range, the scan tests:

```text
10 baud rates x 2 data-bit choices x 5 parity choices x 2 stop-bit choices x 2 flow-control choices = 400 combinations
```

The default baud list is:

```text
110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400
```

The scan tries the fastest selected baud rate first, then works downward. With the default range, it starts at `38400` and ends at `110`. It still tests every data-bit, parity, stop-bit, and flow-control combination.

## Speed

The default menu settings are tuned for a practical scan:

- `180` bytes per setting.
- `1` test per setting.
- Every selected serial setting is tested.
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
22:10:02 *              SETTING 1/400  38400 8N1 FLOW=NONE            *
22:10:02 [0001/0400 38400 8N1 FLOW=NONE] TEST 1/1: SEND 180 BYTES
22:10:03 [0001/0400 38400 8N1 FLOW=NONE] TEST 1/1: RESULT FAIL SCORE=0.00
SCAN TIME 0001/0400: ELAPSED=1S AVG=1S/SET LEFT=6M39S FINISH=22:16:42
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

- Compare `none` and `xon/xoff`; those are the flow-control modes in the scan.
- If hardware flow control is required, this scan range would need to be expanded again.

Stale data:

- Clear/reset the physical buffer.
- Let it finish dumping old data.
- Increase the old-output clearing time from the menu if you want the tool to wait longer before marking a setting `STALE`.
- For a 16K buffer, the default max clear value of `32768` bytes should usually be enough. If more than that keeps arriving, the output is probably repeating, noisy, or not really stale buffer contents.
