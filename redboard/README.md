# RedBoard pulse trigger

A small Arduino that turns an analog level into a timed digital pulse: when
the voltage on **A0** rises above a threshold, it waits a set delay, drives
**D7** high for a set time, then ignores A0 for a lockout period. PALmixer
restarts its host script every time the mixer head starts moving onto the
carousel, and will not mix until the board is armed -- see
[RedBoard trigger](../README.md#redboard-trigger) in the main README.

| File | What it is |
|---|---|
| `redboard_trigger.py` | Host script: finds the board, configures it, prints what it reports. Run by PALmixer, or by hand. |
| `firmware/redboard_trigger/redboard_trigger.ino` | The sketch on the board (formerly `sketch_sep11a.ino`). |

## The board

The board on this rig enumerates as **Arduino Leonardo ETH** (USB
`2A03:8040`, an ATmega32U4 with native USB), currently on `COM17`. Two things
follow from native USB:

- Opening the serial port does **not** reset it, so the sketch -- including a
  lockout in progress -- keeps running while hosts come and go.
- The COM number follows the physical USB socket, since the board has no USB
  serial number. The script finds the board by asking each board-like port,
  so a new COM number needs no configuration.

The board triggers on its own. With no host connected it keeps triggering with
whatever parameters are in its RAM; the host only configures it and reports.

## Parameters

Edit the four constants at the top of `redboard_trigger.py`:

| Constant | Here | Meaning |
|---|---|---|
| `THRESHOLD_VOLTAGE` | 3.0 V | A0 level that triggers |
| `DELAYTIME` | 0.26 s | from crossing the threshold to the start of the pulse |
| `OPTIME` | 0.39 s | pulse width on D7 |
| `LOCKOUT_TIME` | 10.0 s | A0 ignored after each pulse |

They live only in the board's RAM: after a power cycle or an upload, the board
runs on the sketch's defaults (2.5 V, 0.1 s, 0.4 s, 10 s) until a host
configures it again. Under PALmixer that happens at the next move to the
carousel, which is also when edits take effect.

The board stores the two times as whole milliseconds and reports values to two
decimals, so its acknowledgement (`UPDATED: ...`) confirms them to 10 ms.

## Running it by hand

From an Anaconda Prompt, in any environment with `pyserial` (and `psutil`, for
the takeover below):

```
python redboard_trigger.py                            find the board, configure it, stream its output
python redboard_trigger.py --port COM17               skip the search
python redboard_trigger.py --restart-after-trigger 0  never reconnect on its own
```

By default the script reconnects and reconfigures 20 s after each trigger;
`0` turns that off. PALmixer runs it that way, since it restarts the script
itself. Ctrl-C stops it.

Starting it **stops any other copy already running**, because only one program
can hold the port. With the PALmixer server up and `redboard.enabled` on, a
copy started by hand is itself stopped at the next move to the carousel. So
set `redboard.enabled` to `false` in `json/palmixer_config.json` first; the
server then stops its own copy within a couple of seconds.

Only a Python process *running* a file named `redboard_trigger.py` counts as a
copy -- not an editor that has it open, IDLE and Spyder included. A copy
running inside an IDE's interactive kernel is not recognized, so stop it
yourself; until then, the port shows up as busy.

Only ports that look like the board are probed -- Arduino, SparkFun or CH340
USB IDs, or those names in the port's description. A probe writes `?` to the
port, and the pumps' RS-232 adapters on the same machine must not receive it.
Name any other port with `--port`.

The script reconnects by itself when the port is lost or the board stops
answering for 5 s, and gives up (exit status 1) when no board is found within
15 s (`--connect-timeout`).

## Protocol

Plain serial at 9600 baud, not Firmata.

| Direction | Line | Meaning |
|---|---|---|
| host -> board | `<threshold>,<delay>,<optime>,<lockout>` | configure (V, s, s, s) |
| host -> board | `?` | report A0 now |
| board -> host | `UPDATED: Threshold=3.00V, Delay=0.26s, OpTime=0.39s, Lockout=10.00s` | configuration taken |
| board -> host | `A0: 0.12V, locked=0` | reply to `?` |
| board -> host | `TRIGGER: 3.21V` | threshold crossed; the pulse follows |
| board -> host | `LOCKOUT: 10.00s` | pulse done, lockout started |
| board -> host | `READY: monitoring A0` | lockout over |

Lines the script adds, which PALmixer reads:

| Line | Meaning |
|---|---|
| `ARMED: port=COM17 threshold=3.00V delay=0.26s pulse=0.39s lockout=10.00s locked=0` | the board acknowledged exactly what was sent |
| `DISCONNECTED: ...` | the port was lost or the board stopped answering; looking for it again |
| `Error: ...` | why it is giving up, just before exiting with status 1 |

## Uploading the sketch

Open `firmware/redboard_trigger/redboard_trigger.ino` in the Arduino IDE,
select the board the IDE detects on that port (Arduino Leonardo ETH), and
upload. Nothing else may hold the port during the upload: set
`redboard.enabled` to `false` (the server stops its copy within a couple of
seconds), or stop the PALmixer server, and close any copy of the script you
started yourself. After an upload the board runs on the sketch's default
parameters until the script configures it again -- with `redboard.enabled`
back on, at the next move to the carousel.

## Behaviour worth knowing

- **Level triggered.** If A0 stays above the threshold, the board fires again
  each time its lockout ends -- every delay + pulse + lockout, about 10.65 s
  here. The TRIGGER lines in `logs/redboard_trigger.log` show whether that
  happens during a mix.
- **A lockout cannot be cleared from the host**; it ends on the board's timer.
  A firmware command to clear it would be the way to change that.
