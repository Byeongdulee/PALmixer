# -*- coding: utf-8 -*-
"""Persistent tracking state for PALmixer: what is where, and the carousel table.

Five things are tracked, backed by ``palmixer_state.ini`` next to this file so
they survive a server restart:

    A) the mixer head's location   -- at the mixer, or at its cleaning station
    B) flowcell 1 and 2 locations  -- independently, at the beam or at cleaning
    C) the carousel slot last moved to (the EPICS motor holds the true position)
    D) which flowcell is in use    -- PAL12idb's flowcell_ID
    E) the carousel's slot inventory -- one taught reference position, and the
       sample ID held in each slot

The carousel's *geometry* is not state: how many slots it has and how far the
motor moves between two adjacent ones are properties of the hardware, so they
are configuration (``carousel.size`` / ``carousel.step`` in
json/palmixer_config.json) and are only read here. Slots are evenly spaced, so
only one has to be taught: every other slot's position is derived from that
reference and the step, which means re-teaching after a carousel swap is one
move rather than N.

A slot is "used" exactly when it has a sample ID -- make_sample assigns one
(auto-generating a timestamp ID if the operator did not name the sample) as
soon as the mix step succeeds, since mixing is what consumes the vial. When
every slot is used the carousel is a spent consumable: the operator swaps in a
fresh one and calls reset_carousel(), which drops the sample IDs and the taught
reference, because a replacement carousel is not guaranteed to seat where the
old one did.

IMPORTANT -- this module must stay dependency-free (stdlib only). PAL12idb.py
imports ``camera_tools`` at module scope, which only resolves inside the
beamline ``aps12robot`` environment, so server.py imports PAL12idb only when
not simulating. Keeping the store here means simulate mode, and anything else
that just wants to read state, can import it anywhere. PAL12idb imports this
module; never the reverse.

Every location has an ``unknown`` value and that is the default. On first run,
and after any failed move or manual intervention, ``unknown`` is the honest
answer -- recording a location that was never reached is worse than admitting
we do not know. Callers are expected to refuse to move rather than guess.
"""

import configparser
import os
import threading
import time

from . import config  # stdlib-only itself, so this keeps state.py importable anywhere

# -- location vocabulary ----------------------------------------------------
UNKNOWN = "unknown"

MIXER_AT_MIXER = "mixer_station"
MIXER_AT_CLEANING = "mixer_cleaning_station"
MIXER_LOCATIONS = (MIXER_AT_MIXER, MIXER_AT_CLEANING, UNKNOWN)

FC_AT_CLEANING = "cleaning_station"
FC_AT_BEAM = "sample_table"
FC_IN_GRIPPER = "gripper"  # transient: held by the robot, in transit at the mixer
FC_LOCATIONS = (FC_AT_CLEANING, FC_AT_BEAM, FC_IN_GRIPPER, UNKNOWN)

FLOWCELL_IDS = (1, 2)

# Names used on the wire by "set_location <what> <value>".
WHAT_MIXER_HEAD = "mixer_head"
WHAT_FLOWCELL_1 = "flowcell_1"
WHAT_FLOWCELL_2 = "flowcell_2"
LOCATION_KEYS = (WHAT_MIXER_HEAD, WHAT_FLOWCELL_1, WHAT_FLOWCELL_2)

_INI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "palmixer_state.ini")
_TRACKING = "tracking"
_CAROUSEL = "carousel"            # the taught reference: ref_slot + ref_position
_CAROUSEL_SAMPLES = "carousel_samples"  # slot -> sample ID; a key here == used

# Only the MOUNTED carousel is tracked -- there is no per-carousel history. A
# carousel taken off and put back later arrives as a fresh mount and must be
# re-registered with everything in it, which is what mount_carousel() is for.
MAX_CAROUSEL_ID_LEN = 64

# A sample ID is free text the operator types, so it gets bounded here rather
# than trusted: long enough for a real label, short enough that it cannot bloat
# the ini or the retained MQTT snapshot.
MAX_SAMPLE_ID_LEN = 128

# Re-entrant: the setters below call the getters while holding it.
_lock = threading.RLock()
_listeners = []

# See _write(): bounded retry for a transient Windows file-lock on os.replace.
_REPLACE_RETRIES = 5
_REPLACE_BACKOFF_S = 0.05


# -- ini plumbing -----------------------------------------------------------
def _read():
    # Locked, not just the writes: the server is multi-threaded (a fast ZMQ
    # command answers on its own thread while the worker thread is mid-workflow,
    # and both write state), and on Windows os.replace() below fails outright
    # with "Access is denied" if any other handle has the file open. A lock-free
    # read racing a write is therefore not a stale read, it is a crashed write.
    # _lock is re-entrant, so _mutate() can hold it across read-modify-write.
    with _lock:
        # interpolation=None: sample IDs are free text, and configparser's
        # default BasicInterpolation treats "%" in a *value* as a syntax error
        # -- raised not on write but on the next get()/items(), so one "%" in an
        # ID would poison every later read. Nothing here wants interpolation.
        parser = configparser.ConfigParser(interpolation=None)
        parser.read(_INI_PATH)
    return parser


def _write(parser):
    # Written on every transition, so use a temp file + replace: a crash
    # mid-write must not leave a half-truncated state file behind.
    tmp = _INI_PATH + ".tmp"
    with open(tmp, "w") as fp:
        parser.write(fp)
    for attempt in range(_REPLACE_RETRIES):
        try:
            os.replace(tmp, _INI_PATH)
            return
        except PermissionError:
            # Windows only, and not our own threads (those are serialized on
            # _lock): a virus scanner or the search indexer transiently holding
            # the file it just saw us create. Backing off briefly clears it;
            # losing a state transition to it would not.
            if attempt == _REPLACE_RETRIES - 1:
                raise
            time.sleep(_REPLACE_BACKOFF_S)


def _get(section, key, default=UNKNOWN):
    parser = _read()
    if not parser.has_option(section, key):
        return default
    value = parser.get(section, key).strip()
    return value or default


def _mutate(fn):
    """Apply ``fn(parser)`` to the stored ini: one read, one atomic write, one
    notification. Multi-key changes (clearing the carousel) have to land as a
    single transition -- a half-cleared inventory must never be observable, and
    listeners should see one snapshot, not one per key."""
    with _lock:
        parser = _read()
        fn(parser)
        _write(parser)
    _notify()


def _set(section, key, value):
    """Set one key, preserving every other section/key in the file."""
    def apply(parser):
        if not parser.has_section(section):
            parser.add_section(section)
        parser.set(section, key, str(value))
    _mutate(apply)


def _slot_keys(parser, section):
    """The ``slot_<n>`` keys of ``section`` as ``{int: raw string value}``."""
    if not parser.has_section(section):
        return {}
    out = {}
    for key, value in parser.items(section):
        if not key.startswith("slot_"):
            continue
        try:
            out[int(key.split("_", 1)[1])] = value
        except ValueError:
            continue  # ignore a hand-edited line we cannot parse
    return out


# -- change notification ----------------------------------------------------
def add_listener(fn):
    """Register ``fn(snapshot_dict)``, called after every change.

    server.py uses this to publish the tracking state over MQTT without every
    call site having to remember to do it. Listener errors are swallowed --
    telemetry must never break hardware control.
    """
    _listeners.append(fn)


def _notify():
    # Called with the lock released: a listener publishing over MQTT should
    # never be able to stall a state write.
    snap = snapshot()
    for fn in list(_listeners):
        try:
            fn(snap)
        except Exception as e:
            print("WARNING: state listener failed: %s" % e)


# -- A) mixer head ----------------------------------------------------------
def get_mixer_head():
    return _get(_TRACKING, WHAT_MIXER_HEAD)


def set_mixer_head(location):
    if location not in MIXER_LOCATIONS:
        raise ValueError("mixer head location must be one of %s, got %r"
                         % (list(MIXER_LOCATIONS), location))
    _set(_TRACKING, WHAT_MIXER_HEAD, location)


# -- B) flowcell locations --------------------------------------------------
def _flowcell_key(flowcell_id):
    fc = int(flowcell_id)
    if fc not in FLOWCELL_IDS:
        raise ValueError("flowcell id must be one of %s, got %r"
                         % (list(FLOWCELL_IDS), flowcell_id))
    return "flowcell_%d" % fc


def get_flowcell_location(flowcell_id):
    return _get(_TRACKING, _flowcell_key(flowcell_id))


def set_flowcell_location(flowcell_id, location):
    if location not in FC_LOCATIONS:
        raise ValueError("flowcell location must be one of %s, got %r"
                         % (list(FC_LOCATIONS), location))
    _set(_TRACKING, _flowcell_key(flowcell_id), location)


def set_location(what, location):
    """Operator reconcile: declare where something actually is.

    ``what`` is one of LOCATION_KEYS. This is the way out of ``unknown`` after
    a manual intervention -- deliberately a separate, explicit action rather
    than a force flag on the workflows.
    """
    if what == WHAT_MIXER_HEAD:
        set_mixer_head(location)
    elif what in (WHAT_FLOWCELL_1, WHAT_FLOWCELL_2):
        set_flowcell_location(int(what.rsplit("_", 1)[1]), location)
    else:
        raise ValueError("cannot set %r; expected one of %s"
                         % (what, list(LOCATION_KEYS)))


# -- C) carousel ------------------------------------------------------------
def get_carousel_slot():
    """The slot last moved to, as an int, or UNKNOWN. Int rather than the raw
    ini string so the JSON snapshot types it like flowcell_in_use."""
    value = _get(_TRACKING, "carousel_slot", UNKNOWN)
    if value == UNKNOWN:
        return UNKNOWN
    try:
        return int(value)
    except ValueError:
        return UNKNOWN


def set_carousel_slot(slot):
    _set(_TRACKING, "carousel_slot", int(slot))


# -- carousel geometry (configuration, not state) ---------------------------
# Read from json/palmixer_config.json on every call rather than cached, so
# editing the file and restarting only the *server* is enough -- and so a test
# can point config elsewhere without this module holding a stale copy.
def get_carousel_size():
    """How many slots the mounted carousel has, as an int, or UNKNOWN.

    UNKNOWN when ``carousel.size`` is absent or not a positive integer.
    Carousels differ, so nothing is guessed: slot operations refuse until it is
    configured rather than silently accepting a slot the hardware lacks."""
    try:
        size = int(config.get_section("carousel").get("size", 0))
    except (TypeError, ValueError):
        return UNKNOWN
    return size if size >= 1 else UNKNOWN


def get_carousel_step():
    """Motor travel between two adjacent slots, or UNKNOWN.

    May be negative, for a carousel whose slot numbering runs against the
    motor's positive direction. Zero is not a step, it is an absent one: every
    slot would map to the same position, so it reads as UNKNOWN and the moves
    that need it refuse rather than driving to the wrong slot."""
    try:
        step = float(config.get_section("carousel").get("step", 0.0))
    except (TypeError, ValueError):
        return UNKNOWN
    return step if step else UNKNOWN


def require_carousel_step():
    step = get_carousel_step()
    if step == UNKNOWN:
        raise ValueError("carousel step is not configured; set carousel.step in "
                         "json/palmixer_config.json to the motor travel between "
                         "two adjacent slots")
    return step


def validate_slot(slot):
    """Range-check ``slot`` against the configured size; return it as an int.

    Public because callers want to reject a bad slot *before* doing work for
    it -- server.py checks here before reading the motor, so an out-of-range
    slot reports that rather than whatever the hardware read had to say.

    Raises ValueError, which the ZMQ reply path turns into "ERROR: <reason>"
    and the workflows turn into a WorkflowError."""
    try:
        n = int(slot)
    except (TypeError, ValueError):
        raise ValueError("carousel slot must be an integer, got %r" % (slot,))
    size = get_carousel_size()
    if size == UNKNOWN:
        raise ValueError("carousel size is not configured; set carousel.size in "
                         "json/palmixer_config.json to the number of slots")
    if not 1 <= n <= size:
        raise ValueError("carousel slot must be 1..%d, got %d" % (size, n))
    return n


def carousel_reference():
    """The taught reference as ``(slot, position)``, or None if untaught.

    One reference locates the whole carousel: the slots are evenly spaced, so
    every other position follows from it and ``carousel.step``. A reference for
    a slot outside the configured size is stale -- a carousel of a different
    size was swapped in -- and reads as untaught rather than being trusted."""
    parser = _read()
    if not parser.has_section(_CAROUSEL):
        return None
    try:
        slot = int(parser.get(_CAROUSEL, "ref_slot", fallback=""))
        position = float(parser.get(_CAROUSEL, "ref_position", fallback=""))
    except ValueError:
        return None
    size = get_carousel_size()
    if size == UNKNOWN or not 1 <= slot <= size:
        return None
    return slot, position


def teach_carousel_slot(slot, position):
    """Record ``position`` (motor engineering units) as carousel slot ``slot``,
    and with it the position of every other slot.

    Teaching a second slot does not add to a table -- it replaces the
    reference, re-locating the whole carousel from the slot just taught."""
    ref_slot = validate_slot(slot)
    require_carousel_step()
    # Stamped with the carousel it was taught on, so a later mount can tell
    # whether this reference belongs to the hardware now in the machine.
    mounted = get_carousel_id()

    def apply(parser):
        if not parser.has_section(_CAROUSEL):
            parser.add_section(_CAROUSEL)
        parser.set(_CAROUSEL, "ref_slot", str(ref_slot))
        parser.set(_CAROUSEL, "ref_position", repr(float(position)))
        parser.set(_CAROUSEL, "ref_carousel_id", "" if mounted == UNKNOWN else mounted)
    _mutate(apply)


def slot_position(slot):
    """Motor position for ``slot``, derived from the reference and the step.

    Raises KeyError if the carousel has not been located yet -- the message is
    surfaced to the operator verbatim, so it says what to do about it."""
    n = validate_slot(slot)
    reference = carousel_reference()
    if reference is None:
        raise KeyError("the carousel has not been taught: drive the motor to any "
                       "slot and record it with \"teach_carousel_slot <n>\", and "
                       "every other slot follows from carousel.step")
    ref_slot, ref_position = reference
    return ref_position + (n - ref_slot) * require_carousel_step()


def carousel_slots():
    """The full slot -> motor position table, as ``{int: float}``.

    Derived, not stored: once the reference is taught every slot has a
    position, so this is either empty or complete."""
    size = get_carousel_size()
    if size == UNKNOWN or carousel_reference() is None:
        return {}
    try:
        return {n: slot_position(n) for n in range(1, size + 1)}
    except (KeyError, ValueError):
        return {}


# -- carousel sample IDs ----------------------------------------------------
# A slot is "used" exactly when it holds a sample ID. Keeping those the same
# fact, rather than a separate used-flag beside an ID, means the two can never
# disagree about whether a slot has been consumed.
def new_sample_id():
    """The auto-assigned ID for a sample the operator did not name."""
    return time.strftime("S%Y%m%d-%H%M%S")


def clean_sample_id(sample_id):
    """Normalise and bounds-check an operator-supplied sample ID.

    Public for the same reason as validate_slot: a workflow validates the ID on
    the synchronous accept path, so a bad one is refused before a multi-minute
    sequence starts rather than failing at the marking step in the middle."""
    # Collapsing whitespace also strips the newlines and tabs that would
    # otherwise break the one-line-per-key ini format on the way back out.
    text = " ".join(str(sample_id).split())
    if not text:
        raise ValueError("sample ID must not be empty")
    if len(text) > MAX_SAMPLE_ID_LEN:
        raise ValueError("sample ID must be at most %d characters, got %d"
                         % (MAX_SAMPLE_ID_LEN, len(text)))
    if text.lower() == UNKNOWN:
        raise ValueError('%r is not a usable sample ID: it is what get_sample_id '
                         'reports for a slot that has none' % UNKNOWN)
    return text


# -- which carousel is mounted ----------------------------------------------
def clean_carousel_id(carousel_id):
    """Normalise and bounds-check a carousel ID, as clean_sample_id does for samples."""
    text = " ".join(str(carousel_id).split())
    if not text:
        raise ValueError("carousel ID must not be empty")
    if len(text) > MAX_CAROUSEL_ID_LEN:
        raise ValueError("carousel ID must be at most %d characters, got %d"
                         % (MAX_CAROUSEL_ID_LEN, len(text)))
    if text.lower() == UNKNOWN:
        raise ValueError('%r is not a usable carousel ID: it is what an unmounted '
                         'carousel reports' % UNKNOWN)
    return text


def get_carousel_id():
    """The mounted carousel's ID, or UNKNOWN if nobody has said which one it is."""
    return _get(_TRACKING, "carousel_id", UNKNOWN)


def mount_carousel(carousel_id, samples=None):
    """Replace the mounted carousel: its ID and its whole slot inventory, atomically.

    One command rather than a clear followed by N tags, because a half-applied
    swap is the worst possible state: PALmixer's idea of what is in each slot
    would disagree with the physical carousel, and every sample measured
    afterwards would carry the wrong ID with nothing to reveal it.

    The taught reference is **kept**, but stamped with the carousel it was taught
    on -- see carousel_reference_stale(). Dropping it would mean re-teaching a
    slot on every batch; trusting it blindly would drive the robot to the wrong
    place if the new carousel does not seat identically. Stamping it lets the
    operator decide, once, in configuration.
    """
    text = clean_carousel_id(carousel_id)
    cleaned = {}
    for slot, sample_id in dict(samples or {}).items():
        cleaned[validate_slot(slot)] = clean_sample_id(sample_id)

    def apply(parser):
        if not parser.has_section(_TRACKING):
            parser.add_section(_TRACKING)
        parser.set(_TRACKING, "carousel_id", text)
        # Removed and rebuilt, not merged: a slot the new carousel does not use
        # must not inherit what the old one had in it.
        if parser.has_section(_CAROUSEL_SAMPLES):
            parser.remove_section(_CAROUSEL_SAMPLES)
        if cleaned:
            parser.add_section(_CAROUSEL_SAMPLES)
            for slot, sample_id in sorted(cleaned.items()):
                parser.set(_CAROUSEL_SAMPLES, "slot_%d" % slot, sample_id)
    _mutate(apply)
    return text


def reference_carousel_id():
    """Which carousel the taught reference was recorded on, or "" if unstamped."""
    return _get(_CAROUSEL, "ref_carousel_id", "")


def carousel_reference_stale():
    """True when the taught reference belongs to a different carousel than the
    mounted one.

    Not an error by itself -- a repeatable mount makes it harmless, which is what
    ``carousel.keep_reference_on_mount`` declares. Without that declaration the
    workflows refuse rather than drive to a position taught on other hardware.
    """
    if carousel_reference() is None:
        return False
    if _keep_reference_on_mount():
        return False
    taught_on = reference_carousel_id()
    mounted = get_carousel_id()
    if not taught_on or mounted == UNKNOWN:
        # Nothing to compare: a reference taught before carousel IDs existed, or
        # no carousel declared. Treated as usable -- this is the pre-existing
        # single-carousel way of working, which must keep working.
        return False
    return taught_on != mounted


def _keep_reference_on_mount():
    try:
        return bool(config.get_section("carousel").get("keep_reference_on_mount", False))
    except Exception:                                       # noqa: BLE001
        return False


def carousel_samples():
    """The slot -> sample ID table, as ``{int: str}``. Its keys are the used slots.

    Bounded by the configured size: if a smaller carousel is configured while
    IDs for higher slots are still on disk, those slots do not exist on the
    carousel that is mounted, and counting them would make it look full."""
    size = get_carousel_size()
    limit = size if size != UNKNOWN else 0
    return {slot: value for slot, value in _slot_keys(_read(), _CAROUSEL_SAMPLES).items()
            if value.strip() and 1 <= slot <= limit}


def get_sample_id(slot):
    """The sample ID held in ``slot``, or None if the slot is unused."""
    return carousel_samples().get(validate_slot(slot))


def set_sample_id(slot, sample_id):
    """Tag ``slot`` with ``sample_id``, marking it used. Returns the stored ID.

    Overwrites an existing ID: re-mixing into a slot that already holds a
    sample is allowed, and the new sample is what is in there afterwards."""
    text = clean_sample_id(sample_id)
    _set(_CAROUSEL_SAMPLES, "slot_%d" % validate_slot(slot), text)
    return text


def clear_sample_id(slot):
    """Drop ``slot``'s sample ID, marking it unused again -- the way back from a
    slot marked used by mistake, without resetting the whole carousel."""
    key = "slot_%d" % validate_slot(slot)

    def apply(parser):
        if parser.has_section(_CAROUSEL_SAMPLES):
            parser.remove_option(_CAROUSEL_SAMPLES, key)
    _mutate(apply)


def used_slot_count():
    return len(carousel_samples())


def carousel_is_full():
    """True when every slot holds a sample -- time to replace the carousel.

    Advisory only: make_sample still runs on a full carousel, overwriting the
    target slot's ID, because re-mixing a slot is a legitimate thing to do."""
    size = get_carousel_size()
    if size == UNKNOWN:
        return False
    return used_slot_count() >= size


def reset_carousel():
    """Replace the carousel: clear the entire slot inventory in one transition.

    Drops the taught reference along with the sample IDs, because a replacement
    carousel is not guaranteed to seat where the old one did -- keeping a stale
    reference would be the dangerous default, since every slot position is
    derived from it. The slot last moved to goes back to UNKNOWN for the same
    reason. The geometry (size, step) is configuration and is untouched."""
    def apply(parser):
        for section in (_CAROUSEL, _CAROUSEL_SAMPLES):
            if parser.has_section(section):
                parser.remove_section(section)
        for option in ("carousel_slot", "carousel_id"):
            if parser.has_option(_TRACKING, option):
                parser.remove_option(_TRACKING, option)
    _mutate(apply)


# -- D) flowcell in use -----------------------------------------------------
def get_flowcell_in_use():
    value = _get(_TRACKING, "flowcell_in_use", "1")
    try:
        fc = int(value)
    except ValueError:
        return 1
    return fc if fc in FLOWCELL_IDS else 1


def set_flowcell_in_use(flowcell_id):
    fc = int(flowcell_id)
    if fc not in FLOWCELL_IDS:
        raise ValueError("flowcell id must be one of %s, got %r"
                         % (list(FLOWCELL_IDS), flowcell_id))
    _set(_TRACKING, "flowcell_in_use", fc)


# -- the whole picture ------------------------------------------------------
def snapshot():
    """Everything tracked, as a plain dict. This is what goes on the wire.

    Assembled under the lock: each getter below re-reads the file, so without
    it a write landing mid-assembly would produce a snapshot whose fields come
    from different versions of the state -- carousel_full disagreeing with the
    carousel_samples in the very same payload, for instance."""
    with _lock:
        return _snapshot()


def _snapshot():
    return {
        WHAT_MIXER_HEAD: get_mixer_head(),
        WHAT_FLOWCELL_1: get_flowcell_location(1),
        WHAT_FLOWCELL_2: get_flowcell_location(2),
        "flowcell_in_use": get_flowcell_in_use(),
        "carousel_slot": get_carousel_slot(),
        "carousel_slots": carousel_slots(),
        "carousel_size": get_carousel_size(),
        "carousel_step": get_carousel_step(),
        "carousel_reference": carousel_reference(),
        "carousel_samples": carousel_samples(),
        # Which physical carousel is in the machine, and whether the taught
        # reference belongs to it. `stale` is what workflows refuse on: every
        # slot position is derived from that reference, so using one taught on
        # different hardware drives the robot to the wrong place.
        "carousel_id": get_carousel_id(),
        "reference_carousel": reference_carousel_id(),
        "reference_stale": carousel_reference_stale(),
        # Derivable from the two above, but carried explicitly so every
        # consumer of the retained MQTT snapshot -- not just this repo's GUI --
        # gets the "replace the carousel" signal without recomputing it.
        "carousel_full": carousel_is_full(),
    }
