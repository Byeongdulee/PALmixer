# -*- coding: utf-8 -*-
"""Make the bundled ``caRepeater`` findable before Channel Access starts.

On Windows, the first CA call prints::

    Failed to start executable - "caRepeater".
    The system cannot find the file specified.
    caStartRepeaterIfNotInstalled (): unable to start CA repeater daemon
    detached process

libca spawns the repeater by bare name, so it has to be on PATH. pyepics ships
``caRepeater.exe`` in the same directory as the ``ca.dll`` it loads, and does
prepend that directory to PATH -- but only in find_libca()'s last-resort branch
that searches PATH for the DLL. A normal install returns earlier, from the
bundled ``epics/clibs`` copy, so the directory is never added and the repeater
is never found.

Channel Access still works without the repeater -- it is what fans IOC beacons
out to the CA clients on a host -- but each client then misses beacons, so it
notices an IOC restart on its own slow timeout instead of at once, and every
process reprints the message above.

``ensure()`` is idempotent and must run before ``epics`` opens a CA context,
i.e. before the first caget/caput, which is why the two modules doing Channel
Access (motor.py, PAL12idb.py) call it next to their ``epics`` import. It is a
no-op off Windows, and stays silent if anything is unexpected: the goal is to
remove a warning, never to add a failure to a path that was working.
"""

import os

_done = False


def ensure():
    """Prepend pyepics' clibs directory to PATH so libca can spawn caRepeater."""
    global _done
    if _done:
        return
    _done = True                       # one attempt per process, success or not
    if os.name != "nt":
        return
    try:
        from epics.ca import find_libca

        clibs = os.path.dirname(str(find_libca()))
        # Only worth doing if the repeater really is there: with epicscorelibs,
        # or a PYEPICS_LIBCA pointing elsewhere, libca can come from a
        # directory that holds no caRepeater.exe.
        if not os.path.isfile(os.path.join(clibs, "caRepeater.exe")):
            return
        entries = os.environ.get("PATH", "").split(os.pathsep)
        if not any(os.path.normcase(e.strip()) == os.path.normcase(clibs)
                   for e in entries):
            os.environ["PATH"] = os.pathsep.join([clibs] + entries)
    except Exception:                  # pyepics missing, or its internals moved
        pass
