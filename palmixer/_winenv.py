# -*- coding: utf-8 -*-
"""Make a conda environment's shared libraries findable when it was not activated.

Imported for its effect by palmixer/__init__.py, so every entry point in this
package gets it before anything else runs.
"""

import os
import sys


def ensure_conda_dll_path():
    """Put ``<sys.prefix>/Library/bin`` on PATH on Windows, as activation would.

    Starting a conda environment by its interpreter path -- what a service
    wrapper, an IDE run configuration, or a bare
    ``...\\envs\\aps12robot\\python.exe -m palmixer.server`` all do -- skips the
    activation script, so ``<env>\\Library\\bin`` never reaches PATH. Almost
    everything still works, which is what makes the one thing that doesn't so
    unpleasant:

    conda-forge's numpy is linked against MKL; MKL delay-loads
    ``libiomp5md.dll``; and in these environments that file is a *forwarder*
    onto ``libomp.dll`` (llvm-openmp ships both, with identical export tables).
    Loading the forwarder succeeds. Resolving an export *through* it sends the
    loader off to find ``libomp.dll`` by the default search order, which does
    not include ``Library\\bin``. ``GetProcAddress`` then fails with
    ERROR_PROC_NOT_FOUND and the delay-load helper raises 0xC06D007F, which
    kills the process outright: no Python traceback, no exception to catch,
    and nothing in the log but the last line that happened to be flushed.

    It also fires at the first LAPACK call rather than at import, and for this
    package that call is ``np.linalg.inv()`` inside
    ``math3d.Transform.get_inverse()``, reached from urx's ``get_pose()``. So
    the server starts, connects, answers commands, and then dies part-way
    through an AprilTag search with the arm in motion -- looking for all the
    world like a bug in the vision or motion code.

    ``os.add_dll_directory()`` is not a substitute: it only affects loads made
    with LOAD_LIBRARY_SEARCH_USER_DIRS, and forwarder resolution does not use
    that flag. PATH is what works.

    Returns the directory that was prepended, or None when there was nothing
    to do (not Windows, not a conda layout, or already activated).
    """
    if sys.platform != "win32":
        return None
    libbin = os.path.join(sys.prefix, "Library", "bin")
    if not os.path.isdir(libbin):
        return None                       # not a conda-style prefix
    wanted = os.path.normcase(os.path.normpath(libbin))
    path = os.environ.get("PATH", "")
    for entry in path.split(os.pathsep):
        if entry and os.path.normcase(os.path.normpath(entry)) == wanted:
            return None                   # already on PATH; env is activated
    os.environ["PATH"] = libbin + os.pathsep + path
    return libbin
