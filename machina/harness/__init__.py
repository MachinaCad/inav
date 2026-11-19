"""machina's harness for this INAV fork: the fast checks of machina/README.md.

Frames follow machina's convention (closed-loop-plan.md C2): ENU world and FLU body, SI units,
integer-nanosecond ticks. INAV's own conventions appear only in `frames` (the wire layouts of
src/main/fc/machina_sim.h, and the X-Plane drefs of the G0 probe) and in `sitl.InavState`.
"""
