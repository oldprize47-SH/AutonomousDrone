//
// PID Controller Implementation - Cascaded Architecture
//
// NOTE: All controller *instances* (attitude + altitude) are now owned by the
// Autopilot module (Autopilot.cpp). This translation unit no longer defines any
// globals; the controller *struct definitions* live in PIDControl.h and are
// header-only. Kept as a placeholder in case shared non-inline helpers are
// added later.
//

#include "PIDControl.h"
