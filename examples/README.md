This folder contains several serialized ContactSequence objects for different scenarios. They are all made for the Talos humanoid robot, and a simple environment with a flat floor at z=0.

All this file have been generated with the [multicontact-locomotion-planning](https://github.com/loco-3d/multicontact-locomotion-planning) framework.

## Loading the files:

### C++

```c
#include "multicontact-api/scenario/contact-sequence.hpp"


ContactSequence cs;
cs.loadFromBinary(filename);
```

### Python

```python
from multicontact_api import ContactSequence

cs = ContactSequence()
cs.loadFromBinary(filename)
```

## Prefix notation

For the same scenario, several files may exist with different prefixes, here is the meaning of this prefixes:

* No prefix: only contains the contacts placements, the initial and final CoM position and the initial and final wholeBody configuration. 
* _COM prefix: also contains the phases duration and the centroidal trajectories (c, dc, ddc, L, dL)
* _REF prefix: also contains the end-effector trajectories for each swing phases
* _WB prefix: also contains all the WholeBody data (q, dq, ddq, tau, contact forces). 
Note that the centroidal and end-effector trajectories contained in this file are not exactly the same as the ones in the _REF file, they are computed from the wholeBody motion.

## Scenarios

### com_motion_above_feet


### step_in_place_quasistatic


### step_in_place


### walk_10cm

