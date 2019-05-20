# Multicontact API

[![Pipeline status](https://gepgitlab.laas.fr/loco-3d/multicontact-api/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/loco-3d/multicontact-api/commits/master)
[![Coverage report](https://gepgitlab.laas.fr/loco-3d/multicontact-api/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/loco-3d/multicontact-api/master/coverage/)


This package is extracted from an original work of Justin Carpentier (jcarpent@laas.fr),
with the goal to simplify the library and remove old dependencies.

This package installs a python module used to define, store and use ContactSequence objects. 

Basic usage of the ContactSequence object is to initialize either an empty sequence of given size : 

``` Python
cs = ContactSequenceHumanoid(num_contact_phases)
```

Or load it from file : 

``` Python
cs = ContactSequenceHumanoid(0)
cs.loadFromXML(filename, "ContactSequence") # From an XML file
cs.loadFromBinary(filename) # From a binary file (with .cs extension)
```

This ContactSequence object store a sequence of ContactPhases, defining a set of active contact with their placement. 
A ContactPhase can also store centroidal data : state trajectory (c,dc,L) and control trajectory (ddc,dL). 
This trajectories are stored as a set of discretized points, the time corresponding to each point can be found in 'time trajectory'.

For example, to access the position of the center of mass during the first contact phase of a motion :  

``` Python
phase = cs.contact_phases[0]
for k in range(len(phase.state_trajectory)):
    print "c ("+str(phase.time_trajectory[k])+") = "+str(phase.state_trajectory[k][0:3])
    
```
