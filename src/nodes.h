// Shared global property nodes that any code module can read or write to.
//
// 1. This emulates the convenient aspects of simple publish and subscribe
//    system.
// 2. It allows separate code modules to be loosely bound (versus hardwired
//    C/function parameter passing that is often grown organtically/ad-hoc and
//    can become very convoluted.)
// 3. It provides some conveniences of scripted language system in connecting an
//    ascii name to a C variable.
// 4. It offers some typing flexibility similar to many scripting langagues.
// 5. It enables software developers to write much simpler, more concise, and
//    more readable code.  This is more important than many people realize.
// 6. It supports an easy way to manage config files by loading a json file
//    directly into the property tree where submodules can easily access the
//    configuration values.
//
// We all have been taught that global variables are bad, classes and
// abstractions are good. I propose (based on extensive real world experience)
// that many projects can benefit from a balanced approach: a structured and
// planned hierarchy of global variables, combined with a simple API to access
// this tree, used wisely with simple cooperative conventions about which
// modules "own" which data and can write to it.
//
// No approach works for every project, but the "property tree" system has been
// a valuable infrastructure tool for many of my projects over the years. Credit
// David Megginson for showing/teaching me the idea originally.
//
// We could structure the code so that each class or module that needs to access
// the property tree could define and initialize their own property nodes, but
// here we are creating a set of global nodes to save memory and lean into the
// idea that this is a shared structure for all modules to use cooperatively.
//
// Also note, it's no problem if a module wants to create a set of property
// nodes for it's own internal use, or create a node temporarily to access some
// value from the tree.  Just keep in mind there is overhead (cpu and memory
// usage) to creating new property nodes on the fly, so consider trying to
// structure your code so that allocations are only in the init() function.

#include "props2.h"

// Configuration
extern PropertyNode config_node;
extern PropertyNode config_L1_node;   // used in circle and route mgrs
extern PropertyNode config_tecs_node; // only used in tecs.cpp but useful info? fixme: convert tecs to a class for namespace protection
extern PropertyNode imu_calib_node;

// Sensors
extern PropertyNode airdata_node;
extern PropertyNode gps_node;
extern PropertyNode imu_node;
extern PropertyNode power_node;
extern PropertyNode rcin_node;

// INS/GNSS
extern PropertyNode nav_node;

// State
extern PropertyNode orient_node;
// extern PropertyNode pos_node;
extern PropertyNode vel_node;
extern PropertyNode wind_node;

// Inceptors, Effectors, Flight Control Commands
extern PropertyNode inceptors_node;
extern PropertyNode effectors_node;
extern PropertyNode controls_node;

// Status and Comms
extern PropertyNode comms_node;
extern PropertyNode status_node;

// Control Laws
extern PropertyNode fcs_node;
extern PropertyNode locks_node;
extern PropertyNode refs_node;
extern PropertyNode tecs_node;

// Mission and Tasks
extern PropertyNode mission_node;
extern PropertyNode circle_node;
extern PropertyNode home_node;
extern PropertyNode route_node;
extern PropertyNode task_node;

extern void PropertyNodes_init();
