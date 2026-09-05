#ifndef PATHS_HPP
#define PATHS_HPP

// Autonomous routines. runAutonomous() dispatches to one of them based on
// currentStartingPos, which the driver sets from the controller before a match.

namespace Paths {
    void path1();
    void path2();
    void path3();
    void path4();
    void runAutonomous();
}

#endif
