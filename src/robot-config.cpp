#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Lmotor = motor(PORT14, ratio18_1, false);
motor Rmotor = motor(PORT17, ratio18_1, true);
encoder Ltrack = encoder(Brain.ThreeWirePort.G);
encoder Rtrack = encoder(Brain.ThreeWirePort.A);
encoder TurnTrack = encoder(Brain.ThreeWirePort.E);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}