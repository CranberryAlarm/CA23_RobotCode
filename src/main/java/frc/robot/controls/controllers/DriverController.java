package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.RobotBase;

public class DriverController extends FilteredController {
  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Drive
  public double getForwardAxis() {
    return this.getFilteredAxis(1);
  }

  public double getTurnAxis() {
    if (RobotBase.isReal()) {
      return this.getFilteredAxis(4);
    } else {
      return this.getFilteredAxis(2);
    }
  }

  // Intake
  public boolean getWantsIntakeOpen() {
    return this.getLeftBumper();
  }

  public boolean getWantsIntakeClose() {
    return this.getRightBumper();
  }

  // Elevator
  public boolean getWantsExtend() {
    return this.getXButton();
  }

  public boolean getWantsRetract() {
    return this.getBButton();
  }

  public boolean getWantsRaise() {
    return this.getAButton();
  }

  public boolean getWantsLower() {
    return this.getYButton();
  }

}

// a is pivot up (should be Y)
// b is extend down (should be A)
// x is extend up (should be B)
// y is pivot down (should be X)
// set pivot to brake
// set extend to brake

// public boolean getWantsExtend() {
// return this.getBButton();
// }

// public boolean getWantsRetract() {
// return this.getAButton();
// }

// public boolean getWantsRaise() {
// return this.getYButton();
// }

// public boolean getWantsLower() {
// return this.getXButton();
// }
