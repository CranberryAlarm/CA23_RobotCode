package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {
  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Pivot
  public boolean getWantsGroundPosition() {
    return this.getRawButton(1);
  }

  public boolean getWantsScorePosition() {
    return this.getRawButton(2);
  }

  public boolean getWantsPreGoalPosition() {
    return this.getRawButton(3);
  }

  public boolean getWantsStowPosition() {
    return this.getRawButton(4);
  }

  public boolean getWantsPivotAntiBoost() {
    return this.getRawButton(6);
  }

  public boolean getWantsPivotBoost() {
    return this.getRawButton(5);
  }

  // Extension
  public boolean getWantsExtensionStow() {
    return this.getPOV() == 0;
  }
  
  public boolean getWantsExtensionMidGoal() {
    return this.getPOV() == 90;
  }
  
  public boolean getWantsExtensionHighGoal() {
    return this.getPOV() == 180;
  }

  // Reset Encoders
  public boolean getWantsResetPivotEncoder() {
    return this.getRawButton(8);
  }

  public boolean getWantsResetExtensionEncoder() {
    return this.getRawButton(7);
  }

  // public boolean getWantsScorePosition() {
  //   return this.getPOV() == 90;
  // }

  // public boolean getWantsPreGoalPosition() {
  //   return this.getPOV() == 270;
  // }

  // public boolean getWantsStowPosition() {
  //   return this.getPOV() == 0;
  // }

  // Elevator
  public boolean getWantsExtensionStow() {
    return this.getPOV() == 0;
  }

  public boolean getWantsExtensionMidGoal() {
    return this.getPOV() == 90;
  }

  public boolean getWantsExtensionHighGoal() {
    return this.getPOV() == 180;
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
