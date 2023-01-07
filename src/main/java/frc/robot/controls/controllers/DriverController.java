package frc.robot.controls.controllers;

public class DriverController extends FilteredController {
  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  public boolean getWantsIntake() {
    return this.getBButton();
  }

  public boolean getWantsExhaust() {
    return this.getXButton();
  }
}
