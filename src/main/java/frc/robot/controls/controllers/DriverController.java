package frc.robot.controls.controllers;

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
    return this.getFilteredAxis(2);
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
