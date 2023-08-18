package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Drivetrain;

public class BrakeTask extends Task {
  private Drivetrain m_drive;
  private boolean m_brake;

  public BrakeTask(boolean brake) {
    m_brake = brake;
    m_drive = Drivetrain.getInstance();
  }

  @Override
  public void start() {
    if (m_brake) {
      m_drive.brakeOn();
    } else {
      m_drive.brakeOff();
    }
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {
    m_drive.drive(0, 0);
  }
}
