package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;

public class DriveTask extends Task {

  private Drivetrain m_drive = Drivetrain.getInstance();
  private double m_speed, m_time, m_startTime;
  private boolean m_finished = false;

  public DriveTask(double speed, double time) {
    m_speed = speed;
    m_time = time;
  }

  @Override
  public void start() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {
    if (Timer.getFPGATimestamp() <= m_startTime + m_time && !m_finished) {
      m_drive.drive(m_speed, 0);
    } else {
      m_drive.drive(0, 0);
      m_finished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }

}
