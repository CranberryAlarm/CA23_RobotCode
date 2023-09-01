package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;

public class TurnTask extends Task {

  private Drivetrain m_drive = Drivetrain.getInstance();
  private double m_radPerSec, m_time, m_startTime;
  private boolean m_finished = false;

  public TurnTask(double radPerSec, double time) {
    m_radPerSec = radPerSec;
    m_time = time;
  }

  @Override
  public void start() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {
    if (Timer.getFPGATimestamp() <= m_startTime + m_time && !m_finished) {
      m_drive.drive(0, m_radPerSec);
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
