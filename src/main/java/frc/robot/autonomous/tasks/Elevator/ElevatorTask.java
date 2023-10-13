package frc.robot.autonomous.tasks.Elevator;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.Elevator;

public class ElevatorTask extends Task {
  private Elevator m_elevator = Elevator.getInstance();
  private boolean m_finished = false;
  private double m_power, m_time, m_startTime;
  private double m_currentTime = 0.0;

  public ElevatorTask(double power, double time) {
    m_power = power;
    m_time = time;
  }

  @Override
  public void start() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {
    if (Timer.getFPGATimestamp() <= m_startTime + m_time && !m_finished) {
      m_elevator.setPivotPower(m_power);
    } else {
      m_elevator.setPivotPower(0);
      m_finished = true;
      System.out.println("Elevator Task Finished!");
    }
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }

}
