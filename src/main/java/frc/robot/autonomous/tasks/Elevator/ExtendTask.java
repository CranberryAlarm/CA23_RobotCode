package frc.robot.autonomous.tasks.Elevator;

import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.Elevator;

public class ExtendTask extends Task {
  private Elevator m_elevator = Elevator.getInstance();
  private boolean m_extend, m_finished;

  public ExtendTask(boolean extend) {
    m_extend = extend;
  }

  @Override
  public void start() {

  }

  @Override
  public void update() {
    if (m_extend) {
      m_elevator.goToExtensionHighGoal();
    } else {
      m_elevator.goToExtensionStow();
    }
    m_finished = true;
    System.out.println("Extend Task Finished!");
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }

}
