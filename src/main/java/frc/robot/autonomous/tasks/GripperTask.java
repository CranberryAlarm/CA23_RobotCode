package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Intake;

public class GripperTask extends Task {
  private Intake m_intake = Intake.getInstance();
  private boolean m_open;
  private boolean m_finished = false;

  public GripperTask(boolean open) {
    m_open = open;
  }

  @Override
  public void start() {
    System.out.println("Gripper task start");
  }

  @Override
  public void update() {
    if (m_open) {
      m_intake.open();
    } else {
      m_intake.close();
    }
    m_finished = true;
    System.out.println("Gripper task completed!");
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
