package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DriveTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.TurnTask;
import frc.robot.autonomous.tasks.WaitTask;

public class TestMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  public void queueTasks() {
    // queueTask(new GripperTask(true));
    // queueTask(new WaitTask(2));
    // queueTask(new GripperTask(false));
    // queueTask(new WaitTask(5));
    // queueTask(new DriveTask(0.75, 2));
    // queueTask(new WaitTask(3));
    queueTask(new TurnTask(-1, 20));
  }
}
