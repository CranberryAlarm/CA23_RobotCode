package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DriveTask;

public class GoForwardMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  @Override
  public void queueTasks() {
    queueTask(new DriveTask(1.0, 3));
  }
}
