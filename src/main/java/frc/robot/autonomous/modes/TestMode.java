package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.DriveTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.autonomous.tasks.Elevator.ElevatorTask;
import frc.robot.autonomous.tasks.Elevator.ExtendTask;

public class TestMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  public void queueTasks() {
    // queueTask(new ParallelTask(
    queueTask(new ExtendTask(true));
    queueTask(new ElevatorTask(0.2, 2));
    queueTask(new GripperTask(true));
    queueTask(new WaitTask(1));
    queueTask(new ExtendTask(false));
    // new ParallelTask(
    queueTask(new DriveTask(0.5, 5));
    queueTask(new ElevatorTask(0.2, 3));
  }
}
