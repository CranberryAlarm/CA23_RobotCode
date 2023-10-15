package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.AutoRunner.AutoMode;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;

import java.io.IOException;
import java.nio.file.Path;

public class Robot extends TimedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private final Drivetrain m_drive = Drivetrain.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private UsbCamera m_camera;

  //
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();

  private final String trajectoryJSON = "Back-up.wpilib.json";
  private Trajectory m_trajectory;

  private AutoRunner m_autoRunner = AutoRunner.getInstance();
  private Task m_currentTask;

  private final Field2d m_field = new Field2d();

  private final PoseEstimator m_poseEstimator = PoseEstimator.getInstance();
  private Limelight m_rightLL = new Limelight("LL_R");
  private Limelight m_leftLL = new Limelight("LL_L");

  @Override
  public void robotInit() {
    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);

    // Camera server
    m_camera = CameraServer.startAutomaticCapture();
    m_camera.setFPS(30);
    m_camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
    m_intake.periodic();
    m_poseEstimator.periodic();

    if (m_elevator.getHomeState() == 0) {
      if (m_elevator.isLimitStartedPushed()) {
        m_elevator.setHomingState(1);
      } else {
        m_elevator.setHomingState(2);
      }
    }
    m_elevator.writePeriodicOutputs();

    if (m_rightLL.seesAprilTag() && m_leftLL.seesAprilTag()) {
      m_poseEstimator.addVisionMeasurement(m_rightLL.getBotpose2D());
      m_poseEstimator.addVisionMeasurement(m_leftLL.getBotpose2D());
    }
    if (m_rightLL.seesAprilTag() && m_leftLL.seesAprilTag()) {
      m_poseEstimator.addVisionMeasurement(m_rightLL.getBotpose2D());
      m_poseEstimator.addVisionMeasurement(m_leftLL.getBotpose2D());
    }

    m_field.setRobotPose(m_poseEstimator.getPose());

    m_elevator.outputTelemetry();
    m_poseEstimator.outputTelemetry();
  }

  @Override
  public void autonomousInit() {
    m_drive.brakeOff();

    m_autoRunner.setAutoMode(AutoMode.TEST_MODE);
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_speedLimiter.calculate(-m_driverController.getTurnAxis()) *
        Drivetrain.kMaxSpeed;

    // // Get the rate of angular rotation. We are inverting this because we want a
    // // positive value when we pull to the left (remember, CCW is positive in
    // // mathematics). Xbox controllers return positive values when you pull to
    // // the right by default.
    m_drive.slowMode(m_driverController.getWantsSlowMode());
    m_drive.speedMode(m_driverController.getWantsSpeedMode());
    double rot = -m_rotLimiter.calculate(-m_driverController.getForwardAxis()) *
        Drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, rot);

    // // Intake controls
    if (m_driverController.getWantsIntakeOpen()) {
      m_intake.open();
    } else if (m_driverController.getWantsIntakeClose()) {
      m_intake.close();
    }

    // Elevator controls
    // if (m_driverController.getWantsExtend()) {
    //   m_elevator.extend();
    // } else if (m_driverController.getWantsRetract()) {
    //   m_elevator.retract();
    // }
    if (m_operatorController.getWantsExtensionStow()) {
      m_elevator.goToExtensionStow();
    } else if (m_operatorController.getWantsExtensionMidGoal()) {
      m_elevator.goToExtensionMidGoal();
    } else if (m_operatorController.getWantsExtensionHighGoal()) {
      m_elevator.goToExtensionHighGoal();
    } else {
      m_elevator.stopExtension();
    }

    // Pivot controls
    m_elevator.antiBoostPivot(m_operatorController.getWantsPivotAntiBoost());
    m_elevator.boostPivot(m_operatorController.getWantsPivotBoost() || m_driverController.getWantsPivotBoost());

    // if (m_driverController.getWantsLower()) {
    // m_elevator.lower();
    // } else if (m_driverController.getWantsRaise()) {
    // m_elevator.raise();
    if (m_operatorController.getWantsGroundPosition()) {
      m_elevator.goToPivotGround();
    } else if (m_operatorController.getWantsPreGoalPosition()) {
      m_elevator.goToPivotPreScore();
    } else if (m_operatorController.getWantsScorePosition()) {
      m_elevator.goToPivotScore();
    } else if (m_operatorController.getWantsStowPosition()) {
      m_elevator.goToPivotStow();
    } else if (m_operatorController.getWantsResetPivotEncoder()) {
      m_elevator.resetPivotEncoder();
    } else {
      if (m_elevator.getHomeState() == 3) {
        m_elevator.stopPivot();
      }
    }

    if (m_driverController.getWantsAutoScore()) {
      m_intake.open();
      m_elevator.goToExtensionStow();
      m_elevator.goToPivotScore();
    }

    m_elevator.periodic();
    m_elevator.outputTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }

  @Override
  public void disabledInit() {
    m_intake.stop();
    m_elevator.stop();
  }

  @Override
  public void disabledPeriodic() {
    // Stop the robot when disabled.
    m_drive.drive(0.0, 0.0);

    updateSim();
  }

  private void updateSim() {
    // Update the odometry in the sim.
    m_drive.simulationPeriodic();
    m_field.setRobotPose(m_drive.getPose());
  }
}
