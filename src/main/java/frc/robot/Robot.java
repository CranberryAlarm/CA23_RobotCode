package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.controllers.DriverController;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;

public class Robot extends TimedRobot {
  private final DriverController m_driverController = new DriverController(0, true, true);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Robot subsystems
  private final Drivetrain m_drive = new Drivetrain();
  // private final Intake m_intake = Intake.getInstance();

  //
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();

  private final String trajectoryJSON = "Back-up.wpilib.json";
  private Trajectory m_trajectory;

  private final Field2d m_field = new Field2d();

  @Override
  public void robotInit() {
    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);

    // Use the pathweaver trajectory
    try {
      Path trajectoryPath = null;

      if (RobotBase.isReal()) {
        System.out.println("Running on the robot!");
        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + trajectoryJSON);
      } else {
        System.out.println("Running in simulation!");
        trajectoryPath = Filesystem.getLaunchDirectory().toPath().resolve("PathWeaver/output/" + trajectoryJSON);
      }

      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      m_field.getObject("traj").setTrajectory(m_trajectory);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
    // m_intake.periodic();
  }

  @Override
  public void autonomousInit() {
    // m_timer.reset();
    // m_timer.start();

    // m_drive.resetOdometry(m_trajectory.getInitialPose());
    // m_field.setRobotPose(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    // double elapsed = m_timer.get();
    // Trajectory.State reference = m_trajectory.sample(elapsed);
    // ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    // m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_speedLimiter.calculate(m_driverController.getFilteredAxis(1)) * Drivetrain.kMaxSpeed;
    System.out.println(m_driverController.getFilteredAxis(1));

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -m_rotLimiter.calculate(m_driverController.getFilteredAxis(2)) * Drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, rot);

    // Intake controls
    // if (m_driverController.getWantsIntake()) {
    // m_intake.setSystemState(Intake.SystemState.INTAKING);
    // } else if (m_driverController.getWantsExhaust()) {
    // m_intake.setSystemState(Intake.SystemState.EXHAUSTING);
    // } else {
    // m_intake.setSystemState(Intake.SystemState.IDLE);
    // }
  }

  @Override
  public void simulationPeriodic() {
    updateSim();
  }

  @Override
  public void disabledInit() {
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
