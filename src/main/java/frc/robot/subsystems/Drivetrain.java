// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Drivetrain {
  // 3 meters per second.
  public static final double kMaxSpeed = 1.0;

  // 3 meters per second.
  public static final double kMaxAcceleration = 2.0;

  // 1/2 rotation per second.
  public static final double kMaxAngularSpeed = Math.PI * 0.7;

  private static final double kTrackWidth = Units.inchesToMeters(22.0);
  private static final double kWheelRadius = Units.inchesToMeters(3.0);
  private static final double kGearRatio = 10.61;
  private static final int kEncoderResolution = 42;
  private static final double kMetersPerEncTick = (2 * Math.PI * kWheelRadius) / (kEncoderResolution * kGearRatio);

  private static final double kSlowModeRotScale = 0.1;

  private final CANSparkMax m_leftLeader = new CANSparkMax(Constants.kDrivetrainFLMotorId,
      MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(Constants.kDrivetrainBLMotorId,
      MotorType.kBrushless);
  private final CANSparkMax m_rightLeader = new CANSparkMax(Constants.kDrivetrainFRMotorId,
      MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(Constants.kDrivetrainBRMotorId,
      MotorType.kBrushless);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final PIDController m_leftPIDController = new PIDController(4, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(4, 0, 0);

  private final AHRS m_gyro = new AHRS();

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_leftFeedforward = new SimpleMotorFeedforward(1, 2);
  private final SimpleMotorFeedforward m_rightFeedforward = new SimpleMotorFeedforward(1, 2);

  // Simulation classes help us simulate our robot
  // private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  // private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  // private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
      m_drivetrainSystem, DCMotor.getCIM(2), kGearRatio, kTrackWidth, kWheelRadius, null);

  public Drivetrain() {
    m_leftLeader.restoreFactoryDefaults();
    m_leftLeader.setIdleMode(IdleMode.kCoast);
    m_leftFollower.restoreFactoryDefaults();
    m_leftFollower.setIdleMode(IdleMode.kCoast);
    m_rightLeader.restoreFactoryDefaults();
    m_rightLeader.setIdleMode(IdleMode.kCoast);
    m_rightFollower.restoreFactoryDefaults();
    m_rightFollower.setIdleMode(IdleMode.kCoast);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true);

    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setPositionConversionFactor(kMetersPerEncTick);
    m_rightEncoder.setPositionConversionFactor(kMetersPerEncTick);
    m_leftEncoder.setVelocityConversionFactor(kMetersPerEncTick);
    m_rightEncoder.setVelocityConversionFactor(kMetersPerEncTick);

    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
        m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    SmartDashboard.putData("Field", m_fieldSim);
  }

  private boolean m_slowMode = false;

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // var leftFeedforward =
    // m_leftFeedforward.calculate(speeds.leftMetersPerSecond);
    // var rightFeedforward =
    // m_rightFeedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(-m_rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

    System.out.println("LeftSet: " + speeds.leftMetersPerSecond);
    System.out.println("LeftActual: " + m_leftEncoder.getVelocity());
    System.out.println("RightSet: " + speeds.rightMetersPerSecond);
    System.out.println("RightActual: " + -m_rightEncoder.getVelocity());

    // m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    // m_rightGroup.setVoltage(rightOutput + rightFeedforward);

    m_leftGroup.setVoltage(leftOutput);
    m_rightGroup.setVoltage(rightOutput);

    // m_leftGroup.setVoltage(speeds.leftMetersPerSecond);
    // m_rightGroup.setVoltage(speeds.rightMetersPerSecond);
  }

  public void slowMode(boolean slow) {
    m_slowMode = slow;
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis
   * @param rot    the rotation
   */
  public void drive(double xSpeed, double rot) {
    if (m_slowMode) {
      setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot * kSlowModeRotScale)));
    } else {
      setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
    m_drivetrainSimulator.setPose(pose);

    m_odometry.resetPosition(pose.getRotation(), m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition(), pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_leftGroup.get() * RobotController.getInputVoltage(),
        m_rightGroup.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    // m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    // m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    // m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    // m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
    updateOdometry();
    m_fieldSim.setRobotPose(getPose());
  }
}
