package frc.robot;

public class Constants {
  // PCM
  public static final int kPCMId = 0;
  public static final int kIntakeSolenoidForwardId = 2;

  // DIO
  public static final int kExtensionUpperLimitPort = 2;
  public static final int kExtensionLowerLimitPort = 1;
  public static final int kPivotLowerLimitPort = 0;

  // Intake
  public static final int kIntakeMasterId = 14;

  // Elevator
  public static final int kElevatorPivotMotorId = 10;
  public static final int kElevatorExtensionMotorId = 9;

  // Drivetrain
  public static final int kDrivetrainFLMotorId = 5;
  public static final int kDrivetrainBLMotorId = 6;
  public static final int kDrivetrainFRMotorId = 7;
  public static final int kDrivetrainBRMotorId = 8;

  // Operator set points
  public static final double kPivotGroundCount = -145;
  public static final double kPivotReadyToScoreCount = -80;
  public static final double kPivotScoreCount = -40;
  public static final double kPivotStowCount = -10;
}
