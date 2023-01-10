package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Elevator extends Subsystem {
  private static final double kPivotPowerOut = 0.40;
  private static final double kPivotPowerIn = -0.4;
  private static final double kExtensionPowerOut = 0.35;
  private static final double kExtensionPowerIn = -0.35;

  private static Elevator mInstance;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private CANSparkMax mPivotMotor;
  private RelativeEncoder mPivotEncoder;
  private SparkMaxPIDController mPivotPIDController;

  private SimulatableCANSparkMax mExtensionMotor;
  private RelativeEncoder mExtensionEncoder;

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  private Elevator() {
    mPivotMotor = new CANSparkMax(Constants.kElevatorPivotMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotPIDController = mPivotMotor.getPIDController();
    mPivotEncoder = mPivotMotor.getEncoder();

    mPivotPIDController.setP(0.1);
    mPivotPIDController.setI(1e-4);
    mPivotPIDController.setD(1);
    mPivotPIDController.setIZone(0);
    mPivotPIDController.setFF(0);
    mPivotPIDController.setOutputRange(-1, 1);

    // OLD
    // mPivotMotor.restoreFactoryDefaults();
    // mPivotMotor.setIdleMode(IdleMode.kBrake);
    // mPivotEncoder = mPivotMotor.getEncoder();
    // mPivotPIDController = mPivotMotor.getPIDController();
    // mPivotPIDController.setFeedbackDevice(mPivotEncoder);
    // mPivotPIDController.setP(1);
    // mPivotPIDController.setI(1e-4);
    // mPivotPIDController.setD(1);
    // mPivotPIDController.setIZone(0);
    // mPivotPIDController.setFF(0);
    // mPivotPIDController.setOutputRange(-1, 1);

    mExtensionMotor = new SimulatableCANSparkMax(Constants.kElevatorExtensionMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mExtensionMotor.setIdleMode(IdleMode.kBrake);
    mExtensionMotor.setInverted(true);
    mExtensionEncoder = mExtensionMotor.getEncoder();

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    double pivot_power = 0.0;
    double pivot_target = 0.0;
    boolean is_pivot_pos_control = false;

    double extension_power = 0.0;
  }

  public void extend() {
    mPeriodicIO.extension_power = kExtensionPowerOut;
  }

  public void retract() {
    mPeriodicIO.extension_power = kExtensionPowerIn;
  }

  public void raise() {
    mPeriodicIO.is_pivot_pos_control = false;
    mPeriodicIO.pivot_power = kPivotPowerIn;
  }

  public void lower() {
    mPeriodicIO.is_pivot_pos_control = false;
    mPeriodicIO.pivot_power = kPivotPowerOut;
  }

  public void goToGround() {
    System.out.println("----------------------------------------");
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotGroundCount;
  }

  public void goToStow() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotStowCount;
  }

  @Override
  public void periodic() {
    // writePeriodicOutputs();
    System.out.println("=====================");
    mPivotPIDController.setReference(50, ControlType.kPosition);
  }

  @Override
  public void writePeriodicOutputs() {
    // System.out.println("WPO");

    if (mPeriodicIO.is_pivot_pos_control) {
      // System.out.println("POS");
      System.out.println(mPeriodicIO.pivot_target);

      mPivotPIDController.setReference(mPeriodicIO.pivot_target, CANSparkMax.ControlType.kPosition);
    } else {
      System.out.println("POW");
      mPivotMotor.set(mPeriodicIO.pivot_power);
    }

    System.out.println(mPivotMotor.getOutputCurrent());

    mExtensionMotor.set(mPeriodicIO.extension_power);
  }

  @Override
  public void stop() {
    stopPivot();
    stopExtension();
  }

  public void stopPivot() {
    mPeriodicIO.pivot_power = 0.0;
    mPeriodicIO.is_pivot_pos_control = false;

    mPivotMotor.set(0.0);
  }

  public void stopExtension() {
    mPeriodicIO.extension_power = 0.0;

    mExtensionMotor.set(0.0);
  }

  public void resetEncoders() {
    mPivotEncoder.setPosition(0);
  }

  @Override
  public void outputTelemetry() {
    // Pivot telemetry
    SmartDashboard.putNumber("Pivot motor power:", mPeriodicIO.pivot_power);
    SmartDashboard.putNumber("Pivot encoder count:", mPivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot PID target:", mPeriodicIO.pivot_power);

    // Extension telemetry
    SmartDashboard.putNumber("Extension motor power:", mPeriodicIO.extension_power);
    SmartDashboard.putNumber("Extension encoder count:", mExtensionEncoder.getPosition());
  }
}
