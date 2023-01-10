package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
  private static final double kPivotBoostAmount = -15;

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
    mPivotMotor = new CANSparkMax(Constants.kElevatorPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotPIDController = mPivotMotor.getPIDController();
    mPivotEncoder = mPivotMotor.getEncoder();

    mPivotPIDController.setP(0.1);
    // mPivotPIDController.setI(0);
    mPivotPIDController.setI(1e-8);
    mPivotPIDController.setD(1);
    mPivotPIDController.setIZone(0);
    mPivotPIDController.setFF(0);
    mPivotPIDController.setOutputRange(kPivotPowerIn, kPivotPowerOut);

    mExtensionMotor = new SimulatableCANSparkMax(Constants.kElevatorExtensionMotorId,
        MotorType.kBrushless);
    mExtensionMotor.restoreFactoryDefaults();
    mExtensionMotor.setIdleMode(IdleMode.kBrake);
    mExtensionMotor.setInverted(true);
    mExtensionEncoder = mExtensionMotor.getEncoder();

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    double pivot_power = 0.0;
    double pivot_target = 0.0;
    boolean is_pivot_pos_control = false;
    boolean is_pivot_boosted = false;

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
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotGroundCount;
  }

  public void goToScore() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotScoreCount;
  }

  public void goToPreScore() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotPreScoreCount;
  }

  public void goToStow() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotStowCount;
  }

  public void boostPivot(boolean boost) {
    mPeriodicIO.is_pivot_boosted = boost;
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void writePeriodicOutputs() {
    if (mPeriodicIO.is_pivot_pos_control) {
      if (mPeriodicIO.is_pivot_boosted) {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotBoostAmount,
            CANSparkMax.ControlType.kPosition);
      } else {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotBoostAmount,
            CANSparkMax.ControlType.kPosition);
      }
    } else {
      mPivotMotor.set(mPeriodicIO.pivot_power);
    }

    mExtensionMotor.set(mPeriodicIO.extension_power);
  }

  @Override
  public void stop() {
    stopPivot();
    stopExtension();
  }

  public void stopPivot() {
    if (!mPeriodicIO.is_pivot_pos_control) {
      mPeriodicIO.pivot_power = 0.0;
      mPeriodicIO.is_pivot_pos_control = false;

      mPivotMotor.set(0.0);
    }
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
