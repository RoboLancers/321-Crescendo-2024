/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.revrobotics.*;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import java.util.function.DoubleSupplier;
import org.robolancers321.util.InterpolationTable;

public class Pivot extends ProfiledPIDSubsystem {
  /*
   * Singleton
   */

  private static Pivot instance;

  public static Pivot getInstance() {
    if (instance == null) instance = new Pivot();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 0;

  private static final boolean kInvertMotor = false;
  private static final boolean kInvertEncoder = false;
  private static final int kCurrentLimit = 40;

  private static final double kGearRatio = 360.0;
  private static final double kRotPerMinToDegPerSec = kGearRatio / 60.0;

  private static final float kMinAngle = 0.0f;
  private static final float kMaxAngle = 270.0f;

  private static final double kS = 0.0;
  private static final double kG = 0.0;
  private static final double kV = 0.0;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kMaxVelocityDeg = 0.0;
  private static final double kMaxAccelerationDeg = 0.0;

  private static final double kToleranceDeg = 0.0;

  private static final double kInterpolationThreshold = 0.0;

  private double latest_distance = 0.0;

  private InterpolationTable.AimCharacteristic latest_characteristic;

  public enum PivotSetpoint {
    kRetracted(0.0),
    kMating(0.0),
    kSpeaker(0.0),
    kAmp(0.0);

    private double angle;

    PivotSetpoint(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return this.angle;
    }
  }

  /*
   * Implementation
   */

  private CANSparkMax motor;
  private AbsoluteEncoder absoluteEncoder;
  private ArmFeedforward feedforwardController;

  private Pivot() {
    super(
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocityDeg, kMaxAccelerationDeg)));

    this.motor = new CANSparkMax(kMotorPort, kBrushless);
    this.absoluteEncoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedforwardController = new ArmFeedforward(kS, kG, kV);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);

    this.motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) kMaxAngle);
    this.motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) kMinAngle);
    this.motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    this.motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
  }

  private void configureEncoder() {
    this.absoluteEncoder.setInverted(kInvertEncoder);
    this.absoluteEncoder.setPositionConversionFactor(kGearRatio);
    this.absoluteEncoder.setVelocityConversionFactor(kRotPerMinToDegPerSec);
  }

  @Override
  public double getMeasurement() {
    return getVelocityRad();
  }

  private void configureController() {
    super.m_controller.setTolerance(kToleranceDeg);
  }

  public double getPositionDeg() {
    return this.absoluteEncoder.getPosition();
  }

  public double getPositionRad() {
    return Math.toRadians(this.getPositionDeg());
  }

  public double getVelocityDeg() {
    return this.absoluteEncoder.getVelocity();
  }

  public double getVelocityRad() {
    return Math.toRadians(this.getVelocityDeg());
  }

  private boolean atGoal() {
    return super.m_controller.atGoal();
  }

  public void setAngleGoal(double goal) {
    super.setGoal(new TrapezoidProfile.State(goal, 0.0));
  }

  public void setAngleGoal(PivotSetpoint goal) {
    super.setGoal(goal.getAngle());
  }

  public void setAngleGoal(DoubleSupplier distance) {
    if (!epsilonEquals(distance.getAsDouble(), latest_distance, kInterpolationThreshold))
      latest_characteristic = InterpolationTable.interpolate(distance.getAsDouble());

    setAngleGoal(latest_characteristic.getPitchAngle());
  }

  public Command aimAtSpeaker(DoubleSupplier distance) {

    return run(() -> setAngleGoal(distance)).until(this::atGoal);
  }

  public Command positionAmp() {
    return run(() -> setAngleGoal(PivotSetpoint.kAmp)).until(this::atGoal);
  }

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforwardOutput =
        this.feedforwardController.calculate(setpoint.position, setpoint.velocity);

    double feedbackOutput = super.m_controller.calculate(this.getPositionDeg());

    double controllerOutput = feedforwardOutput + feedbackOutput;

    this.motor.setVoltage(controllerOutput);
  }

  public void doSendables() {
    SmartDashboard.putBoolean("Pivot At Goal", this.atGoal());
    SmartDashboard.putNumber("Pivot Position (deg)", this.getPositionDeg());
  }

  @Override
  public void periodic() {

    this.doSendables();
  }

  public void initTuning() {
    SmartDashboard.putNumber("pivot ks", SmartDashboard.getNumber("pivot ks", kS));
    SmartDashboard.putNumber("pivot kg", SmartDashboard.getNumber("pivot kg", kG));
    SmartDashboard.putNumber("pivot kv", SmartDashboard.getNumber("pivot kv", kV));

    SmartDashboard.putNumber("pivot kp", SmartDashboard.getNumber("pivot kp", kP));
    SmartDashboard.putNumber("pivot ki", SmartDashboard.getNumber("pivot ki", kI));
    SmartDashboard.putNumber("pivot kd", SmartDashboard.getNumber("pivot kd", kD));
  }

  public void tune() {
    double tunedS = SmartDashboard.getNumber("pivot ks", kS);
    double tunedG = SmartDashboard.getNumber("pivot kg", kG);
    double tunedV = SmartDashboard.getNumber("pivot kv", kV);

    // TODO: is there a better way to update this?
    if (tunedS != this.feedforwardController.ks
        || tunedG != this.feedforwardController.kg
        || tunedV != this.feedforwardController.kv) {
      this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);
    }

    double tunedP = SmartDashboard.getNumber("pivot kp", kP);
    double tunedI = SmartDashboard.getNumber("pivot ki", kI);
    double tunedD = SmartDashboard.getNumber("pivot kd", kD);

    super.m_controller.setPID(tunedP, tunedI, tunedD);
  }
}
