/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import java.util.function.DoubleSupplier;

public class Pivot extends ProfiledPIDSubsystem {
  /*
   * Singleton
   */

  private static Pivot instance = null;

  public static Pivot getInstance() {
    if (instance == null) instance = new Pivot();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 15;

  private static final boolean kInvertMotor = false;
  private static final int kCurrentLimit = 40;

  private static final double kGearRatio = 360.0;
  private static final double kRotPerMinToDegPerSec = kGearRatio / 60.0;

  private static final float kMinAngle = -5.0f;
  private static final float kMaxAngle = 90.0f;

  private static final double kS = 0.0;
  private static final double kG = 0.0;
  private static final double kV = 0.0;

  private static final double kP = 0.04;
  private static final double kI = 0.0;
  private static final double kD = 0.02;
  private static final double kMaxVelocityDeg = 140;
  private static final double kMaxAccelerationDeg = 140;

  private static final double kToleranceDeg = 0.0;

  public enum PivotSetpoint {
    kRetracted(0.0),
    kMating(0.0),
    kAmp(0.0);

    public final double angle;

    PivotSetpoint(double angle) {
      this.angle = angle;
    }
  }

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final AbsoluteEncoder absoluteEncoder;
  private ArmFeedforward feedforwardController; // TODO: make this final when not tuning

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
    this.motor.burnFlash();

    this.m_enabled = true;
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
    this.absoluteEncoder.setPositionConversionFactor(kGearRatio);
    this.absoluteEncoder.setVelocityConversionFactor(kRotPerMinToDegPerSec);
  }

  private void configureController() {
    super.m_controller.setTolerance(kToleranceDeg);
    super.m_controller.enableContinuousInput(0.0, 360.0);
  }

  @Override
  public double getMeasurement() {
    return this.getPositionDeg();
  }

  public double getPositionDeg() {
    return this.absoluteEncoder.getPosition();
  }

  public double getVelocityDeg() {
    return this.absoluteEncoder.getVelocity();
  }

  private boolean atGoal() {
    return super.m_controller.atGoal();
  }

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforwardOutput =
        this.feedforwardController.calculate(setpoint.position * Math.PI / 180.0, setpoint.velocity * Math.PI / 180.0);

    SmartDashboard.putNumber("pivot position setpoint mp (deg)", setpoint.position);
    SmartDashboard.putNumber("pivot velocity setpoint mp (deg)", setpoint.velocity);

    SmartDashboard.putNumber("pivot position setpoint error (deg)", setpoint.position - this.getPositionDeg());

    SmartDashboard.putNumber("pivot ff output", feedforwardOutput);

    double feedbackOutput = super.m_controller.calculate(this.getPositionDeg());

    SmartDashboard.putNumber("pivot fb output", feedbackOutput);

    double controllerOutput = feedforwardOutput + feedbackOutput;

    SmartDashboard.putNumber("pivot controller output", controllerOutput);

    this.motor.set(controllerOutput);
  }

  public void doSendables() {
    SmartDashboard.putBoolean("pivot at goal", this.atGoal());
    SmartDashboard.putNumber("pivot position (deg)", this.getPositionDeg());
  }

  @Override
  public void periodic() {
    super.periodic();

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("pivot kp", SmartDashboard.getNumber("pivot kp", kP));
    SmartDashboard.putNumber("pivot ki", SmartDashboard.getNumber("pivot ki", kI));
    SmartDashboard.putNumber("pivot kd", SmartDashboard.getNumber("pivot kd", kD));

    SmartDashboard.putNumber("pivot ks", SmartDashboard.getNumber("pivot ks", kS));
    SmartDashboard.putNumber("pivot kg", SmartDashboard.getNumber("pivot kg", kG));
    SmartDashboard.putNumber("pivot kv", SmartDashboard.getNumber("pivot kv", kV));

    SmartDashboard.putNumber("pivot target position (deg)", this.getPositionDeg());
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("pivot kp", kP);
    double tunedI = SmartDashboard.getNumber("pivot ki", kI);
    double tunedD = SmartDashboard.getNumber("pivot kd", kD);

    super.m_controller.setPID(tunedP, tunedI, tunedD);

    double tunedS = SmartDashboard.getNumber("pivot ks", kS);
    double tunedG = SmartDashboard.getNumber("pivot kg", kG);
    double tunedV = SmartDashboard.getNumber("pivot kv", kV);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);

    this.setGoal(MathUtil.clamp(SmartDashboard.getNumber("pivot target position (deg)", this.getPositionDeg()), kMinAngle, kMaxAngle));
  }

  private Command moveToAngle(DoubleSupplier angleDegSupplier) {
    return run(() -> this.setGoal(MathUtil.clamp(angleDegSupplier.getAsDouble(), kMinAngle, kMaxAngle))).until(this::atGoal);
  }

  private Command moveToAngle(double angleDeg) {
    return this.moveToAngle(() -> angleDeg);
  }

  public Command moveToRetracted() {
    return this.moveToAngle(AimTable.kRetractedAimCharacteristic.angle);
  }

  public Command moveToMating() {
    return this.moveToAngle(AimTable.kMatingAimCharacteristic.angle);
  }

  public Command aimAtAmp() {
    return this.moveToAngle(AimTable.kAmpAimCharacteristic.angle);
  }

  public Command aimAtSpeaker(DoubleSupplier angleDegSupplier) {
    return this.moveToAngle(angleDegSupplier);
  }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}
