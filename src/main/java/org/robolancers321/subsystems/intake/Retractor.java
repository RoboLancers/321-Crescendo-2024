/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Retractor extends ProfiledPIDSubsystem {
  /*
   * Singleton
   */

  private static Retractor instance = null;

  public static Retractor getInstance() {
    if (instance == null) instance = new Retractor();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 13;

  private static final boolean kInvertMotor = false;
  private static final int kCurrentLimit = 30;
  
  private static final double kGearRatio = 360.0;
  private static final double kRotPerMinToDegPerSec = kGearRatio / 60.0;

  private static final float kMinAngle = 0.0f;
  private static final float kMaxAngle = 180.0f;

  private static final double kP = 0.000;
  private static final double kI = 0.000;
  private static final double kD = 0.000;

  private static final double kS = 0.000;
  private static final double kG = 0.000;
  private static final double kV = 0.000;

  private static final double kMaxVelocityDeg = 30.0;
  private static final double kMaxAccelerationDeg = 45.0;

  private static final double kToleranceDeg = 0.5;

  private enum RetractorSetpoint {
    kRetracted(0),
    kMating(0),
    kIntake(0);

    public final double angle;

    private RetractorSetpoint(double angleDeg) {
      this.angle = angleDeg;
    }
  }

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;
  private ArmFeedforward feedforwardController;

  private Retractor() {
    super(
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocityDeg, kMaxAccelerationDeg)));

    this.motor = new CANSparkMax(kMotorPort, kBrushless);
    this.encoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedforwardController = new ArmFeedforward(kS, kG, kV);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
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
    this.encoder.setPositionConversionFactor(kGearRatio);
    this.encoder.setVelocityConversionFactor(kRotPerMinToDegPerSec);
  }

  private void configureController() {
    super.m_controller.enableContinuousInput(0.0, 360.0);
    super.m_controller.setTolerance(kToleranceDeg);
    super.m_controller.setGoal(this.getPositionDeg());

    this.m_enabled = true;
  }
  
  @Override
  public double getMeasurement() {
    return this.getPositionDeg();
  }

  public double getPositionDeg() {
    return this.encoder.getPosition();
  }

  public double getVelocityDeg() {
    return this.encoder.getVelocity();
  }

  private boolean atGoal() {
    return super.m_controller.atGoal();
  }

  @Override
  protected void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforwardOutput =
        this.feedforwardController.calculate(setpoint.position * Math.PI / 180.0, setpoint.velocity * Math.PI / 180.0);

    SmartDashboard.putNumber("retractor position setpoint mp (deg)", setpoint.position);
    SmartDashboard.putNumber("retractor velocity setpoint mp (deg)", setpoint.velocity);

    SmartDashboard.putNumber("retractor ff output", feedforwardOutput);

    double feedbackOutput = super.m_controller.calculate(this.getPositionDeg());

    SmartDashboard.putNumber("retractor fb output", feedbackOutput);

    double controllerOutput = feedforwardOutput + feedbackOutput;

    SmartDashboard.putNumber("retractor controller output", controllerOutput);

    this.motor.set(controllerOutput);
  }

  private void doSendables() {
    SmartDashboard.putBoolean("retractor at goal", this.atGoal());
    SmartDashboard.putNumber("retractor position (deg)", this.getPositionDeg());
  }
  
  @Override
  public void periodic() {
    super.periodic();

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("retractor kp", SmartDashboard.getNumber("retractor kp", kP));
    SmartDashboard.putNumber("retractor ki", SmartDashboard.getNumber("retractor ki", kI));
    SmartDashboard.putNumber("retractor kd", SmartDashboard.getNumber("retractor kd", kD));

    SmartDashboard.putNumber("retractor ks", SmartDashboard.getNumber("retractor ks", kS));
    SmartDashboard.putNumber("retractor kv", SmartDashboard.getNumber("retractor kv", kV));
    SmartDashboard.putNumber("retractor kg", SmartDashboard.getNumber("retractor kg", kG));

    SmartDashboard.putNumber("retractor max vel (deg)", SmartDashboard.getNumber("retractor max vel (deg)", kMaxVelocityDeg));
    SmartDashboard.putNumber("retractor max acc (deg)", SmartDashboard.getNumber("retractor max acc (deg)", kMaxAccelerationDeg));

    SmartDashboard.putNumber("retractor target position (deg)", this.getPositionDeg());
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("retractor kp", kP);
    double tunedI = SmartDashboard.getNumber("retractor ki", kI);
    double tunedD = SmartDashboard.getNumber("retractor kd", kD);

    super.m_controller.setPID(tunedP, tunedI, tunedD);

    double tunedS = SmartDashboard.getNumber("retractor ks", kS);
    double tunedV = SmartDashboard.getNumber("retractor kv", kV);
    double tunedG = SmartDashboard.getNumber("retractor kg", kG);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);

    double tunedMaxVel = SmartDashboard.getNumber("retractor max vel (deg)", kMaxVelocityDeg);
    double tunedMaxAcc = SmartDashboard.getNumber("retractor max acc (deg)", kMaxAccelerationDeg);

    this.m_controller.setConstraints(new Constraints(tunedMaxVel, tunedMaxAcc));

    this.setGoal(MathUtil.clamp(SmartDashboard.getNumber("retractor target position (deg)", this.getPositionDeg()), kMinAngle, kMaxAngle));
  }
  
  private Command moveToAngle(double angleDeg) {
    return run(() -> this.setGoal(MathUtil.clamp(angleDeg, kMinAngle, kMaxAngle))).until(this::atGoal);
  }

  public Command moveToRetracted() {
    return this.moveToAngle(RetractorSetpoint.kRetracted.angle);
  }

  public Command moveToMating() {
    return this.moveToAngle(RetractorSetpoint.kMating.angle);
  }

  public Command moveToIntake() {
    return this.moveToAngle(RetractorSetpoint.kIntake.angle);
  }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}
