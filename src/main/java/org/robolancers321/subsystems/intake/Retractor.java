/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.DoubleSupplier;
import org.robolancers321.Constants.RetractorConstants;

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
   * Implementation
   */

  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;
  private ArmFeedforward feedforwardController;

  private Retractor() {
    super(
        new ProfiledPIDController(
            RetractorConstants.kP,
            RetractorConstants.kI,
            RetractorConstants.kD,
            new TrapezoidProfile.Constraints(
                RetractorConstants.kMaxVelocityDeg, RetractorConstants.kMaxAccelerationDeg)));

    this.motor = new CANSparkMax(RetractorConstants.kMotorPort, kBrushless);
    this.encoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedforwardController =
        new ArmFeedforward(RetractorConstants.kS, RetractorConstants.kG, RetractorConstants.kV);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(RetractorConstants.kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(RetractorConstants.kCurrentLimit);
    this.motor.enableVoltageCompensation(12); 
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 30000);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 30000);


    // this.motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) kMaxAngle);
    // this.motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) kMinAngle);
    // this.motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    // this.motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

    this.motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    this.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  private void configureEncoder() {
    // this.motor.getEncoder().setPositionConversionFactor(kGearRatio);
    // this.motor.getEncoder().setPosition(this.getPositionDeg());

    this.encoder.setInverted(RetractorConstants.kInvertEncoder);
    this.encoder.setPositionConversionFactor(RetractorConstants.kGearRatio);
    this.encoder.setVelocityConversionFactor(RetractorConstants.kGearRatio);
  }

  private void configureController() {
    // super.m_controller.enableContinuousInput(-180.0, 180.0);
    super.m_controller.disableContinuousInput();
    super.m_controller.setTolerance(RetractorConstants.kToleranceDeg);
    super.m_controller.setGoal(this.getPositionDeg());

    this.m_enabled = true;
  }

  @Override
  public double getMeasurement() {
    return this.getPositionDeg();
  }

  public double getPositionDeg() {
    return MathUtil.clamp(
        this.encoder.getPosition() > 270
            ? this.encoder.getPosition() - 360.0
            : this.encoder.getPosition(),
        RetractorConstants.kMinAngle,
        RetractorConstants.kMaxAngle);
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
        this.feedforwardController.calculate(
            setpoint.position * Math.PI / 180.0, setpoint.velocity * Math.PI / 180.0);

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
    SmartDashboard.putNumber("retractor mp goal (deg)", this.getVelocityDeg());
    SmartDashboard.putNumber("retractor position actual", this.encoder.getPosition());
  }

  @Override
  public void periodic() {
    super.periodic();

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber(
        "retractor kp", SmartDashboard.getNumber("retractor kp", RetractorConstants.kP));
    SmartDashboard.putNumber(
        "retractor ki", SmartDashboard.getNumber("retractor ki", RetractorConstants.kI));
    SmartDashboard.putNumber(
        "retractor kd", SmartDashboard.getNumber("retractor kd", RetractorConstants.kD));

    SmartDashboard.putNumber(
        "retractor ks", SmartDashboard.getNumber("retractor ks", RetractorConstants.kS));
    SmartDashboard.putNumber(
        "retractor kv", SmartDashboard.getNumber("retractor kv", RetractorConstants.kV));
    SmartDashboard.putNumber(
        "retractor kg", SmartDashboard.getNumber("retractor kg", RetractorConstants.kG));

    SmartDashboard.putNumber(
        "retractor max vel (deg)",
        SmartDashboard.getNumber("retractor max vel (deg)", RetractorConstants.kMaxVelocityDeg));
    SmartDashboard.putNumber(
        "retractor max acc (deg)",
        SmartDashboard.getNumber(
            "retractor max acc (deg)", RetractorConstants.kMaxAccelerationDeg));

    SmartDashboard.putNumber("retractor target position (deg)", this.getPositionDeg());
  }

  private void 
  tune() {
    double tunedP = SmartDashboard.getNumber("retractor kp", RetractorConstants.kP);
    double tunedI = SmartDashboard.getNumber("retractor ki", RetractorConstants.kI);
    double tunedD = SmartDashboard.getNumber("retractor kd", RetractorConstants.kD);

    super.m_controller.setPID(tunedP, tunedI, tunedD);

    double tunedS = SmartDashboard.getNumber("retractor ks", RetractorConstants.kS);
    double tunedV = SmartDashboard.getNumber("retractor kv", RetractorConstants.kV);
    double tunedG = SmartDashboard.getNumber("retractor kg", RetractorConstants.kG);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);

    double tunedMaxVel =
        SmartDashboard.getNumber("retractor max vel (deg)", RetractorConstants.kMaxVelocityDeg);
    double tunedMaxAcc =
        SmartDashboard.getNumber("retractor max acc (deg)", RetractorConstants.kMaxAccelerationDeg);

    this.m_controller.setConstraints(new Constraints(tunedMaxVel, tunedMaxAcc));

    this.setGoal(
        MathUtil.clamp(
            SmartDashboard.getNumber("retractor target position (deg)", this.getPositionDeg()),
            RetractorConstants.kMinAngle,
            RetractorConstants.kMaxAngle));
  }

  private Command moveToAngle(DoubleSupplier angleDegSupplier) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () ->
                this.setGoal(
                    MathUtil.clamp(
                        angleDegSupplier.getAsDouble(),
                        RetractorConstants.kMinAngle,
                        RetractorConstants.kMaxAngle))),
        this.run(
                () ->
                    this.setGoal(
                        MathUtil.clamp(
                            angleDegSupplier.getAsDouble(),
                            RetractorConstants.kMinAngle,
                            RetractorConstants.kMaxAngle)))
            .until(this::atGoal));
  }

  private Command moveToAngle(double angleDeg) {
    return this.moveToAngle(() -> angleDeg);
  }

  public Command moveToRetracted() {
    return this.moveToAngle(RetractorConstants.RetractorSetpoint.kRetracted.angle);
  }

  public Command moveToMating() {
    return this.moveToAngle(RetractorConstants.RetractorSetpoint.kMating.angle);
  }

  public Command moveToIntake() {
    return this.moveToAngle(RetractorConstants.RetractorSetpoint.kIntake.angle);
  }

  public Command moveToOuttake() {
    return this.moveToAngle(RetractorConstants.RetractorSetpoint.kOuttake.angle);
  }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}
