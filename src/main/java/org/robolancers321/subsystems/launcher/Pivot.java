/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.robolancers321.Constants;
import org.robolancers321.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /*
   * Singleton
   */

  private static Pivot instance = null;

  public static Pivot getInstance() {
    if (instance == null) instance = new Pivot();

    return instance;
  }

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;
  private ArmFeedforward feedforwardController;
  private PIDController feedbackController;
  private TrapezoidProfile motionProfile;
  private TrapezoidProfile.State previousReference;
  private TrapezoidProfile.State goalReference;

  private Pivot() {
    this.motor = new CANSparkMax(PivotConstants.kMotorPort, kBrushless);
    this.encoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedforwardController =
        new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV);
    this.feedbackController =
        new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
    this.motionProfile = new TrapezoidProfile(Constants.PivotConstants.kProfileConstraints);
    this.previousReference = new TrapezoidProfile.State(this.getPositionDeg(), 0);
    this.goalReference = previousReference;

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(PivotConstants.kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(PivotConstants.kCurrentLimit);
    this.motor.enableVoltageCompensation(12);

    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 30000); // analog sensor
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 30000); // alternate encoder
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // abs encoder position
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // abs encoder velocity

    this.motor.setSoftLimit(
        CANSparkBase.SoftLimitDirection.kForward, (float) PivotConstants.kMaxAngle);
    this.motor.setSoftLimit(
        CANSparkBase.SoftLimitDirection.kReverse, (float) PivotConstants.kMinAngle);

    this.motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    this.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  private void configureEncoder() {
    this.encoder.setInverted(PivotConstants.kInvertEncoder);
    this.encoder.setPositionConversionFactor(PivotConstants.kGearRatio);
    this.encoder.setVelocityConversionFactor(PivotConstants.kGearRatio);
  }

  private void configureController() {
    feedbackController.enableContinuousInput(-180.0, 180.0);
    feedbackController.setTolerance(PivotConstants.kToleranceDeg);
    feedbackController.setSetpoint(previousReference.position);
  }

  public double getPositionDeg() {
    double angle =
        this.encoder.getPosition() > 180
            ? this.encoder.getPosition() - 360.0
            : this.encoder.getPosition();

    if (angle > PivotConstants.kMinAngle && angle < PivotConstants.kMaxAngle + 0.5) return angle;

    if (angle > PivotConstants.kMinAngle - 40.0 && angle < PivotConstants.kMaxAngle)
      return PivotConstants.kMinAngle;

    return PivotConstants.kMaxAngle;
  }

  public double getVelocityDeg() {
    return this.encoder.getVelocity();
  }

  public boolean atGoal() {
    return Math.abs(goalReference.position - getPositionDeg()) < PivotConstants.kToleranceDeg;
  }

  private void setGoal(double position) {
    goalReference = new TrapezoidProfile.State(position, 0);
  }

  protected void useOutput(TrapezoidProfile.State setpoint) {
    double feedforwardOutput =
        this.feedforwardController.calculate(
            setpoint.position * Math.PI / 180.0, setpoint.velocity * Math.PI / 180.0);

    SmartDashboard.putNumber("pivot position setpoint mp (deg)", setpoint.position);
    SmartDashboard.putNumber("pivot velocity setpoint mp (deg)", setpoint.velocity);

    SmartDashboard.putNumber("pivot ff output", feedforwardOutput);

    double feedbackOutput = feedbackController.calculate(this.getPositionDeg(), setpoint.position);

    SmartDashboard.putNumber("pivot fb output", feedbackOutput);

    double controllerOutput = feedforwardOutput + feedbackOutput;

    SmartDashboard.putNumber("pivot controller output", controllerOutput);

    this.motor.set(controllerOutput);
  }

  public void doSendables() {
    SmartDashboard.putBoolean("pivot at goal", this.atGoal());
    SmartDashboard.putNumber("pivot position (deg)", this.getPositionDeg());
    SmartDashboard.putNumber("pivot velocity (deg)", this.getVelocityDeg());
  }

  @Override
  public void periodic() {
    // update assumed position with next profile timestamp
    previousReference = motionProfile.calculate(0.02, previousReference, goalReference);

    // set position to where we're supposed to be. Velocity isn't an input, so can't interupt
    useOutput(previousReference);

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("pivot kp", SmartDashboard.getNumber("pivot kp", PivotConstants.kP));
    SmartDashboard.putNumber("pivot ki", SmartDashboard.getNumber("pivot ki", PivotConstants.kI));
    SmartDashboard.putNumber("pivot kd", SmartDashboard.getNumber("pivot kd", PivotConstants.kD));

    SmartDashboard.putNumber("pivot ks", SmartDashboard.getNumber("pivot ks", PivotConstants.kS));
    SmartDashboard.putNumber("pivot kg", SmartDashboard.getNumber("pivot kg", PivotConstants.kG));
    SmartDashboard.putNumber("pivot kv", SmartDashboard.getNumber("pivot kv", PivotConstants.kV));

    SmartDashboard.putNumber(
        "pivot max vel (deg)",
        SmartDashboard.getNumber("pivot max vel (deg)", PivotConstants.kMaxVelocityDeg));
    SmartDashboard.putNumber(
        "pivot max acc (deg)",
        SmartDashboard.getNumber("pivot max acc (deg)", PivotConstants.kMaxAccelerationDeg));

    SmartDashboard.putNumber("pivot target position (deg)", this.getPositionDeg());
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("pivot kp", PivotConstants.kP);
    double tunedI = SmartDashboard.getNumber("pivot ki", PivotConstants.kI);
    double tunedD = SmartDashboard.getNumber("pivot kd", PivotConstants.kD);

    this.feedbackController.setPID(tunedP, tunedI, tunedD);

    double tunedS = SmartDashboard.getNumber("pivot ks", PivotConstants.kS);
    double tunedG = SmartDashboard.getNumber("pivot kg", PivotConstants.kG);
    double tunedV = SmartDashboard.getNumber("pivot kv", PivotConstants.kV);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);

    double tunedMaxVel =
        SmartDashboard.getNumber("pivot max vel (deg)", PivotConstants.kMaxVelocityDeg);
    double tunedMaxAcc =
        SmartDashboard.getNumber("pivot max acc (deg)", PivotConstants.kMaxAccelerationDeg);

    this.motionProfile = new TrapezoidProfile(new Constraints(tunedMaxVel, tunedMaxAcc));

    this.setGoal(
        MathUtil.clamp(
            SmartDashboard.getNumber("pivot target position (deg)", goalReference.position),
            PivotConstants.kMinAngle,
            PivotConstants.kMaxAngle));
  }

  public Command moveToAngle(DoubleSupplier angleDegSupplier) {
    return new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    this.setGoal(
                        MathUtil.clamp(
                            angleDegSupplier.getAsDouble(),
                            PivotConstants.kMinAngle,
                            PivotConstants.kMaxAngle))),
            this.run(
                    () ->
                        this.setGoal(
                            MathUtil.clamp(
                                angleDegSupplier.getAsDouble(),
                                PivotConstants.kMinAngle,
                                PivotConstants.kMaxAngle)))
                .until(this::atGoal))
        .withTimeout(4.0);
  }

  public Command moveToAngle(double angleDeg) {
    return this.moveToAngle(() -> angleDeg);
  }

  public Command moveToRetracted() {
    return this.moveToAngle(PivotConstants.PivotSetpoint.kRetracted.angle);
  }

  public Command moveToMating() {
    return this.moveToAngle(PivotConstants.PivotSetpoint.kMating.angle);
  }

  public Command moveToShift() {
    return this.moveToAngle(PivotConstants.PivotSetpoint.kShift.angle);
  }

  public Command aimAtAmp() {
    return this.moveToAngle(PivotConstants.PivotSetpoint.kAmp.angle);
  }

  public Command aimAtTrap() {
    return this.moveToAngle(PivotConstants.PivotSetpoint.kTrap.angle);
  }

  // public Command aimAtSource() {
  //   return this.moveToAngle()
  // }

  public Command aimAtSpeaker(DoubleSupplier angleDegSupplier) {
    return this.moveToAngle(angleDegSupplier);
  }

  public Command aimAtSpeakerFixed() {
    return this.moveToAngle(PivotConstants.PivotSetpoint.kSpeaker.angle);
  }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}
