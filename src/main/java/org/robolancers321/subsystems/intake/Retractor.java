/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Retractor extends SubsystemBase {
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

  private static final boolean kInvertMotor = true;
  private static final int kCurrentLimit = 20;

  private static final double kP = 0.000;
  private static final double kI = 0.000;
  private static final double kD = 0.000;

  private static final double kS = 0.000;
  private static final double kG = 0.000;
  private static final double kV = 0.000;

  private static final double kMaxVelocityDeg = 0.0;
  private static final double kMaxAccelerationDeg = 0.0;

  private static final double kErrorThreshold = 2.0;

  public enum RetractorPosition {
    kRetracted(0),
    kMating(0),
    kIntake(0);

    private final double angle;

    RetractorPosition(double angleDeg) {
      this.angle = angleDeg;
    }

    public double getAngle() {
      return this.angle;
    }
  }

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;
  private final ProfiledPIDController feedbackController;
  private ArmFeedforward feedforwardController; // TODO: make this final when not tuning

  private Retractor() {
    this.motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    this.encoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedbackController =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocityDeg, kMaxAccelerationDeg));

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setPositionConversionFactor(360); // TODO: potentially a gear ratio on this
  }

  private void configureController() {
    this.feedbackController.setP(kP);
    this.feedbackController.setI(kI);
    this.feedbackController.setD(kD);
    this.feedbackController.setTolerance(kErrorThreshold);

    this.feedforwardController = new ArmFeedforward(kS, kG, kV);
  }

  public double getPosition() {
    return this.encoder.getPosition();
  }

  public boolean isAtGoal() {
    return this.feedbackController.atGoal();
  }

  private void setGoal(double goalDeg) {
    this.feedbackController.setGoal(goalDeg * Math.PI / 180.0);
  }

  private void setGoal(RetractorPosition goal) {
    this.setGoal(goal.getAngle());
  }

  private void useControllers() {
    State setpointState = this.feedbackController.getSetpoint();

    double feedforwardOutput =
        this.feedforwardController.calculate(setpointState.position, setpointState.velocity);

    SmartDashboard.putNumber("retractor ff output", feedforwardOutput);

    double feedbackOutput = this.feedbackController.calculate(this.getPosition());

    SmartDashboard.putNumber("retractor fb output", feedbackOutput);

    double controllerOutput = feedforwardOutput + feedbackOutput;

    SmartDashboard.putNumber("retractor controller output", controllerOutput);

    this.motor.set(controllerOutput);
  }

  private void doSendables() {
    SmartDashboard.putBoolean("retractor at goal", this.isAtGoal());
    SmartDashboard.putNumber("retractor position (deg)", this.getPosition());
  }

  @Override
  public void periodic() {
    useControllers();
    doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("retractor kp", SmartDashboard.getNumber("retractor kp", kP));
    SmartDashboard.putNumber("retractor ki", SmartDashboard.getNumber("retractor ki", kI));
    SmartDashboard.putNumber("retractor kd", SmartDashboard.getNumber("retractor kd", kD));

    SmartDashboard.putNumber("retractor ks", SmartDashboard.getNumber("retractor ks", kS));
    SmartDashboard.putNumber("retractor kv", SmartDashboard.getNumber("retractor kv", kV));
    SmartDashboard.putNumber("retractor kg", SmartDashboard.getNumber("retractor kg", kG));

    SmartDashboard.putNumber("retractor target position (deg)", this.getPosition());
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("retractor kp", kP);
    double tunedI = SmartDashboard.getNumber("retractor ki", kI);
    double tunedD = SmartDashboard.getNumber("retractor kd", kD);

    this.feedbackController.setP(tunedP);
    this.feedbackController.setI(tunedI);
    this.feedbackController.setD(tunedD);

    double tunedS = SmartDashboard.getNumber("retractor ks", kS);
    double tunedV = SmartDashboard.getNumber("retractor kv", kV);
    double tunedG = SmartDashboard.getNumber("retractor kg", kG);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);

    double targetRetractorPosition =
        SmartDashboard.getNumber("retractor target position (deg)", this.getPosition());

    this.setGoal(targetRetractorPosition);
  }

  private Command moveToPosition(RetractorPosition position) {
    this.setGoal(position);

    return new WaitUntilCommand(this::isAtGoal);
  }

  public Command moveToRetracted() {
    return this.moveToPosition(RetractorPosition.kRetracted);
  }

  public Command moveToMating() {
    return this.moveToPosition(RetractorPosition.kMating);
  }

  public Command moveToIntake() {
    return this.moveToPosition(RetractorPosition.kIntake);
  }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}
