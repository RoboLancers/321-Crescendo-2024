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

  private static final int kMotorPort = 0;

  private static final boolean kInvertMotor = false;
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

  public static enum RetractorPosition {
    kRetracted(0),
    kMating(0),
    kIntake(0);

    private double angle;

    private RetractorPosition(double angle) {
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
  private AbsoluteEncoder encoder;
  private ProfiledPIDController feedbackController;
  private ArmFeedforward feedforwardController;

  private Retractor() {
    this.motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    this.encoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedbackController =
        new ProfiledPIDController(
            kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocityDeg, kMaxAccelerationDeg));

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setInverted(kInvertMotor);
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

  private void setGoal(double goal){
    this.feedbackController.setGoal(goal);
  }

  private void setGoal(RetractorPosition goal){
    this.setGoal(goal.getAngle());
  }

  private void useController() {
    State setpointState = this.feedbackController.getSetpoint();

    double feedforwardOutput =
        this.feedforwardController.calculate(setpointState.position, setpointState.velocity);
    double feedbackOutput = this.feedbackController.calculate(this.getPosition());

    double controllerOutput = feedforwardOutput + feedbackOutput;

    this.motor.set(controllerOutput);
  }

  private void doSendables() {
    SmartDashboard.putNumber("current position (deg)", this.getPosition());
    SmartDashboard.putBoolean("retractor at goal", this.isAtGoal());
  }

  @Override
  public void periodic() {
    useController();
    doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("retractor kP", SmartDashboard.getNumber("retractor kP", kP));
    SmartDashboard.putNumber("retractor kI", SmartDashboard.getNumber("retractor kI", kI));
    SmartDashboard.putNumber("retractor kD", SmartDashboard.getNumber("retractor kD", kD));

    SmartDashboard.putNumber("retractor kS", SmartDashboard.getNumber("retractor kS", kS));
    SmartDashboard.putNumber("retractor kV", SmartDashboard.getNumber("retractor kV", kV));
    SmartDashboard.putNumber("retractor kG", SmartDashboard.getNumber("retractor kG", kG));

    SmartDashboard.putNumber("target retractor position", this.getPosition());
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("retractor kp", kP);
    double tunedI = SmartDashboard.getNumber("retractor kI", kI);
    double tunedD = SmartDashboard.getNumber("retractor kD", kD);

    this.feedbackController.setP(tunedP);
    this.feedbackController.setI(tunedI);
    this.feedbackController.setD(tunedD);

    double tunedS = SmartDashboard.getNumber("retractor kS", kS);
    double tunedV = SmartDashboard.getNumber("retractor kV", kV);
    double tunedG = SmartDashboard.getNumber("retractor kG", kG);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);
    
    double targetRetractorPosition = SmartDashboard.getNumber("target retractor position", this.getPosition());
    
    this.setGoal(targetRetractorPosition);
  }

  private Command moveToPosition(RetractorPosition position){
    this.setGoal(position);

    return new WaitUntilCommand(this::isAtGoal);
  }

  public Command moveToRetracted(){
    return this.moveToPosition(RetractorPosition.kRetracted);
  }

  public Command moveToMating(){
    return this.moveToPosition(RetractorPosition.kMating);
  }

  public Command moveToIntake(){
    return this.moveToPosition(RetractorPosition.kIntake);
  }

  public Command tuneControllers(){
    this.initTuning();

    return run(this::tune);
  }
}
