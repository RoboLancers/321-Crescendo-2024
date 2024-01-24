/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeRetractor extends SubsystemBase {
  /*
   * Singleton
   */

  private static IntakeRetractor instance = null;

  public static IntakeRetractor getInstance(){
    if (instance == null) instance = new IntakeRetractor();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 0;
  private static final int kLimitSwitchPort = 0;
  private static final int kBeamBreakPort = 0;

  private static final boolean kInvertMotor = false;
  private static final int kCurrentLimit = 20;

  private static final double kP = 0.000;
  private static final double kI = 0.000;
  private static final double kD = 0.000;
  private static final double kS = 0.000;
  private static final double kV = 0.000;
  private static final double kA = 0.000;
  private static final double kG = 0.000;

  private static final double kMaxVelocityDeg = 0.0;
  private static final double kMaxAccelerationDeg = 0.0;
  
  private static final double kErrorThreshold = 2.0;

  public static enum RetractorSetpoint {
    kRetracted(0),
    kMating(0),
    kIntake(0);

    private double angle;

    private RetractorSetpoint(double angle){
      this.angle = angle;
    }

    public double getAngle(){
      return this.angle;
    }
  }

  /*
   * Implementation
   */

  private CANSparkMax motor;
  private AbsoluteEncoder absoluteEncoder;
  private RelativeEncoder relativeEncoder;
  private ProfiledPIDController feedbackController;
  private ArmFeedforward feedforwardController;

  private DigitalInput limitSwitch;
  private DigitalInput beamBreak;

  private RetractorSetpoint setpoint;

  private IntakeRetractor() {
    this.motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    this.absoluteEncoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.relativeEncoder = this.motor.getEncoder();
    this.feedbackController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocityDeg, kMaxAccelerationDeg));

    this.limitSwitch = new DigitalInput(kLimitSwitchPort);
    this.beamBreak = new DigitalInput(kBeamBreakPort);

    this.configureMotor();
    this.configureEncoder();
    this.configureController(); 
    this.configureLimitSwitchResponder();
  }

  private void configureMotor(){
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  
  }

  private void configureEncoder(){
    this.relativeEncoder.setInverted(kInvertMotor);
    this.absoluteEncoder.setInverted(kInvertMotor);
    this.relativeEncoder.setPositionConversionFactor(360);
    this.absoluteEncoder.setPositionConversionFactor(360);

    /*
     * TODO
     * 
     * set position conversion rate (store this rate as a constant), set encoder inversion
     * 
     * this should probably be done for both encoders just in case
     */
  }

  private void configureController(){
    this.feedforwardController = new ArmFeedforward(kS, kG, kV);

    this.feedbackController.setP(kP);
    this.feedbackController.setI(kI);
    this.feedbackController.setD(kD);
  }

  private void configureLimitSwitchResponder(){
    new Trigger(this::isLimitSwitchTriggered).whileTrue(new RunCommand(this::resetEncoder));
  }

  public double getPosition() {
    return this.absoluteEncoder.getPosition();

    // if using relative encoder
    // return this.relativeEncoder.getPosition();
  }

  public boolean isLimitSwitchTriggered() {
    return !this.limitSwitch.get();
  }

  public boolean isBeamBroken(){
    return !this.beamBreak.get();
  }

  // TODO: this method is totally fine but may be scrapped when we move to a motion profiling implementation
  public boolean isAtPosition(double position) {
    return Math.abs(this.getPosition() - position) < kErrorThreshold;
  }

  public boolean isAtSetpoint(){
    return this.isAtPosition(this.setpoint.getAngle());
  }

  public void resetEncoder(double position){
    this.relativeEncoder.setPosition(position);
  }

  public void resetEncoder(){
    this.resetEncoder(RetractorSetpoint.kRetracted.getAngle());
  }

  public void setSetpoint(RetractorSetpoint setpoint){
    this.setpoint = setpoint;
  }

  private void useController(){
    State setpointState = this.feedbackController.getSetpoint();

    double feedforwardOutput = this.feedforwardController.calculate(setpointState.position, setpointState.velocity);
    double feedbackOutput = this.feedbackController.calculate(this.getPosition());

    double controllerOutput = feedforwardOutput + feedbackOutput;
    this.motor.setVoltage(controllerOutput);
  }

  private void doSendables(){
    SmartDashboard.putNumber("current position", relativeEncoder.getPosition());
    SmartDashboard.putBoolean("note detected", this.isBeamBroken());
    SmartDashboard.putBoolean("Retractor at position(deg)", this.isAtSetpoint());
    }

  @Override
  public void periodic() {
    useController();
    doSendables();
  }

  public void initTuning(){
    SmartDashboard.putNumber("retractor kP", SmartDashboard.getNumber("retractor kP", kP));
    SmartDashboard.putNumber("retractor kI", SmartDashboard.getNumber("retractor kI", kI));
    SmartDashboard.putNumber("retractor kD", SmartDashboard.getNumber("retractor kD", kD));
    SmartDashboard.putNumber("retractor kS", SmartDashboard.getNumber("retractor kS", kS));
    SmartDashboard.putNumber("retractor kV", SmartDashboard.getNumber("retractor kV", kV));
    SmartDashboard.putNumber("retractor kA", SmartDashboard.getNumber("retractor kA", kA));
    SmartDashboard.putNumber("retractor kG", SmartDashboard.getNumber("retractor kG", kG));
  }

  public void tune(){ 
    double tunedP = SmartDashboard.getNumber("retractor kp", kP);
    double tunedI = SmartDashboard.getNumber("retractor kI", kI);
    double tunedD = SmartDashboard.getNumber("retractor kD", kD);

    this.feedforwardController = new ArmFeedforward(kS, kG, kV);

    this.feedbackController.setP(tunedP);
    this.feedbackController.setI(tunedI);
    this.feedbackController.setD(tunedD);

  }
}
