/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

  // TODO: put constants for motor and encoder configuration here

  private static final double kP = 0.000;
  private static final double kI = 0.000;
  private static final double kD = 0.000;
  
  // TODO: feed forward controller using wpilib ArmFeedforward class

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
  private SparkPIDController feedbackController;

  private DigitalInput limitSwitch;
  private DigitalInput beamBreak;

  private RetractorSetpoint setpoint;

  private IntakeRetractor() {
    this.motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    this.absoluteEncoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.relativeEncoder = this.motor.getEncoder();
    this.feedbackController = this.motor.getPIDController();

    this.limitSwitch = new DigitalInput(kLimitSwitchPort);
    this.beamBreak = new DigitalInput(kBeamBreakPort);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();

    this.configureLimitSwitchResponder();
  }

  private void configureMotor(){
    /*
     * TODO
     * 
     * using constants set:
     *    motor inversion, current limit (30?)
     * 
     * without constants set:
     *    idle mode (brake), voltage compensation (12)
     */
  }

  private void configureEncoder(){
    /*
     * TODO
     * 
     * set position conversion rate (store this rate as a constant), set encoder inversion
     * 
     * this should probably be done for both encoders just in case
     */
  }

  private void configureController(){
    this.feedbackController.setD(kD);
    this.feedbackController.setP(kP);
    this.feedbackController.setI(kI);

    // TODO: also configure feed forward controller once it it written
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
    // TODO: is this negation correct?

    return !this.limitSwitch.get();
  }

  public boolean isBeamBroken(){
    // TODO: is this negation correct?
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

    // TODO: does it make sense to implement this for the mag offset of the absolute encoder?
  }

  public void resetEncoder(){
    this.resetEncoder(RetractorSetpoint.kRetracted.getAngle());
  }

  public void setSetpoint(RetractorSetpoint setpoint){
    this.setpoint = setpoint;
  }

  private void useController(){
    /*
     * TODO
     * 
     * this method will be called periodically to set the motor voltage based on feedback and feedforward controllers
     * 
     * add the outputs of the feedback controller (pid) and feedforward controller (once it is written) and supply them to the motor via set()
     * 
     * good examples of this code are this years shooter pivot and last years arm
     */
  }

  private void doSendables(){
    // TODO: use smart dashboard to log the position, limit switch status, beam break status, etc.
  }

  @Override
  public void periodic() {
    useController();
    doSendables();
  }

  public void initTuning(){
    SmartDashboard.putNumber("retractor kp", SmartDashboard.getNumber("retractor kp", kP));

    // TODO: ^ do this for kI, kD, and any constants for the feedforward controller
  }

  public void tune(){
    double tunedP = SmartDashboard.getNumber("retractor kp", kP);

    this.feedbackController.setP(tunedP);

    // TODO: ^ do this for kI, kD, and any constants for the feedforward controller
  }
}
