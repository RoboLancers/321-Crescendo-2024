/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
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

  private static enum PivotSetpoint {
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
  private ProfiledPIDController feedbackController;

  private Pivot() {
    this.motor = new CANSparkMax(kMotorPort, kBrushless);
    this.absoluteEncoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedforwardController = new ArmFeedforward(kS, kG, kV);
    this.feedbackController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(kMaxVelocityDeg, kMaxAccelerationDeg));

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

  private void configureEncoder(){
    this.absoluteEncoder.setInverted(kInvertEncoder);
    this.absoluteEncoder.setPositionConversionFactor(kGearRatio);
    this.absoluteEncoder.setVelocityConversionFactor(kRotPerMinToDegPerSec);
  }

  private void configureController(){
    this.feedbackController.setTolerance(kToleranceDeg);
  }

  public double getPositionDeg(){
    return this.absoluteEncoder.getPosition();
  }

  public double getPositionRad(){
    return this.getPositionDeg() * Math.PI / 180.0;
  }

  public double getVelocityDeg(){
    return this.absoluteEncoder.getVelocity();
  }

  public double getVelocityRad(){
    return this.getVelocityDeg() * Math.PI / 180.0;
  }

  private boolean atGoal(){
    return this.feedbackController.atGoal();
  }

  private void setGoal(double goal){
    this.feedbackController.setGoal(new TrapezoidProfile.State(goal, 0.0));
  }

  private void setGoal(PivotSetpoint goal){
    this.setGoal(goal.getAngle());
  }

  private void useController(){
    // TODO: docs are not specific as to units of measurement for velocity
    double feedforwardOutput = this.feedforwardController.calculate(this.getPositionRad(), this.getVelocityRad());

    double feedbackOutput = this.feedbackController.calculate(this.getPositionDeg());

    double controllerOutput = feedforwardOutput + feedbackOutput;

    this.motor.setVoltage(controllerOutput);
  }

  public void doSendables(){
    SmartDashboard.putBoolean("Pivot At Goal", this.atGoal());
    SmartDashboard.putNumber("Pivot Position (deg)", this.getPositionDeg());
  }

  @Override
  public void periodic(){
    this.useController();
    this.doSendables();
  }

  public void initTuning(){
    SmartDashboard.putNumber("pivot ks", SmartDashboard.getNumber("pivot ks", kS));
    SmartDashboard.putNumber("pivot kg", SmartDashboard.getNumber("pivot kg", kG));
    SmartDashboard.putNumber("pivot kv", SmartDashboard.getNumber("pivot kv", kV));

    SmartDashboard.putNumber("pivot kp", SmartDashboard.getNumber("pivot kp", kP));
    SmartDashboard.putNumber("pivot ki", SmartDashboard.getNumber("pivot ki", kI));
    SmartDashboard.putNumber("pivot kd", SmartDashboard.getNumber("pivot kd", kD));
  }

  public void tune(){
    double tunedS = SmartDashboard.getNumber("pivot ks", kS);
    double tunedG = SmartDashboard.getNumber("pivot kg", kG);
    double tunedV = SmartDashboard.getNumber("pivot kv", kV);

    // TODO: is there a better way to update this?
    if(tunedS != this.feedforwardController.ks || tunedG != this.feedforwardController.kg || tunedV != this.feedforwardController.kv){
      this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);
    }

    double tunedP = SmartDashboard.getNumber("pivot kp", kP);
    double tunedI = SmartDashboard.getNumber("pivot ki", kI);
    double tunedD = SmartDashboard.getNumber("pivot kd", kD);

    this.feedbackController.setPID(tunedP, tunedI, tunedD);
  }
}
