/* (C) Robolancers 2024 */
package org.robolancers321.subsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sucker extends SubsystemBase {
  /*
   * Singleton
   */

  private static Sucker instance = null;

  public static Sucker getInstance() {
    if (instance == null) instance = new Sucker();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 0;

  private static final boolean kInvertMotor = false;
  private static final int kCurrentLimit = 20;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kFF = 0.01;

  private static final double kInRPM = -2000;
  private static final double kOutRPM = 1000;

  /*
   * Implementation
   */

  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkPIDController controller;

  private Sucker() {
    this.motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    this.encoder = this.motor.getEncoder();
    this.controller = this.motor.getPIDController();

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
  }

  private void configureMotor() {

    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
    // TODO: inversion, current limit, idle mode, voltage compensation
  }

  public void configureEncoder() {

    this.encoder.setVelocityConversionFactor(1);
    // TODO: set velocity conversion factor to 1
  }

  public void configureController() {

    this.controller.setP(kP);
    this.controller.setI(kI);
    this.controller.setD(kD);
    this.controller.setFF(kFF);
    // TODO: set kP, kI, kD, kFF on controller
  }

  public double getVelocityRPM() {
    return this.encoder.getVelocity();
  }

  private void doSendables() {
    SmartDashboard.putNumber("sucker velocity rpm", this.getVelocityRPM());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  public void initTuning() {

    SmartDashboard.putNumber("kP", SmartDashboard.getNumber("kP", kP));
    SmartDashboard.putNumber("kI", SmartDashboard.getNumber("kI", kI));
    SmartDashboard.putNumber("kD", SmartDashboard.getNumber("kD", kD));
    SmartDashboard.putNumber("kFF", SmartDashboard.getNumber("kFF", kFF));
    // TODO: put default values for kP, kI, kD, kFF on SmartDashboard
  }

  public void tune() {

    double PTuned = SmartDashboard.getNumber("kP", kP);
    double ITuned = SmartDashboard.getNumber("kI", kI);
    double DTuned = SmartDashboard.getNumber("kD", kD);
    double FFTuned = SmartDashboard.getNumber("kFF", kFF);

    this.controller.setP(PTuned);
    this.controller.setI(ITuned);
    this.controller.setD(DTuned);
    this.controller.setFF(FFTuned);
    // TODO: read the values for kP, kI, kD, kFF from SmartDashboard and update your controller
  }

  public Command in() {
    return run(() -> this.controller.setReference(kInRPM, ControlType.kVelocity))
        .andThen(() -> this.controller.setReference(0, ControlType.kVelocity));
  }

  public Command out(double power) {
    return run(() -> this.controller.setReference(kOutRPM, ControlType.kVelocity))
        .andThen(() -> this.controller.setReference(0, ControlType.kVelocity));
  }

  // public Command off() {
  //   return runOnce(() -> this.controller.setReference(0.0, ControlType.kVelocity));
  // }

  public Command tuneController() {
    initTuning();

    return run(this::tune);
  }
}
