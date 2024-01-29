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
  }

  private void configureEncoder() {
    this.encoder.setVelocityConversionFactor(1);
  }

  private void configureController() {
    this.controller.setP(kP);
    this.controller.setI(kI);
    this.controller.setD(kD);
    this.controller.setFF(kFF);
  }

  public double getVelocityRPM() {
    return this.encoder.getVelocity();
  }

  private void useController(double desiredRPM){
    this.controller.setReference(desiredRPM, ControlType.kVelocity);
  }

  private void doSendables() {
    SmartDashboard.putNumber("sucker velocity rpm", this.getVelocityRPM());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("sucker kP", SmartDashboard.getNumber("sucker kP", kP));
    SmartDashboard.putNumber("sucker kI", SmartDashboard.getNumber("sucker kI", kI));
    SmartDashboard.putNumber("sucker kD", SmartDashboard.getNumber("sucker kD", kD));
    SmartDashboard.putNumber("sucker kFF", SmartDashboard.getNumber("sucker kFF", kFF));

    SmartDashboard.putNumber("target sucker rpm", 0.0);
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("sucker kP", kP);
    double tunedI = SmartDashboard.getNumber("sucker kI", kI);
    double tunedD = SmartDashboard.getNumber("sucker kD", kD);
    double tunedFF = SmartDashboard.getNumber("sucker kFF", kFF);

    this.controller.setP(tunedP);
    this.controller.setI(tunedI);
    this.controller.setD(tunedD);
    this.controller.setFF(tunedFF);

    double targetRPM = SmartDashboard.getNumber("target sucker rpm", 0.0);

    this.useController(targetRPM);
  }

  public Command in() {
    return run(() -> this.useController(kInRPM))
        .finallyDo(() -> this.useController(0.0));
  }

  public Command out(double power) {
    return run(() -> this.useController(kOutRPM))
        .finallyDo(() -> this.useController(0.0));
  }

  public Command tuneController() {
    initTuning();

    return run(this::tune);
  }
}
