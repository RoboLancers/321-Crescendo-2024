/* (C) Robolancers 2024 */
package org.robolancers321.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final CANSparkMax leftClimber;
  private final CANSparkMax rightClimber;

  private final RelativeEncoder leftClimberEncoder;
  private final RelativeEncoder rightClimberEncoder;

  private final PIDController leftClimberPID;
  private final PIDController rightClimberPID;

  private final DigitalInput leftLimitSwitch;
  private final DigitalInput rightLimitSwitch;

  public static final int leftClimberPort = 0;
  public static final int leftLimitSwitchPort = 0;
  public static final int leftClimberCurrentLimit = 40;
  public static final boolean leftClimberInverted = false;

  public static final int rightClimberPort = 0;
  public static final int rightLimitSwitchPort = 1;
  public static final int rightClimberCurrentLimit = 40;
  public static final boolean rightClimberInverted = false;

  public static final double climberEncoderPCF = 1;

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kFF = 0;

  public Climber() {
    this.leftClimber = new CANSparkMax(leftClimberPort, MotorType.kBrushless);
    this.rightClimber = new CANSparkMax(rightClimberPort, MotorType.kBrushless);

    this.leftClimberEncoder = leftClimber.getEncoder();
    this.rightClimberEncoder = rightClimber.getEncoder();

    this.leftLimitSwitch = new DigitalInput(leftLimitSwitchPort);
    this.rightLimitSwitch = new DigitalInput(rightLimitSwitchPort);

    this.leftClimberPID = new PIDController(kP, kI, kD);
    this.rightClimberPID = new PIDController(kP, kI, kD);

    configMotors();
    configEncoders();
    resetEncoders();
  }

  public void configMotors() {
    leftClimber.setInverted(leftClimberInverted);
    leftClimber.setIdleMode(IdleMode.kBrake);
    leftClimber.setSmartCurrentLimit(leftClimberCurrentLimit);
    leftClimber.enableVoltageCompensation(12.0);

    rightClimber.setInverted(rightClimberInverted);
    rightClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setSmartCurrentLimit(rightClimberCurrentLimit);
    rightClimber.enableVoltageCompensation(12.0);

    // leftClimber.burnFlash();
    // rightClimber.burnFlash();
  }

  public void configEncoders() {
    leftClimberEncoder.setPositionConversionFactor(climberEncoderPCF);
    rightClimberEncoder.setPositionConversionFactor(climberEncoderPCF);

    leftClimberEncoder.setVelocityConversionFactor(climberEncoderPCF / 60);
    rightClimberEncoder.setVelocityConversionFactor(climberEncoderPCF / 60);
  }

  public void resetEncoders() {
    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);
  }

  public double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  public double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }

  public double getLeftClimberVelocity() {
    return leftClimberEncoder.getVelocity();
  }

  public double getRightClimberVelocity() {
    return rightClimberEncoder.getVelocity();
  }

  public void setLeftClimberSetpoint(double setpoint) {
    leftClimberPID.setSetpoint(setpoint);
  }

  public void setRightClimberSetpoint(double setpoint) {
    rightClimberPID.setSetpoint(setpoint);
  }

  public void setLeftPower(double speed) {
    leftClimber.set(speed);
  }

  public void setRightPower(double speed) {
    rightClimber.set(speed);
  }

  public Command leftUp(double setpoint) {
    setLeftClimberSetpoint(setpoint);
    return run(() -> setLeftPower(leftClimberPID.calculate(getLeftClimberPosition())))
        .until(leftClimberPID::atSetpoint);
  }

  public Command rightUp(double setpoint) {
    setRightClimberSetpoint(setpoint);
    return run(() -> setRightPower(rightClimberPID.calculate(getRightClimberPosition())))
        .until(rightClimberPID::atSetpoint);
  }

  public Command bothUp(double setpoint) {
    setLeftClimberSetpoint(setpoint);
    setRightClimberSetpoint(setpoint);
    return run(() -> {
          setLeftPower(leftClimberPID.calculate(getLeftClimberPosition()));
          setRightPower(rightClimberPID.calculate(getRightClimberPosition()));
        })
        .until(() -> leftClimberPID.atSetpoint() && rightClimberPID.atSetpoint())
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command zeroLeft() {
    return run(() -> setLeftPower(-0.2))
        .until(leftLimitSwitch::get)
        .finallyDo(
            () -> {
              setLeftPower(0);
              leftClimberEncoder.setPosition(0);
            });
  }

  public Command zeroRight() {
    return run(() -> setRightPower(-0.2))
        .until(rightLimitSwitch::get)
        .finallyDo(
            () -> {
              setRightPower(0);
              rightClimberEncoder.setPosition(0);
            });
  }

  public Command zeroBoth() {
    return run(() -> {
          setRightPower(-0.2);
          setLeftPower(-0.2);
        })
        .until(() -> leftLimitSwitch.get() && rightLimitSwitch.get())
        .finallyDo(
            () -> {
              setRightPower(0);
              setLeftPower(0);
              resetEncoders();
            });
  }
}
