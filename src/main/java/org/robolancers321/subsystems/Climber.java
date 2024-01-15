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
  /*
   * Singleton
   */

  private static Climber instance = null;
  public static Climber getInstance(){
    if (instance == null) instance = new Climber();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kLeftClimberPort = 0;
  private static final int kLeftLimitSwitchPort = 0;
  private static final boolean kLeftClimberInverted = false;

  private static final int kRightClimberPort = 0;
  private static final int kRightLimitSwitchPort = 1;
  private static final boolean kRightClimberInverted = false;

  private static final int kCurrentLimit = 40;
  private static final double kMetersPerRot = 1;
  private static final double kRPMToMPS = kMetersPerRot / 60.0;

  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;

  private static final double kErrorTolerance = 0.1;

  // used to zero the climber at a safe speed
  private static final double kDownwardZeroingSpeed = -0.2;

  /*
   * Implementation
   */

  private final CANSparkMax leftClimberMotor;
  private final CANSparkMax rightClimberMotor;

  private final RelativeEncoder leftClimberEncoder;
  private final RelativeEncoder rightClimberEncoder;

  private final PIDController leftClimberPID;
  private final PIDController rightClimberPID;

  private final DigitalInput leftLimitSwitch;
  private final DigitalInput rightLimitSwitch;

  private Climber() {
    this.leftClimberMotor = new CANSparkMax(kLeftClimberPort, MotorType.kBrushless);
    this.rightClimberMotor = new CANSparkMax(kRightClimberPort, MotorType.kBrushless);

    this.leftClimberEncoder = leftClimberMotor.getEncoder();
    this.rightClimberEncoder = rightClimberMotor.getEncoder();

    this.leftClimberPID = new PIDController(kP, kI, kD);
    this.rightClimberPID = new PIDController(kP, kI, kD);

    this.leftLimitSwitch = new DigitalInput(kLeftLimitSwitchPort);
    this.rightLimitSwitch = new DigitalInput(kRightLimitSwitchPort);

    configMotors();
    configEncoders();
    configController();
  }

  // TODO: soft limits
  private void configMotors() {
    leftClimberMotor.setInverted(kLeftClimberInverted);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setSmartCurrentLimit(kCurrentLimit);
    leftClimberMotor.enableVoltageCompensation(12.0);

    rightClimberMotor.setInverted(kRightClimberInverted);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setSmartCurrentLimit(kCurrentLimit);
    rightClimberMotor.enableVoltageCompensation(12.0);

    // leftClimberMotor.burnFlash();
    // rightClimberMotor.burnFlash();
  }

  private void configEncoders() {
    leftClimberEncoder.setPositionConversionFactor(kMetersPerRot);
    rightClimberEncoder.setPositionConversionFactor(kMetersPerRot);

    leftClimberEncoder.setVelocityConversionFactor(kRPMToMPS);
    rightClimberEncoder.setVelocityConversionFactor(kRPMToMPS);

    resetEncoders();
  }

  private void resetLeftEncoder(){
    leftClimberEncoder.setPosition(0);
  }

  private void resetRightEncoder(){
    rightClimberEncoder.setPosition(0);
  }

  private void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  private void configController(){
    this.leftClimberPID.setTolerance(kErrorTolerance);
    this.rightClimberPID.setTolerance(kErrorTolerance);
  }

  private double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  private double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }

  private double getLeftClimberVelocity() {
    return leftClimberEncoder.getVelocity();
  }

  private double getRightClimberVelocity() {
    return rightClimberEncoder.getVelocity();
  }

  private void setLeftClimberSetpoint(double setpoint) {
    leftClimberPID.setSetpoint(setpoint);
  }

  private void setRightClimberSetpoint(double setpoint) {
    rightClimberPID.setSetpoint(setpoint);
  }

  private void setLeftPower(double speed) {
    leftClimberMotor.set(speed);
  }

  private void setRightPower(double speed) {
    rightClimberMotor.set(speed);
  }

  private void doSendables(){
    // TODO: log position, velocity, limit switch, controller output
  }

  @Override
  public void periodic(){
    doSendables();
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
    return run(() -> setLeftPower(kDownwardZeroingSpeed))
        .until(leftLimitSwitch::get)
        .finallyDo(
            () -> {
              setLeftPower(0);
              resetLeftEncoder();
            });
  }

  public Command zeroRight() {
    return run(() -> setRightPower(kDownwardZeroingSpeed))
        .until(rightLimitSwitch::get)
        .finallyDo(
            () -> {
              setRightPower(0);
              resetRightEncoder();
            });
  }

  public Command zeroBoth() {
    return run(() -> {
          if (!leftLimitSwitch.get()) setLeftPower(kDownwardZeroingSpeed);
          if (!rightLimitSwitch.get()) setRightPower(kDownwardZeroingSpeed);
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
