/* (C) Robolancers 2024 */
package org.robolancers321.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.robolancers321.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /*
   * Singleton
   */

  private static Climber instance = null;

  public static Climber getInstance() {
    if (instance == null) instance = new Climber();

    return instance;
  }

  /*
   * Implementation
   */

  private final CANSparkMax leftClimberMotor;
  private final CANSparkMax rightClimberMotor;

  private final RelativeEncoder leftClimberEncoder;
  private final RelativeEncoder rightClimberEncoder;

  private final PIDController leftClimberPID;
  private final PIDController rightClimberPID;

  // private final DigitalInput leftLimitSwitch;
  // private final DigitalInput rightLimitSwitch;

  private Climber() {
    this.leftClimberMotor =
        new CANSparkMax(ClimberConstants.kLeftClimberPort, MotorType.kBrushless);
    this.rightClimberMotor =
        new CANSparkMax(ClimberConstants.kRightClimberPort, MotorType.kBrushless);

    this.leftClimberEncoder = leftClimberMotor.getEncoder();
    this.rightClimberEncoder = rightClimberMotor.getEncoder();

    this.leftClimberPID =
        new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
    this.rightClimberPID =
        new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);

    // this.leftLimitSwitch = new DigitalInput(ClimberConstants.kLeftLimitSwitchPort);
    // this.rightLimitSwitch = new DigitalInput(ClimberConstants.kRightLimitSwitchPort);

    configMotors();
    configEncoders();
    configController();
  }

  // TODO: soft limits
  private void configMotors() {
    leftClimberMotor.setInverted(ClimberConstants.kLeftClimberInverted);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    leftClimberMotor.enableVoltageCompensation(12.0);
    leftClimberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kMaxSoftLimit);
    leftClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kMinSoftLimit);

    leftClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftClimberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    rightClimberMotor.setInverted(ClimberConstants.kRightClimberInverted);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.kCurrentLimit);
    rightClimberMotor.enableVoltageCompensation(12.0);
    rightClimberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kMaxSoftLimit);
    rightClimberMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kMinSoftLimit);

    rightClimberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // leftClimberMotor.burnFlash();
    // rightClimberMotor.burnFlash();
  }

  private void configEncoders() {
    leftClimberEncoder.setPositionConversionFactor(ClimberConstants.kMetersPerRot);
    rightClimberEncoder.setPositionConversionFactor(ClimberConstants.kMetersPerRot);

    resetEncoders();
  }

  private void resetLeftEncoder() {
    leftClimberEncoder.setPosition(0);
  }

  private void resetRightEncoder() {
    rightClimberEncoder.setPosition(0);
  }

  private void resetEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  private void configController() {
    this.leftClimberPID.setTolerance(ClimberConstants.kErrorTolerance);
    this.rightClimberPID.setTolerance(ClimberConstants.kErrorTolerance);
  }

  private double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  private double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }

  private void setLeftClimberSetpoint(double setpoint) {
    leftClimberPID.setSetpoint(setpoint);
  }

  private void setRightClimberSetpoint(double setpoint) {
    rightClimberPID.setSetpoint(setpoint);
  }

  public void setLeftPower(double speed) {
    leftClimberMotor.set(speed);
  }

  public void setRightPower(double speed) {
    rightClimberMotor.set(speed);
  }

  private void doSendables() {
    // TODO: log position, velocity, limit switch, controller output
    // tried new naming convention from 6328 Mechanical Advantage

    SmartDashboard.putNumber("climberLeft Position", getLeftClimberPosition());
    // SmartDashboard.putBoolean("climberLeft LimitSwitch", leftLimitSwitch.get());
    SmartDashboard.putNumber("climberLeft MotorOutput", leftClimberMotor.getAppliedOutput());
    SmartDashboard.putNumber(
        "climberLeft PIDOutput", leftClimberPID.calculate(getLeftClimberPosition()));

    SmartDashboard.putNumber("climberRight Position", getRightClimberPosition());
    // SmartDashboard.putBoolean("climberRight LimitSwitch", rightLimitSwitch.get());
    SmartDashboard.putNumber("climberRight MotorOutput", rightClimberMotor.getAppliedOutput());
    SmartDashboard.putNumber(
        "climberRight PIDOutput", rightClimberPID.calculate(getRightClimberPosition()));
  }

  private void initTuning() {
    SmartDashboard.putNumber(
        "climber kP", SmartDashboard.getNumber("climber kP", ClimberConstants.kP));
    SmartDashboard.putNumber(
        "climber kI", SmartDashboard.getNumber("climber kI", ClimberConstants.kI));
    SmartDashboard.putNumber(
        "climber kD", SmartDashboard.getNumber("climber kD", ClimberConstants.kD));

    SmartDashboard.putNumber("climber target position (deg)", 0);
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("climber kP", ClimberConstants.kP);
    double tunedI = SmartDashboard.getNumber("climber kI", ClimberConstants.kI);
    double tunedD = SmartDashboard.getNumber("climber kD", ClimberConstants.kD);

    leftClimberPID.setPID(tunedP, tunedI, tunedD);
    rightClimberPID.setPID(tunedP, tunedI, tunedD);

    double setpoint =
        MathUtil.clamp(
            SmartDashboard.getNumber("climber target position (deg)", 0),
            ClimberConstants.kMinSoftLimit,
            ClimberConstants.kMaxSoftLimit);

    setRightClimberSetpoint(setpoint);
    setLeftClimberSetpoint(setpoint);
  }

  @Override
  public void periodic() {
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
        .until(() -> leftClimberPID.atSetpoint() && rightClimberPID.atSetpoint());
  }

  // public Command zeroLeft() {
  //   return run(() -> setLeftPower(ClimberConstants.kDownwardZeroingSpeed))
  //       .until(leftLimitSwitch::get)
  //       .finallyDo(
  //           () -> {
  //             setLeftPower(0);
  //             resetLeftEncoder();
  //           });
  // }

  // public Command zeroRight() {
  //   return run(() -> setRightPower(ClimberConstants.kDownwardZeroingSpeed))
  //       .until(rightLimitSwitch::get)
  //       .finallyDo(
  //           () -> {
  //             setRightPower(0);
  //             resetRightEncoder();
  //           });
  // }

  // public Command zeroBoth() {
  //   return run(() -> {
  //         if (!leftLimitSwitch.get()) setLeftPower(ClimberConstants.kDownwardZeroingSpeed);
  //         if (!rightLimitSwitch.get()) setRightPower(ClimberConstants.kDownwardZeroingSpeed);
  //       })
  //       .until(() -> leftLimitSwitch.get() && rightLimitSwitch.get())
  //       .finallyDo(
  //           () -> {
  //             setRightPower(0);
  //             setLeftPower(0);
  //             resetEncoders();
  //           });
  // }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}
