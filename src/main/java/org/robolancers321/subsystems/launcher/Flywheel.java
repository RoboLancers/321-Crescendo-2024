/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import org.robolancers321.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  private static Flywheel instance = null;

  public static Flywheel getInstance() {
    if (instance == null) instance = new Flywheel();

    return instance;
  }

  /*
   * Implementation
   */

  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController controller;

  private final SlewRateLimiter limiter;

  private double goalRPM = 0.0;

  private Flywheel() {
    this.motor =
        new CANSparkFlex(FlywheelConstants.kMotorPort, CANSparkLowLevel.MotorType.kBrushless);

    this.encoder = this.motor.getEncoder();

    this.controller = this.motor.getPIDController();

    this.limiter = new SlewRateLimiter(FlywheelConstants.kRampUpRate);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(FlywheelConstants.kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(FlywheelConstants.kCurrentLimit);
    this.motor.enableVoltageCompensation(12);

    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 30000);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 30000);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 30000);
    this.motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 30000);
  }

  private void configureEncoder() {
    this.encoder.setVelocityConversionFactor(1.0);
  }

  private void configureController() {
    this.controller.setP(0.0);
    this.controller.setI(0.0);
    this.controller.setD(0.0);
    this.controller.setFF(FlywheelConstants.kFF);
  }

  private double getRPM() {
    return this.encoder.getVelocity();
  }

  public double getGoalRPM() {
    return this.goalRPM;
  }

  private void useController() {
    if (this.goalRPM - this.getRPM() > 2 * FlywheelConstants.kToleranceRPM)
      this.controller.setFF(10 * FlywheelConstants.kFF);
    else this.controller.setFF(FlywheelConstants.kFF);

    this.controller.setReference(
        this.limiter.calculate(this.goalRPM), CANSparkBase.ControlType.kVelocity);
  }

  public boolean isRevved() {
    return epsilonEquals(this.getRPM(), this.goalRPM, FlywheelConstants.kToleranceRPM);
  }

  private void doSendables() {
    SmartDashboard.putNumber("flywheel rpm", this.getRPM());
    SmartDashboard.putNumber("flywheel voltage", this.motor.getBusVoltage());
    SmartDashboard.putNumber("flywheel current (amps)", this.motor.getOutputCurrent());
    SmartDashboard.putBoolean("flywheel isRevved", this.isRevved());

    SmartDashboard.putNumber("flywheel mp goal (rpm)", this.goalRPM);
  }

  @Override
  public void periodic() {
    this.useController();

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber(
        "flywheel kff", SmartDashboard.getNumber("flywheel kff", FlywheelConstants.kFF));
    SmartDashboard.putNumber("flywheel target rpm", 0.0);
  }

  private void tune() {
    double tunedFF = SmartDashboard.getNumber("flywheel kff", FlywheelConstants.kFF);

    this.controller.setFF(tunedFF);

    this.goalRPM = SmartDashboard.getNumber("flywheel target rpm", 0.0);
  }

  public Command off() {
    return runOnce(
        () -> {
          this.goalRPM = 0.0;
        });
  }

  public Command acceptHandoff() {
    return runOnce(
        () -> {
          this.goalRPM = FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm;
        });
  }

  public Command shiftForward() {
    return runOnce(
        () -> {
          this.goalRPM = FlywheelConstants.FlywheelSetpoint.kShiftForward.rpm;
        });
  }

  public Command shiftBackwardFast() {
    return runOnce(
        () -> {
          this.goalRPM = FlywheelConstants.FlywheelSetpoint.kShiftBackwardFast.rpm;
        });
  }

  public Command shiftBackwardSlow() {
    return runOnce(
        () -> {
          this.goalRPM = FlywheelConstants.FlywheelSetpoint.kShiftBackwardSlow.rpm;
        });
  }

  public Command revAmp() {
    return runOnce(
            () -> {
              this.goalRPM = FlywheelConstants.FlywheelSetpoint.kAmp.rpm;
            })
        .alongWith(new WaitUntilCommand(this::isRevved));
  }

  public Command revSpeaker() {
    return runOnce(
            () -> {
              this.goalRPM = FlywheelConstants.FlywheelSetpoint.kSpeaker.rpm;
            })
        .alongWith(new WaitUntilCommand(this::isRevved).withTimeout(4.0));
  }

  public Command revTrap() {
    return runOnce(
            () -> {
              this.goalRPM = FlywheelConstants.FlywheelSetpoint.kTrap.rpm;
            })
        .alongWith(new WaitUntilCommand(this::isRevved));
  }

  public Command revFeeder() {
    return runOnce(
            () -> {
              this.goalRPM = FlywheelConstants.FlywheelSetpoint.kFeeder.rpm;
            })
        .alongWith(new WaitUntilCommand(this::isRevved));
  }

  public Command revSpeakerFromRPM(DoubleSupplier rpm) {
    return runOnce(
            () -> {
              this.goalRPM = rpm.getAsDouble();
            })
        .alongWith(new WaitUntilCommand(this::isRevved));
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune);
  }
}
