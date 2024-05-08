/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.robolancers321.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final PIDController controller;

  public final Trigger atSetpoint;

  public Climber(ClimbConstants.Config config) {
    this.motor = new CANSparkMax(config.motorId(), MotorType.kBrushless);

    this.encoder = this.motor.getEncoder();

    this.controller = new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD);

    this.atSetpoint = new Trigger(this.controller::atSetpoint);

    this.setName(config.name());

    configMotor(config);
    configEncoder();
    configController();

    initLog();
  }

  private void configMotor(ClimbConstants.Config config) {
    this.motor.setInverted(config.motorInverted());

    this.motor.setIdleMode(IdleMode.kBrake);
    this.motor.enableVoltageCompensation(12.0);
    this.motor.setSmartCurrentLimit(ClimbConstants.kCurrentLimit);

    this.motor.setSoftLimit(SoftLimitDirection.kForward, ClimbConstants.kMaxSoftLimit);
    this.motor.setSoftLimit(SoftLimitDirection.kReverse, ClimbConstants.kMinSoftLimit);
    this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  private void configEncoder() {
    this.encoder.setPositionConversionFactor(ClimbConstants.kMetersPerRot);
    this.encoder.setVelocityConversionFactor(ClimbConstants.kRPMToMPS);

    this.encoder.setPosition(0);
  }

  private void configController() {
    this.controller.setTolerance(ClimbConstants.kErrorTolerance);
  }

  private void initLog() {
    final var tab = Shuffleboard.getTab(this.getName());

    tab.addDouble("Position (m)", this.encoder::getPosition);
    tab.addDouble("Applied Output", this.motor::getAppliedOutput);
    tab.addDouble("PID Output (V)", () -> this.controller.calculate(this.encoder.getPosition()));
  }

  public Command tune(
      DoubleSupplier setpoint, DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD) {
    return runOnce(
            () -> this.controller.setPID(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble()))
        .andThen(goTo(setpoint));
  }

  /* Does not end, will hold setpoint */
  public Command goTo(DoubleSupplier setpoint) {
    return runOnce(() -> this.controller.setSetpoint(setpoint.getAsDouble()))
        .andThen(runVolts(() -> this.controller.calculate(this.encoder.getPosition())));
    // .until(this.controller::atSetpoint));
  }

  public Command runPower(DoubleSupplier power) {
    return runVolts(() -> 12.0 * power.getAsDouble());
  }

  /* Non-terminating, which is typically easier to extend to terminating than the other way around */
  public Command runVolts(DoubleSupplier volts) {
    return run(() -> this.motor.setVoltage(volts.getAsDouble()))
        .finallyDo(() -> this.motor.setVoltage(0));
  }

  public Command stop() {
    return runOnce(() -> this.motor.setVoltage(0));
  }
}
