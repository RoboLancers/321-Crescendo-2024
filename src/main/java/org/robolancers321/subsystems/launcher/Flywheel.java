/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Flywheel extends SubsystemBase {
  private static Flywheel instance = null;

  public static Flywheel getInstance() {
    if (instance == null) instance = new Flywheel();

    return instance;
  }

  /* Constants */

  private static final int kMotorPort = 17;

  private static final boolean kInvertMotor = false;
  private static final int kCurrentLimit = 40;

  private static final double kRampUpRate = 0.5;

  private static final double kFF = 0.0;

  private final double kErrorTolerance = 0.0;

  /*
   * Implementation
   */

  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController controller;

  private final SlewRateLimiter limiter;

  private double goalRPM = 0.0;

  private Flywheel() {
    this.motor = new CANSparkFlex(kMotorPort, CANSparkLowLevel.MotorType.kBrushless);

    this.encoder = this.motor.getEncoder();

    this.controller = this.motor.getPIDController();

    this.limiter = new SlewRateLimiter(kRampUpRate);

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setVelocityConversionFactor(1.0);
  }

  private void configureController() {
    this.controller.setP(0.0);
    this.controller.setI(0.0);
    this.controller.setD(0.0);
    this.controller.setFF(kFF);
  }

  private double getRPM() {
    // TODO: filter here?
    return this.encoder.getVelocity();
  }

  private void useController() {
    this.controller.setReference(this.goalRPM, ControlType.kVelocity);

    // For tuning, no limiter
    // this.controller.setReference(
    //     this.limiter.calculate(this.goalRPM), CANSparkBase.ControlType.kVelocity);
  }

  public boolean isRevved() {
    return epsilonEquals(this.getRPM(), this.goalRPM, kErrorTolerance);
  }

  private void dangerouslySetSpeed(double speed) {
    this.motor.set(speed);
  }

  private void doSendables() {
    SmartDashboard.putNumber("flywheel rpm", this.getRPM());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("shooter kff", SmartDashboard.getNumber("shooter kff", kFF));

    SmartDashboard.putNumber("target shooter rpm", 0.0);
  }

  private void tune() {
    double tunedFF = SmartDashboard.getNumber("shooter kff", kFF);

    this.controller.setFF(tunedFF);

    this.goalRPM = SmartDashboard.getNumber("target shooter rpm", 0.0);

    this.useController();
  }

  public Command dangerouslyYeet(double speed) {
    return run(() -> this.dangerouslySetSpeed(speed))
        .finallyDo(() -> this.dangerouslySetSpeed(0.0));
  }

  private Command off() {
    return runOnce(
        () -> {
          this.goalRPM = 0.0;
          this.useController();
        });
  }

  public Command yeetNoteAmp() {
    this.goalRPM = AimTable.kAmpAimCharacteristic.rpm;

    return run(this::useController).finallyDo(this::off);
  }

  public Command yeetNoteSpeaker(DoubleSupplier rpmSupplier) {
    return run(() -> {
          this.goalRPM = rpmSupplier.getAsDouble();
          this.useController();
        })
        .finallyDo(this::off);
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune).finallyDo(this::off);
  }
}
