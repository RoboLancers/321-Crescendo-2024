/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.revrobotics.*;
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

  private static final double kRampUpRate = 8000;

  private static final double kFF = 0.000153;

  private static final double kToleranceRPM = 40.0;

  private enum FlywheelSetpoint {
    kAmp(1000.0),
    kSpeaker(6400.0);

    public final double rpm;

    private FlywheelSetpoint(double rpm) {
      this.rpm = rpm;
    }
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
    // TODO
    this.controller.setReference(
        this.limiter.calculate(this.goalRPM), CANSparkBase.ControlType.kVelocity);
  }

  public boolean isRevved() {
    return epsilonEquals(this.getRPM(), this.goalRPM, kToleranceRPM);
  }

  private void doSendables() {
    SmartDashboard.putNumber("flywheel rpm", this.getRPM());

    SmartDashboard.putNumber("flywheel mp goal (rpm)", this.goalRPM);
  }

  @Override
  public void periodic() {
    this.useController();

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("flywheel kff", SmartDashboard.getNumber("flywheel kff", kFF));
    SmartDashboard.putNumber("flywheel target rpm", 0.0);
  }

  private void tune() {
    double tunedFF = SmartDashboard.getNumber("flywheel kff", kFF);

    this.controller.setFF(tunedFF);

    this.goalRPM = SmartDashboard.getNumber("flywheel target rpm", 0.0);

    this.useController();
  }

  public Command off() {
    return runOnce(
        () -> {
          this.goalRPM = 0.0;
        });
  }

  public Command yeetNoteAmp() {
    return runOnce(
        () -> {
          this.goalRPM = FlywheelSetpoint.kAmp.rpm;
        });
  }

  public Command yeetNoteSpeaker(DoubleSupplier rpmSupplier) {
    return runOnce(
        () -> {
          this.goalRPM = rpmSupplier.getAsDouble();
        });
  }

  public Command yeetNoteSpeakerFixed() {
    return runOnce(
        () -> {
          this.goalRPM = FlywheelSetpoint.kSpeaker.rpm;
        });
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune);
  }
}
