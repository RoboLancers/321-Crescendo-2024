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

  private static final int kTopMotorPort = 0;
  private static final int kBottomMotorPort = 0;

  private static final int kCurrentLimit = 40;

  private static final boolean kInvertTopMotor = false;
  private static final boolean kInvertBottomMotor = false;

  private static final double kRampUpRate = 0.5;

  private static final double kFF = 0.0;

  private final double kErrorTolerance = 0.0;

  /*
   * Implementation
   */

  private final CANSparkMax topMotor;
  private final CANSparkMax bottomMotor;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private final SparkPIDController topController;
  private final SparkPIDController bottomController;

  private final SlewRateLimiter topLimiter;
  private final SlewRateLimiter bottomLimiter;

  private double goalRPM = 0.0;

  private Flywheel() {
    this.topMotor = new CANSparkMax(kTopMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    this.bottomMotor = new CANSparkMax(kBottomMotorPort, CANSparkLowLevel.MotorType.kBrushless);

    this.topEncoder = this.topMotor.getEncoder();
    this.bottomEncoder = this.bottomMotor.getEncoder();

    this.topController = this.topMotor.getPIDController();
    this.bottomController = this.bottomMotor.getPIDController();

    this.topLimiter = new SlewRateLimiter(kRampUpRate);
    this.bottomLimiter = new SlewRateLimiter(kRampUpRate);

    this.configureMotors();
    this.configureEncoders();
    this.configureControllers();
  }

  private void configureMotors() {
    this.topMotor.setInverted(kInvertTopMotor);
    this.topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.topMotor.setSmartCurrentLimit(kCurrentLimit);
    this.topMotor.enableVoltageCompensation(12);
    this.topMotor.burnFlash();

    this.bottomMotor.setInverted(kInvertBottomMotor);
    this.bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.bottomMotor.setSmartCurrentLimit(kCurrentLimit);
    this.bottomMotor.enableVoltageCompensation(12);
    this.bottomMotor.burnFlash();
  }

  private void configureEncoders() {
    this.topEncoder.setVelocityConversionFactor(1.0);
    this.bottomEncoder.setVelocityConversionFactor(1.0);
  }

  private void configureControllers() {
    this.topController.setP(0.0);
    this.topController.setI(0.0);
    this.topController.setD(0.0);
    this.topController.setFF(kFF);

    this.bottomController.setP(0.0);
    this.bottomController.setI(0.0);
    this.bottomController.setD(0.0);
    this.bottomController.setFF(kFF);
  }

  private double getTopRPM() {
    // TODO: filter here?
    return this.topEncoder.getVelocity();
  }

  private double getBottomRPM() {
    // TODO: filter here?
    return this.bottomEncoder.getVelocity();
  }

  private void useControllers() {
    this.topController.setReference(
        this.topLimiter.calculate(this.goalRPM), CANSparkBase.ControlType.kVelocity);
    this.bottomController.setReference(
        this.bottomLimiter.calculate(this.goalRPM), CANSparkBase.ControlType.kVelocity);
  }

  private boolean isTopRevved() {
    return epsilonEquals(this.getTopRPM(), this.goalRPM, kErrorTolerance);
  }

  private boolean isBottomRevved() {
    return epsilonEquals(this.getBottomRPM(), this.goalRPM, kErrorTolerance);
  }

  public boolean isRevved() {
    return this.isTopRevved() && this.isBottomRevved();
  }

  private void dangerouslySetSpeed(double speed) {
    this.topMotor.set(speed);
    this.bottomMotor.set(speed);
  }

  private void doSendables() {
    SmartDashboard.putNumber("top flywheel rpm", this.getTopRPM());
    SmartDashboard.putNumber("bottom flywheel rpm", this.getBottomRPM());
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

    this.topController.setFF(tunedFF);
    this.bottomController.setFF(tunedFF);

    this.goalRPM = SmartDashboard.getNumber("target shooter rpm", 0.0);

    this.useControllers();
  }

  public Command dangerouslyYeet(double speed) {
    return run(() -> this.dangerouslySetSpeed(speed))
        .finallyDo(() -> this.dangerouslySetSpeed(0.0));
  }

  private Command off() {
    return runOnce(
        () -> {
          this.goalRPM = 0.0;
          this.useControllers();
        });
  }

  public Command yeetNoteAmp() {
    this.goalRPM = AimTable.kAmpAimCharacteristic.rpm;

    return run(this::useControllers).finallyDo(this::off);
  }

  public Command yeetNoteSpeaker(DoubleSupplier rpmSupplier) {
    return run(() -> {
          this.goalRPM = rpmSupplier.getAsDouble();
          this.useControllers();
        })
        .finallyDo(this::off);
  }

  public Command tuneController() {
    this.initTuning();

    return run(this::tune).finallyDo(this::off);
  }
}
