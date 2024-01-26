/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.revrobotics.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.robolancers321.util.InterpolationTable;

public class Shooter extends SubsystemBase {
  private static Shooter instance = null;

  public static Shooter getInstance() {
    if (instance == null) instance = new Shooter();

    return instance;
  }

  /* Constants */

  private static final int kLeftMotorID = 0;
  private static final int kRightMotorID = 0;
  private static final int kBeamBreakPort = 0;

  private static final int kCurrentLimit = 40;

  private static final boolean kInvertLeftMotor = false;
  private static final boolean kInvertRightMotor = false;

  // TODO: store this better?
  private static final double kAmpLeftSpeed = 0.0;
  private static final double kRightAmpSpeed = 0.0;

  private static double kRampUpRate = 0.5;

  private static final double kFF = 0.0;

  private final double kErrorTolerance = 0.0;
  private final double kInterpolationCacheThreshold =
      0.0; // the distance at which interpolation table recalculates setpoints

  /*
   * Implementation
   */

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private SparkPIDController leftController;
  private SparkPIDController rightController;

  private SlewRateLimiter leftLimiter;
  private SlewRateLimiter rightLimiter;

  private DigitalInput beamBreak;

  private double latestDistance = 0.0;
  private InterpolationTable.AimCharacteristic latestCharacteristic = null;

  private double leftSetpoint = 0.0;
  private double rightSetpoint = 0.0;

  private Shooter() {
    this.leftMotor = new CANSparkMax(kLeftMotorID, CANSparkLowLevel.MotorType.kBrushless);
    this.rightMotor = new CANSparkMax(kRightMotorID, CANSparkLowLevel.MotorType.kBrushless);

    this.leftEncoder = this.leftMotor.getEncoder();
    this.rightEncoder = this.rightMotor.getEncoder();

    this.leftController = this.leftMotor.getPIDController();
    this.rightController = this.rightMotor.getPIDController();

    this.leftLimiter = new SlewRateLimiter(kRampUpRate);
    this.rightLimiter = new SlewRateLimiter(kRampUpRate);

    this.beamBreak = new DigitalInput(kBeamBreakPort);

    this.configureMotors();
    this.configureControllers();
    this.configureEncoders();
  }

  private void configureControllers() {
    this.leftController.setFF(kFF);
    this.rightController.setFF(kFF);
  }

  private void configureMotors() {
    this.leftMotor.setInverted(kInvertLeftMotor);
    this.leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.leftMotor.setSmartCurrentLimit(kCurrentLimit);
    this.leftMotor.enableVoltageCompensation(12);
    this.leftMotor.burnFlash();

    this.rightMotor.setInverted(kInvertRightMotor);
    this.rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.rightMotor.setSmartCurrentLimit(kCurrentLimit);
    this.rightMotor.enableVoltageCompensation(12);
    this.rightMotor.burnFlash();
  }

  private void configureEncoders() {
    this.leftEncoder.setInverted(kInvertLeftMotor);
    this.rightEncoder.setInverted(kInvertRightMotor);
  }

  private double getLeftVelocity() {
    // TODO: filter here?
    return this.leftEncoder.getVelocity();
  }

  private double getRightVelocity() {
    // TODO: filter here?
    return this.rightEncoder.getVelocity();
  }

  private void useControllers(double leftSpeed, double rightSpeed) {
    this.leftController.setReference(
        this.leftLimiter.calculate(leftSpeed), CANSparkBase.ControlType.kVelocity);
    this.rightController.setReference(
        this.rightLimiter.calculate(rightSpeed), CANSparkBase.ControlType.kVelocity);
  }

  public void yeetNoteSpeaker(double distance) {
    if (!epsilonEquals(distance, latestDistance, kInterpolationCacheThreshold)) {
      latestDistance = distance;
      latestCharacteristic = InterpolationTable.interpolate(distance);
    }

    this.leftSetpoint = latestCharacteristic.getLeftSpeed();
    this.rightSetpoint = latestCharacteristic.getRightSpeed();

    this.useControllers(this.leftSetpoint, this.rightSetpoint);
  }

  public Command yeetNoteSpeaker(DoubleSupplier distance) {
    // TODO: until beam break goes from true to false, also maybe add a time delay
    return run(() -> this.yeetNoteSpeaker(distance.getAsDouble())).until(beamBreak::get);
  }

  public boolean atLeftSetpoint() {
    return epsilonEquals(this.getLeftVelocity(), this.leftSetpoint, kErrorTolerance);
  }

  public boolean atRightSetpoint() {
    return epsilonEquals(this.getRightVelocity(), this.rightSetpoint, kErrorTolerance);
  }

  public boolean atSetpoint() {
    return this.atLeftSetpoint() && this.atLeftSetpoint();
  }

  public void dangerouslyYeet(double leftSpeed, double rightSpeed) {
    this.leftMotor.set(leftSpeed);
    this.rightMotor.set(rightSpeed);
  }

  public void yeetNoteAmp(double distance) {
    this.leftSetpoint = kAmpLeftSpeed;
    this.rightSetpoint = kRightAmpSpeed;

    this.useControllers(this.leftSetpoint, this.rightSetpoint);
  }

  public Command yeetNoteAmp() {
    // TODO: is this the right end condition?
    return run(this::yeetNoteAmp).until(beamBreak::get);
  }

  private void doSendables() {

    SmartDashboard.putNumber("Left flywheel velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right flywheel velocity", getRightVelocity());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  public void initTuning() {
    SmartDashboard.putNumber("shooter kff", SmartDashboard.getNumber("shooter kff", kFF));
  }

  public void tune() {
    double tunedFF = SmartDashboard.getNumber("shooter kff", kFF);

    leftController.setFF(tunedFF);
    rightController.setFF(tunedFF);
  }
}
