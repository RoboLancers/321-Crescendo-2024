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
  private static Shooter INSTANCE;

  public static Shooter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Shooter();
    }
    return INSTANCE;
  }

  /* Constants */

  private final int kLeftMotorID = 0;
  private final int kRightMotorID = 0;

  private int kLeftCurrentLimit = 40;

  private int kRightCurrentLimit = 40;

  private boolean kInvertLeftMotor = false;

  private boolean kInvertRightMotor = false;

  private double kLeftRotPerMinToDegPerSec = 0.0;

  private double kRightRotPerMinToDegPerSec = 0.0;

  private double kLeftFF = 0.0;

  private double kRightFF = 0.0;

  private double kAmpLeftSpeed = 0.0;

  private double kRightAmpSpeed = 0.0;

  private double kRampUpRate = 0.5;

  private double kErrorTolerance = 0.0;

  private double kInterpolationThreshold = 0.0;

  private int kBeamBreakChannelPort = 0;
  private double leftSetpointSpeed;

  private double rightSetpointSpeed;

  private CANSparkMax left_motor;

  private CANSparkMax right_motor;

  private AbsoluteEncoder left_encoder;

  private AbsoluteEncoder right_encoder;

  private double latest_distance = 0.0;

  private InterpolationTable.AimCharacteristic latest_characteristic;

  private SparkPIDController left_controller;
  private SparkPIDController right_controller;

  private SlewRateLimiter left_limiter;
  private SlewRateLimiter right_limiter;

  private DigitalInput beam_break;

  private Shooter() {

    beam_break = new DigitalInput(kBeamBreakChannelPort);

    left_motor = new CANSparkMax(kLeftMotorID, CANSparkLowLevel.MotorType.kBrushless);
    right_motor = new CANSparkMax(kRightMotorID, CANSparkLowLevel.MotorType.kBrushless);

    left_encoder = left_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    right_encoder = right_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    left_controller = left_motor.getPIDController();
    right_controller = right_motor.getPIDController();

    left_limiter = new SlewRateLimiter(kRampUpRate);
    right_limiter = new SlewRateLimiter(kRampUpRate);

    configureMotors();
    configureControllers();
    configureEncoders();
  }

  private void configureControllers() {
    left_controller.setFF(kLeftFF);
    right_controller.setFF(kRightFF);
  }

  private void configureMotors() {
    left_motor.setInverted(kInvertLeftMotor);
    left_motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    left_motor.setSmartCurrentLimit(kLeftCurrentLimit);
    left_motor.enableVoltageCompensation(12);
    left_motor.burnFlash();

    right_motor.setInverted(kInvertRightMotor);
    right_motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    right_motor.setSmartCurrentLimit(kRightCurrentLimit);
    right_motor.enableVoltageCompensation(12);
    right_motor.burnFlash();
  }

  private void configureEncoders() {
    left_encoder.setInverted(kInvertLeftMotor);
    left_encoder.setVelocityConversionFactor(kLeftRotPerMinToDegPerSec);

    right_encoder.setInverted(kInvertRightMotor);
    right_encoder.setVelocityConversionFactor(kRightRotPerMinToDegPerSec);
  }

  private double getLeftVelocity() {
    return left_encoder.getVelocity();
  }

  private double getRightVelocity() {
    return right_encoder.getVelocity();
  }

  private void setGoalLR(double leftSpeed, double rightSpeed) {
    left_controller.setReference(
        left_limiter.calculate(leftSpeed), CANSparkBase.ControlType.kVelocity);
    right_controller.setReference(
        right_limiter.calculate(rightSpeed), CANSparkBase.ControlType.kVelocity);
  }

  public void yeetNoteSpeaker(double distance) {
    if (!epsilonEquals(distance, latest_distance, kInterpolationThreshold))
      latest_characteristic = InterpolationTable.interpolate(distance);

    leftSetpointSpeed = latest_characteristic.getLeftSpeed();
    rightSetpointSpeed = latest_characteristic.getRightSpeed();

    setGoalLR(leftSetpointSpeed, rightSetpointSpeed);
  }

  public Command yeetNoteSpeaker(DoubleSupplier distance) {
    return run(() -> yeetNoteSpeaker(distance.getAsDouble())).until(beam_break::get);
  }

  public void setRampUpRate(double rate) {
    kRampUpRate = rate;
  }

  public boolean atLeftSetpoint() {
    return epsilonEquals(getLeftVelocity(), leftSetpointSpeed, kErrorTolerance);
  }

  public boolean atRightSetpoint() {
    return epsilonEquals(getRightVelocity(), leftSetpointSpeed, kErrorTolerance);
  }

  public boolean atSetpoint() {
    return atLeftSetpoint() && atLeftSetpoint();
  }

  public void yeet(double speed) {
    left_motor.set(speed);
    right_motor.set(-speed);
  }

  public void yeetNoteAmp(double distance) {
    leftSetpointSpeed = kAmpLeftSpeed;
    rightSetpointSpeed = kRightAmpSpeed;
    setGoalLR(kAmpLeftSpeed, kRightAmpSpeed);
  }

  public Command yeetNoteAmp() {
    return run(this::yeetNoteAmp).until(beam_break::get);
  }

  @Override
  public void periodic() {
    doSendables();
  }

  private void doSendables() {

    SmartDashboard.putNumber("Left flywheel velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right flywheel velocity", getRightVelocity());
  }

  public void initTuning() {
    SmartDashboard.putNumber("Left FF", kLeftFF);
    SmartDashboard.putNumber("Right FF", kRightFF);
  }

  public void tune() {
    double leftFF = SmartDashboard.getNumber("Left FF", kLeftFF);
    double rightFF = SmartDashboard.getNumber("Right FF", kRightFF);
    left_controller.setFF(leftFF);

    right_controller.setFF(rightFF);
  }
}
