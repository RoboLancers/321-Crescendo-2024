/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  /*
   * Singletons
   */

  // TODO: change these for new robot

  private static SwerveModule frontLeft = null;

  public static SwerveModule getFrontLeft() {
    if (frontLeft == null)
      frontLeft = new SwerveModule("Front Left", 2, 1, 9, false, true, false, -0.265869140625);

    return frontLeft;
  }

  private static SwerveModule frontRight = null;

  public static SwerveModule getFrontRight() {
    if (frontRight == null)
      frontRight = new SwerveModule("Front Right", 4, 3, 10, true, true, false, 0.291748046875);

    return frontRight;
  }

  private static SwerveModule backLeft = null;

  public static SwerveModule getBackLeft() {
    if (backLeft == null)
      backLeft = new SwerveModule("Back Left", 8, 7, 12, true, true, false, -0.35986328125);

    return backLeft;
  }

  private static SwerveModule backRight = null;

  public static SwerveModule getBackRight() {
    if (backRight == null)
      backRight = new SwerveModule("Back Right", 6, 5, 11, true, true, false, -0.4599609375);

    return backRight;
  }

  /*
   * Constants
   */

  // TODO: change these for new robot

  private static final CANcoderConfiguration kCANCoderConfig = new CANcoderConfiguration();

  private static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
  private static final double kGearRatio = 6.8;
  private static final double kRPMToMPS = 2 * Math.PI * kWheelRadiusMeters / (kGearRatio * 60.0);

  private static final double kDriveP = 0.00;
  private static final double kDriveI = 0.00;
  private static final double kDriveD = 0.00;
  private static final double kDriveFF = 0.22;

  private static final double kTurnP = 0.4; // 0.50;
  private static final double kTurnI = 0.00;
  private static final double kTurnD = 0.005; // 0.005;

  /*
   * Implementation
   */

  private String id;

  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private CANcoder turnEncoder;

  private SparkPIDController driveController;
  private PIDController turnController;

  private SwerveModule(
      String id,
      int driveMotorPort,
      int turnMotorPort,
      int turnEncoderPort,
      boolean invertDriveMotor,
      boolean invertTurnMotor,
      boolean invertTurnEncoder,
      double turnEncoderOffset) {
    this.id = id;

    this.configDrive(driveMotorPort, invertDriveMotor);
    this.configTurn(
        turnMotorPort, turnEncoderPort, invertTurnMotor, invertTurnEncoder, turnEncoderOffset);
  }

  private void configDrive(int driveMotorPort, boolean invertDriveMotor) {
    this.driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);

    this.driveMotor.setInverted(invertDriveMotor);
    this.driveMotor.setIdleMode(IdleMode.kBrake);
    this.driveMotor.setSmartCurrentLimit(40);
    this.driveMotor.enableVoltageCompensation(12);
    this.driveMotor.burnFlash();

    this.driveEncoder = this.driveMotor.getEncoder();
    this.driveEncoder.setVelocityConversionFactor(kRPMToMPS);

    this.driveController = this.driveMotor.getPIDController();
    this.driveController.setP(kDriveP);
    this.driveController.setI(kDriveI);
    this.driveController.setD(kDriveD);
    this.driveController.setFF(kDriveFF);
  }

  private void configTurn(
      int turnMotorPort,
      int turnEncoderPort,
      boolean invertTurnMotor,
      boolean invertTurnEncoder,
      double turnEncoderOffset) {
    this.turnMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);

    this.turnMotor.setInverted(invertTurnMotor);
    this.turnMotor.setIdleMode(IdleMode.kBrake);
    this.turnMotor.setSmartCurrentLimit(40);
    this.turnMotor.enableVoltageCompensation(12);
    this.turnMotor.burnFlash();

    this.turnEncoder = new CANcoder(turnEncoderPort);

    CANcoderConfiguration config = kCANCoderConfig;
    config.MagnetSensor.withMagnetOffset(turnEncoderOffset);
    config.MagnetSensor.withSensorDirection(
        invertTurnEncoder
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive);

    this.turnEncoder.getConfigurator().apply(config);

    this.turnController = new PIDController(kTurnP, kTurnI, kTurnD);
    this.turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDriveVelocity() {
    return this.driveEncoder.getVelocity();
  }

  public double getTurnAngleRad() {
    return this.turnEncoder.getAbsolutePosition().getValue()
        * 2
        * Math.PI; // TODO: conversion rate?
  }

  public double getTurnAngleDeg() {
    return this.getTurnAngleRad() * 180.0 / Math.PI;
  }

  public SwerveModulePosition getPosition() {
    // TODO: should drive encoder position have a conversion factor?
    return new SwerveModulePosition(
        this.driveEncoder.getPosition(), Rotation2d.fromRadians(this.getTurnAngleRad()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        this.getDriveVelocity(), Rotation2d.fromRadians(this.getTurnAngleRad()));
  }

  public void update(SwerveModuleState desiredState) {
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(this.getTurnAngleRad()));

    this.driveController.setReference(optimized.speedMetersPerSecond, ControlType.kVelocity);

    this.turnController.setSetpoint(optimized.angle.getRadians());
    this.turnMotor.set(
        -MathUtil.clamp(this.turnController.calculate(this.getTurnAngleRad()), -1.0, 1.0));
  }

  public void doSendables() {
    SmartDashboard.putNumber(this.id + " Drive Vel (m/s)", this.getDriveVelocity());
    SmartDashboard.putNumber(this.id + " Turn Angle (deg)", this.getTurnAngleDeg());
  }

  public static void initTuning() {
    SmartDashboard.putNumber(
        "module drive kp", SmartDashboard.getNumber("module drive kp", kDriveP));
    SmartDashboard.putNumber(
        "module drive ki", SmartDashboard.getNumber("module drive ki", kDriveI));
    SmartDashboard.putNumber(
        "module drive kd", SmartDashboard.getNumber("module drive kd", kDriveD));
    SmartDashboard.putNumber(
        "module drive kff", SmartDashboard.getNumber("module drive kff", kDriveFF));

    SmartDashboard.putNumber("module turn kp", SmartDashboard.getNumber("module turn kp", kTurnP));
    SmartDashboard.putNumber("module turn ki", SmartDashboard.getNumber("module turn ki", kTurnI));
    SmartDashboard.putNumber("module turn kd", SmartDashboard.getNumber("module turn kd", kTurnD));
  }

  public void tune() {
    double tunedDriveP = SmartDashboard.getNumber("module drive kp", kDriveP);
    double tunedDriveI = SmartDashboard.getNumber("module drive ki", kDriveI);
    double tunedDriveD = SmartDashboard.getNumber("module drive kd", kDriveD);
    double tunedDriveFF = SmartDashboard.getNumber("module drive kff", kDriveFF);

    this.driveController.setP(tunedDriveP);
    this.driveController.setI(tunedDriveI);
    this.driveController.setD(tunedDriveD);
    this.driveController.setFF(tunedDriveFF);

    double tunedTurnP = SmartDashboard.getNumber("module turn kp", kTurnP);
    double tunedTurnI = SmartDashboard.getNumber("module turn ki", kTurnI);
    double tunedTurnD = SmartDashboard.getNumber("module turn kd", kTurnD);

    this.turnController.setPID(tunedTurnP, tunedTurnI, tunedTurnD);
  }
}
