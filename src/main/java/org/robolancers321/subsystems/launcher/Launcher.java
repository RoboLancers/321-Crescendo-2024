/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Launcher extends SubsystemBase {
  private static Launcher INSTANCE;

  public static Launcher getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Launcher();
    }
    return INSTANCE;
  }

  private static double kFlywheelFF = 0.0;

  private static double kFlywheelTolerance = 150;

  private static double kIndexerFF = 0.0;

  private static double kFlywheelInRPM = 0;
  private static double kFlywheelOutRPM = 0;
  private static double kIndexerInRPM = 0;
  private static double kIndexerOutRPM = 0;

  private static boolean kInvertFlywheel = false;

  private static boolean kInvertIndexer = false;

  private static double kFlywheelConversionFactor = 1;

  private static double kIndexerConversionFactor = 0.0;
  private static int kFlywheelPort = 14;

  private static int kIndexerPort = 13;

  private CANSparkMax flywheelMotor;

  private CANSparkMax indexerMotor;

  private RelativeEncoder flywheelEncoder;

  private RelativeEncoder indexerEncoder;

  private SparkPIDController flywheelPIDController;

  private SparkPIDController indexerPIDController;

  private Launcher() {
    this.flywheelMotor = new CANSparkMax(kFlywheelPort, CANSparkLowLevel.MotorType.kBrushless);
    this.indexerMotor = new CANSparkMax(kIndexerPort, CANSparkLowLevel.MotorType.kBrushless);

    this.flywheelEncoder = this.flywheelMotor.getEncoder();
    this.indexerEncoder = this.indexerMotor.getEncoder();

    this.flywheelPIDController = this.flywheelMotor.getPIDController();
    this.indexerPIDController = this.indexerMotor.getPIDController();
  }

  private void configureMotors() {
    this.flywheelMotor.setInverted(kInvertFlywheel);
    this.indexerMotor.setInverted(kInvertIndexer);
  }

  private void configureEncoders() {
    this.flywheelMotor.setInverted(kInvertFlywheel);
    this.indexerEncoder.setInverted(kInvertIndexer);

    this.flywheelEncoder.setVelocityConversionFactor(kFlywheelConversionFactor);
    this.indexerEncoder.setVelocityConversionFactor(kIndexerConversionFactor);
  }

  private void configureControllers() {
    this.flywheelPIDController.setFF(kFlywheelFF);

    this.flywheelPIDController.setP(0.0);
    this.flywheelPIDController.setI(0.0);
    this.flywheelPIDController.setD(0.0);

    this.indexerPIDController.setFF(kIndexerFF);

    this.indexerPIDController.setP(0.0);
    this.indexerPIDController.setI(0.0);
    this.indexerPIDController.setD(0.0);
  }

  private void setFlywheelSpeedRPM(double speed) {
    this.flywheelPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  private void setIndexerSpeedRPM(double speed) {
    this.indexerPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  private double getFlywheelVelocity() {
    return this.flywheelEncoder.getVelocity();
  }

  private double getIndexerVelocity() {
    return this.indexerEncoder.getVelocity();
  }

  private boolean isFlywheelAtSpeed() {
    return epsilonEquals(getFlywheelVelocity(), kFlywheelOutRPM, kFlywheelTolerance);
  }

  public Command yeet() {

    return run(() -> {
          this.setFlywheelSpeedRPM(kFlywheelOutRPM);
          if (this.isFlywheelAtSpeed()) {
            this.setIndexerSpeedRPM(kIndexerOutRPM);
          }
        })
        .raceWith(run(() -> {}).until(this::isFlywheelAtSpeed).andThen(new WaitCommand(0.4)))
        .finallyDo(
            () -> {
              setFlywheelSpeedRPM(0);
              setIndexerSpeedRPM(0);
            });
  }

  public Command pullIn() {
    return run(() -> {
          setFlywheelSpeedRPM(kFlywheelInRPM);
          setIndexerSpeedRPM(kIndexerInRPM);
        })
        .finallyDo(
            () -> {
              setIndexerSpeedRPM(0);
              setFlywheelSpeedRPM(0);
            });
  }

  private void doSendables() {
    SmartDashboard.putNumber("flywheel velocity", getFlywheelVelocity());
    SmartDashboard.putNumber("indexer velocity", getIndexerVelocity());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("flywheel FF", SmartDashboard.getNumber("flywheel FF", kFlywheelFF));
    SmartDashboard.putNumber("indexer FF", SmartDashboard.getNumber("indexer FF", kIndexerFF));

    SmartDashboard.putNumber("target flywheel velocity", 0);
    SmartDashboard.putNumber("target indexer velocity", 0);
  }

  private void tune() {

    double tunedFlywheelFF = SmartDashboard.getNumber("flywheel FF", kFlywheelFF);
    double tunedIndexerFF = SmartDashboard.getNumber("indexer FF", kIndexerFF);

    double targetFlywheelRPM = SmartDashboard.getNumber("target flywheel velocity", 0);
    double targetIndexerRPM = SmartDashboard.getNumber("target flywheel velocity", 0);

    this.flywheelPIDController.setFF(tunedFlywheelFF);
    this.indexerPIDController.setFF(tunedIndexerFF);

    setFlywheelSpeedRPM(targetFlywheelRPM);
    setIndexerSpeedRPM(targetIndexerRPM);
  }

  public Command tuneSpeeds() {
    initTuning();

    return run(this::tune);
  }
}
