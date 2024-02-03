/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static org.robolancers321.util.MathUtils.epsilonEquals;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class Launcher extends SubsystemBase {
  /*
   * Singleton
   */

  private static Launcher instance = null;

  public static Launcher getInstance() {
    if (instance == null) {
      instance = new Launcher();
    }
    return instance;
  }

  /*
   * Constants
   */

  private static int kIndexerPort = 14;
  private static int kFlywheelPort = 13;

  private static boolean kInvertIndexer = false;
  private static boolean kInvertFlywheel = false;

  private static double kIndexerConversionFactor = 1;
  private static double kFlywheelConversionFactor = 1;

  private static double kIndexerFF = 0.0001735;
  private static double kFlywheelFF = 0.000178;

  private static double kFlywheelTolerance = 250;

  private static double kIndexerInRPM = -1000;
  private static double kIndexerOutRPM = 5600;
  private static double kFlywheelInRPM = -1000;
  private static double kFlywheelOutRPM = 5600;

  /*
   * Implementation
   */

  private CANSparkMax indexerMotor;
  private CANSparkMax flywheelMotor;

  private RelativeEncoder indexerEncoder;
  private RelativeEncoder flywheelEncoder;

  private SparkPIDController indexerPIDController;
  private SparkPIDController flywheelPIDController;

  private Launcher() {
    this.indexerMotor = new CANSparkMax(kIndexerPort, CANSparkLowLevel.MotorType.kBrushless);
    this.flywheelMotor = new CANSparkMax(kFlywheelPort, CANSparkLowLevel.MotorType.kBrushless);

    this.indexerEncoder = this.indexerMotor.getEncoder();
    this.flywheelEncoder = this.flywheelMotor.getEncoder();

    this.indexerPIDController = this.indexerMotor.getPIDController();
    this.flywheelPIDController = this.flywheelMotor.getPIDController();

    this.configureMotors();
    this.configureEncoders();
    this.configureControllers();
  }

  private void configureMotors() {
    this.indexerMotor.setInverted(kInvertIndexer);
    this.flywheelMotor.setInverted(kInvertFlywheel);
  }

  private void configureEncoders() {
   
    this.indexerEncoder.setVelocityConversionFactor(kIndexerConversionFactor);
    this.flywheelEncoder.setVelocityConversionFactor(kFlywheelConversionFactor);
  }

  private void configureControllers() {
    this.indexerPIDController.setP(0.0);
    this.indexerPIDController.setI(0.0);
    this.indexerPIDController.setD(0.0);
    this.indexerPIDController.setFF(kIndexerFF);

    this.flywheelPIDController.setP(0.0);
    this.flywheelPIDController.setI(0.0);
    this.flywheelPIDController.setD(0.0);
    this.flywheelPIDController.setFF(kFlywheelFF);
  }

  private void setIndexerSpeedRPM(double speed) {
    this.indexerPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  private void setFlywheelSpeedRPM(double speed) {
    this.flywheelPIDController.setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  private double getIndexerVelocity() {
    return this.indexerEncoder.getVelocity();
  }

  private double getFlywheelVelocity() {
    return this.flywheelEncoder.getVelocity();
  }

  private boolean isFlywheelAtSpeed() {
    return epsilonEquals(getFlywheelVelocity(), kFlywheelOutRPM, kFlywheelTolerance);
  }

  public Command yeet() {
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new RunCommand(() -> {
          this.setFlywheelSpeedRPM(kFlywheelOutRPM);
        }),
        new WaitCommand(0.4)
      ),
      new ParallelRaceGroup(
        new RunCommand(() -> {
          this.setFlywheelSpeedRPM(kFlywheelOutRPM);
          this.setIndexerSpeedRPM(kIndexerOutRPM);
        }),
        new WaitCommand(0.4)
      ),
      new InstantCommand(() -> {
        setIndexerSpeedRPM(0);
        setFlywheelSpeedRPM(0);
      })
    );
  }

  public Command pullIn() {
    return run(() -> {
          setIndexerSpeedRPM(kIndexerInRPM);
          setFlywheelSpeedRPM(kFlywheelInRPM);
        })
        .finallyDo(
            () -> {
              setIndexerSpeedRPM(0);
              setFlywheelSpeedRPM(0);
            });
  }

  private void doSendables() {
    SmartDashboard.putNumber("indexer velocity", getIndexerVelocity());
    SmartDashboard.putNumber("flywheel velocity", getFlywheelVelocity());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("indexer FF", SmartDashboard.getNumber("indexer FF", kIndexerFF));
    SmartDashboard.putNumber("flywheel FF", SmartDashboard.getNumber("flywheel FF", kFlywheelFF));

    SmartDashboard.putNumber("target indexer velocity", 0);
    SmartDashboard.putNumber("target flywheel velocity", 0);
  }

  private void tune() {
    double tunedIndexerFF = SmartDashboard.getNumber("indexer FF", kIndexerFF);
    double tunedFlywheelFF = SmartDashboard.getNumber("flywheel FF", kFlywheelFF);

    double targetIndexerRPM = SmartDashboard.getNumber("target indexer velocity", 0);
    double targetFlywheelRPM = SmartDashboard.getNumber("target flywheel velocity", 0);

    this.indexerPIDController.setFF(tunedIndexerFF);
    this.flywheelPIDController.setFF(tunedFlywheelFF);

    setIndexerSpeedRPM(targetIndexerRPM);
    setFlywheelSpeedRPM(targetFlywheelRPM);
  }

  public Command tuneSpeeds() {
    initTuning();

    return run(this::tune);
  }
}
