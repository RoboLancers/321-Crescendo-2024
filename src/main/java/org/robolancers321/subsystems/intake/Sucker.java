/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.robolancers321.Constants.SuckerConstants;

public class Sucker extends SubsystemBase {
  /*
   * Singleton
   */

  private static Sucker instance = null;

  public static Sucker getInstance() {
    if (instance == null) instance = new Sucker();

    return instance;
  }

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final DigitalInput touchSensor;

  private Sucker() {
    this.motor = new CANSparkMax(SuckerConstants.kMotorPort, MotorType.kBrushless);
    this.encoder = this.motor.getEncoder();
    this.touchSensor = new DigitalInput(SuckerConstants.kTouchSensorPort);

    this.configureMotor();
    this.configureEncoder();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(SuckerConstants.kInvertMotor);
    this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(SuckerConstants.kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setVelocityConversionFactor(1.0);
  }

  public double getVelocityRPM() {
    return this.encoder.getVelocity();
  }

  public boolean noteDetected() {
    return !this.touchSensor.get();
  }

  private void doSendables() {
    SmartDashboard.putNumber("sucker rpm", this.getVelocityRPM());
    SmartDashboard.putBoolean("sucker detects note", this.noteDetected());
    SmartDashboard.putNumber("sucker output", this.motor.get());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  public Command off() {
    return run(
        () -> {
          this.motor.set(0.0);
        });
  }

  public Command offInstantly() {
    return runOnce(
        () -> {
          this.motor.set(0.0);
        });
  }

  public Command in() {
    return run(() -> {
          this.motor.set(SuckerConstants.kInSpeed);
        })
        .finallyDo(
            () -> {
              this.motor.set(0.0);
            });
  }

  public Command ampShot() {
    return run(
        () -> {
          this.motor.set(SuckerConstants.kAmpShot);
        });
  }

  public Command out() {
    return run(
        () -> {
          this.motor.set(SuckerConstants.kOutSpeed);
        });
  }
}
