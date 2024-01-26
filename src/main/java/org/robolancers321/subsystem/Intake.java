/* (C) Robolancers 2024 */
package org.robolancers321.subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public CANSparkMax intakeMotor;
  SparkPIDController intakPidController;

  public int kPort = 0;
  public double power = 0;
  public double kFF = 0;

  public void Intake() {

    this.intakeMotor = new CANSparkMax(kPort, MotorType.kBrushless);
    this.intakeMotor.setSmartCurrentLimit(20);
    intakPidController = intakeMotor.getPIDController();
    SmartDashboard.putNumber("kFF", kFF);
  }

  @Override
  public void periodic() {

    double kFF = SmartDashboard.getNumber("kFF", 0);
    intakPidController.setFF(kFF);
  }

  public Command in(double power) {

    return runOnce(() -> intakeMotor.set(/*Math.abs*/(power)));
  }

  public Command out(double power) {

    return runOnce(() -> intakeMotor.set(-/*Math.abs*/(power)));
  }

  public Command off(double power) {

    return runOnce(() -> intakeMotor.set(0));
  }
}
