/* (C) Robolancers 2024 */
package org.robolancers321.subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public CANSparkMax intakemotor;
  SparkPIDController IntakePIDController;

  public int kport = 0;
  public double power = 0;
  public double kFF = 0;

  public void intake() {

    this.intakemotor = new CANSparkMax(kport, MotorType.kBrushless);
    this.intakemotor.setSmartCurrentLimit(20);
    SmartDashboard.putNumber("intake-kFF", kFF);
  }

  @Override
  public void periodic() {

    intakemotor.set(power);
    double kFF = SmartDashboard.getNumber("kFF", 0);
    IntakePIDController.setFF(kFF);
  }

  public Command on (double power) {

    return runOnce(() -> intakemotor.set(power));
  }

  public Command off (double power) {

    return runOnce(() -> intakemotor.set(0));
  }
}
