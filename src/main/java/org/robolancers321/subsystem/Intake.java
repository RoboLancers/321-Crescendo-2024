/* (C) Robolancers 2024 */
package org.robolancers321.subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public CANSparkMax intakeMotor;

  public int kPort = 0;
  public double power = 0;

  public void Intake() {

    this.intakeMotor = new CANSparkMax(kPort, MotorType.kBrushless);
    this.intakeMotor.setSmartCurrentLimit(20);
  }

  public Command in(double power) {

    return runOnce(() -> intakeMotor.set(power));
  }

  public Command out(double power) {

    return runOnce(() -> intakeMotor.set(-power));
  }

  public Command off(double power) {

    return runOnce(() -> intakeMotor.set(0));
  }
}
