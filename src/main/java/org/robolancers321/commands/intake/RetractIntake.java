/* (C) Robolancers 2024 */
package org.robolancers321.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import org.robolancers321.subsystems.intake.IntakeRetractor;
import org.robolancers321.subsystems.intake.IntakeRetractor.RetractorSetpoint;

public class RetractIntake extends Command {
  private IntakeRetractor intakeRetractor;

  public RetractIntake() {
    this.intakeRetractor = IntakeRetractor.getInstance();

    addRequirements(this.intakeRetractor);
  }

  @Override
  public void initialize() {
    this.intakeRetractor.setSetpoint(RetractorSetpoint.kRetracted);
  }

  @Override
  public boolean isFinished() {
    return this.intakeRetractor.isAtSetpoint();
  }
}
