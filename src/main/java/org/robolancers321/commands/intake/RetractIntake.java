package org.robolancers321.commands.intake;

import org.robolancers321.subsystems.intake.IntakeRetractor;
import org.robolancers321.subsystems.intake.IntakeRetractor.RetractorSetpoint;

import edu.wpi.first.wpilibj2.command.Command;

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
