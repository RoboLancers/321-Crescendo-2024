package org.robolancers321.subsystems.commands.retractor;

import edu.wpi.first.wpilibj2.command.Command;
import org.robolancers321.subsystems.IntakeRetractor;

public class retractIntake extends Command {
    public IntakeRetractor intakeRetractor;

    public retractIntake() {
        addRequirements(this.intakeRetractor);
    }
    
    @Override
    public void initialize() {
       intakeRetractor.resetEncoder(); 
    }

    @Override 
    public void execute() {
        intakeRetractor.extendRetractor();
    }

    @Override
    public boolean isFinished() {
        return intakeRetractor.beamBroken();
    }

    @Override 
    public void end(boolean interrupted) {
      intakeRetractor.retractedPosition();      
    }
}
