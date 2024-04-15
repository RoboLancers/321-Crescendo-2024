package org.robolancers321.commands;

import javax.sound.midi.Sequence;

import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.launcher.AimTable;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TrapShot extends SequentialCommandGroup {
private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  public TrapShot(){
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    // NOTE: actual usage of these are now inside of aim table
    SmartDashboard.putNumber("trap pivot angle", 0);
    SmartDashboard.putNumber("trap flywheel rpm", 0);

    this.addCommands(
        this.indexer.shiftBackFromExit().withTimeout(0.1),
        this.indexer.off(),
        new ParallelCommandGroup(
            this.pivot.aimAtSpeaker(
                () -> SmartDashboard.getNumber("trap pivot angle", 0)),
                // TODO
            this.flywheel.revSpeakerFromRPM(
                () -> SmartDashboard.getNumber("trap flywheel rpm", 0))),
                // TODO
        this.indexer.outtake(),
        this.indexer.off(),
        this.flywheel.off());
  }
}
