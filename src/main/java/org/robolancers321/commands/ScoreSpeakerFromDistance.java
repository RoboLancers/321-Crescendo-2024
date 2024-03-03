package org.robolancers321.commands;

import java.util.function.DoubleSupplier;

import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.AimTable;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreSpeakerFromDistance extends SequentialCommandGroup {
    private Retractor retractor;
    private Sucker sucker;
    private Pivot pivot;
    private Indexer indexer;
    private Flywheel flywheel;
    private Drivetrain drivetrain;

    public ScoreSpeakerFromDistance(){
        this.retractor = Retractor.getInstance();
        this.sucker = Sucker.getInstance();
        this.pivot = Pivot.getInstance();
        this.indexer = Indexer.getInstance();
        this.flywheel = Flywheel.getInstance();
        this.drivetrain = Drivetrain.getInstance();

        SmartDashboard.putNumber("pivot interpolation angle", 0);

        this.addCommands(
            this.drivetrain.turnToSpeaker(),
            this.drivetrain.stop(),
            // this.indexer.shiftBackward(),
            // this.sucker.offInstantly(),
            new ParallelCommandGroup(
                // new ConditionalCommand(this.retractor.moveToClearFromLauncher(), new InstantCommand(), () -> SmartDashboard.getNumber("pivot interpolation angle", 0) < 10.0),
                // new ConditionalCommand(this.retractor.moveToClearFromLauncher(), new InstantCommand(), () -> AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker()) < 20.0),
                // this.pivot.moveToAngle(() -> SmartDashboard.getNumber("pivot interpolation angle", 0))
                this.pivot.aimAtSpeaker(() -> AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker()))
            ),
            this.indexer.shiftForward(),
            this.indexer.shiftBackward(),
            this.flywheel.revSpeaker(() -> AimTable.interpolateFlywheelRPM(this.drivetrain.getDistanceToSpeaker())),
            this.indexer.outtake(),
            this.flywheel.off(),
            new ParallelCommandGroup(
                // this.retractor.moveToRetracted(),
                this.pivot.moveToRetracted()
            )
        );
    }
}
