package org.robolancers321.commands.autonomous;

import org.robolancers321.commands.DeployIntake;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.ScoreSpeakerFixed;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Auto3NBSweep extends SequentialCommandGroup {
    private Drivetrain drivetrain;
    private Retractor retractor;
    private Sucker sucker;
    private Pivot pivot;
    private Indexer indexer;
    private Flywheel flywheel;

    public Auto3NBSweep(){
        this.drivetrain = Drivetrain.getInstance();
        this.retractor = Retractor.getInstance();
        this.sucker = Sucker.getInstance();
        this.pivot = Pivot.getInstance();
        this.indexer = Indexer.getInstance();
        this.flywheel = Flywheel.getInstance();

        this.addCommands(
            // new InstantCommand(() -> this.drivetrain.setYaw(60.0)),
            new ScoreSpeakerFixedAuto(),
            new ParallelRaceGroup(
                new WaitCommand(0.6).andThen(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("NoTeamBottom.1")).andThen(this.drivetrain.stop()).andThen(new WaitCommand(0.5))),
                new DeployIntake()
            ),
            new Mate(),
            new ScoreSpeakerFromDistance(),
            new ParallelRaceGroup(
                new WaitCommand(0.6).andThen(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("NoTeamBottom.2")).andThen(this.drivetrain.stop()).andThen(new WaitCommand(0.5))),
                new DeployIntake()
            ),
            new Mate(),
            new ScoreSpeakerFromDistance(),
            new ParallelRaceGroup(
                new WaitCommand(0.6).andThen(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("NoTeamBottom.3")).andThen(this.drivetrain.stop()).andThen(new WaitCommand(0.5))),
                new DeployIntake()
            ),
            new Mate(),
            new ScoreSpeakerFromDistance()
        );
    }
}
