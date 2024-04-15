package org.robolancers321.commands;

import java.util.Set;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoScoreTrap extends SequentialCommandGroup {
    private Drivetrain drivetrain;

    public AutoScoreTrap(){
        this.drivetrain = Drivetrain.getInstance();

        this.addCommands(
            Commands.defer(() -> this.drivetrain.pathfindToTrap(), Set.of(drivetrain)).onlyIf(() -> this.drivetrain.getClosestTrapPosition().getDistance() < 3.0),
            new Mate().andThen(new Shift()).onlyIf(() -> this.drivetrain.getClosestTrapPosition().getDistance() < 0.5),
            new TrapShot().onlyIf(() -> this.drivetrain.getClosestTrapPosition().getDistance() < 0.5)
        );
    }
}
