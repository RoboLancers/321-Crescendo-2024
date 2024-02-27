/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Launcher extends SubsystemBase {
  /*
   * Singleton
   */

  private static Launcher instance = null;

  public static Launcher getInstance() {
    if (instance == null) instance = new Launcher();

    return instance;
  }

  /*
   * Constants
   */

  // TODO
  private static final int kBeamBreakPort = 9;

  // TODO
  // private static final double kDebounceTime = 0.05;

  /*
   * Implementation
   */

  public final Pivot pivot;
  public final Indexer indexer;
  public final Flywheel flywheel;

  // TODO
  private final DigitalInput beamBreak;

  // TODO
  // private final Debouncer beamBreakDebouncer;

  // TODO
  // private final AimTable aimTable;

  private Launcher() {
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    // TODO
    this.beamBreak = new DigitalInput(kBeamBreakPort);

    // TODO
    // this.beamBreakDebouncer = new Debouncer(kDebounceTime, Debouncer.DebounceType.kBoth);

    // TODO
    // this.aimTable = new AimTable();
  }

  public boolean getBeamBreakState() {
    // return false;
    return !beamBreak.get();
    // return beamBreakDebouncer.calculate(beamBreak.get()); // TODO
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("launcher middle beam broken", this.getBeamBreakState());
  }

  public Command acceptHandoff() {
    return this.indexer.acceptHandoff(this::getBeamBreakState);
  }

  public Command scoreAmp() {
    return new SequentialCommandGroup(
        this.pivot.aimAtAmp(),
        new ParallelRaceGroup(
            this.flywheel.yeetNoteAmp(),
            new SequentialCommandGroup(
                new WaitCommand(1.0),
                new ParallelRaceGroup(
                    this.indexer.outtake(this::getBeamBreakState), new WaitCommand(0.5)))),
        this.pivot.moveToRetracted());
  }

  public Command scoreSpeaker() {
    return new SequentialCommandGroup(
        this.pivot.aimAtSpeakerFixed(),
        new ParallelRaceGroup(
            this.flywheel.yeetNoteSpeakerFixed(),
            new SequentialCommandGroup(
                new WaitCommand(3.0),
                new ParallelRaceGroup(
                    this.indexer.outtake(this::getBeamBreakState), new WaitCommand(2.0)))),
        this.pivot.moveToRetracted());
  }

  /*

  public Command yeetSpeaker(DoubleSupplier distanceSupplier) {
    return new ParallelRaceGroup(
        new RunCommand(
            () -> this.aimTable.updateSpeakerAimCharacteristic(distanceSupplier.getAsDouble())),
        new SequentialCommandGroup(
            indexer.shiftIntoPosition(this::getBeamBreakState),
            new ParallelRaceGroup(
                flywheel.yeetNoteSpeaker(() -> this.aimTable.getSpeakerAimCharacteristic().rpm),
                new SequentialCommandGroup(
                    pivot.aimAtSpeaker(() -> this.aimTable.getSpeakerAimCharacteristic().angle),
                    new WaitUntilCommand(flywheel::isRevved),
                    indexer.outtake(this::getBeamBreakState),
                    new WaitCommand(0.2)))));
  }

  */
}
