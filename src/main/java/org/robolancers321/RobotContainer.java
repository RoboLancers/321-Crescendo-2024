/* (C) Robolancers 2024 */
package org.robolancers321;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.Constants.FlywheelConstants;
import org.robolancers321.Constants.PivotConstants;
import org.robolancers321.commands.AutoPickupNote;
import org.robolancers321.commands.EmergencyCancel;
import org.robolancers321.commands.IntakeNote;
import org.robolancers321.commands.IntakeNoteManual;
import org.robolancers321.commands.Mate;
import org.robolancers321.commands.OuttakeNote;
import org.robolancers321.commands.ScoreAmp;
import org.robolancers321.commands.ScoreSpeakerFixedAuto;
import org.robolancers321.commands.ScoreSpeakerFixedTeleop;
import org.robolancers321.commands.ScoreSpeakerFromDistance;
import org.robolancers321.commands.Shift;
import org.robolancers321.commands.autonomous.Auto3NBClose;
import org.robolancers321.commands.autonomous.Auto3NMClose;
import org.robolancers321.commands.autonomous.Auto3NTClose;
import org.robolancers321.commands.autonomous.Auto4NBSkip;
import org.robolancers321.commands.autonomous.Auto4NBSweep;
import org.robolancers321.commands.autonomous.Auto4NBSweepStraight;
import org.robolancers321.commands.autonomous.Auto4NMSweep;
import org.robolancers321.commands.autonomous.Auto4NMSweepFender;
import org.robolancers321.commands.autonomous.Auto4NMSweepFenderStraight;
import org.robolancers321.commands.autonomous.Auto4NMSweepFenderStraightAutoPickup;
import org.robolancers321.commands.autonomous.Auto4NTClose;
import org.robolancers321.commands.autonomous.Auto4NTSweep;
import org.robolancers321.subsystems.LED.LED;
import org.robolancers321.subsystems.LED.LED.Section;
import org.robolancers321.subsystems.drivetrain.Drivetrain;
import org.robolancers321.subsystems.intake.Retractor;
import org.robolancers321.subsystems.intake.Sucker;
import org.robolancers321.subsystems.launcher.AimTable;
import org.robolancers321.subsystems.launcher.Flywheel;
import org.robolancers321.subsystems.launcher.Indexer;
import org.robolancers321.subsystems.launcher.Pivot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
  private Drivetrain drivetrain;
  private Retractor retractor;
  private Sucker sucker;
  private Pivot pivot;
  private Indexer indexer;
  private Flywheel flywheel;

  private XboxController driverController;
  private XboxController manipulatorController;

  private SendableChooser<Command> autoChooser;

  private LED led;
  private AddressableLEDSim ledSim;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();
    this.retractor = Retractor.getInstance();
    this.sucker = Sucker.getInstance();
    this.pivot = Pivot.getInstance();
    this.indexer = Indexer.getInstance();
    this.flywheel = Flywheel.getInstance();

    this.driverController = new XboxController(0);
    this.manipulatorController = new XboxController(1);

    this.autoChooser = new SendableChooser<Command>();

    this.led = new LED();
    this.ledSim = new AddressableLEDSim(led.ledStrip);

    this.configureLEDs();
    this.configureDefaultCommands();
    this.configureDriverController();
    this.configureManipulatorController();
    this.configureAuto();
  }

  private void configureLEDs() {
    // default, meteor red
    LED.registerSignal(1, () -> true, LED.meteorRain(0.02, LED.kDefaultMeteorColors));

    // sees note, blink orange
    LED.registerSignal(
        2, this.drivetrain::seesNote, LED.strobe(Section.FULL, new Color(180, 30, 0)));

    // note in sucker, solid white
    LED.registerSignal(
        3, this.sucker::noteDetected, LED.solid(Section.FULL, Color.kWhite));

    // flywheel is revving, solid yellow
    LED.registerSignal(
        4,
        () ->
            (!this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.solid(Section.FULL, new Color(255, 255, 0)));

    // flywheel is revved, blink green
    LED.registerSignal(
        5,
        () ->
            (this.pivot.atGoal() && this.flywheel.isRevved()
                && this.flywheel.getGoalRPM()
                    > FlywheelConstants.FlywheelSetpoint.kAcceptHandoff.rpm),
        LED.strobe(Section.FULL, new Color(0, 255, 0)));
  }

  private void configureDefaultCommands() {
    this.drivetrain.setDefaultCommand(this.drivetrain.teleopDrive(driverController, true));

    this.sucker.setDefaultCommand(this.sucker.off());
    this.indexer.setDefaultCommand(this.indexer.off());

    this.flywheel.setDefaultCommand(this.flywheel.revSpeakerFromRPM(() -> {
        if (this.indexer.entranceBeamNotBroken()) return 0.0;

        if (this.drivetrain.getDistanceToSpeaker() < 4.0) return Math.min(0.7 * AimTable.interpolateFlywheelRPM(this.drivetrain.getDistanceToSpeaker()), 2000);

        return 0.0;
    }));

    this.retractor.setDefaultCommand(this.retractor.moveToRetracted());
    this.pivot.setDefaultCommand(this.pivot.aimAtSpeaker(() -> {
        if (this.indexer.entranceBeamNotBroken()) return PivotConstants.PivotSetpoint.kRetracted.angle;

        if (this.drivetrain.getDistanceToSpeaker() < 4.0) return AimTable.interpolatePivotAngle(this.drivetrain.getDistanceToSpeaker());

        return PivotConstants.PivotSetpoint.kRetracted.angle;
    }));
  }

  /*
   * Press Bumpers Together: zero gyro
   * 
   * Hold Right Bumper: toggle slow mode
   * 
   * Hold Right Trigger: deploy intake
   * Release Right Trigger: retract intake
   * 
   * Hold Left Trigger: deploy outtake
   * Release Left Trigger: retract intake
   * 
   * Press A Button: snap to amp angle
   * Press B Button: auto pickup note
   * Press X Button: emergency cancel
   * 
   */
  private void configureDriverController() {
    // TODO: i think we keep this just in case for teleop
    new Trigger(
            () -> this.driverController.getLeftBumper() && this.driverController.getRightBumper())
        .onTrue(this.drivetrain.zeroYaw());

    // TODO: technically we can just pull the bumper state off of the controller inside teleop
    // command but maybe this offers more control programmatically
    new Trigger(this.driverController::getRightBumper)
        .whileTrue(new InstantCommand(() -> this.drivetrain.slowMode = true));
    new Trigger(this.driverController::getRightBumper)
        .whileFalse(new InstantCommand(() -> this.drivetrain.slowMode = false));

    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
        .whileTrue(new IntakeNoteManual());
    new Trigger(() -> this.driverController.getRightTriggerAxis() > 0.8)
        .onFalse((new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected));

    new Trigger(() -> this.driverController.getLeftTriggerAxis() > 0.8)
        .whileTrue(new OuttakeNote());

    new Trigger(this.driverController::getAButton).onTrue(this.drivetrain.turnToAngle(90.0));
    new Trigger(this.driverController::getBButton).whileTrue(new AutoPickupNote());
    new Trigger(this.driverController::getBButton).onFalse((new Mate().andThen(new Shift())).onlyIf(this.sucker::noteDetected));
    new Trigger(this.driverController::getXButton).onTrue(new EmergencyCancel());
  }

  /*
   * Hold A Button: rev for score amp
   * Release A Button: eject for score amp
   * 
   * Press Y Button: score speaker from distance
   * 
   * Hold X Button: rev for fixed speaker shot
   * Release X Button: eject for fixed speaker shot
   * 
   */
  private void configureManipulatorController() {
    new Trigger(this.manipulatorController::getAButton).whileTrue(new ScoreAmp());
    new Trigger(this.manipulatorController::getAButton).onFalse(this.indexer.outtake().alongWith(new InstantCommand(() -> {}, this.flywheel)).andThen(
        new ParallelCommandGroup(this.retractor.moveToRetracted(), this.pivot.moveToRetracted())
    ));

    new Trigger(this.manipulatorController::getYButton)
        .onTrue(
            new ScoreSpeakerFromDistance()
                .finallyDo(
                    () -> {
                      CommandScheduler.getInstance()
                          .schedule(
                              new ParallelCommandGroup(
                                  this.retractor.moveToRetracted(), this.pivot.moveToRetracted()));
                    }));

    new Trigger(this.manipulatorController::getXButton).whileTrue(new ScoreSpeakerFixedTeleop());
    new Trigger(this.manipulatorController::getXButton)
        .onFalse(
            this.indexer
                .outtake()
                .raceWith(this.sucker.out())
                .alongWith(new InstantCommand(() -> {}, this.flywheel)));

    new Trigger(() -> this.manipulatorController.getLeftY() < -0.8).onTrue(this.pivot.aimAtAmp());
    new Trigger(() -> this.manipulatorController.getLeftY() > 0.8).onTrue(this.pivot.moveToRetracted());
  }

  private void configureAuto() {
    /*
    Sweep - pickup front 3 notes with curves
    Sweep straight - pickup front 3 notes with straight line
    Close - pickup front note first
    Skip - pickup center note first then go back for front note
     */

    this.autoChooser.setDefaultOption("Score And Sit", new ScoreSpeakerFixedAuto());

    this.autoChooser.addOption("4NT Sweep", new Auto4NTSweep());
    this.autoChooser.addOption("4NT Close", new Auto4NTClose());
    this.autoChooser.addOption("3NT Close", new Auto3NTClose());

    this.autoChooser.addOption("4NM Sweep", new Auto4NMSweep());
    this.autoChooser.addOption("3NM Close", new Auto3NMClose());
    this.autoChooser.addOption("4NM Sweep Fender", new Auto4NMSweepFender());
    this.autoChooser.addOption("4NM Sweep Fender Straight", new Auto4NMSweepFenderStraight());
    this.autoChooser.addOption("4NM Sweep Fender Straight Auto Pickup", new Auto4NMSweepFenderStraightAutoPickup());

    this.autoChooser.addOption("4NB Sweep", new Auto4NBSweep());
    this.autoChooser.addOption("4NB Skip", new Auto4NBSkip());
    this.autoChooser.addOption("3NB Sweep Straight", new Auto4NBSweepStraight());
    this.autoChooser.addOption("3NB Close", new Auto3NBClose());

    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    // return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("4NMSweep"));

    return this.autoChooser.getSelected();
  }
}
