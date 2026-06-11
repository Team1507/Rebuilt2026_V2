//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import org.team1507.robot.Constants.kHopper;
import org.team1507.robot.subsystems.Agitator;
import org.team1507.robot.subsystems.Feeder;
import org.team1507.robot.subsystems.Hopper;
import org.team1507.robot.subsystems.IntakeArm;
import org.team1507.robot.subsystems.IntakeRoller;
import org.team1507.robot.subsystems.Shooter;

// Coordinated, multi-subsystem robot behaviors.
// Each method is the single definition used in both teleop (Robot.java) and auto (AutoSequence.java).
// Name methods by what the robot DOES: shoot(), intakePiece(), ejectPiece().
public final class RobotBehaviors {

    // Prevent instantiation — this is a static utility class.
    private RobotBehaviors() {}

    // ─────────────────────────────────────────────────────────────────
    // FAILSAFE
    // ─────────────────────────────────────────────────────────────────

    /**
     * Cancels every running command immediately.
     *
     * <p>Use when a mechanism appears stuck or unresponsive. All commands are
     * cancelled on the spot; subsystem default commands (e.g. the swerve drive)
     * are automatically rescheduled by the scheduler on the next cycle, so
     * driving is restored within one loop.
     *
     * <p>Works while disabled ({@code ignoringDisable(true)}) so operators can
     * clear a stuck state before enabling.
     *
     * <p>Binding: {@code driver.back().onTrue(RobotBehaviors.failsafe());}
     */
    public static Command failsafe() {
        return Commands.runOnce(CommandScheduler.getInstance()::cancelAll)
            .withName("Failsafe.cancelAll")
            .ignoringDisable(true);
    }

    // ─────────────────────────────────────────────────────────────────
    // INTAKE
    // ─────────────────────────────────────────────────────────────────

    /**
     * Extends the hopper to EXTENDED_POS while simultaneously deploying the intake
     * arm and roller once the hopper clears its safe threshold.
     *
     * <p>The hopper and arm/roller move in parallel: the hopper drives to 12" and
     * holds there; as soon as the hopper passes 10" (SAFE_EXTENDED), the arm deploys
     * and the roller spins. This saves time vs. waiting for full hopper extension
     * before starting arm movement. Runs until interrupted (button released).
     *
     * <p>Binding: {@code driver.leftTrigger(0.5).whileTrue(RobotBehaviors.deployAndIntake(...));}
     */
    public static Command deployAndIntake(Hopper hopper, IntakeArm intakeArm, IntakeRoller intakeRoller) {
        return Commands.parallel(
            hopper.holdExtendedCommand(),
            Commands.sequence(
                // Race the safe-threshold check against a timeout so a stalled hopper
                // never blocks the arm from deploying indefinitely.
                Commands.race(
                    Commands.waitUntil(hopper::isHopperSafeForIntake),
                    Commands.waitSeconds(kHopper.DEPLOY_TIMEOUT_SECONDS)
                ),
                Commands.parallel(
                    intakeArm.deployCommand(),
                    intakeRoller.runCommand()
                )
            )
        ).withName("Behaviors.deployAndIntake");
    }

    /**
     * Retracts the intake arm and stops the roller. Used to stow after intaking.
     *
     * <p>Binding: release of the intake trigger ({@code .onFalse(...)}).
     */
    public static Command stowIntake(IntakeArm intakeArm, IntakeRoller intakeRoller) {
        return Commands.parallel(
            intakeArm.retractCommand(),
            intakeRoller.stopCommand()
        ).withName("Behaviors.stowIntake");
    }

    // ─────────────────────────────────────────────────────────────────
    // SHOOT
    // ─────────────────────────────────────────────────────────────────

    /**
     * Spins the shooter to the target RPM, then—once up to speed—runs the
     * feeder and agitator to push the game piece through. Stops everything
     * on interrupt.
     *
     * <p>Binding: {@code operator.rightBumper().whileTrue(RobotBehaviors.shootFixedRPM(..., kShooter.SAFE_RPM));}
     */
    public static Command shootFixedRPM(Shooter shooter, Feeder feeder, Agitator agitator, double rpm) {
        return Commands.deadline(
            shooter.spinUpCommand(rpm),
            Commands.sequence(
                Commands.waitUntil(shooter::atVelocity),
                Commands.parallel(
                    feeder.feedCommand(),
                    agitator.toShooterCommand()
                )
            )
        ).withName("Behaviors.shootFixedRPM(" + rpm + ")");
    }

    // ─────────────────────────────────────────────────────────────────
    // OUTTAKE
    // ─────────────────────────────────────────────────────────────────

    /**
     * Deploys the intake arm, reverses the intake roller, and runs the agitator
     * in the outtake direction simultaneously. Runs until interrupted.
     *
     * <p>Binding: {@code operator.b().whileTrue(RobotBehaviors.outtake(...));}
     */
    public static Command outtake(IntakeArm intakeArm, IntakeRoller intakeRoller, Agitator agitator) {
        return Commands.parallel(
            intakeArm.deployCommand(),
            intakeRoller.runReverseCommand(),
            agitator.toOuttakeCommand()
        ).withName("Behaviors.outtake");
    }
}
