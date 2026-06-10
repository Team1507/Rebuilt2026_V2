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

import org.team1507.robot.subsystems.Agitator;
import org.team1507.robot.subsystems.Feeder;
import org.team1507.robot.subsystems.Hopper;
import org.team1507.robot.subsystems.IntakeArm;
import org.team1507.robot.subsystems.IntakeRoller;
import org.team1507.robot.subsystems.Shooter;

// ─────────────────────────────────────────────────────────────────────────────
// RobotBehaviors
//
// A single shared library of coordinated, multi-subsystem robot behaviors.
// These are complete robot actions that require two or more subsystems working
// together in sequence or in parallel.
//
// KEY PRINCIPLE:
//   A behavior defined here is the ONE definition used everywhere.
//   Whether triggered by a driver button in teleop or a step in an auto
//   routine, it calls the same method. You never write the same behavior twice.
//
// HOW TELEOP USES THIS:
//   In Robot.java:
//     driver.a().onTrue(RobotBehaviors.myBehavior());
//
// HOW AUTO USES THIS:
//   In AutoSequence.java (add a one-line wrapper):
//     public AutoSequence myBehavior() {
//         steps.add(RobotBehaviors.myBehavior());
//         return this;
//     }
//   Then in a routine:
//     new AutoSequence().myBehavior().driveFieldRelative(...).build();
//
// HOW TO ADD A NEW BEHAVIOR:
//   1. Identify which subsystems are involved.
//   2. Write a static method here that composes their individual commands.
//   3. Use Commands.sequence() for ordered steps.
//      Use Commands.parallel() for simultaneous actions.
//      Use Commands.waitUntil() to gate one action on another subsystem's state.
//   4. Add a wrapper in AutoSequence.java if it's needed in auto routines.
//   5. Bind it in Robot.java if it's a driver control.
//
// NAMING CONVENTION:
//   Name behaviors by what the robot DOES, not what the mechanism is.
//   GOOD:  shoot(), scoreHigh(), intakePiece(), ejectPiece()
//   AVOID: armHighAndIntakeForward(), deployArmRunRoller()
// ─────────────────────────────────────────────────────────────────────────────
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
     * Extends the hopper, then—once extended—deploys the intake arm and spins
     * the intake roller. All three run until interrupted.
     *
     * <p>Binding: {@code operator.leftTrigger(0.5).whileTrue(RobotBehaviors.deployAndIntake(...));}
     */
    public static Command deployAndIntake(Hopper hopper, IntakeArm intakeArm, IntakeRoller intakeRoller) {
        return Commands.sequence(
            hopper.extendCommand(),
            Commands.parallel(
                intakeArm.deployCommand(),
                intakeRoller.runCommand()
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
        return Commands.sequence(
            shooter.setRPMCommand(rpm),
            Commands.waitUntil(shooter::atVelocity),
            Commands.parallel(
                feeder.feedCommand(),
                agitator.toShooterCommand()
            )
        ).withName("Behaviors.shootFixedRPM(" + rpm + ")");
    }
}
