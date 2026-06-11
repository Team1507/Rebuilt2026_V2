//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.auto.nodes.Nodes;

// ─────────────────────────────────────────────────────────────────────────────
// AutoSubway6inchRight
//
// Right-side half-width Neutral Zone sweep. "6 inch sub" = drive halfway
// across the Neutral Zone (Y=1.3 to Y=3.0) collecting Fuel, then return
// to start and shoot.
//
// Start:  RIGHT side
// Steps:
//   1. Cross the bump (arm raised during return)
//   2. Slowly sweep right subway entry → halfway exit while collecting
//   3. Raise arm, retract intake + drive back over bump
//   4. Return to start, aim at hub, shoot until t=19.99s
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSubway6inchRight {

    private AutoSubway6inchRight() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)
            // Cross the bump
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            // Intake while sweeping through the right subway corridor
            .deadline(
                seq -> seq
                    .slow().moveThrough(Nodes.Legacy.Midfield.RIGHT_RIGHT_SUBWAY, 0.5)
                    .slow().moveThrough(Nodes.Legacy.Midfield.LEFT_RIGHT_SUBWAY,  0.5),
                seq -> seq.intakeDeploy()
            )
            // Raise arm before returning over the bump
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Legacy.Midfield.RIGHT_BEFORE_BUMP, 0.5)
            )
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            .driveTo(Nodes.Robot.Start.RIGHT)
            .waitSeconds(0.5)
            // Aim and shoot
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            .shootUntil(19.99)
            .stop()
            .build();
    }
}
