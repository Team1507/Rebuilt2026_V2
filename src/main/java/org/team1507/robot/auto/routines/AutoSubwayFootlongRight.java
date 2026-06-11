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
// AutoSubwayFootlongRight
//
// Full-width right-side Neutral Zone sweep. "Footlong" = drive the entire
// field width (Y=1.3 to Y=7.2), collecting as much Fuel as possible.
//
// Start:  RIGHT side
// Steps:
//   1. Cross the bump
//   2. Slowly sweep from right entry all the way to the left footlong endpoint
//      while collecting Fuel
//   3. Raise arm, retract intake + drive back over bump
//   4. Return to start and shoot until t=19.5s
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSubwayFootlongRight {

    private AutoSubwayFootlongRight() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)
            // Cross the bump
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            // Full-width intake sweep: entry → halfway → full extent
            .deadline(
                seq -> seq
                    .slow().moveThrough(Nodes.Legacy.Midfield.RIGHT_RIGHT_SUBWAY,   1.0)
                    .slow().moveThrough(Nodes.Legacy.Midfield.LEFT_RIGHT_SUBWAY,    1.0)
                    .slow().moveThrough(Nodes.Legacy.Midfield.LEFT_FOOTLONG_SUBWAY, 0.5),
                seq -> seq.intakeDeploy()
            )
            // Raise arm before returning
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Legacy.Midfield.RIGHT_BEFORE_BUMP, 0.5)
            )
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            .driveTo(Nodes.Robot.Start.RIGHT)
            // Shoot all collected Fuel
            .shootUntil(19.5)
            .stop()
            .build();
    }
}
