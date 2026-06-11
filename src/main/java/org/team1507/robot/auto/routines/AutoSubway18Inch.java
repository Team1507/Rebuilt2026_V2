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
// AutoSubway18Inch
//
// Fast rush entry into the right Neutral Zone, slightly longer sweep than
// 6-inch, then score from a fixed tuned pose aimed at the Hub.
// "18 inch" is student naming — exact origin unclear.
//
// Key differences from 6-inch:
//   - Rush entry via RIGHT_RUSH_SUBWAY (faster approach angle)
//   - Returns to SHOOTING_SPOT_RIGHT (fixed tuned pose) instead of start
//   - Uses pointToTarget instead of driving back to start for aim
//
// Start:  RIGHT side
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSubway18Inch {

    private AutoSubway18Inch() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)
            // Cross the bump at full speed
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            // Rush into the subway entry, then slow down for the sweep
            .deadline(
                seq -> seq
                    .moveThrough(Nodes.Legacy.Midfield.RIGHT_RUSH_SUBWAY,   0.5)  // fast entry
                    .slow().moveThrough(Nodes.Legacy.Midfield.RIGHT_RIGHT_SUBWAY, 0.5)
                    .slow().moveThrough(Nodes.Legacy.Midfield.LEFT_RIGHT_SUBWAY, 0.5),
                seq -> seq.intakeDeploy()
            )
            // Return over the bump directly (skip BEFORE_BUMP waypoint)
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            )
            // Drive to the fixed tuned scoring pose
            .driveTo(Nodes.Legacy.Start.SHOOTING_SPOT_RIGHT)
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            .shootUntil(19.9)
            .stop()
            .build();
    }
}
