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
// AutoSubway18InchLeftBlue
//
// Left-side mirror of AutoSubway18Inch, tuned for Blue alliance.
// Wide sweep through the left Neutral Zone, then score from a fixed left
// shooting pose.
//
// "Blue" suffix = Blue alliance tuning. Red variants exist in old code but
// were commented out in RobotContainer. Implement field-flip math in Nodes
// if both alliances need support.
//
// Start:  LEFT side
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSubway18InchLeftBlue {

    private AutoSubway18InchLeftBlue() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Legacy.Start.LEFT)
            // Cross the bump
            .moveThrough(Nodes.Legacy.Midfield.LEFT_OVER_BUMP, 0.5)
            // Intake while sweeping through the left subway corridor
            .deadline(
                seq -> seq
                    .slow().moveThrough(Nodes.Legacy.Midfield.LEFT_LEFT_SUBWAY,  0.5)
                    .slow().moveThrough(Nodes.Legacy.Midfield.RIGHT_LEFT_SUBWAY, 0.5),
                seq -> seq.intakeDeploy()
            )
            // Raise arm before returning over the bump
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Legacy.Midfield.LEFT_BEFORE_BUMP, 0.5)
            )
            .moveThrough(Nodes.Legacy.Midfield.LEFT_OVER_BUMP, 0.5)
            // Drive past start to the fixed tuned scoring pose
            .moveThrough(Nodes.Legacy.Start.LEFT, 1.0)
            .driveTo(Nodes.Legacy.Start.SHOOTING_SPOT_LEFT)
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            .shootUntil(19.5)
            .stop()
            .build();
    }
}
