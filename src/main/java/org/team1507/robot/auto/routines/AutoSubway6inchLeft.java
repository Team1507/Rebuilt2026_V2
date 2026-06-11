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
// AutoSubway6inchLeft
//
// Left-side mirror of AutoSubway6inchRight. Sweeps Y=7.0 → Y=2.5 (half the
// left subway corridor), then returns and shoots.
//
// Start:  LEFT side
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSubway6inchLeft {

    private AutoSubway6inchLeft() {}

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
            .driveTo(Nodes.Legacy.Start.LEFT)
            .waitSeconds(0.5)
            // Aim and shoot
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            .shootUntil(19.99)
            .stop()
            .build();
    }
}
