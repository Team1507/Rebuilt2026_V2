//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.robot.Constants.kShooter;
import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.auto.nodes.Nodes;

// ─────────────────────────────────────────────────────────────────────────────
// AutoDoubleSubway
//
// Two full right-side Neutral Zone cycles before end of match.
// Cycle 1 must complete by ~13.5 s; cycle 2 uses a different sweep path
// (deeper entry, center-field exit) to cover a different Fuel zone.
//
// Start:  RIGHT side
//
// Cycle 1 — same path as AutoSubway6inchRight, deadline at 13.5 s
// Cycle 2 — deeper entry (LOWER_RIGHT_RIGHT_SUBWAY) + center sweep (MIDDLE_RIGHT_SUBWAY)
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoDoubleSubway {

    private AutoDoubleSubway() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)

            // ── CYCLE 1 ───────────────────────────────────────────────────
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            .deadline(
                seq -> seq
                    .slow().moveThrough(Nodes.Legacy.Midfield.RIGHT_RIGHT_SUBWAY, 0.5)
                    .slow().moveThrough(Nodes.Legacy.Midfield.LEFT_RIGHT_SUBWAY,  0.5),
                seq -> seq.intakeDeploy()
            )
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Legacy.Midfield.RIGHT_BEFORE_BUMP, 0.5)
            )
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            .driveTo(Nodes.Robot.Start.RIGHT)
            .waitSeconds(0.5)
            .slow().driveTo(Nodes.Legacy.Start.SHOOTING_SPOT_RIGHT)
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            // Hard deadline — cycle 1 must finish by 13.5 s
            .shootRPMUntil(13.5, kShooter.LOB_RPM)

            // ── CYCLE 2 ───────────────────────────────────────────────────
            // Re-enter at a different heading for the approach angle
            .changeHeading(Nodes.Legacy.Midfield.RIGHT_TURN)
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP2, 0.5)
            .deadline(
                seq -> seq
                    // Deeper right entry, then sweep across center of field
                    .slow().moveThrough(Nodes.Legacy.Midfield.LOWER_RIGHT_RIGHT_SUBWAY, 0.5)
                    .slow().moveThrough(Nodes.Legacy.Midfield.MIDDLE_RIGHT_SUBWAY,      0.5),
                seq -> seq.intakeDeploy()
            )
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Legacy.Midfield.RIGHT_BEFORE_BUMP, 0.5)
            )
            .moveThrough(Nodes.Legacy.Midfield.RIGHT_OVER_BUMP, 0.5)
            .driveTo(Nodes.Robot.Start.RIGHT)
            .waitSeconds(0.5)
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            .shootUntil(19.99)
            .stop()
            .build();
    }
}
