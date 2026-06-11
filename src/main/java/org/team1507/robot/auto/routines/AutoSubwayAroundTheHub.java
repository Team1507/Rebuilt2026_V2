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
// AutoSubwayAroundTheHub
//
// The most ambitious auto: full right cycle, transit behind the Hub to the
// left side, full left cycle. Likely never successfully run in competition.
//
// SUBWAY_AROUND_THE_HUB (2.0, 5.0) routes around the back of the Hub
// structure to avoid collision during the cross-field transit.
//
// Cycle 1 must complete by ~9 s — very tight timing constraint.
//
// Start:  RIGHT side
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSubwayAroundTheHub {

    private AutoSubwayAroundTheHub() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)

            // ── CYCLE 1: RIGHT SIDE ───────────────────────────────────────
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
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            // Hard deadline — cycle 1 must finish by 9 s
            .shootUntil(9.0)

            // ── TRANSIT: AROUND THE HUB ───────────────────────────────────
            // Route behind the Hub to reach the left side without collision
            .moveThrough(Nodes.Legacy.Midfield.SUBWAY_AROUND_THE_HUB, 0.5)
            .moveThrough(Nodes.Legacy.Start.LEFT, 0.5)
            .moveThrough(Nodes.Legacy.Midfield.LEFT_OVER_BUMP, 0.5)

            // ── CYCLE 2: LEFT SIDE ────────────────────────────────────────
            .deadline(
                seq -> seq
                    .slow().moveThrough(Nodes.Legacy.Midfield.LEFT_LEFT_SUBWAY,  0.5)
                    .slow().moveThrough(Nodes.Legacy.Midfield.RIGHT_LEFT_SUBWAY, 0.5),
                seq -> seq.intakeDeploy()
            )
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Legacy.Midfield.LEFT_BEFORE_BUMP, 0.5)
            )
            .moveThrough(Nodes.Legacy.Midfield.LEFT_OVER_BUMP, 0.5)
            .driveTo(Nodes.Legacy.Start.LEFT)
            .waitSeconds(0.5)
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            .shootUntil(19.99)
            .stop()
            .build();
    }
}
