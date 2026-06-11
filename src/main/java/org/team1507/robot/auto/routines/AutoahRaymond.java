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
// AutoahRaymond
//
// Named after Mr. Raymond — shoot 8 preloaded Fuel, never move.
// The safest possible auto when all driving options are risky.
//
// Start:  RIGHT side
// Action: spin up to LOB_RPM, feed all 8 Fuel, hold until 4 s
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoahRaymond {

    private AutoahRaymond() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)
            .shootRPMUntil(4.0, kShooter.LOB_RPM)
            .stop()
            .build();
    }
}
