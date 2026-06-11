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
// AutoHumanPlayerQuest
//
// Shoot preload, receive a human player feed at the Outpost, return and shoot.
// Uses QuestNav-tuned approach pose for the final press to the station wall.
//
// Start:  RIGHT side
// Steps:
//   1. Shoot 8 preloaded Fuel until t=3.5s
//   2. Drive to Outpost approach (QuestNav-tuned pose)
//   3. Deploy intake
//   4. Creep into station wall (0.15 m at 70% speed)
//   5. Wait 1.5 s for human player to load
//   6. Back away to approach point, retract intake, drive back to start
//   7. Shoot remaining Fuel until t=19.5s
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoHumanPlayerQuest {

    private AutoHumanPlayerQuest() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)
            // Shoot preload
            .shootRPMUntil(3.5, kShooter.LOB_RPM)
            // Drive to outpost and deploy intake
            .driveTo(Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT_QUEST)
            .intakeLow()
            // Creep forward into the human player station wall
            .creep().driveForwardMeters(0.15, true)
            // Wait for human player to load Fuel
            .waitSeconds(1.5)
            // Back away and retract intake simultaneously with return drive
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.driveTo(Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT)
            )
            .driveTo(Nodes.Robot.Start.RIGHT)
            // Shoot remaining Fuel
            .pointToTarget(Nodes.Legacy.Hub.CENTER)
            .shootUntil(19.5)
            .stop()
            .build();
    }
}
