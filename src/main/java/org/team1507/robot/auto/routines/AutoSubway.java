//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.auto.nodes.Nodes;

// AutoSubway — drives through the bump crossing into the neutral-zone subway,
// sweeps for fuel with the intake deployed, then returns to score.
public final class AutoSubway {

    public enum Side { LEFT, RIGHT }

    private AutoSubway() {}

    public static Command build(Side side) {
        boolean left = side == Side.LEFT;

        Pose2d start       = left ? Nodes.Robot.Start.LEFT          : Nodes.Robot.Start.RIGHT;
        Pose2d score       = left ? Nodes.Robot.Score.LEFT          : Nodes.Robot.Score.RIGHT;
        Pose2d pickupStart = left ? Nodes.Robot.Pickup.START        : mirrorY(Nodes.Robot.Pickup.START);
        Pose2d pickupEnd   = left ? Nodes.Robot.Pickup.END          : mirrorY(Nodes.Robot.Pickup.END);
        Pose2d hubTarget   = new Pose2d(Nodes.FieldElements.Hub.CENTER, new Rotation2d());

        // BUMP_ALLIANCE and BUMP_NEUTRAL are defined for the LEFT (high-Y) side.
        // moveThroughBy applies FieldFlip for Red automatically; mirrorY handles the
        // Left→Right Y-flip for Blue side (Left subway is high Y, Right is low Y).
        Pose2d bumpAlliance = left ? Nodes.Robot.Waypoint.BUMP_ALLIANCE : mirrorY(Nodes.Robot.Waypoint.BUMP_ALLIANCE);
        Pose2d bumpNeutral  = left ? Nodes.Robot.Waypoint.BUMP_NEUTRAL  : mirrorY(Nodes.Robot.Waypoint.BUMP_NEUTRAL);

        return new AutoSequence()
            .withDebug()
            .startTimer()
            .resetPose(start)
            .moveThroughBy(bumpAlliance,  0.2,  1.6).withName("Outbound: alliance side of bump")
            .moveThroughBy(bumpNeutral,   0.2,  2.1).withName("Outbound: neutral side of bump")
            .moveThroughBy(pickupStart,   0.2,  2.7).withName("Outbound: enter subway")
            .deadline(
                seq -> seq.driveToBy(pickupEnd, 99.0),
                seq -> seq.intakeDeploy()
            )
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThroughBy(bumpNeutral, 0.2, 99.0)
            )
            .moveThroughBy(bumpAlliance,  0.2,  5.0).withName("Return: alliance side of bump")
            .driveToBy(score,                              5.9).withName("Return: drive to score")
            .pointToTarget(hubTarget)
            .shootUntil(19.99)
            .stop()
            .build();
    }

    private static Pose2d mirrorY(Pose2d pose) {
        return new Pose2d(
            pose.getX(),
            Nodes.Field.WIDTH - pose.getY(),
            pose.getRotation().unaryMinus()
        );
    }
}
