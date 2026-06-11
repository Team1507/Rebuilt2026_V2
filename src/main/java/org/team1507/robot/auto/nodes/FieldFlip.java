//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.nodes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.team1507.lib.core.util.Alliance;

/**
 * Field coordinate flip utilities for the 2026 REBUILT season.
 *
 * <p>All robot nodes in {@link Nodes} are defined in <b>Blue-origin coordinates</b>:
 * (0, 0) at the Blue driver station corner, X increasing toward Red, Y increasing
 * to the left from the driver's perspective.
 *
 * <p>REBUILT 2026 uses <b>rotational symmetry</b> — the Red alliance side of the
 * field is the Blue side rotated 180° around the field center point
 * ({@code LENGTH / 2}, {@code WIDTH / 2}). This is the same symmetry type used in
 * Rapid React (2022) and Reefscape (2025), and is sometimes called a "pizza-pie" flip.
 * It is different from the "hamburger" mirror flip used in games like 2024 Crescendo,
 * where only the X coordinate is inverted.
 *
 * <p>To convert a Blue-origin coordinate to the equivalent Red-alliance position:
 * <pre>
 *   X_red     = LENGTH - X_blue
 *   Y_red     = WIDTH  - Y_blue
 *   heading_red = heading_blue + 180°
 * </pre>
 *
 * <p>These methods are called automatically by {@link org.team1507.robot.auto.AutoSequence}
 * when {@link Alliance#isRed()} returns true. Auto routines and {@link Nodes} never need
 * to call FieldFlip directly — write all coordinates in Blue-origin and the flip is
 * applied at runtime.
 *
 * <p><b>Season note:</b> If a future game uses mirror symmetry instead of rotational
 * symmetry (like 2024 Crescendo), the formulas in this class must be updated. Mirror
 * symmetry keeps Y unchanged and inverts only X; the heading formula also differs.
 * Check the game manual's field layout section before each season, and update
 * {@code Nodes.Field.LENGTH} / {@code Nodes.Field.WIDTH} to match the new dimensions.
 *
 * @see Alliance#isRed()
 * @see Nodes.Field#LENGTH
 * @see Nodes.Field#WIDTH
 */
public final class FieldFlip {

    private FieldFlip() {}

    /**
     * Converts a robot pose from Blue-origin to Red-alliance coordinates.
     *
     * <p>Applies a 180° rotation around the field center to both the XY position
     * and the heading. Call this only when {@link Alliance#isRed()} returns
     * {@code true}; on Blue alliance the original pose is correct as-is.
     *
     * @param blue pose defined in Blue-origin field coordinates
     * @return equivalent pose in Red-alliance field coordinates
     */
    public static Pose2d pose(Pose2d blue) {
        return new Pose2d(
            translation(blue.getTranslation()),
            rotation(blue.getRotation())
        );
    }

    /**
     * Converts a field position from Blue-origin to Red-alliance coordinates.
     *
     * <p>Result is {@code (LENGTH - x, WIDTH - y)}, reflecting the 180° rotational
     * symmetry of the REBUILT 2026 field. Both X and Y are inverted relative to the
     * field center, unlike a simple mirror which would invert only one axis.
     *
     * @param blue position defined in Blue-origin field coordinates
     * @return equivalent position in Red-alliance field coordinates
     */
    public static Translation2d translation(Translation2d blue) {
        return new Translation2d(
            Nodes.Field.LENGTH - blue.getX(),
            Nodes.Field.WIDTH  - blue.getY()
        );
    }

    /**
     * Converts a heading from Blue-origin to Red-alliance direction.
     *
     * <p>Adds 180° to the heading. A robot facing the Blue scoring hub
     * (heading ≈ 0°) will face the Red scoring hub on the opposite side
     * (heading ≈ 180°) after flipping.
     *
     * @param blue heading defined relative to Blue-origin field frame
     * @return equivalent heading in Red-alliance field frame
     */
    public static Rotation2d rotation(Rotation2d blue) {
        return blue.rotateBy(Rotation2d.fromDegrees(180.0));
    }
}
