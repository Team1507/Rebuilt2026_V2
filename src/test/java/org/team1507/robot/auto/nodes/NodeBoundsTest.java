//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.nodes;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.junit.jupiter.api.Test;
import org.team1507.robot.Constants.kSwerve;

// ─────────────────────────────────────────────────────────────────────────────
// NodeBoundsTest
//
// Build-time guard — runs automatically as part of `./gradlew build`.
// Both tests below must pass before the build produces a deployable binary,
// so out-of-bounds or colliding coordinates are caught before the robot ever
// moves.
//
// ── WHAT IS TESTED ──────────────────────────────────────────────────────────
//
//   allNodesWithinBounds
//     Every Robot node (Pose2d) must lie within the active field boundary.
//     Location nodes (Translation2d, used for field reference / obstacle math)
//     are NOT checked — the robot never drives to those.
//     Red-side flipped nodes are not checked separately: FieldFlip maps
//     (x,y)→(16.54−x,8.21−y), so any in-bounds point always flips to an
//     in-bounds point. nodeFlipRoundTrip confirms the math is self-consistent.
//
//   noRobotNodeCollidesWithObstacles
//     No Robot node may place the robot's spin circle inside or touching a
//     field structure. Each obstacle is defined by a CORNERS[] polygon in
//     Nodes.FieldElements. All nodes are Blue-origin; Red-side obstacles are
//     symmetric by the same FieldFlip argument above.
//
// ── WHICH NODES ARE CHECKED ─────────────────────────────────────────────────
//
//   Robot.*   — validated in-season nodes used in active auto routines.
//   Legacy.*  — nodes copied verbatim from Rebuilt2026 (the previous season's
//               code). These have NOT been confirmed against the V2 field model.
//               Testing them catches coordinates that are grossly wrong (off-field,
//               inside an obstacle) before an old routine is accidentally run.
//               When you confirm a Legacy node is correct and move it into
//               Robot.*, delete the Legacy entry and its test call here.
//
//   Location nodes (Nodes.Field.*, Nodes.FieldElements.*) are intentionally
//   excluded — they describe field geometry, not robot destinations.
//
// ── FIELD BOUNDARY ──────────────────────────────────────────────────────────
//
//   Set Nodes.Field.PRACTICE_MODE = true when building at the field room.
//   The test then validates against PRACTICE_LENGTH / PRACTICE_WIDTH instead
//   of competition dimensions. Set it back to false before a competition build.
//
// ── COLLISION DETECTION ─────────────────────────────────────────────────────
//
//   Each robot node is modeled as a circle:
//     center = node XY position
//     radius = robot half-diagonal (center-to-module corner) + BUMPER_BUFFER
//   Each field element is a convex polygon defined by its CORNERS[] array.
//   Collision is true when:
//     (a) the node center is inside the polygon, OR
//     (b) the circle's edge reaches any polygon edge (distance < radius).
//
//   Works for any polygon shape. To add a new obstacle each season, add one
//   obstacle() call inside clearanceCheck() for the new FieldElements class.
//
// ── ADDING NEW NODES ────────────────────────────────────────────────────────
//
//   For each new Robot.* constant: add one check() call in allNodesWithinBounds()
//   and one clearanceCheck() call in noRobotNodeCollidesWithObstacles().
//   Use the full Nodes path as the name string — it appears in failure messages.
//
//   For each new Legacy.* constant: same pattern, grouped under the Legacy.*
//   section in each test.
// ─────────────────────────────────────────────────────────────────────────────
class NodeBoundsTest {

    // =========================================================================
    // Field boundary limits
    // =========================================================================

    private static final double MAX_X = Nodes.Field.PRACTICE_MODE
        ? Nodes.Field.PRACTICE_LENGTH
        : Nodes.Field.LENGTH;

    private static final double MAX_Y = Nodes.Field.PRACTICE_MODE
        ? Nodes.Field.PRACTICE_WIDTH
        : Nodes.Field.WIDTH;

    // =========================================================================
    // Robot footprint
    //
    // The robot is approximated as a circle for collision purposes.
    // Radius = distance from robot center to the outermost swerve module corner,
    // plus bumper thickness. This is conservative (a circle is larger than the
    // actual rectangular footprint), which is the safe direction to err.
    // =========================================================================

    private static final double ROBOT_CLEARANCE_RADIUS = kSwerve.ROBOT_CLEARANCE_RADIUS;

    // =========================================================================
    // Test 1 — Field boundary
    // =========================================================================

    @Test
    void allNodesWithinBounds() {

        // ── Robot.Start ─────────────────────────────────────────────────────────────────────
        check("Robot.Start.RIGHT",                                  Nodes.Robot.Start.RIGHT);
        check("Robot.Start.CENTER",                                 Nodes.Robot.Start.CENTER);
        check("Robot.Start.LEFT",                                   Nodes.Robot.Start.LEFT);

        // ── Robot.Score ─────────────────────────────────────────────────────────────────────
        check("Robot.Score.RIGHT",                                  Nodes.Robot.Score.RIGHT);
        check("Robot.Score.LEFT",                                   Nodes.Robot.Score.LEFT);

        // ── Robot.Pickup ────────────────────────────────────────────────────────────────────
        check("Robot.Pickup.APPROACH_RIGHT",                        Nodes.Robot.Pickup.APPROACH_RIGHT);
        check("Robot.Pickup.STATION_RIGHT",                         Nodes.Robot.Pickup.STATION_RIGHT);
        check("Robot.Pickup.APPROACH_LEFT",                         Nodes.Robot.Pickup.APPROACH_LEFT);
        check("Robot.Pickup.STATION_LEFT",                          Nodes.Robot.Pickup.STATION_LEFT);
        check("Robot.Pickup.START",                                  Nodes.Robot.Pickup.START);
        check("Robot.Pickup.END",                                    Nodes.Robot.Pickup.END);

        // ── Robot.Waypoint ──────────────────────────────────────────────────────────────────
        check("Robot.Waypoint.MIDFIELD_RIGHT",                      Nodes.Robot.Waypoint.MIDFIELD_RIGHT);
        check("Robot.Waypoint.MIDFIELD_CENTER",                     Nodes.Robot.Waypoint.MIDFIELD_CENTER);
        check("Robot.Waypoint.MIDFIELD_LEFT",                       Nodes.Robot.Waypoint.MIDFIELD_LEFT);
        check("Robot.Waypoint.BUMP_CROSS_LEFT",                     Nodes.Robot.Waypoint.BUMP_CROSS_LEFT);

        // ── Legacy.Start ────────────────────────────────────────────────────────────────────
        // Copied from Rebuilt2026 — not yet confirmed against V2 field model.
        check("Legacy.Start.RIGHT",                                 Nodes.Legacy.Start.RIGHT);
        check("Legacy.Start.LEFT",                                  Nodes.Legacy.Start.LEFT);
        check("Legacy.Start.SHOOTING_SPOT_RIGHT",                   Nodes.Legacy.Start.SHOOTING_SPOT_RIGHT);
        check("Legacy.Start.SHOOTING_SPOT_LEFT",                    Nodes.Legacy.Start.SHOOTING_SPOT_LEFT);

        // ── Legacy.Midfield ─────────────────────────────────────────────────────────────────
        check("Legacy.Midfield.RIGHT_OVER_BUMP",                    Nodes.Legacy.Midfield.RIGHT_OVER_BUMP);
        check("Legacy.Midfield.RIGHT_OVER_BUMP2",                   Nodes.Legacy.Midfield.RIGHT_OVER_BUMP2);
        check("Legacy.Midfield.LEFT_OVER_BUMP",                     Nodes.Legacy.Midfield.LEFT_OVER_BUMP);
        check("Legacy.Midfield.RIGHT_BEFORE_BUMP",                  Nodes.Legacy.Midfield.RIGHT_BEFORE_BUMP);
        check("Legacy.Midfield.LEFT_BEFORE_BUMP",                   Nodes.Legacy.Midfield.LEFT_BEFORE_BUMP);
        check("Legacy.Midfield.RIGHT_TURN",                         Nodes.Legacy.Midfield.RIGHT_TURN);
        check("Legacy.Midfield.RIGHT_RIGHT_SUBWAY",                 Nodes.Legacy.Midfield.RIGHT_RIGHT_SUBWAY);
        check("Legacy.Midfield.LEFT_RIGHT_SUBWAY",                  Nodes.Legacy.Midfield.LEFT_RIGHT_SUBWAY);
        check("Legacy.Midfield.RIGHT_RUSH_SUBWAY",                  Nodes.Legacy.Midfield.RIGHT_RUSH_SUBWAY);
        check("Legacy.Midfield.LOWER_RIGHT_RIGHT_SUBWAY",           Nodes.Legacy.Midfield.LOWER_RIGHT_RIGHT_SUBWAY);
        check("Legacy.Midfield.RIGHT_LEFT_SUBWAY",                  Nodes.Legacy.Midfield.RIGHT_LEFT_SUBWAY);
        check("Legacy.Midfield.MIDDLE_RIGHT_SUBWAY",                Nodes.Legacy.Midfield.MIDDLE_RIGHT_SUBWAY);
        check("Legacy.Midfield.LEFT_LEFT_SUBWAY",                   Nodes.Legacy.Midfield.LEFT_LEFT_SUBWAY);
        check("Legacy.Midfield.LEFT_FOOTLONG_SUBWAY",               Nodes.Legacy.Midfield.LEFT_FOOTLONG_SUBWAY);
        check("Legacy.Midfield.SUBWAY_AROUND_THE_HUB",              Nodes.Legacy.Midfield.SUBWAY_AROUND_THE_HUB);

        // ── Legacy.Outpost ──────────────────────────────────────────────────────────────────
        check("Legacy.Outpost.RIGHT_APPROACH_POINT",                Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT);
        check("Legacy.Outpost.RIGHT_APPROACH_POINT_QUEST",          Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT_QUEST);
    }

    // =========================================================================
    // Test 2 — Field element collision
    // =========================================================================

    @Test
    void noRobotNodeCollidesWithObstacles() {

        // ── Robot.Start ─────────────────────────────────────────────────────────────────────
        clearanceCheck("Robot.Start.RIGHT",                         Nodes.Robot.Start.RIGHT);
        clearanceCheck("Robot.Start.CENTER",                        Nodes.Robot.Start.CENTER);
        clearanceCheck("Robot.Start.LEFT",                          Nodes.Robot.Start.LEFT);

        // ── Robot.Score ─────────────────────────────────────────────────────────────────────
        clearanceCheck("Robot.Score.RIGHT",                         Nodes.Robot.Score.RIGHT);
        clearanceCheck("Robot.Score.LEFT",                          Nodes.Robot.Score.LEFT);

        // ── Robot.Pickup ────────────────────────────────────────────────────────────────────
        clearanceCheck("Robot.Pickup.APPROACH_RIGHT",               Nodes.Robot.Pickup.APPROACH_RIGHT);
        clearanceCheck("Robot.Pickup.STATION_RIGHT",                Nodes.Robot.Pickup.STATION_RIGHT);
        clearanceCheck("Robot.Pickup.APPROACH_LEFT",                Nodes.Robot.Pickup.APPROACH_LEFT);
        clearanceCheck("Robot.Pickup.STATION_LEFT",                 Nodes.Robot.Pickup.STATION_LEFT);
        clearanceCheck("Robot.Pickup.START",                        Nodes.Robot.Pickup.START);
        clearanceCheck("Robot.Pickup.END",                          Nodes.Robot.Pickup.END);

        // ── Robot.Waypoint ──────────────────────────────────────────────────────────────────
        clearanceCheck("Robot.Waypoint.MIDFIELD_RIGHT",             Nodes.Robot.Waypoint.MIDFIELD_RIGHT);
        clearanceCheck("Robot.Waypoint.MIDFIELD_CENTER",            Nodes.Robot.Waypoint.MIDFIELD_CENTER);
        clearanceCheck("Robot.Waypoint.MIDFIELD_LEFT",              Nodes.Robot.Waypoint.MIDFIELD_LEFT);
        clearanceCheck("Robot.Waypoint.BUMP_CROSS_LEFT",            Nodes.Robot.Waypoint.BUMP_CROSS_LEFT);

        // ── Legacy.Start ────────────────────────────────────────────────────────────────────
        // Copied from Rebuilt2026 — not yet confirmed against V2 field model.
        clearanceCheck("Legacy.Start.RIGHT",                        Nodes.Legacy.Start.RIGHT);
        clearanceCheck("Legacy.Start.LEFT",                         Nodes.Legacy.Start.LEFT);
        clearanceCheck("Legacy.Start.SHOOTING_SPOT_RIGHT",          Nodes.Legacy.Start.SHOOTING_SPOT_RIGHT);
        clearanceCheck("Legacy.Start.SHOOTING_SPOT_LEFT",           Nodes.Legacy.Start.SHOOTING_SPOT_LEFT);

        // ── Legacy.Midfield ─────────────────────────────────────────────────────────────────
        clearanceCheck("Legacy.Midfield.RIGHT_OVER_BUMP",           Nodes.Legacy.Midfield.RIGHT_OVER_BUMP);
        clearanceCheck("Legacy.Midfield.RIGHT_OVER_BUMP2",          Nodes.Legacy.Midfield.RIGHT_OVER_BUMP2);
        clearanceCheck("Legacy.Midfield.LEFT_OVER_BUMP",            Nodes.Legacy.Midfield.LEFT_OVER_BUMP);
        clearanceCheck("Legacy.Midfield.RIGHT_BEFORE_BUMP",         Nodes.Legacy.Midfield.RIGHT_BEFORE_BUMP);
        clearanceCheck("Legacy.Midfield.LEFT_BEFORE_BUMP",          Nodes.Legacy.Midfield.LEFT_BEFORE_BUMP);
        clearanceCheck("Legacy.Midfield.RIGHT_TURN",                Nodes.Legacy.Midfield.RIGHT_TURN);
        clearanceCheck("Legacy.Midfield.RIGHT_RIGHT_SUBWAY",        Nodes.Legacy.Midfield.RIGHT_RIGHT_SUBWAY);
        clearanceCheck("Legacy.Midfield.LEFT_RIGHT_SUBWAY",         Nodes.Legacy.Midfield.LEFT_RIGHT_SUBWAY);
        clearanceCheck("Legacy.Midfield.RIGHT_RUSH_SUBWAY",         Nodes.Legacy.Midfield.RIGHT_RUSH_SUBWAY);
        clearanceCheck("Legacy.Midfield.LOWER_RIGHT_RIGHT_SUBWAY",  Nodes.Legacy.Midfield.LOWER_RIGHT_RIGHT_SUBWAY);
        clearanceCheck("Legacy.Midfield.RIGHT_LEFT_SUBWAY",         Nodes.Legacy.Midfield.RIGHT_LEFT_SUBWAY);
        clearanceCheck("Legacy.Midfield.MIDDLE_RIGHT_SUBWAY",       Nodes.Legacy.Midfield.MIDDLE_RIGHT_SUBWAY);
        clearanceCheck("Legacy.Midfield.LEFT_LEFT_SUBWAY",          Nodes.Legacy.Midfield.LEFT_LEFT_SUBWAY);
        clearanceCheck("Legacy.Midfield.LEFT_FOOTLONG_SUBWAY",      Nodes.Legacy.Midfield.LEFT_FOOTLONG_SUBWAY);
        clearanceCheck("Legacy.Midfield.SUBWAY_AROUND_THE_HUB",     Nodes.Legacy.Midfield.SUBWAY_AROUND_THE_HUB);

        // ── Legacy.Hub ──────────────────────────────────────────────────────────────────────
        // Legacy.Hub.CENTER is a rotation-pointing target — the robot faces this point to aim
        // at the hub but never drives there. It is intentionally inside the hub structure.
        // clearanceCheck("Legacy.Hub.CENTER", Nodes.Legacy.Hub.CENTER);  // excluded: rotation-only

        // ── Legacy.Outpost ──────────────────────────────────────────────────────────────────
        clearanceCheck("Legacy.Outpost.RIGHT_APPROACH_POINT",       Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT);
        clearanceCheck("Legacy.Outpost.RIGHT_APPROACH_POINT_QUEST", Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT_QUEST);
    }

    // =========================================================================
    // Test 3 — Flip round-trip (math sanity)
    // =========================================================================

    @Test
    void nodeFlipRoundTrip() {
        // Flipping any pose twice must return the original within floating-point tolerance.
        // Catches sign errors or missing axes in FieldFlip's math.
        final double EPSILON = 1e-9;

        Pose2d[] nodes = {
            Nodes.Robot.Start.RIGHT,    Nodes.Robot.Start.CENTER,    Nodes.Robot.Start.LEFT,
            Nodes.Robot.Score.RIGHT,    Nodes.Robot.Score.LEFT,
            Nodes.Robot.Pickup.APPROACH_RIGHT, Nodes.Robot.Pickup.STATION_RIGHT,
            Nodes.Robot.Pickup.APPROACH_LEFT,  Nodes.Robot.Pickup.STATION_LEFT,
            Nodes.Robot.Pickup.START,          Nodes.Robot.Pickup.END,
            Nodes.Robot.Waypoint.MIDFIELD_RIGHT, Nodes.Robot.Waypoint.MIDFIELD_CENTER, Nodes.Robot.Waypoint.MIDFIELD_LEFT,
            Nodes.Robot.Waypoint.BUMP_CROSS_LEFT,
            Nodes.Legacy.Start.RIGHT,   Nodes.Legacy.Start.LEFT,
            Nodes.Legacy.Hub.CENTER,
            Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT, Nodes.Legacy.Outpost.RIGHT_APPROACH_POINT_QUEST,
        };

        for (Pose2d original : nodes) {
            Pose2d roundTripped = FieldFlip.pose(FieldFlip.pose(original));
            assertEquals(original.getX(), roundTripped.getX(), EPSILON,
                "Round-trip flip changed X for pose " + original);
            assertEquals(original.getY(), roundTripped.getY(), EPSILON,
                "Round-trip flip changed Y for pose " + original);
            // Compare cos/sin directly — getDegrees() can return un-normalized values
            // (e.g. 305° instead of -55°) when created from Node.at(), but Rotation2d's
            // internal cos/sin are always canonical and survive the round-trip exactly.
            assertEquals(original.getRotation().getCos(), roundTripped.getRotation().getCos(), EPSILON,
                "Round-trip flip changed heading (cos) for pose " + original);
            assertEquals(original.getRotation().getSin(), roundTripped.getRotation().getSin(), EPSILON,
                "Round-trip flip changed heading (sin) for pose " + original);
        }
    }

    /**
     * Checks a single robot node against every registered field element obstacle.
     * Add one obstacle() call here for each new FieldElements structure each season.
     *
     * Bumps (BumpLeft, BumpRight, and their Red counterparts) are intentionally
     * omitted — the robot drives through the subway lanes and many start/score
     * nodes sit inside the bump footprint. Bumps are passable floor elevations,
     * not structures the robot must avoid.
     */
    private void clearanceCheck(String name, Pose2d pose) {
        Translation2d position = pose.getTranslation();
        obstacle(name, position, "FieldElements.Hub",         Nodes.FieldElements.Hub.CORNERS);
        obstacle(name, position, "FieldElements.TrenchLeft",  Nodes.FieldElements.TrenchLeft.CORNERS);
        obstacle(name, position, "FieldElements.TrenchRight", Nodes.FieldElements.TrenchRight.CORNERS);
        // Bumps omitted — robot drives through subway lanes; many nodes sit inside the bump footprint
        // Red-side obstacles computed via FieldFlip when needed in auto or future tests
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private void check(String name, Pose2d pose) {
        check(name, pose.getX(), pose.getY());
    }

    private void check(String name, double x, double y) {
        String bounds = String.format("%.2f m × %.2f m", MAX_X, MAX_Y);
        assertTrue(x >= 0,    name + " — x=" + x + " is negative (outside field)");
        assertTrue(x <= MAX_X, name + " — x=" + x + " exceeds field length " + MAX_X + " (" + bounds + ")");
        assertTrue(y >= 0,    name + " — y=" + y + " is negative (outside field)");
        assertTrue(y <= MAX_Y, name + " — y=" + y + " exceeds field width "  + MAX_Y + " (" + bounds + ")");
    }

    /**
     * Circle-polygon collision check between a robot node and a field obstacle.
     *
     * The robot is modeled as a circle centered at the node position with radius
     * ROBOT_CLEARANCE_RADIUS. The obstacle is a convex polygon defined by its corners.
     *
     * Two failure conditions:
     *   (a) node center is inside the polygon — the robot would be occupying the structure
     *   (b) distance from node center to any polygon edge < ROBOT_CLEARANCE_RADIUS —
     *       the robot's spin circle overlaps the structure
     *
     * Works for any polygon shape — square, octagon, etc. Add the corners in CORNERS[]
     * and the test handles the rest.
     *
     * @param nodeName     display name of the robot node (shown in failure message)
     * @param nodePos      XY position of the robot node
     * @param obstacleName display name of the obstacle (shown in failure message)
     * @param corners      ordered polygon corners (clockwise or counter-clockwise)
     */
    private void obstacle(
            String nodeName, Translation2d nodePos,
            String obstacleName, Translation2d[] corners) {

        // (a) Node is inside the polygon — definite collision regardless of radius
        if (isInsidePolygon(nodePos, corners)) {
            fail(String.format(
                "%s is inside %s — node at (%.3f, %.3f) is within the structure boundary",
                nodeName, obstacleName, nodePos.getX(), nodePos.getY()
            ));
            return;
        }

        // (b) Robot's spin circle reaches a polygon edge
        double minDistance = Double.MAX_VALUE;
        int n = corners.length;
        for (int i = 0; i < n; i++) {
            double d = pointToSegmentDistance(nodePos, corners[i], corners[(i + 1) % n]);
            if (d < minDistance) minDistance = d;
        }

        assertTrue(minDistance >= ROBOT_CLEARANCE_RADIUS, String.format(
            "%s — robot spin circle (r=%.3f m) intersects %s: "
            + "closest edge distance=%.3f m",
            nodeName, ROBOT_CLEARANCE_RADIUS, obstacleName, minDistance
        ));
    }

    /**
     * Ray-casting point-in-polygon test. Works for any simple polygon regardless
     * of winding order. Returns true if {@code point} is strictly inside {@code polygon}.
     */
    private boolean isInsidePolygon(Translation2d point, Translation2d[] polygon) {
        boolean inside = false;
        int n = polygon.length;
        double px = point.getX(), py = point.getY();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i].getX(), yi = polygon[i].getY();
            double xj = polygon[j].getX(), yj = polygon[j].getY();
            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    /**
     * Returns the shortest distance from point {@code p} to the line segment {@code (a, b)}.
     * Projects {@code p} onto the segment, clamping to [0, 1] so the result is always
     * the distance to a point on the segment (not its infinite extension).
     */
    private double pointToSegmentDistance(Translation2d p, Translation2d a, Translation2d b) {
        double ax = a.getX(), ay = a.getY();
        double bx = b.getX(), by = b.getY();
        double px = p.getX(), py = p.getY();

        double abx = bx - ax, aby = by - ay;
        double ab2 = abx * abx + aby * aby;

        if (ab2 == 0.0) return p.getDistance(a); // degenerate segment (a == b)

        double t = ((px - ax) * abx + (py - ay) * aby) / ab2;
        t = Math.max(0.0, Math.min(1.0, t));

        return p.getDistance(new Translation2d(ax + t * abx, ay + t * aby));
    }
}
