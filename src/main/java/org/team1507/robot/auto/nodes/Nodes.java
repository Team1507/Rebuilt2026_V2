//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.nodes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

// ─────────────────────────────────────────────────────────────────────────────
// Nodes
//
// Named navigation poses organized by category. All coordinates are in meters,
// in WPILib field coordinates with (0, 0) at the Blue DS corner.
//
// Type convention — visible at a glance in any editor:
//   Pose2d       — robot nodes (Node.at);      the robot drives here with a heading
//   Translation2d — field nodes (Node.location); describes where a structure is
//
// Three categories:
//   Field        — physical boundary corners (Translation2d — locations only)
//   Robot        — where the robot is commanded to drive, organized by role:
//                    Start, Score, Pickup, Waypoint (all Pose2d)
//   FieldElements — where physical field structures are located (Translation2d);
//                  repopulate each season after game reveal
//
// Naming conventions:
//   - LEFT/RIGHT are always driver-relative (from driver's perspective looking
//     onto the field). Add a comment if you deviate from this.
//   - APPROACH nodes are 1+ m from their target, giving the APF controller
//     room to decelerate before the final position.
//   - STATION nodes are the final docked positions at a field structure.
//
// To add a robot node:   Node.at(x, y, degrees)  → Pose2d       → goes in Robot.*
// To add a field node:   Node.location(x, y)     → Translation2d → goes in Field or FieldElements.*
// ─────────────────────────────────────────────────────────────────────────────
public final class Nodes {

    private Nodes() {}

    // ─────────────────────────────────────────────────────────────────────
    // Robot — where the robot is commanded to drive (Pose2d)
    //
    // Organized by role. LEFT/RIGHT are driver-relative throughout.
    // Change a node here once and every auto that references it updates.
    // ─────────────────────────────────────────────────────────────────────
    public static final class Robot {

        // Starting positions — where the robot is placed before the match.
        public static final class Start {
            public static final Pose2d RIGHT  = Node.at(3.5, 2.5,  55.0);
            public static final Pose2d CENTER = Node.at(2.0, 4.1,   0.0);
            public static final Pose2d LEFT   = Node.at(3.5, 5.7, 305.0);
        }

        // Scoring poses — where the robot faces to score a game piece.
        public static final class Score {
            public static final Pose2d RIGHT = Node.at(3.3, 2.8, 49.52);
            public static final Pose2d LEFT  = Node.at(3.4, 5.3, 320.3);
        }

        // Pickup poses — APPROACH is 1 m out from the station wall,
        // STATION is the final docked position against the wall.
        // START/END are the entry and exit of the neutral-zone subway sweep (Left side).
        // For Right side, the auto routine applies a Y-mirror: y → 8.21−y, heading → −heading.
        public static final class Pickup {
            public static final Pose2d APPROACH_RIGHT = Node.at(0.9, 0.70, 180.0);
            public static final Pose2d STATION_RIGHT  = Node.at(0.5, 0.70, 180.0);
            public static final Pose2d APPROACH_LEFT  = Node.at(0.9, 7.12, 180.0);
            public static final Pose2d STATION_LEFT   = Node.at(0.5, 7.51, 180.0);

            // Subway sweep intent nodes — Left side (high Y). Validate on-field before competition.
            // Derived from Legacy.Midfield.LEFT_LEFT_SUBWAY and RIGHT_LEFT_SUBWAY.
            public static final Pose2d START = Node.at(7.40, 7.00, 270.0); // left subway entry
            public static final Pose2d END   = Node.at(7.95, 2.50, 270.0); // sweep exit toward center
        }

        // Waypoints — intermediate navigation poses used when routing across
        // the field. Not tied to any specific game action.
        public static final class Waypoint {
            public static final Pose2d MIDFIELD_RIGHT  = Node.at(8.27, 2.0, 0.0);
            public static final Pose2d MIDFIELD_CENTER = Node.at(8.27, 4.1, 0.0);
            public static final Pose2d MIDFIELD_LEFT   = Node.at(8.27, 6.2, 0.0);

            // Bump crossing waypoint on the neutral side. X=5.5 is just past the hub far edge
            // (X=5.223). Y=5.6 centers the robot in the BumpLeft corridor [4.632, 6.556].
            // For Right side, auto routines apply a Y-mirror: (5.5, 8.21 − 5.6) = (5.5, 2.61).
            public static final Pose2d BUMP_CROSS_LEFT = Node.at(5.5, 5.6, 45.0);

            // Alliance-zone approach node for bump crossing. X=3.7 is just before the hub
            // near edge (X=4.029). Y=5.6 centers the robot in the BumpLeft corridor [4.632,
            // 6.556] — above the Hub wall and below the TrenchLeft wall.
            // FieldFlip handles the Red-side mirror at use-time; no separate RIGHT node needed.
            public static final Pose2d BUMP_ALLIANCE = Node.at(3.7, 5.6, 0.0);

            // Neutral-zone exit node for bump crossing. X=5.4 is just past the hub far edge
            // (X=5.223). Same Y=5.6 for a straight crossing.
            // FieldFlip handles the Red-side mirror at use-time; no separate RIGHT node needed.
            public static final Pose2d BUMP_NEUTRAL  = Node.at(5.4, 5.6, 0.0);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Field — physical boundary dimensions and corners (Translation2d)
    //
    // No heading — corners are reference locations, not robot destinations.
    //
    // PRACTICE_MODE — set to true when working in the field room.
    // NodeBoundsTest reads this flag and validates all nodes against the
    // smaller practice dimensions. Any node outside those bounds fails the
    // build so the robot cannot be deployed with an out-of-bounds coordinate.
    //
    // Measure your field room and update PRACTICE_LENGTH / PRACTICE_WIDTH.
    // ─────────────────────────────────────────────────────────────────────
    public static final class Field {

        // Competition field — 2025/2026 standard dimensions
        public static final double LENGTH = 16.54; // meters
        public static final double WIDTH  =  8.21; // meters

        // Practice field room — update these to match your actual space
        public static final double PRACTICE_LENGTH = 8.27; // TODO: measure your field room
        public static final double PRACTICE_WIDTH  = 8.21; // TODO: measure your field room

        // ── Flip this to true when building for the practice field room ──
        public static final boolean PRACTICE_MODE = false;

        public static final Translation2d ORIGIN          = Node.location(  0.00,  0.00);
        public static final Translation2d BLUE_FAR_CORNER = Node.location(  0.00, WIDTH);
        public static final Translation2d RED_NEAR_CORNER = Node.location(LENGTH,  0.00);
        public static final Translation2d RED_FAR_CORNER  = Node.location(LENGTH, WIDTH);
    }

    // ─────────────────────────────────────────────────────────────────────
    // FieldElements — where physical field structures are located (Translation2d)
    //
    // One nested class per structure. Each class exposes:
    //   CORNER_* — named corners of the structure (Translation2d), for readability
    //   CORNERS  — ordered array of all corners, used by NodeBoundsTest.
    //              Walk edges as consecutive pairs (wrapping last→first).
    //              Works for any convex polygon — square, octagon, etc.
    //              No bounding box needed; the test derives geometry from CORNERS.
    //
    // Corner naming:
    //   NEAR = closer to Blue DS (low X), FAR = far side (high X).
    //   LEFT/RIGHT are driver-relative (left = high Y in WPILib coords).
    //   INNER = hub-facing side, WALL = field boundary side (for Trench).
    //   HUB = hub-adjacent side, TRENCH = trench-adjacent side (for Bump).
    // Winding order in CORNERS: clockwise when viewed from above (x right, y up).
    //
    // Hub position measured from AdvantageScope field data: center = (182.11 in, 158.84 in) = (4.626, 4.035) m.
    //   HUB_WIDTH = 47 in = 1.194 m (half = 0.597 m). Hub X: [4.029, 5.223]. Hub Y: [3.438, 4.632].
    //   TRENCH_OFFSET = 96.5 in = 2.451 m from field center Y (4.105). TrenchLeft inner = 6.556. TrenchRight inner = 1.654.
    //   Red-side coordinates: compute at use-time via FieldFlip.translation() / FieldFlip.pose().
    //
    // Field cross-section (Y axis, same X range [4.029, 5.223] for all structures):
    // Hub center measured from AdvantageScope field data: 182.11 in × 158.84 in = (4.626, 4.035) m.
    //   Y=8.21  ┌──────────────┐  TrenchLeft (impassable — robot too tall)
    //   Y=6.556 ├──────────────┤
    //           │  BumpLeft    │  Elevated floor — passable, robot crosses here
    //   Y=4.632 ├──────────────┤
    //   Y=3.438 │    Hub       │  Solid structure — robot cannot enter
    //           ├──────────────┤
    //   Y=1.654 │  BumpRight   │  Elevated floor — passable, robot crosses here
    //   Y=0.0   ├──────────────┤
    //           └──────────────┘  TrenchRight (impassable)
    // ─────────────────────────────────────────────────────────────────────
    public static final class FieldElements {

        // ── Hub ──────────────────────────────────────────────────────────────
        public static final class Hub {
            // 47 in × 47 in solid box. Measured from AdvantageScope field data:
            // center = (182.11 in, 158.84 in) = (4.626, 4.035) m. Half-width = 23.5 in = 0.597 m.
            // NEAR_X = 4.626 − 0.597 = 4.029.  FAR_X = 4.626 + 0.597 = 5.223.
            // LEFT_Y = 4.035 + 0.597 = 4.632.  RIGHT_Y = 4.035 − 0.597 = 3.438.
            public static final Translation2d CORNER_NEAR_LEFT  = Node.location(4.029, 4.632);
            public static final Translation2d CORNER_FAR_LEFT   = Node.location(5.223, 4.632);
            public static final Translation2d CORNER_FAR_RIGHT  = Node.location(5.223, 3.438);
            public static final Translation2d CORNER_NEAR_RIGHT = Node.location(4.029, 3.438);
            public static final Translation2d CENTER            = Node.location(4.626, 4.035);
            public static final Translation2d[] CORNERS = {
                CORNER_NEAR_LEFT,
                CORNER_FAR_LEFT,
                CORNER_FAR_RIGHT,
                CORNER_NEAR_RIGHT,
            };
        }

        // ── Left Trench (high Y — impassable) ──────────────────────────────
        public static final class TrenchLeft {
            // Occupies X [4.029, 5.223], Y [6.556, 8.21] in Blue-origin coordinates.
            // Includes the solid trench base (12 in wide) and the open corridor to the
            // field wall. Both sections are impassable — the robot is too tall.
            // INNER = hub-facing edge at field_center_Y + 96.5 in = 4.105 + 2.451 = 6.556 m.
            // WALL  = field boundary at Y = 8.21 m.
            public static final Translation2d CORNER_NEAR_WALL  = Node.location(4.029, 8.210);
            public static final Translation2d CORNER_FAR_WALL   = Node.location(5.223, 8.210);
            public static final Translation2d CORNER_FAR_INNER  = Node.location(5.223, 6.556);
            public static final Translation2d CORNER_NEAR_INNER = Node.location(4.029, 6.556);
            public static final Translation2d[] CORNERS = {
                CORNER_NEAR_WALL,
                CORNER_FAR_WALL,
                CORNER_FAR_INNER,
                CORNER_NEAR_INNER,
            };
        }

        // ── Right Trench (low Y — impassable) ───────────────────────────────
        public static final class TrenchRight {
            // Occupies X [4.029, 5.223], Y [0.0, 1.654] in Blue-origin coordinates.
            // INNER = hub-facing edge at field_center_Y − 96.5 in = 4.105 − 2.451 = 1.654 m.
            // WALL  = field boundary at Y = 0.0 m.
            public static final Translation2d CORNER_NEAR_INNER = Node.location(4.029, 1.654);
            public static final Translation2d CORNER_FAR_INNER  = Node.location(5.223, 1.654);
            public static final Translation2d CORNER_FAR_WALL   = Node.location(5.223, 0.000);
            public static final Translation2d CORNER_NEAR_WALL  = Node.location(4.029, 0.000);
            public static final Translation2d[] CORNERS = {
                CORNER_NEAR_INNER,
                CORNER_FAR_INNER,
                CORNER_FAR_WALL,
                CORNER_NEAR_WALL,
            };
        }

        // ── Left Bump (high Y subway lane — passable) ────────────────────────
        public static final class BumpLeft {
            // Elevated floor between the Hub's left wall and the Left Trench inner edge.
            // Occupies X [4.029, 5.223], Y [4.632, 6.556] in Blue-origin coordinates.
            // The robot CAN drive through here (this is the subway lane). Not an obstacle.
            // Used for field visualization. Not used in NodeBoundsTest collision checks.
            public static final Translation2d CORNER_NEAR_TRENCH = Node.location(4.029, 6.556);
            public static final Translation2d CORNER_FAR_TRENCH  = Node.location(5.223, 6.556);
            public static final Translation2d CORNER_FAR_HUB     = Node.location(5.223, 4.632);
            public static final Translation2d CORNER_NEAR_HUB    = Node.location(4.029, 4.632);
            public static final Translation2d[] CORNERS = {
                CORNER_NEAR_TRENCH,
                CORNER_FAR_TRENCH,
                CORNER_FAR_HUB,
                CORNER_NEAR_HUB,
            };
        }

        // ── Right Bump (low Y subway lane — passable) ───────────────────────
        public static final class BumpRight {
            // Elevated floor between the Right Trench inner edge and the Hub's right wall.
            // Occupies X [4.029, 5.223], Y [1.654, 3.438] in Blue-origin coordinates.
            public static final Translation2d CORNER_NEAR_HUB    = Node.location(4.029, 3.438);
            public static final Translation2d CORNER_FAR_HUB     = Node.location(5.223, 3.438);
            public static final Translation2d CORNER_FAR_TRENCH  = Node.location(5.223, 1.654);
            public static final Translation2d CORNER_NEAR_TRENCH = Node.location(4.029, 1.654);
            public static final Translation2d[] CORNERS = {
                CORNER_NEAR_HUB,
                CORNER_FAR_HUB,
                CORNER_FAR_TRENCH,
                CORNER_NEAR_TRENCH,
            };
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Legacy — navigation poses copied verbatim from Rebuilt2026 (old code).
    //
    // These have NOT been validated against the V2 field model or the V2
    // Robot.* node map. Use them only as a reference when porting old auto
    // routines. Once a node is confirmed correct, move it into Robot.* and
    // delete the Legacy entry.
    //
    // Note: Robot.Start.LEFT in V2 is (3.5, 5.7, 305°) — old code used
    // (3.5, 5.6, 315°). Confirm against the actual starting box before running.
    // ─────────────────────────────────────────────────────────────────────
    public static final class Legacy {

        public static final class Start {
            public static final Pose2d RIGHT               = Node.at(3.5,  2.5,  55.0);
            public static final Pose2d LEFT                = Node.at(3.5,  5.6, 315.0);
            public static final Pose2d SHOOTING_SPOT_RIGHT = Node.at(3.3,  2.8,  49.52);
            public static final Pose2d SHOOTING_SPOT_LEFT  = Node.at(3.4,  5.3, 320.3);
        }

        public static final class Midfield {
            // Bump crossing — safe headings to pass over the 6.5" alliance/neutral-zone ramps
            public static final Pose2d RIGHT_OVER_BUMP          = Node.at(6.30, 2.30,  55.0);
            public static final Pose2d RIGHT_OVER_BUMP2         = Node.at(6.30, 2.30, 170.0); // DoubleSubway cycle-2 return heading
            public static final Pose2d LEFT_OVER_BUMP           = Node.at(6.30, 5.60, 315.0);
            public static final Pose2d RIGHT_BEFORE_BUMP        = Node.at(5.85, 2.30, 315.0);
            public static final Pose2d LEFT_BEFORE_BUMP         = Node.at(7.50, 5.60, 315.0);
            public static final Pose2d RIGHT_TURN               = Node.at(3.50, 2.50, 140.0);

            // Right-side Neutral Zone ("subway") pickup nodes
            public static final Pose2d RIGHT_RIGHT_SUBWAY       = Node.at(7.40, 1.30,  90.0); // right entry
            public static final Pose2d LEFT_RIGHT_SUBWAY        = Node.at(7.55, 3.00,  90.0); // 6-inch exit
            public static final Pose2d RIGHT_RUSH_SUBWAY        = Node.at(6.75, 1.30,  90.0); // fast entry (18-inch)
            public static final Pose2d LOWER_RIGHT_RIGHT_SUBWAY = Node.at(7.00, 0.85,  90.0); // deep right (DoubleSubway cycle 2)
            public static final Pose2d RIGHT_LEFT_SUBWAY        = Node.at(7.95, 2.50, 270.0); // right-side exit of left sweep
            public static final Pose2d MIDDLE_RIGHT_SUBWAY      = Node.at(7.00, 4.00,  90.0); // center sweep (DoubleSubway cycle 2)

            // Left-side Neutral Zone ("subway") pickup nodes
            public static final Pose2d LEFT_LEFT_SUBWAY         = Node.at(7.40, 7.00, 270.0); // left entry
            public static final Pose2d LEFT_FOOTLONG_SUBWAY     = Node.at(7.95, 7.20, 270.0); // full-width endpoint

            // Cross-field waypoint used in AroundTheHub transit
            public static final Pose2d SUBWAY_AROUND_THE_HUB   = Node.at(2.00, 5.00, 270.0);
        }

        // Hub center — rotation target only. The robot faces this point to aim at the hub;
        // it never drives here. Updated to match FieldElements.Hub.CENTER measured coordinates.
        // NodeBoundsTest intentionally excludes this from collision checks for this reason.
        public static final class Hub {
            public static final Pose2d CENTER = Node.at(4.626, 4.035, 0.0);
        }

        // Human player station — right side
        public static final class Outpost {
            public static final Pose2d RIGHT_APPROACH_POINT       = Node.at(0.70, 1.10, 180.0);
            public static final Pose2d RIGHT_APPROACH_POINT_QUEST = Node.at(1.10, 1.10, 180.0); // QuestNav-tuned variant
        }
    }
}
