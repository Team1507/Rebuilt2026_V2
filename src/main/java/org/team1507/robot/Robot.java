//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

import static org.team1507.robot.Constants.*;
import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.lib.core.ml.ShooterModel;
import org.team1507.lib.core.vision.QuestNavSubsystem;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.auto.routines.*;
import org.team1507.robot.subsystems.*;

public final class Robot extends LoggedRobot {

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    public final Agitator          agitator;
    public final Feeder            feeder;
    public final Hopper            hopper;
    public final IntakeArm         intakeArm;
    public final IntakeRoller      intakeRoller;
    public final QuestNavSubsystem questNav;
    public final Shooter           shooter;
    public final Swerve            swerve;

    // -------------------------------------------------------------------------
    // ML Model
    // -------------------------------------------------------------------------

    private final ShooterModel shooterModel;

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    private final CommandXboxController topDriver;
    private final CommandXboxController bottomDriver;
    

    // -------------------------------------------------------------------------
    // Autonomous
    // -------------------------------------------------------------------------

    private Command m_autoCommand = null;
    private final SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();

    // =========================================================================
    // Constructor
    // =========================================================================

    public Robot() {

        // Load shooter model before anything else — no subsystem dependencies.
        shooterModel = ShooterModel.load();

        // Swerve first — questNav takes method references from it.
        swerve   = new Swerve();
        questNav = new QuestNavSubsystem(
            swerve::addVisionMeasurement,
            swerve::resetPose,
            kQuest.ROBOT_TO_QUEST
        );
        agitator     = new Agitator();
        feeder       = new Feeder();
        hopper       = new Hopper();
        intakeArm    = new IntakeArm();
        intakeRoller = new IntakeRoller();
        shooter      = new Shooter();

        // Safe Check for IntakeArm
        intakeArm.setHopperSafeSupplier(hopper::isHopperSafeForIntake);

        // Pre-match pose preset buttons (visible in Elastic while disabled).
        // Place the robot at the known starting position and press the matching
        // button. The command waits for Quest to confirm before snapping odometry.
        questNav.setKnownPoseCommand(Nodes.Robot.Start.LEFT)
            .named("Set Pose Left")
            .publishToDashboard();
        questNav.setKnownPoseCommand(Nodes.Robot.Start.CENTER)
            .named("Set Pose Start")
            .publishToDashboard();
        questNav.setKnownPoseCommand(Nodes.Robot.Start.RIGHT)
            .named("Set Pose Right")
            .publishToDashboard();

        // Autonomous chooser
        AutoBuilder.init(swerve, intakeArm, intakeRoller, hopper, agitator, feeder, shooter, shooterModel);
        autoChooser.setDefaultOption("Drive Forward",          DriveForwardAuto::build);
        autoChooser.addOption("Raymond (shoot only)",          AutoahRaymond::build);
        autoChooser.addOption("Human Player Quest",            AutoHumanPlayerQuest::build);
        autoChooser.addOption("Subway 6-inch Right",           AutoSubway6inchRight::build);
        autoChooser.addOption("Subway 6-inch Left",            AutoSubway6inchLeft::build);
        autoChooser.addOption("Subway Footlong Right",         AutoSubwayFootlongRight::build);
        autoChooser.addOption("Subway 18-inch Right",          AutoSubway18Inch::build);
        autoChooser.addOption("Subway 18-inch Left",           AutoSubway18InchLeftBlue::build);
        autoChooser.addOption("Double Subway",                 AutoDoubleSubway::build);
        autoChooser.addOption("Subway Around The Hub",         AutoSubwayAroundTheHub::build);
        autoChooser.addOption("Subway Left",  () -> AutoSubway.build(AutoSubway.Side.LEFT));
        autoChooser.addOption("Subway Right", () -> AutoSubway.build(AutoSubway.Side.RIGHT));
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Controllers and bindings — bottomDriver = port 0 (driver), topDriver = port 1 (operator)
        bottomDriver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER);
        topDriver    = new CommandXboxController(RobotMap.OPERATOR_CONTROLLER);
        configureBindings();
        configureDefaultBindings();
    }

    // =========================================================================
    // Bindings
    // =========================================================================

    private void configureBindings() {

        // ── bottomDriver (port 0) — driver: swerve + all robot actions ─────

        // Swerve brake — hold Start to lock wheels in X pattern
        bottomDriver.start().whileTrue(swerve.brakeCommand());

        // Failsafe — cancels all running commands (see RobotBehaviors for details)
        bottomDriver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot toward the opposing alliance wall, then press A
        // to zero the gyro. Do this after any hot code deploy without a power cycle.
        bottomDriver.a().onTrue(swerve.zeroHeadingCommand());

        // ----------------------------
        // Intake
        // ----------------------------

        // Left trigger: deploy hopper → arm → roller
        bottomDriver.leftTrigger(0.5)
            .whileTrue(RobotBehaviors.deployAndIntake(hopper, intakeArm, intakeRoller))
            .onFalse(RobotBehaviors.stowIntake(intakeArm, intakeRoller));

        // B button: outtake (arm deploy + roller reverse + agitator out)
        bottomDriver.b()
            .whileTrue(RobotBehaviors.outtake(intakeArm, intakeRoller, agitator))
            .onFalse(Commands.parallel(RobotBehaviors.stowIntake(intakeArm, intakeRoller), agitator.stopCommand()));

        // ----------------------------
        // Shooter
        // ----------------------------

        // Right stick: auto-rotate to face hub while driver translates freely (matches V1)
        bottomDriver.rightStick()
            .whileTrue(swerve.maintainHeadingToTarget(
                () -> new Pose2d(Nodes.FieldElements.Hub.CENTER, new Rotation2d()),
                () -> MathUtil.applyDeadband(-bottomDriver.getLeftY(),  0.12) * kSwerve.MAX_SPEED,
                () -> MathUtil.applyDeadband(-bottomDriver.getLeftX(),  0.12) * kSwerve.MAX_SPEED
            ));

        // Right bumper: lob shot (matches old code rightBumper = LOB_RPM)
        bottomDriver.rightBumper()
            .whileTrue(RobotBehaviors.shootFixedRPM(shooter, feeder, agitator, kShooter.LOB_RPM));

        // Right trigger: ML auto-aim shot (matches V1 right trigger behavior)
        bottomDriver.rightTrigger(0.5)
            .whileTrue(RobotBehaviors.shootAutoAim(shooter, feeder, agitator, shooterModel, swerve::getPose));

        // ── topDriver (port 1) — operator: mechanisms ──────────────────────

        // ----------------------------
        // Agitator manual overrides
        // ----------------------------

        // POV Up: nudge game piece toward shooter
        topDriver.povUp()
            .whileTrue(agitator.toShooterCommand())
            .onFalse(agitator.stopCommand());

        // POV Down: nudge game piece back toward intake
        topDriver.povDown()
            .whileTrue(agitator.toIntakeCommand())
            .onFalse(agitator.stopCommand());
    }

    private void configureDefaultBindings() {

        // bottomDriver (port 0) drives the robot field-relative
        swerve.setDefaultCommand(
            swerve.driveCommand(() -> {
                double x   = MathUtil.applyDeadband(-bottomDriver.getLeftY(),  0.12);
                double y   = MathUtil.applyDeadband(-bottomDriver.getLeftX(),  0.12);
                double rot = MathUtil.applyDeadband(-bottomDriver.getRightX(), 0.12);

                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    x   * kSwerve.MAX_SPEED,
                    y   * kSwerve.MAX_SPEED,
                    rot * Math.PI,
                    swerve.getHeading()
                );
            })
        );

        // topDriver (port 1) manually adjusts arm and hopper with joystick axes
        intakeArm.setDefaultCommand(intakeArm.manualJoystickCommand(() -> topDriver.getLeftY()));
        hopper.setDefaultCommand(hopper.manualPowerCommand(() -> topDriver.getRightY()));
    }

    // =========================================================================
    // Mode callbacks
    // =========================================================================

    @Override
    public void autonomousInit() {
        Supplier<Command> factory = autoChooser.getSelected();
        m_autoCommand = (factory != null) ? factory.get() : null;
        if (m_autoCommand != null) {
            CommandScheduler.getInstance().schedule(m_autoCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null) {
            m_autoCommand.cancel();
        }
    }
}
