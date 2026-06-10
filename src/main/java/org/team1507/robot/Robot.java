//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.lib.core.vision.QuestNavSubsystem;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.auto.routines.*;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kQuest;
import org.team1507.robot.Constants.kShooter;
import org.team1507.robot.Constants.kSwerve;
import org.team1507.robot.subsystems.*;

public final class Robot extends LoggedRobot {

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    public final Swerve            swerve;
    public final QuestNavSubsystem questNav;
    public final IntakeArm         intakeArm;
    public final IntakeRoller      intakeRoller;
    public final Hopper            hopper;
    public final Agitator          agitator;
    public final Feeder            feeder;
    public final Shooter           shooter;

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    private final CommandXboxController topDriver;
    private final CommandXboxController bottomDriver;
    

    // -------------------------------------------------------------------------
    // Autonomous
    // -------------------------------------------------------------------------

    private Command m_autoCommand = null;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // =========================================================================
    // Constructor
    // =========================================================================

    public Robot() {

        // Subsystems — swerve first; questNav takes method references from it.
        swerve    = new Swerve();
        questNav  = new QuestNavSubsystem(
            swerve::addVisionMeasurement,
            swerve::resetPose,
            kQuest.ROBOT_TO_QUEST
        );
        intakeArm    = new IntakeArm();
        intakeRoller = new IntakeRoller();
        hopper       = new Hopper();
        agitator     = new Agitator();
        feeder       = new Feeder();
        shooter      = new Shooter();

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
        AutoBuilder.init(swerve, intakeArm, intakeRoller, hopper, agitator, feeder, shooter);
        autoChooser.setDefaultOption("Drive Forward", DriveForwardAuto.build());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Controllers and bindings
        topDriver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER);
        bottomDriver = new CommandXboxController(RobotMap.OPERATOR_CONTROLLER);
        configureBindings();
        configureDefaultBindings();
    }

    // =========================================================================
    // Bindings
    // =========================================================================

    private void configureBindings() {

        // ── Driver — swerve utilities ──────────────────────────────────────

        // Swerve brake — hold Start to lock wheels in X pattern, release to resume driving
        topDriver.start().whileTrue(swerve.brakeCommand());

        // Failsafe — cancels all running commands (see RobotBehaviors for details)
        topDriver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot toward the opposing alliance wall, then press A
        // to zero the gyro. Do this after any hot code deploy without a power cycle.
        topDriver.a().onTrue(swerve.zeroHeadingCommand());

        // ----------------------------
        // Intake
        // ----------------------------

        // Left trigger: deploy hopper → arm → roller
        bottomDriver.leftTrigger(0.5)
            .whileTrue(RobotBehaviors.deployAndIntake(hopper, intakeArm, intakeRoller))
            .onFalse(RobotBehaviors.stowIntake(intakeArm, intakeRoller));

        // B button: outtake (arm deploy + roller reverse + agitator out)
        bottomDriver.b()
            .whileTrue(intakeArm.deployCommand())
            .whileTrue(intakeRoller.runReverseCommand())
            .whileTrue(agitator.toOuttakeCommand())
            .onFalse(intakeArm.retractCommand())
            .onFalse(intakeRoller.stopCommand())
            .onFalse(agitator.stopCommand());

        // ----------------------------
        // Shooter
        // ----------------------------

        // Right bumper: safe shot (spin up + feed on velocity)
        bottomDriver.rightBumper()
            .whileTrue(shooter.spinUpCommand(kShooter.SAFE_RPM))
            .whileTrue(Commands.waitUntil(shooter::atVelocity)
                .andThen(Commands.parallel(feeder.feedCommand(), agitator.toShooterCommand())))
            .onFalse(Commands.parallel(shooter.stopCommand(), feeder.stopCommand(), agitator.stopCommand()));

        // Left bumper: lob shot
        bottomDriver.leftBumper()
            .whileTrue(shooter.spinUpCommand(kShooter.LOB_RPM))
            .whileTrue(Commands.waitUntil(shooter::atVelocity)
                .andThen(Commands.parallel(feeder.feedCommand(), agitator.toShooterCommand())))
            .onFalse(Commands.parallel(shooter.stopCommand(), feeder.stopCommand(), agitator.stopCommand()));

        // Right trigger (legacy / direct spin-up without feed)
        bottomDriver.rightTrigger(0.5)
            .whileTrue(shooter.spinUpCommand(kShooter.SAFE_RPM))
            .onFalse(shooter.stopCommand());

        // ----------------------------
        // Agitator manual overrides
        // ----------------------------

        // POV Up: nudge game piece toward shooter
        bottomDriver.povUp()
            .whileTrue(agitator.toShooterCommand())
            .onFalse(agitator.stopCommand());

        // POV Down: nudge game piece back toward intake
        bottomDriver.povDown()
            .whileTrue(agitator.toIntakeCommand())
            .onFalse(agitator.stopCommand());
    }

    private void configureDefaultBindings() {

        swerve.setDefaultCommand(
            swerve.driveCommand(() -> {
                double x   = MathUtil.applyDeadband(-topDriver.getLeftY(),  0.12);
                double y   = MathUtil.applyDeadband(-topDriver.getLeftX(),  0.12);
                double rot = MathUtil.applyDeadband(-topDriver.getRightX(), 0.12);

                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    x   * kSwerve.MAX_SPEED,
                    y   * kSwerve.MAX_SPEED,
                    rot * Math.PI,
                    swerve.getHeading()
                );
            })
        );
    }

    // =========================================================================
    // Mode callbacks
    // =========================================================================

    @Override
    public void autonomousInit() {
        m_autoCommand = autoChooser.getSelected();
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
