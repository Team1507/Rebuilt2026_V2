//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import static org.team1507.robot.Constants.kAgitator.*;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kAgitator;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj2.command.Command;

public class Agitator extends Subsystem1507 {

    private final Motor1507 bluMotor;
    private final Motor1507 yelMotor;
    private final BaseStatusSignal[] agitatorSignals;

    private double targetAmps = 0.0;

    public Agitator() {
        super("Agitator");

        bluMotor = new Motor1507(key("BLU"), Motor1507.Type.FX, RobotMap.AGITATOR_BLU, BLU_CONFIG);
        yelMotor = new Motor1507(key("YEL"), Motor1507.Type.FX, RobotMap.AGITATOR_YEL, YEL_CONFIG);

        BaseStatusSignal[] blu = bluMotor.getSignals();
        BaseStatusSignal[] yel = yelMotor.getSignals();
        agitatorSignals = new BaseStatusSignal[blu.length + yel.length];
        System.arraycopy(blu, 0, agitatorSignals, 0, blu.length);
        System.arraycopy(yel, 0, agitatorSignals, blu.length, yel.length);
    }

    // =========================================================================
    // Control
    // =========================================================================


    public void toShooter() {
        targetAmps = kAgitator.TORQUE_LOW;
        bluMotor.runTorqueCurrent(targetAmps);
        yelMotor.runTorqueCurrent(targetAmps);
    }

    public void toIntake() {
        targetAmps = -kAgitator.TORQUE_LOW;
        bluMotor.runTorqueCurrent(targetAmps);
        yelMotor.runTorqueCurrent(targetAmps);
    }

    public void toOuttake() {
        targetAmps = kAgitator.OUTTAKE_TORQUE;
        bluMotor.runTorqueCurrent(targetAmps);
        yelMotor.runTorqueCurrent(targetAmps);
    }

    public void stop() {
        targetAmps = 0.0;
        bluMotor.stop();
        yelMotor.stop();
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void simulationPeriodic() {
        bluMotor.simulationPeriodic(0.02);
        yelMotor.simulationPeriodic(0.02);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(agitatorSignals);

        log("VelocityRPS",      (bluMotor.getRotorVelocity() + yelMotor.getRotorVelocity()) / 2.0);
        log("TargetAmps",       targetAmps);
        log("TotalCurrentAmps", bluMotor.getStatorCurrent() + yelMotor.getStatorCurrent());
        log("BLU/VelocityRPS",  bluMotor.getRotorVelocity());
        log("YEL/VelocityRPS",  yelMotor.getRotorVelocity());
        log("BLU/StatorAmps",   bluMotor.getStatorCurrent());
        log("YEL/StatorAmps",   yelMotor.getStatorCurrent());
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /** Runs agitators toward the shooter. Stops on interrupt. */
    public Command toShooterCommand() {
        return new CommandBuilder(this)
            .named("agitator.toShooter")
            .onExecute(this::toShooter)
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Runs agitators toward the intake. Stops on interrupt. */
    public Command toIntakeCommand() {
        return new CommandBuilder(this)
            .named("agitator.toIntake")
            .onExecute(this::toIntake)
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Runs agitators in outtake direction. Stops on interrupt. */
    public Command toOuttakeCommand() {
        return new CommandBuilder(this)
            .named("agitator.toOuttake")
            .onExecute(this::toOuttake)
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Stops both agitators immediately. Finishes in one loop. */
    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("agitator.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
