//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import static org.team1507.robot.Constants.kFeeder.*;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kFeeder;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj2.command.Command;

public class Feeder extends Subsystem1507 {

    private final Motor1507 bluMotor;
    private final Motor1507 yelMotor;
    private final BaseStatusSignal[] feederSignals;

    private double targetRPM = 0.0;

    public Feeder() {
        super("Feeder");

        bluMotor = new Motor1507(key("BLU"), Motor1507.Type.FX, RobotMap.FEEDER_BLU, BLU_CONFIG);
        yelMotor = new Motor1507(key("YEL"), Motor1507.Type.FX, RobotMap.FEEDER_YEL, YEL_CONFIG);

        BaseStatusSignal[] blu = bluMotor.getSignals();
        BaseStatusSignal[] yel = yelMotor.getSignals();
        feederSignals = new BaseStatusSignal[blu.length + yel.length];
        System.arraycopy(blu, 0, feederSignals, 0, blu.length);
        System.arraycopy(yel, 0, feederSignals, blu.length, yel.length);
    }

    // =========================================================================
    // Control
    // =========================================================================

    public void feed() {
        targetRPM = kFeeder.FEED_RPM;
        bluMotor.setVelocityRPS(kFeeder.FEED_RPM / 60.0);
        yelMotor.setVelocityRPS(kFeeder.FEED_RPM / 60.0);
    }

    public void vomit() {
        targetRPM = kFeeder.VOMIT_RPM;
        bluMotor.setVelocityRPS(kFeeder.VOMIT_RPM / 60.0);
        yelMotor.setVelocityRPS(kFeeder.VOMIT_RPM / 60.0);
    }

    public void stop() {
        targetRPM = 0.0;
        bluMotor.stop();
        yelMotor.stop();
    }

    // =========================================================================
    // State
    // =========================================================================

    public double getBluRPM() {
        return bluMotor.getRotorVelocity() * 60.0;
    }

    public double getYelRPM() {
        return yelMotor.getRotorVelocity() * 60.0;
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
        BaseStatusSignal.refreshAll(feederSignals);

        log("BLU/RPM",        getBluRPM());
        log("YEL/RPM",        getYelRPM());
        log("TargetRPM",      targetRPM);
        log("BLU/StatorAmps", bluMotor.getStatorCurrent());
        log("YEL/StatorAmps", yelMotor.getStatorCurrent());
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /** Runs feeders at FEED_RPM to push game pieces toward the shooter. Stops on interrupt. */
    public Command feedCommand() {
        return new CommandBuilder(this)
            .named("feeder.feed")
            .onExecute(this::feed)
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Runs feeders in reverse at VOMIT_RPM to eject game pieces. Stops on interrupt. */
    public Command vomitCommand() {
        return new CommandBuilder(this)
            .named("feeder.vomit")
            .onExecute(this::vomit)
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Stops both feeder motors immediately. Finishes in one loop. */
    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("feeder.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
