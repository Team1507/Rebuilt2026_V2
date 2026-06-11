//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team1507.lib.core.logging.Telemetry;

/**
 * Base robot class for Team 1507.
 *
 * <p>Extends WPILib's {@link TimedRobot} with two additions:
 * <ul>
 *   <li><b>DataLogManager</b> — starts WPILib's binary {@code .wpilog} logger on boot so
 *       match data is always captured without any extra setup in Robot.java.</li>
 *   <li><b>Telemetry.update()</b> — flushes all {@link org.team1507.lib.core.logging.InputField}
 *       snapshots and NetworkTables writes each loop after the CommandScheduler runs.</li>
 * </ul>
 *
 * <p>Robot.java extends this instead of {@code TimedRobot} directly. All other robot lifecycle
 * methods ({@code autonomousInit}, {@code teleopInit}, etc.) are inherited from TimedRobot and
 * overridden in Robot.java as normal.
 */
public abstract class LoggedRobot extends TimedRobot {

    /**
     * Initializes WPILib data logging and wires DriverStation events into the log.
     * Called once at robot startup before any mode callbacks.
     */
    protected LoggedRobot() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    /**
     * Runs every robot loop (20 ms by default) regardless of mode.
     * Advances the WPILib CommandScheduler and flushes Telemetry outputs.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Telemetry.update();
    }
}