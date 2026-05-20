//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;
import static org.team1507.robot.Constants.kHopper;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.RobotMap;

import com.ctre.phoenix6.BaseStatusSignal;

import java.util.function.DoubleSupplier;

public class Hopper extends Subsystem1507 {

  // TODO: Declare hardware (motors, sensors, etc.) as private fields
  private final Motor1507 hopperMotor;
  private final BaseStatusSignal[] hopperSignals;

  //change later
  private final double targetPosition = 0.0;
  
  /** Creates a new Hopper. */
  public Hopper() {

    super("Hopper");
    
    hopperMotor = new Motor1507(key("Hopper"), Motor1507.Type.FX, RobotMap.HOPPER, kHopper.CONFIG);
    hopperSignals = hopperMotor.getSignals();
  }

  // TODO: Add control methods
  // private void setOutput(double output) { ... }

  // ---- Command Factories ----

  public Command exampleCommand() {
    return runOnce(
        () -> {
          // TODO: Add command logic
        });
  }

  @Override
  public void periodic() {
    // TODO: Refresh hardware signals
    BaseStatusSignal.refreshAll(hopperSignals);

    // TODO: Log telemetry
    // log("someField", someValue);
  }
}