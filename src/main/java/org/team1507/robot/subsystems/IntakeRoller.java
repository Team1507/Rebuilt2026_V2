//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.RobotMap;
import com.ctre.phoenix6.BaseStatusSignal;
import static org.team1507.robot.Constants.kIntake.kRoller.*;

public class IntakeRoller extends Subsystem1507 {

   private final Motor1507 intakeRollerMotor;
   private final BaseStatusSignal[] motorSignals;

  /** Creates a new IntakeRoller. */
  public IntakeRoller() {
    super("IntakeRoller");


    intakeRollerMotor = new Motor1507(("IntakeRoller"), Motor1507.Type.FX, RobotMap.INTAKE_ROLLER, CONFIG);
    motorSignals = intakeRollerMotor.getSignals();
  }

  @Override
  public void simulationPeriodic() {
      intakeRollerMotor.simulationPeriodic(0.02);
  }

  @Override
  public void periodic() {
 
    BaseStatusSignal.refreshAll(motorSignals);

    log("VelocityRPS", intakeRollerMotor.getRotorVelocity());
    log("StatorAmps",  intakeRollerMotor.getStatorCurrent());
    log("Stalled",     isStalled());

  }

  public void run() {
    intakeRollerMotor.runTorqueCurrent(TORQUE_HIGH);
  }

  public void run(double torque) {
    intakeRollerMotor.runTorqueCurrent(torque);
  }

  public void stop() {
    intakeRollerMotor.stop();
  }

  public boolean isStalled() {
    return intakeRollerMotor.isStalled();
  }

   public Command runCommand() {
    return new CommandBuilder(this)
        .named("roller.run")
        .onExecute(() -> run())
        .stallFinish(this::isStalled)
        .onEnd((interupted, timedOut, stalled) ->{
        if(stalled)stop();
        });
  }

  public Command runCommand(double torque) {
    return new CommandBuilder(this)
        .named("roller.run")
        .onExecute(() -> run(torque))
        .stallFinish(this::isStalled)
        .onEnd((interupted, timedOut, stalled) ->{
        if(stalled)stop();
        });
  }

  public Command runReverseCommand() {
    return new CommandBuilder(this)
        .named("roller.runreverse")
        .onExecute(() -> run(OUTTAKE_TORQUE))
        .stallFinish(this::isStalled)
        .onEnd((interupted, timedOut, stalled) ->{
        if(stalled)stop();
        });
  }

   public Command stopCommand() {
    return new CommandBuilder(this)
        .named("roller.stop")
        .onInitialize(this::stop)
        .isFinished(true);
  }

}