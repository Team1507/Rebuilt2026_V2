//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import static org.team1507.robot.Constants.kHopper;

import java.util.function.DoubleSupplier;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.RobotMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Hopper extends Subsystem1507 {

    private final Motor1507 hopperMotor;
    private final BaseStatusSignal[] hopperSignals;
    private final DigitalInput halSensor;

    private double targetPosition = 0.0;

    public Hopper() {
        super("Hopper");

        hopperMotor = new Motor1507(key("Motor"), Motor1507.Type.FX, RobotMap.HOPPER, kHopper.CONFIG);
        hopperSignals = hopperMotor.getSignals();
        halSensor = new DigitalInput(RobotMap.HOPPER_DIO);
    }

    // =========================================================================
    // Control
    // =========================================================================

    public void setPosition(double degrees) {
        double safePos = Math.min(degrees, kHopper.MAX_POS);
        if (isAtReverseLimit() && safePos < 0) safePos = 0.0;
        targetPosition = safePos;
        hopperMotor.setPositionVoltage(safePos * kHopper.DEGREES_TO_MOTOR_ROTATIONS);
    }

    public void runPower(double power) {
        if (isAtReverseLimit() && power < 0) power = 0.0;
        if (power > 0) {
            hopperMotor.runDuty(kHopper.MANUAL_POSITIVE_POWER);
        } else {
            hopperMotor.stop();
        }
    }

    public void stop() {
        hopperMotor.stop();
    }

    // =========================================================================
    // State
    // =========================================================================

    public double getPositionDegrees() {
        return hopperMotor.getRotorPosition() / kHopper.DEGREES_TO_MOTOR_ROTATIONS;
    }

    /** True when the hopper's reverse Hall-effect sensor is triggered (fully retracted). */
    public boolean isAtReverseLimit() {
        if (RobotBase.isSimulation()) return false;
        return !halSensor.get();
    }

    /** True when the hopper has extended past the safe threshold for intake deployment. */
    public boolean isHopperFullyExtended() {
        return getPositionDegrees() >= kHopper.SAFE_EXTENDED;
    }

    /** Alias for {@link #isHopperFullyExtended()} — used by RobotBehaviors to gate intake deployment. */
    public boolean isHopperSafeForIntake() {
        return isHopperFullyExtended();
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void simulationPeriodic() {
        hopperMotor.simulationPeriodic(0.02);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(hopperSignals);

        // Auto-zero the encoder whenever the reverse limit switch is active.
        if (isAtReverseLimit()) {
            ((TalonFX) hopperMotor.getDevice()).setPosition(0.0);
        }

        log("PositionDegrees", getPositionDegrees());
        log("TargetDegrees",   targetPosition);
        log("ReverseLimit",    isAtReverseLimit());
        log("FullyExtended",   isHopperFullyExtended());
        log("StatorAmps",      hopperMotor.getStatorCurrent());
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /** Extends the hopper to EXTENDED_POS and finishes when fully extended. */
    public Command extendCommand() {
        return new CommandBuilder(this)
            .named("hopper.extend")
            .onExecute(() -> setPosition(kHopper.EXTENDED_POS))
            .isFinished(this::isHopperFullyExtended)
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Retracts the hopper to RETRACTED_POS. Runs until interrupted. */
    public Command retractCommand() {
        return new CommandBuilder(this)
            .named("hopper.retract")
            .onExecute(() -> setPosition(kHopper.RETRACTED_POS))
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Moves the hopper to an arbitrary angle in degrees. Runs until interrupted. */
    public Command moveToCommand(double degrees) {
        return new CommandBuilder(this)
            .named("hopper.moveTo(" + degrees + ")")
            .onExecute(() -> setPosition(degrees))
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Manual joystick control — power supplier should be in [-1, 1]. */
    public Command manualPowerCommand(DoubleSupplier power) {
        return new CommandBuilder(this)
            .named("hopper.manualPower")
            .onExecute(() -> runPower(power.getAsDouble()))
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /** Stops hopper motion immediately. Finishes in one loop. */
    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("hopper.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
