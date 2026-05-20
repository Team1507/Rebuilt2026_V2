//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants.RobotMap;
import static org.team1507.robot.Constants.kIntake.kArm.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeArm extends Subsystem1507{
    private final Motor1507 BLUmotor;
    private final Motor1507 YELmotor;
    private final BaseStatusSignal[] armSignals;

    private final double targetAngleDeg = 0.0;
    public IntakeArm() {
        super ("IntakeArm");
        BLUmotor = new Motor1507(key("BLU"), Motor1507.Type.FXS, RobotMap.INTAKE_ARM_BLUE);
        YELmotor = new Motor1507(key("YEL"), Motor1507.Type.FXS, RobotMap.INTAKE_ARM_YELLOW);

        BaseStatusSignal[] BLU = BLUmotor.getSignals();
        BaseStatusSignal[] YEL = YELmotor.getSignals();
        armSignals = new BaseStatusSignal[12];
        System.arraycopy(BLU, 0,armSignals,0,6);
        System.arraycopy(YEL, 0,armSignals,6,6);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(armSignals);

        log("AngleDegrees",getAverageAngle());
        log("TargetDegrees",getTargetAngle());
        log("Stalled",isStalled());
    }

    public void setAngle(double angleDeg) {
        double targetAngleDeg = clampAngle(angleDeg);
        double rotations = degToRotations(targetAngleDeg);
        
        BLUmotor.setPositionVoltage(rotations, 0.0);
        YELmotor.setPositionVoltage(rotations, 0.0);
    }

    public void deploy() {
        setAngle(DEPLOYED_ANGLE_DEGREES);
    }

    public void retract() {
        setAngle(RETRACTED_ANGLE_DEGREES);
    }

    public void runManual(double duty) {
        double positionDeg = getAverageAngle();

        if (duty > 0 && positionDeg >= MAX_ANGLE_DEGREES) {
            stop();
            return;
        }
        if (duty < 0 && positionDeg <= MIN_ANGLE_DEGREES) {
            stop();
            return;
        }

        if(duty > 0.5) {
            BLUmotor.runDuty(MANUAL_POSITIVE_POWER);
            YELmotor.runDuty(MANUAL_POSITIVE_POWER);
        }
        else if(duty < -0.5) {
            BLUmotor.runDuty(MANUAL_NEGATIVE_POWER);
            YELmotor.runDuty(MANUAL_NEGATIVE_POWER);
        }
        else {
            stop();
        }
    }

    public void stop() {
        BLUmotor.stop();
        YELmotor.stop();
    }

    public double getCurrentBLUAngle() {
        return rotationsToDeg(BLUmotor.getRotorPosition());
    }

    public double getCurrentYELAngle() {
        return rotationsToDeg(YELmotor.getRotorPosition());
    }

    public double getAverageAngle() {
        return (getCurrentBLUAngle() + getCurrentYELAngle()) / 2.0;
    }

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public boolean isStalled() {
        return BLUmotor.isStalled() || YELmotor.isStalled();
    }

    public boolean isAtTarget() {
        return Math.abs(getAverageAngle() - targetAngleDeg) < ANGLE_TOLERANCE_DEGREES;
    }

    //uttility methods
    private static double clampAngle(double angleDeg) {
        return MathUtil.clamp(angleDeg, MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    }

    private static double degToRotations(double angleDeg) {
        return angleDeg / 360.0;
    }

    private static double rotationsToDeg(double rotations) {
        return rotations * 360.0;
    }

    // command factory methods
    public Command deployCommand() {
        return new CommandBuilder(this)
        .named("arm.deploy")
        .onInitialize(this ::deploy)
        .isFinished(this::isAtTarget)
        .stallFinish(this::isStalled)
        .onEnd((interupted, timedOut, stalled) ->{
        if(timedOut || stalled)stop();
    });

    }

    public Command retractCommand() {
        return new CommandBuilder(this)
        .named("arm.retract")
        .onInitialize(this ::retract)
        .isFinished(this::isAtTarget)
        .stallFinish(this::isStalled)
        .onEnd((interupted, timedOut, stalled) ->{
        if(timedOut || stalled)stop();
    });
    
    }

    public Command goToAngleCommand (double angleDeg) {
        return new CommandBuilder(this)
        .named("arm.goToAngle")
        .onInitialize(() -> setAngle(angleDeg))
        .isFinished(this::isAtTarget)
        .stallFinish(this::isStalled)
        .onEnd((interupted, timedOut, stalled) ->{
        if(timedOut || stalled)stop();
    });
    }

    public Command manualJoystickCommand(DoubleSupplier inputSupplier) {
        return new CommandBuilder(this)
        .named("arm.manualJoystick")
        .onExecute(() -> runManual(inputSupplier.getAsDouble()))
        .stallFinish(this::isStalled)
        .onEnd((interupted, timedOut, stalled) -> stop())
        .runsUntilInterrupted();
    }

}
