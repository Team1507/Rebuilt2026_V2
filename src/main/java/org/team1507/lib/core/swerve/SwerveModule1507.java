//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;

import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.Telemetry;

/**
 * One physical swerve module — a paired drive motor, steer motor, and absolute CANcoder.
 *
 * <p>Wraps all per-module math behind three public APIs:
 * <ul>
 *   <li>{@link #setDesiredState} — apply a {@link SwerveModuleState} each loop</li>
 *   <li>{@link #getState} / {@link #getPosition} — read measured speed and distance</li>
 *   <li>{@link #getAllSignals} — return all 14 CAN signals for batched refresh in
 *       {@code Swerve.periodic()}</li>
 * </ul>
 *
 * <p>Drive velocity and position are corrected for mechanical coupling: in SDS-style modules,
 * rotating the steer gear physically turns the drive gear by a small amount
 * ({@link MathConfig#couplingRatio}). Without correction this introduces odometry error
 * whenever the steer angle changes rapidly (e.g., snap-turns in path following).
 *
 * <p>Simulation is supported: {@link #simulationUpdate} ticks the CANcoder sim state and
 * integrates drive position. All observation methods return sim values when
 * {@link edu.wpi.first.wpilibj.RobotBase#isSimulation()} is true.
 */
public final class SwerveModule1507 {

    /**
     * Immutable gear-ratio and wheel-size parameters for a single swerve module.
     *
     * @param driveGearRatio    motor rotations per wheel rotation (drive stage)
     * @param steerGearRatio    motor rotations per full module azimuth rotation (steer stage)
     * @param couplingRatio     drive motor rotations induced per steer motor rotation due to
     *                          mechanical coupling in the module (typically ~3.57 for SDS L2/L3)
     * @param wheelRadiusMeters wheel radius in meters; used to convert rotations to distance
     */
    public record MathConfig(
        double driveGearRatio,
        double steerGearRatio,
        double couplingRatio,
        double wheelRadiusMeters
    ) {
        public double wheelCircumferenceMeters() {
            return 2.0 * Math.PI * wheelRadiusMeters;
        }
    }

    private final String name;

    private final Motor1507 drive;
    private final Motor1507 steer;
    private final CANcoder encoder;

    private final Rotation2d encoderOffset;
    private final MathConfig math;
    private final double driveMetersScale;

    private Rotation2d lastAngle = Rotation2d.kZero;

    // Sim state
    private double simSteerAngleRotations = 0.0;
    private double simDriveVelocityRps = 0.0;
    private double simDrivePositionRotations = 0.0;

    private final StatusSignal<Angle> absPosition;
    private final StatusSignal<AngularVelocity> azimuthVelocity;

    private final BaseStatusSignal[] allSignals;

    /**
     * Constructs a {@code SwerveModule1507} and wires all CAN devices.
     *
     * @param name            telemetry prefix for this module (e.g., {@code "FL"}, {@code "BR"})
     * @param drive           drive motor configured for velocity control
     * @param steer           steer motor configured for position control
     * @param encoder         absolute CANcoder measuring the steer azimuth angle
     * @param encoderOffset   offset subtracted from the raw CANcoder reading to zero the
     *                        wheel in the robot-forward direction
     * @param math            gear ratio and wheel size parameters — see {@link MathConfig}
     * @param driveMetersScale scalar applied to computed drive distance and velocity;
     *                        use {@code 1.0} unless per-module wheel-wear compensation is needed
     */
    public SwerveModule1507(
        String name,
        Motor1507 drive,
        Motor1507 steer,
        CANcoder encoder,
        Rotation2d encoderOffset,
        MathConfig math,
        double driveMetersScale
    ) {
        this.name = name;
        this.drive = drive;
        this.steer = steer;
        this.encoder = encoder;
        this.encoderOffset = encoderOffset;
        this.math = math;
        this.driveMetersScale = driveMetersScale;

        this.absPosition = encoder.getAbsolutePosition();
        this.azimuthVelocity = encoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            absPosition,
            azimuthVelocity
        );

        BaseStatusSignal[] driveSignals = drive.getSignals(); // 6
        BaseStatusSignal[] steerSignals = steer.getSignals(); // 6
        this.allSignals = new BaseStatusSignal[14];
        System.arraycopy(driveSignals, 0, allSignals, 0, 6);
        System.arraycopy(steerSignals, 0, allSignals, 6, 6);
        allSignals[12] = absPosition;
        allSignals[13] = azimuthVelocity;

        Telemetry.set(key("Drive/MetersScale"), driveMetersScale);
        Telemetry.set(key("Initialized"), true);
    }

    // ============================================================
    // Control
    // ============================================================

    /**
     * Applies a desired swerve module state (speed and angle), optimizing direction
     * to minimize wheel rotation and applying an anti-jitter guard at near-zero speed.
     *
     * @param desired the target module state
     */
    public void setDesiredState(SwerveModuleState desired) {
        Rotation2d current = getAngle();
        SwerveModuleState optimized = new SwerveModuleState(
            desired.speedMetersPerSecond,
            desired.angle
        );

        optimized.optimize(current);

        Rotation2d targetAngle =
            Math.abs(optimized.speedMetersPerSecond) < 0.01
                ? lastAngle
                : optimized.angle;

        double driveRps =
            metersPerSecondToDriveMotorRps(optimized.speedMetersPerSecond);

        drive.setVelocityRPS(driveRps);

        steer.setPositionVoltage(
            targetAngle.getRotations(),
            0.0
        );

        // In sim, instantly snap to the target so getAngle() returns
        // the correct value next loop — simulates a perfect steer controller
        if (RobotBase.isSimulation()) {
            simSteerAngleRotations = targetAngle.getRotations();
            simDriveVelocityRps = driveRps / math.driveGearRatio();
        }

        lastAngle = targetAngle;

        Telemetry.set(key("Drive/TargetMps"), optimized.speedMetersPerSecond);
        Telemetry.set(key("Drive/TargetMotorRps"), driveRps);
        Telemetry.set(key("Steer/TargetAngleRad"), targetAngle.getRadians());
    }

    /** Stops both the drive and steer motors immediately. */
    public void stop() {
        drive.stop();
        steer.stop();

        if (RobotBase.isSimulation()) {
            simDriveVelocityRps = 0.0;
        }
    }

    /**
     * Commands the steer directly to a target angle and stops the drive motor.
     *
     * <p>Unlike {@link #setDesiredState}, this method bypasses the anti-jitter
     * guard that skips steer updates when speed is near zero. Use this for
     * X-brake mode where the target angle must be applied regardless
     * of drive speed.
     *
     * @param angle the target steer angle
     */
    public void brakeToAngle(Rotation2d angle) {
        drive.stop();
        steer.setPositionVoltage(angle.getRotations(), 0.0);

        if (RobotBase.isSimulation()) {
            simSteerAngleRotations = angle.getRotations();
            simDriveVelocityRps    = 0.0;
        }

        lastAngle = angle;
    }

    // ============================================================
    // Simulation Update
    // ============================================================

    /**
     * Called every loop from Swerve.simulationPeriodic().
     * Updates the CANcoder sim state so getAngle() returns real values,
     * and integrates drive position.
     */
    public void simulationUpdate(double dtSeconds) {
        // Update CANcoder sim state so absPosition signal reads correctly
        CANcoderSimState encoderSim = encoder.getSimState();
        encoderSim.setRawPosition(
            simSteerAngleRotations + encoderOffset.getRotations()
        );
        encoderSim.setVelocity(0.0);

        // Integrate drive position
        simDrivePositionRotations += simDriveVelocityRps * dtSeconds;
    }

    // ============================================================
    // Observation
    // ============================================================

    /** Returns the current steer angle, corrected for the encoder offset. */
    public Rotation2d getAngle() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromRotations(simSteerAngleRotations);
        }
        double rotations = absPosition.getValue().in(Rotations);
        return Rotation2d.fromRotations(rotations).minus(encoderOffset);
    }

    /** Raw CANcoder position in rotations (encoder offset not applied), or sim equivalent. */
    private double azimuthRotationsRaw() {
        if (RobotBase.isSimulation()) {
            return simSteerAngleRotations + encoderOffset.getRotations();
        }
        return absPosition.getValue().in(Rotations);
    }

    /** Raw CANcoder velocity in rotations per second, or {@code 0.0} in sim. */
    private double azimuthRpsRaw() {
        return RobotBase.isSimulation() ? 0.0
            : azimuthVelocity.getValue().in(RotationsPerSecond);
    }

    /** Returns the current module state (speed in m/s and steer angle). */
    public SwerveModuleState getState() {
        double mps = RobotBase.isSimulation()
            ? simDriveVelocityRps * math.wheelCircumferenceMeters()
            : wheelRpsToMetersPerSecond(correctedWheelRps()) * driveMetersScale;

        return new SwerveModuleState(mps, getAngle());
    }

    /** Returns the current module position (distance traveled in meters and steer angle). */
    public SwerveModulePosition getPosition() {
        double meters = RobotBase.isSimulation()
            ? simDrivePositionRotations * math.wheelCircumferenceMeters()
            : wheelRotationsToMeters(correctedWheelRotations()) * driveMetersScale;

        return new SwerveModulePosition(meters, getAngle());
    }

    /** Returns all CAN status signals for this module (drive, steer, and encoder), for batched refresh. */
    public BaseStatusSignal[] getAllSignals() {
        return allSignals;
    }

    /** @deprecated Replaced by {@link #getAllSignals()} — Swerve now batches all signals in one call. */
    @Deprecated
    public void refreshSignals() {
        // no-op — Swerve.periodic() calls BaseStatusSignal.refreshAll(allSignals) for all modules
    }

    // ============================================================
    // CAN Bus Access (for bus optimization only)
    // ============================================================

    /** Returns the drive motor's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getDriveDevice() { return drive.getDevice(); }

    /** Returns the steer motor's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getSteerDevice() { return steer.getDevice(); }

    /** Returns the CANcoder's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getEncoderDevice() { return encoder; }

    // ============================================================
    // Faults
    // ============================================================

    /** Returns true if the drive motor is currently reporting a stall condition. */
    public boolean isDriveStalled() {
        return drive.isStalled();
    }

    /** Returns true if the steer motor is currently reporting a stall condition. */
    public boolean isSteerStalled() {
        return steer.isStalled();
    }

    // ============================================================
    // Core math
    // ============================================================

    /**
     * Raw drive rotor position minus the coupling contribution from the current steer angle.
     * Removes odometry error that accumulates when the steer gear turns the drive gear.
     */
    private double correctedDriveMotorRotations() {
        return drive.getRotorPosition()
            - (azimuthRotationsRaw() * math.couplingRatio());
    }

    /**
     * Raw drive rotor velocity minus the coupling contribution from the current steer velocity.
     * Removes speed measurement error during rapid steer changes.
     */
    private double correctedDriveMotorRps() {
        return drive.getRotorVelocity()
            - (azimuthRpsRaw() * math.couplingRatio());
    }

    /** Coupling-corrected drive motor rotations divided by the drive gear ratio → wheel rotations. */
    private double correctedWheelRotations() {
        return correctedDriveMotorRotations() / math.driveGearRatio();
    }

    /** Coupling-corrected drive motor RPS divided by the drive gear ratio → wheel RPS. */
    private double correctedWheelRps() {
        return correctedDriveMotorRps() / math.driveGearRatio();
    }

    /** Converts wheel rotations to meters via wheel circumference. */
    private double wheelRotationsToMeters(double wheelRotations) {
        return wheelRotations * math.wheelCircumferenceMeters();
    }

    /** Converts wheel RPS to meters per second via wheel circumference. */
    private double wheelRpsToMetersPerSecond(double wheelRps) {
        return wheelRps * math.wheelCircumferenceMeters();
    }

    /**
     * Converts a target linear speed in m/s to the required drive motor RPS,
     * adding the coupling feedforward so the drive motor compensates for steer-induced rotation.
     */
    private double metersPerSecondToDriveMotorRps(double mps) {
        double wheelRps = mps / math.wheelCircumferenceMeters();
        return (wheelRps * math.driveGearRatio())
            + (azimuthRpsRaw() * math.couplingRatio());
    }

    private String key(String field) {
        return "Swerve/" + name + "/" + field;
    }
}
