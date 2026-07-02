package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DemoMode {

    public enum ShotPreset {
        CLOSE(30.0),
        MEDIUM(54.0),
        FAR(66.0);

        public final double rps;
        ShotPreset(double rps) { this.rps = rps; }
    }

    private static DemoMode instance;

    private CommandSwerveDrivetrain m_drivetrain;

    private double targetAngle = 0.0;  // radians, field-relative
    private ShotPreset m_preset = ShotPreset.CLOSE;

    private static final double VIRTUAL_TARGET_DISTANCE = 3.0;
    private static final double ANGLE_STEP_DEGREES = 1.5; // degrees per 20ms tick

    private DemoMode() {
        SmartDashboard.putBoolean("Overrides/Demo Mode Enabled", false);
        SmartDashboard.putString("DemoMode/ShotPreset", m_preset.name());
    }

    public static DemoMode getInstance() {
        if (instance == null) instance = new DemoMode();
        return instance;
    }

    public void init(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    /** Called every cycle by Turret.periodic() while demo mode is enabled. */
    public void update() {
        SmartDashboard.putString("DemoMode/ShotPreset", m_preset.name());

        Logger.recordOutput("DemoMode/VirtualTarget", getVirtualTarget());
        Logger.recordOutput("DemoMode/Distance", getDistance());
        Logger.recordOutput("DemoMode/TargetAngle", Math.toDegrees(targetAngle));
        Logger.recordOutput("DemoMode/ShooterRPS", m_preset.rps);
        Logger.recordOutput("DemoMode/Enabled", isEnabled());
    }

    /** Rotate the virtual target one step. Pass +1.0 for clockwise, -1.0 for counter-clockwise. */
    public void adjustAngle(double directionSign) {
        targetAngle += Math.toRadians(ANGLE_STEP_DEGREES * directionSign);
    }

    public Pose2d getVirtualTarget() {
        if (m_drivetrain == null) return new Pose2d();
        Pose2d robotPose = m_drivetrain.getState().Pose;
        double targetX = robotPose.getX() + VIRTUAL_TARGET_DISTANCE * Math.cos(targetAngle);
        double targetY = robotPose.getY() + VIRTUAL_TARGET_DISTANCE * Math.sin(targetAngle);
        return new Pose2d(targetX, targetY, Rotation2d.fromRadians(targetAngle));
    }

    public double getDistance() {
        return VIRTUAL_TARGET_DISTANCE;
    }

    public void setPreset(ShotPreset preset) {
        m_preset = preset;
    }

    public double getShooterRPS() {
        return m_preset.rps;
    }

    public boolean isEnabled() {
        return SmartDashboard.getBoolean("Overrides/Demo Mode Enabled", false);
    }

    public void resetTarget() {
        targetAngle = 0.0;
    }
}
