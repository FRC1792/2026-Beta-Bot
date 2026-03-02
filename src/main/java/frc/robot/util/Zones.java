package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.Vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ZoneConstants;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Zones {
    public static interface Zone {
        public Trigger contains(Supplier<Pose2d> pose);
    }

    public static interface PredictiveXZone extends Zone {
        public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt);
    }

    public static class BaseZone implements Zone {
        protected final double xMin, xMax, yMin, yMax;

        public BaseZone(double xMin, double xMax, double yMin, double yMax) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        public BaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
            this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
        }

        @Override
        public Trigger contains(Supplier<Pose2d> poseSupplier) {
            return new Trigger(() -> this.containsPoint(poseSupplier.get().getTranslation()));
        }

        protected boolean containsPoint(Translation2d point) {
            return point.getX() >= xMin && point.getX() <= xMax && point.getY() >= yMin && point.getY() <= yMax;
        }

        public BaseZone mirroredX() {
            return new BaseZone(
                    aprilTagLayout.getFieldLength() - xMax,
                    aprilTagLayout.getFieldLength() - xMin,
                    yMin,
                    yMax);
        }

        public BaseZone mirroredY() {
            return new BaseZone(
                    xMin,
                    xMax,
                    aprilTagLayout.getFieldWidth() - yMax,
                    aprilTagLayout.getFieldWidth() - yMin);
        }

        /** list of corners, with the bottom left corner repeated at the end to form a closed loop */
        public Translation2d[] getCorners() {
            return new Translation2d[] {
                new Translation2d(xMin, yMin),
                new Translation2d(xMax, yMin),
                new Translation2d(xMax, yMax),
                new Translation2d(xMin, yMax),
                new Translation2d(xMin, yMin)
            };
        }
    }

    public static class PredictiveXBaseZone extends BaseZone implements PredictiveXZone {
        public PredictiveXBaseZone(double xMin, double xMax, double yMin, double yMax) {
            super(xMin, xMax, yMin, yMax);
        }

        public PredictiveXBaseZone(BaseZone baseZone) {
            super(baseZone.xMin, baseZone.xMax, baseZone.yMin, baseZone.yMax);
        }

        public PredictiveXBaseZone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
            super(xMin, xMax, yMin, yMax);
        }

        @Override
        public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
            return new Trigger(() -> willContainPoint(pose.get().getTranslation(), fieldSpeeds.get(), dt));
        }

        protected boolean willContainPoint(Translation2d point, ChassisSpeeds fieldSpeeds, Time dt) {
            return (point.getY() >= yMin && point.getY() <= yMax)
                    && ((point.getX() >= xMin && point.getX() <= xMax)
                            || (point.getX() < xMin
                                    && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) >= xMin - point.getX())
                            || (point.getX() > xMax
                                    && fieldSpeeds.vxMetersPerSecond * dt.in(Seconds) <= xMax - point.getX()));
        }

        @Override
        public PredictiveXBaseZone mirroredX() {
            return new PredictiveXBaseZone(super.mirroredX());
        }

        @Override
        public PredictiveXBaseZone mirroredY() {
            return new PredictiveXBaseZone(super.mirroredY());
        }
    }

    public static class ZoneCollection implements Zone {
        protected final Zone[] zones;

        public ZoneCollection(Zone... zones) {
            this.zones = zones;
        }

        @Override
        public Trigger contains(Supplier<Pose2d> pose) {
            Trigger combined = new Trigger(() -> false);

            for (Zone zone : zones) {
                combined = combined.or(zone.contains(pose));
            }

            return combined;
        }
    }

    public static class PredictiveXZoneCollection extends ZoneCollection implements PredictiveXZone {
        public PredictiveXZoneCollection(PredictiveXZone... zones) {
            super(zones);
        }

        @Override
        public Trigger willContain(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> fieldSpeeds, Time dt) {
            Trigger combined = new Trigger(() -> false);

            for (Zone zone : zones) {
                combined = combined.or(((PredictiveXZone) zone).willContain(pose, fieldSpeeds, dt));
            }

            return combined;
        }
    }

    private static final PredictiveXBaseZone BLUE_BOTTOM_TRENCH = new PredictiveXBaseZone(
            ZoneConstants.TRENCH_BUMP_X
                    .minus(ZoneConstants.TRENCH_BUMP_LENGTH.div(2)),
                    // .minus(Dimensions.FULL_LENGTH.div(2)),
            ZoneConstants.TRENCH_BUMP_X
                    .plus(ZoneConstants.TRENCH_BUMP_LENGTH.div(2)),
                    // .plus(Dimensions.FULL_LENGTH.div(2)),
            Meters.of(0),
            ZoneConstants.TRENCH_WIDTH);
    private static final PredictiveXBaseZone BLUE_TOP_TRENCH = BLUE_BOTTOM_TRENCH.mirroredY();
    private static final PredictiveXBaseZone RED_BOTTOM_TRENCH = BLUE_BOTTOM_TRENCH.mirroredX();
    private static final PredictiveXBaseZone RED_TOP_TRENCH = BLUE_TOP_TRENCH.mirroredX();

    public static final PredictiveXZoneCollection TRENCH_ZONES =
            new PredictiveXZoneCollection(BLUE_BOTTOM_TRENCH, BLUE_TOP_TRENCH, RED_BOTTOM_TRENCH, RED_TOP_TRENCH);

    private static final PredictiveXBaseZone BLUE_BOTTOM_BUMP = new PredictiveXBaseZone(
            ZoneConstants.TRENCH_BUMP_X
                    .minus(ZoneConstants.TRENCH_BUMP_LENGTH.div(2)),
                    // .minus(Dimensions.FULL_LENGTH.div(2)),
            ZoneConstants.TRENCH_BUMP_X
                    .plus(ZoneConstants.TRENCH_BUMP_LENGTH.div(2)),
                    // .plus(Dimensions.FULL_LENGTH.div(2)),
            ZoneConstants.TRENCH_WIDTH.plus(ZoneConstants.TRENCH_BLOCK_WIDTH),
            ZoneConstants.TRENCH_WIDTH.plus(ZoneConstants.TRENCH_BLOCK_WIDTH).plus(ZoneConstants.BUMP_WIDTH));
    private static final PredictiveXBaseZone BLUE_TOP_BUMP = BLUE_BOTTOM_BUMP.mirroredY();
    private static final PredictiveXBaseZone RED_BOTTOM_BUMP = BLUE_BOTTOM_BUMP.mirroredX();
    private static final PredictiveXBaseZone RED_TOP_BUMP = BLUE_TOP_BUMP.mirroredX();

    public static final PredictiveXZoneCollection BUMP_ZONES =
            new PredictiveXZoneCollection(BLUE_BOTTOM_BUMP, BLUE_TOP_BUMP, RED_BOTTOM_BUMP, RED_TOP_BUMP);

    public static void logAllZones() {
        Logger.recordOutput("Zones/Trenches/Blue Bottom", BLUE_BOTTOM_TRENCH.getCorners());
        Logger.recordOutput("Zones/Trenches/Blue Top", BLUE_TOP_TRENCH.getCorners());
        Logger.recordOutput("Zones/Trenches/Red Bottom", RED_BOTTOM_TRENCH.getCorners());
        Logger.recordOutput("Zones/Trenches/Red Top", RED_TOP_TRENCH.getCorners());

        Logger.recordOutput("Zones/Bumps/Blue Bottom", BLUE_BOTTOM_BUMP.getCorners());
        Logger.recordOutput("Zones/Bumps/Blue Top", BLUE_TOP_BUMP.getCorners());
        Logger.recordOutput("Zones/Bumps/Red Bottom", RED_BOTTOM_BUMP.getCorners());
        Logger.recordOutput("Zones/Bumps/Red Top", RED_TOP_BUMP.getCorners());
    }
}