package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.FlippingUtil;

public final class Constants {
    public static class driveConstants {
        public static double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                           // top
        // speed
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity
    }
  public static class QuestNavConstants{
    public static final Pose2d initalPose2dBlue = new Pose2d(3.25,4, Rotation2d.fromDegrees(-90)); 
    public static final Pose2d initalPose2dRed = FlippingUtil.flipFieldPose(initalPose2dBlue); 
    public static final Transform2d ROBOT_TO_QUEST = new Transform2d(-0.24,-0.314,Rotation2d.fromDegrees(-90));
    // public static final Transform2d ROBOT_TO_QUEST = new Transform2d(-0.23-,-0.298,Rotation2d.fromDegrees(-90));


  }

    public static class FieldConstants {
        public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

        public static final Translation3d HUB_BLUE = new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2),
                Inches.of(56.4));
        public static final Translation3d HUB_RED = new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)),
                FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Distance FUNNEL_RADIUS = Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

        private static final Distance TRENCH_BUMP_X = Inches.of(181.56);
        private static final Distance TRENCH_WIDTH = Inches.of(49.86);
        private static final Distance BUMP_INSET = TRENCH_WIDTH.plus(Inches.of(12));
        private static final Distance BUMP_LENGTH = Inches.of(73);

        private static final Distance TRENCH_ZONE_EXTENSION = Inches.of(70);
        private static final Distance BUMP_ZONE_EXTENSION = Inches.of(60);
        private static final Distance TRENCH_BUMP_ZONE_TRANSITION = TRENCH_WIDTH.plus(BUMP_INSET).div(2);

        public static final Translation2d[][] TRENCH_ZONES = {
                new Translation2d[] {
                        new Translation2d(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION), Inches.zero()),
                        new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION)
                },
                new Translation2d[] {
                        new Translation2d(
                                TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION),
                                FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
                        new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH)
                },
                new Translation2d[] {
                        new Translation2d(FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)), Inches.zero()),
                        new Translation2d(
                                FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)),
                                TRENCH_BUMP_ZONE_TRANSITION)
                },
                new Translation2d[] {
                        new Translation2d(
                                FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)),
                                FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
                        new Translation2d(FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)), FIELD_WIDTH)
                }
        };

        public static final Translation2d[][] BUMP_ZONES = {
                new Translation2d[] {
                        new Translation2d(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION),
                        new Translation2d(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), BUMP_INSET.plus(BUMP_LENGTH))
                },
                new Translation2d[] {
                        new Translation2d(
                                TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION),
                                FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
                        new Translation2d(
                                TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
                },
                new Translation2d[] {
                        new Translation2d(
                                FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
                                FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
                        new Translation2d(
                                FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
                                FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
                },
                new Translation2d[] {
                        new Translation2d(
                                FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
                                TRENCH_BUMP_ZONE_TRANSITION),
                        new Translation2d(
                                FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
                                BUMP_INSET.plus(BUMP_LENGTH))
                }
        };

        public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
    }

}
