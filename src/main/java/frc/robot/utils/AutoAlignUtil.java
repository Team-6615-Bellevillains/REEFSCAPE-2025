package frc.robot.utils;

import static edu.wpi.first.units.Units.Meter;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoAlignUtil {
    public static enum CoralScoreDirection {
        LEFT, RIGHT;
    }

    private static Distance CORAL_SCORE_OFFSET = Units.Inches.of(6.5);
    
    private static Distance ROBOT_WIDTH = Units.Centimeters.of(73.5);
    private static Distance BUMPER_THICKNESS = Units.Centimeters.of(8.5);
    private static Distance FUDGE_FACTOR = Units.Centimeters.of(2.5);

    public static Pose2d getClosestAprilTagPose(Pose2d robotPose) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        
        List<Pose2d> coralAprilTagPoses = AprilTagDataUtil.getCoralAprilTagPoses(
            alliance.isPresent() ? alliance.get() : DriverStation.Alliance.Blue
        );

        if (coralAprilTagPoses.size() == 0) {
            return null;
        }

        return robotPose.nearest(coralAprilTagPoses);
    }

    public static Pose2d offsetAprilTagPose(Pose2d aprilTagPose, CoralScoreDirection coralScoreDirection, double backwardsOffset) {
        // An AprilTag with 0 rotation faces the Red alliance wall.
        // This means that scoring "Right" is actually "Up", relative to the field.

        // ------------------------ Top of Field
        //
        //                                      ^ Positive Y             |
        //                                      |                        |
        //    T____T                            |---> Positive X         |
        //   /      \                                                    |
        //  /        \                                                   |
        // T          T ---->   0 rotation                               | Red Alliance Wall
        //  \        /                                                   |
        //   \______/                                                    |
        //   T      T                                                    |
        // 
        // ------------------------ Bottom of Field

        // Left is the opposite direction or Right
        Distance directionalOffset = coralScoreDirection == CoralScoreDirection.LEFT ? CORAL_SCORE_OFFSET.times(-1) : CORAL_SCORE_OFFSET;

        Pose2d poseAdjustment = new Pose2d(
            ROBOT_WIDTH.div(2).plus(BUMPER_THICKNESS).plus(FUDGE_FACTOR).plus(Meter.of(backwardsOffset)), 
            directionalOffset,
            new Rotation2d()
        );

        // Account for AprilTag Angle
        poseAdjustment = poseAdjustment.rotateBy(aprilTagPose.getRotation());

        return new Pose2d(
            aprilTagPose.getTranslation().plus(poseAdjustment.getTranslation()), 
            aprilTagPose.getRotation()
        );
    }

    // To aid in visualizing how the field, AprilTags, and AutoAlignment work, we have a website:
    // https://team-6615-bellevillains.github.io/AprilTagVisualizer/
    // It does not work on mobile.
    public static Pose2d calculateTargetPose(Pose2d robotPose, CoralScoreDirection coralScoreDirection, double backwardsOffset) {
        Pose2d poseOfTargettedTag = getClosestAprilTagPose(robotPose);

        if (poseOfTargettedTag == null) {
            return null;
        }

        return offsetAprilTagPose(poseOfTargettedTag, coralScoreDirection, backwardsOffset);
    }
    
}
