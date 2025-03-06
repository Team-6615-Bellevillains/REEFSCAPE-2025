package frc.robot.utils;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignUtil {
    public static enum CoralScoreDirection {
        LEFT, RIGHT;
    }

    private static Distance CORAL_SCORE_OFFSET = Units.Inches.of(2);
    private static Set<Integer> CORAL_APRIL_TAG_IDS = new HashSet<>(){{
        // Red Reef
        for (int id = 6; id <= 11; id++) {
            add(id);
        }

        // Blue Reef
        for (int id = 17; id <= 22; id++) {
            add(id);
        }
    }};

    // To aid in visualizing how the field, AprilTags, and AutoAlignment work, we have a website:
    // https://team-6615-bellevillains.github.io/AprilTagVisualizer/
    // It does not work on mobile.
    private static Pose2d calculateTargetPose(Pose2d robotPose, CoralScoreDirection coralScoreDirection) {
        List<AprilTag> aprilTags = AprilTagDataUtil.get().getAprilTags();
        if (aprilTags.size() == 0) {
            return null;
        }

        // 1. Get a list of all the AprilTags on the field
        // 2. Filter out the non-Coral AprilTags
        // 2. Get the poses of each AprilTag
        // 3. Out of all of the poses, find the closest one to the current robot pose.
        Pose2d poseOfTargettedTag = robotPose
                        .nearest(
                            aprilTags
                                .stream()
                                .filter((AprilTag aprilTag) -> CORAL_APRIL_TAG_IDS.contains(aprilTag.ID))
                                .map((AprilTag aprilTag) -> aprilTag.pose.toPose2d())
                                .toList()
                        );
        
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

        Pose2d poseAdjustment = new Pose2d(
            Units.Inches.of(0), 
            CORAL_SCORE_OFFSET, 
            new Rotation2d()
        );

        // Left is the opposite direction or Right
        if (coralScoreDirection == CoralScoreDirection.LEFT) {
            poseAdjustment = poseAdjustment.times(-1);
        }

        // Account for AprilTag Angle
        poseAdjustment = poseAdjustment.rotateBy(poseOfTargettedTag.getRotation());

        return poseOfTargettedTag.transformBy(new Transform2d(new Pose2d(), poseAdjustment));
    }

    private static Command buildAutoAlign(SwerveSubsystem swerveSubsystem, CoralScoreDirection coralScoreDirection) {
        Pose2d targetPose = calculateTargetPose(swerveSubsystem.getPose(), coralScoreDirection);
        if (targetPose == null) {
            return Commands.print("Failed to load field data when calculating target pose!");
        }
        
        return Commands.print("Auto Aligning to " + targetPose)
            .andThen(Commands.runOnce(() -> {
                SmartDashboard.putBoolean("Auto Align Status", false);
            }))
            .alongWith(
                AutoBuilder.pathfindToPose(
                    targetPose, 
                    new PathConstraints(
                        Units.FeetPerSecond.of(1), 
                        Units.FeetPerSecondPerSecond.of(4), 
                        Units.RotationsPerSecond.of(120), 
                        Units.RotationsPerSecondPerSecond.of(300)
                    )
                )
                // Cancel pathfinding if Driver wants to take over
                .until(swerveSubsystem::isBeingControlledByHuman) 
            )
            .finallyDo((interrupted) -> {
                SmartDashboard.putBoolean("Auto Align Status", !interrupted);
            });
    }

    // This command is structured with a Supplier and Proxy
    // because Commands are generated when the code is first ran,
    // but we want to be able to generate the alignment command
    // using the current pose.
    //
    // If we weren't using PathPlanner generated commands,
    // it would probably be better to have a regular Command
    // and then pass the "starting robot pose" in with a 
    // Supplier<Pose2d>
    public static Command autoAlign(SwerveSubsystem swerveSubsystem, CoralScoreDirection coralScoreDirection) {
        Supplier<Command> autoAlignSupplier = () -> buildAutoAlign(swerveSubsystem, coralScoreDirection);
        return Commands.deferredProxy(autoAlignSupplier);
    }
    
}
