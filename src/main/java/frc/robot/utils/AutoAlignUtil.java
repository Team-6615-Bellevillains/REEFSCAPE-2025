package frc.robot.utils;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

    private static Pose2d calculateTargetPose(Pose2d robotPose, CoralScoreDirection coralScoreDirection) {
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagDataUtil.get().getFieldLayout();
        if (aprilTagFieldLayout == null) {
            return null;
        }

        // 1. Get a list of all the AprilTags on the field
        // 2. Get the poses of each AprilTag in the list
        // 3. Out of all of the poses, find the closest one to the current robot pose.
        Pose2d poseOfTargettedTag = robotPose
                        .nearest(
                            aprilTagFieldLayout
                                .getTags()
                                .stream()
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

    private static Command buildAutoAlign(Pose2d robotPose, CoralScoreDirection coralScoreDirection) {
        Pose2d targetPose = calculateTargetPose(robotPose, coralScoreDirection);
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
            )
            .finallyDo((interrupted) -> {
                SmartDashboard.putBoolean("Auto Align Status", !interrupted);
            });
    }

    public static Command autoAlign(SwerveSubsystem swerveSubsystem, CoralScoreDirection coralScoreDirection) {
        Supplier<Command> autoAlignSupplier = () -> buildAutoAlign(swerveSubsystem.getPose(), coralScoreDirection);
        return Commands.deferredProxy(autoAlignSupplier);
    }
    
}
