package frc.robot.utils;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignUtil {
    public static enum CoralScoreDirection {
        LEFT, RIGHT;
    }

    private static Distance CORAL_SCORE_OFFSET = Units.Inches.of(6);

    // To aid in visualizing how the field, AprilTags, and AutoAlignment work, we have a website:
    // https://team-6615-bellevillains.github.io/AprilTagVisualizer/
    // It does not work on mobile.
    private static Pose2d calculateTargetPose(Pose2d robotPose, CoralScoreDirection coralScoreDirection) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        
        List<Pose2d> coralAprilTagPoses = AprilTagDataUtil.get().getCoralAprilTagPoses(
            alliance.isPresent() ? alliance.get() : DriverStation.Alliance.Blue
        );

        if (coralAprilTagPoses.size() == 0) {
            return null;
        }

        Pose2d poseOfTargettedTag = robotPose.nearest(coralAprilTagPoses);
        System.out.println(poseOfTargettedTag);
        
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
            Units.Centimeters.of(73.5/2 + 8.5 + 1 + 2), 
            CORAL_SCORE_OFFSET, 
            new Rotation2d()
        );

        // Left is the opposite direction or Right
        if (coralScoreDirection == CoralScoreDirection.LEFT) {
            poseAdjustment = poseAdjustment.times(-1);
        }

        // Account for AprilTag Angle
        poseAdjustment = poseAdjustment.rotateBy(poseOfTargettedTag.getRotation());

        System.out.println(poseAdjustment);

        Pose2d finalPose = new Pose2d(
            poseOfTargettedTag.getTranslation().plus(poseAdjustment.getTranslation()), 
            poseOfTargettedTag.getRotation()
        );

        System.out.println(finalPose);

        return finalPose;
    }

    private static Command buildAutoAlign(SwerveSubsystem swerveSubsystem, CoralScoreDirection coralScoreDirection) {
        Pose2d targetPose = calculateTargetPose(swerveSubsystem.getPose(), coralScoreDirection);
        if (targetPose == null) {
            return Commands.print("Failed to load field data when calculating target pose!");
        }

        // return Commands.print("NOOP");
        //8.5
        
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
                        Units.RotationsPerSecond.of(360), 
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
