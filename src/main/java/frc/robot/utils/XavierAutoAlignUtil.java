package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AutoAlignUtil.CoralScoreDirection;

public class XavierAutoAlignUtil {
    

    // private static Command buildAutoAlign(SwerveSubsystem swerveSubsystem, CoralScoreDirection coralScoreDirection){
    //     Pose2d targetPose1 = AutoAlignUtil.calculateTargetPose(swerveSubsystem.getPose(), coralScoreDirection, 0.3);
    //     Pose2d targetPose2 = AutoAlignUtil.calculateTargetPose(swerveSubsystem.getPose(), coralScoreDirection, 0);

    //     if (targetPose1 == null) {
    //         return Commands.print("Failed to load field data when calculating target pose!");
    //     }


    // }




    // public static Command autoAlignXavier(SwerveSubsystem swerveSubsystem, CoralScoreDirection coralScoreDirection) {
    //     Supplier<Command> autoAlignSupplier = () -> buildAutoAlign(swerveSubsystem, coralScoreDirection);
    //     return Commands.deferredProxy(autoAlignSupplier);
    // }
    
}
