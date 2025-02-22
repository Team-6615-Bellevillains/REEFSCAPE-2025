package frc.robot.subsystems;

import java.io.File; 

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private double maximumSpeed = Units.feetToMeters(100);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive swerveDrive;

    public SwerveSubsystem(){
        try {
           swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed); 
        } catch (Exception e) {
            throw new RuntimeException();
        }
        
    }


  public Command driveCommand(SwerveInputStream velocity, CommandXboxController controller){
    return run(()->{
      SwerveInputStream adjustedInput = controller.leftBumper().getAsBoolean() ?  velocity.scaleTranslation(0.5).scaleRotation(0.5) : velocity;
      swerveDrive.driveFieldOriented(adjustedInput.get());
    });
  }
  
  
  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

}
