package frc.robot.subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

@Logged
public class SwerveSubsystem extends SubsystemBase {
    private LinearVelocity maximumSpeed = FeetPerSecond.of(15);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    private SwerveDrive swerveDrive;
    private Pigeon2 gyro = new Pigeon2(0);
    private final Field2d field = new Field2d();
    private final CommandXboxController driverController;

    public SwerveSubsystem(CommandXboxController driverController){
        this.driverController = driverController;

        try {
           swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed.in(MetersPerSecond));
           swerveDrive.setChassisDiscretization(true, 0.02);
           swerveDrive.setCosineCompensator(true);
           swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        } catch (Exception e) {
            e.printStackTrace();
        }
       
        
        RobotConfig config;
        try {
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          throw new RuntimeException("PathPlanner config file missing");
        }
        
        AutoBuilder.configure(this::getPose, 
        this::resetOdometry, 
        this::getRobotVelocity, 
        (speeds, feedForward)->{swerveDrive.setChassisSpeeds(speeds);}, 
        new PPHolonomicDriveController(new PIDConstants(2.0, 0.0,0.0), 
        new PIDConstants(2.0,0.0,0.0)), 
        config, 
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          }, 
          this);
          SmartDashboard.putData("field", field);


      LimelightHelpers.setCameraPose_RobotSpace("limelight", -0.1524, -.2794, .3683, 0, 0, 180);
      
      // Use only reef tags for pose estimation
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight", new int[]{
         6,  7,  8,  9, 10, 11, // Red Reef Tags
        17, 18, 19, 20, 21, 22  // Blue Reef Tags
      });
    }

    @Override
    public void periodic() {
      


      Pose2d currentRobotPose = getPose();
      field.setRobotPose(currentRobotPose);
      SmartDashboard.putNumber("rotation fed to limelight", currentRobotPose.getRotation().getDegrees());
      LimelightHelpers.SetRobotOrientation("limelight", currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(mt2 != null && !(Math.abs(gyro.getAngularVelocityYWorld().getValueAsDouble())>360 || mt2.tagCount == 0)){
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1
          , 9999999));
        swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }

      SmartDashboard.putNumber("Pose X", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("Pose Y", swerveDrive.getPose().getY());
    }


  public Command driveCommand(SwerveInputStream velocity, SwerveInputStream slowedVelocity, CommandXboxController controller){
    return run(()->{
      if (controller.leftTrigger().getAsBoolean()){
        swerveDrive.driveFieldOriented(slowedVelocity.get());
      } else {
        swerveDrive.driveFieldOriented(velocity.get());
      }
    });
  }
  
  
  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose){
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity(){
    return swerveDrive.getRobotVelocity();
  }

  public Command resetHeading(){
    return this.runOnce(()->{
      swerveDrive.zeroGyro();
    });
  }

  public Command angleNegative60(){
    return this.runOnce(()->{
      swerveDrive.zeroGyro();
      swerveDrive.setGyro(new Rotation3d(Rotation2d.fromDegrees(-60)));
    });
  }

  // Used to retake control from AutoAlign
  public boolean isBeingControlledByHuman() {
    double leftStickPower = Math.abs(new Translation2d(driverController.getLeftX(), driverController.getLeftY()).getNorm());
    double rightStickPower = Math.abs(new Translation2d(driverController.getRightX(), driverController.getRightY()).getNorm());

    return leftStickPower > 0.05 || rightStickPower > 0.05;
  }

  public ChassisSpeeds getFieldVelocity(){
    return swerveDrive.getFieldVelocity();
  }

  public LinearVelocity getVelocityMagnitude() {
    ChassisSpeeds cs = getFieldVelocity();
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  public Rotation2d getHeading()
  {
    return swerveDrive.getYaw();
  }

  public Command resetGyroToPlayground() {
    return this.runOnce(() -> {
      resetOdometry(new Pose2d(
        new Translation2d(),
        Rotation2d.fromDegrees(-60 + 180)
      ));
    });
  }

}
