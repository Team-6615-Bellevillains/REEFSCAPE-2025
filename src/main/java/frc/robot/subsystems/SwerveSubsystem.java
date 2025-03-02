package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.LEDMode;

public class SwerveSubsystem extends SubsystemBase {
    private double maximumSpeed = Units.feetToMeters(10);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive swerveDrive;
    private Limelight limelight;
    private LimelightPoseEstimator limelightPoseEstimator;
    private Pigeon2 gyro = new Pigeon2(0);
    private final Field2d field = new Field2d();

    public SwerveSubsystem(){
        try {
           swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
           swerveDrive.setChassisDiscretization(true, 0.02);
           swerveDrive.setCosineCompensator(true);
           swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        } catch (Exception e) {
            throw new RuntimeException("swerve drive config files missing");
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
        new PPHolonomicDriveController(new PIDConstants(5.0, 0.0,0.0), 
        new PIDConstants(1.0,0.0,0.0)), 
        config, 
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          }, 
          this);
          limelight = new Limelight("limelight");
          limelightPoseEstimator = limelight.getPoseEstimator(true);
          limelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(new Pose3d(.31 , .16, .455, new Rotation3d()));
          SmartDashboard.putData("field", field);
    }

    @Override
    public void periodic() {
      limelight.getSettings()
        .withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(),
          new AngularVelocity3d(gyro.getAngularVelocityXWorld().getValue(), gyro.getAngularVelocityYWorld().getValue(),
              gyro.getAngularVelocityXWorld().getValue())))
        .save();
      
      boolean useVision = Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble())<=720;

      // Get MegaTag2 pose
      Optional<PoseEstimate> visionEstimate = limelightPoseEstimator.getPoseEstimate();
      // If the pose is present
      visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
        // Add it to the pose estimator.
        if ((poseEstimate.tagCount != 0) && useVision){
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
        }
      });
      field.setRobotPose(getPose());
    }

    public Command getAutonomousCommand(){
      return new PathPlannerAuto("New Auto");
    }


  public Command driveCommand(SwerveInputStream velocity, SwerveInputStream slowedVelocity, CommandXboxController controller){
    return run(()->{
      if (controller.leftBumper().getAsBoolean()){
        swerveDrive.driveFieldOriented(slowedVelocity.get());
        //System.out.println("slowing swerve drive");
      } else {
        swerveDrive.driveFieldOriented(velocity.get());
        //System.out.println("not slow");
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
    return this.run(()->{
      swerveDrive.zeroGyro();
    });
  }

}
