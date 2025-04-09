package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import java.util.ArrayList;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.utils.AutoAlignUtil;
import frc.robot.utils.AutoAlignUtil.CoralScoreDirection;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private double maximumSpeed = Units.feetToMeters(10);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private SwerveDrive swerveDrive;
    //private Limelight limelight;
    //private LimelightPoseEstimator limelightPoseEstimator;
    private Pigeon2 gyro = new Pigeon2(0);
    private final Field2d field = new Field2d();
    private final CommandXboxController driverController;
    private int ticks = 0;

    public SwerveSubsystem(CommandXboxController driverController){
        this.driverController = driverController;

        try {
           swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
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


      // 54, 5.25, 
      //36.75
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

      // Every 12 ticks / appx. quarter second
      ticks = (ticks + 1) % 12;
      if (ticks == 0) {
        updateIsAligned();
      }

      SmartDashboard.putNumber("Pose X", swerveDrive.getPose().getX());
      SmartDashboard.putNumber("Pose Y", swerveDrive.getPose().getY());


      // if ((mt1.tagCount != 0) && useVision &&
      // !(mt1.tagCount == 1 && mt1.rawFiducials.length == 1 && (mt1.rawFiducials[0].ambiguity > 0.5 || mt1.rawFiducials[0].distToCamera > 3))
      // && mt1.pose.relativeTo(getPose()).getTranslation().getNorm() < 10){
      //     swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1
      //     , 9999999));
      //     swerveDrive.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      // }
    }


  public Command driveCommand(SwerveInputStream velocity, SwerveInputStream slowedVelocity, CommandXboxController controller){
    return run(()->{
      if (controller.leftTrigger().getAsBoolean()){
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

  private void updateIsAligned() {
    Pose2d robotPose = getPose();

    Pose2d closestAprilTagPose = AutoAlignUtil.getClosestAprilTagPose(robotPose);

    if (closestAprilTagPose == null) {
      return;
    }

    ArrayList<Pose2d> targetPoses = new ArrayList<>(){{
      add(AutoAlignUtil.offsetAprilTagPose(closestAprilTagPose, CoralScoreDirection.LEFT));
      add(AutoAlignUtil.offsetAprilTagPose(closestAprilTagPose, CoralScoreDirection.RIGHT));
    }};

    Pose2d poseDifference = robotPose.relativeTo(robotPose.nearest(targetPoses));

    // Need to ignore rotation because we want to have the 
    // left/right error relative to the AprilTag instead of
    // relative to the field
    Translation2d translationDifferenceIgnoringRotation = 
      poseDifference
        .rotateBy(closestAprilTagPose.getRotation().unaryMinus())
        .getTranslation();
    double frontBackErrorRelativeToAprilTag = translationDifferenceIgnoringRotation.getX();
    double leftRightErrorRelativeToAprilTag = translationDifferenceIgnoringRotation.getY();

    boolean rotationAligned = Math.abs(poseDifference.getRotation().getDegrees()) < 10;
    boolean frontBackAligned = 
      Math.abs(frontBackErrorRelativeToAprilTag) < Inches.of(2.5).in(Meters);
    boolean leftRightAligned = 
      Math.abs(leftRightErrorRelativeToAprilTag) < Inches.of(1.5).in(Meters);

    // BEGIN DEBUG
    // SmartDashboard.putNumber("Rotation Error (Degrees)", poseDifference.getRotation().getDegrees());
    // SmartDashboard.putNumber("Front-Back Error (Inches)", Meters.of(frontBackErrorRelativeToAprilTag).in(Inches));
    // SmartDashboard.putNumber("Left-Right Error (Inches)", Meters.of(leftRightErrorRelativeToAprilTag).in(Inches));
    // SmartDashboard.putBoolean("Rotation Aligned", rotationAligned);
    // SmartDashboard.putBoolean("Front-Back Aligned", frontBackAligned);
    // SmartDashboard.putBoolean("Left-Right Aligned", leftRightAligned);
    // END DEBUG
    
    SmartDashboard.putBoolean("Aligned", rotationAligned && frontBackAligned && leftRightAligned);
    
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

}
