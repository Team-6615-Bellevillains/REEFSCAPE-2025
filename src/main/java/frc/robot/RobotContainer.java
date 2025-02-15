package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    
    private SwerveSubsystem swerve = new SwerveSubsystem();
    private AlgaeGrabberSubsystem algae = new AlgaeGrabberSubsystem();
    private CommandXboxController driverController = new CommandXboxController(0);
    private CommandXboxController operatorController = new CommandXboxController(1);

    // from the YAGSL example project, hence the diabolical formatting  
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
    
    public RobotContainer(){
        configureBindings();
    }

    private void configureBindings(){
        swerve.setDefaultCommand(swerve.driveCommand(driveAngularVelocity));
        driverController.a().onTrue(new GrabAlgaeCommand(algae));
        driverController.b().whileTrue(algae.spitAlgae());
    }
}
