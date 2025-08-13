package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SharedState;
import frc.robot.subsystems.PivotArmSubsystem;

public class SpitCoralAutonCommand extends Command{
    
    private double startingRotations;
    private static final double endRotations = 16;
    private final PivotArmSubsystem pivotArmSubsystem;
    
    public SpitCoralAutonCommand(PivotArmSubsystem subsystem){
        this.pivotArmSubsystem = subsystem;
        this.addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startingRotations = pivotArmSubsystem.grabberMotorRotations();
        pivotArmSubsystem.setGrabberMotor(0.5);
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return true;
        }
        
        return pivotArmSubsystem.grabberMotorRotations() - startingRotations >= endRotations;
    }

    @Override
    public void end(boolean interrupted) {
        SharedState.get().setLoaded(false);
    }


}
