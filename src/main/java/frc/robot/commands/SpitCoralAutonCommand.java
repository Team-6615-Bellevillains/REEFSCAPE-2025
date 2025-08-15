package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SharedState;
import frc.robot.subsystems.PivotArmSubsystem;

public class SpitCoralAutonCommand extends Command{
    
    private Angle startingRotations;
    private static final Angle endRotations = Rotations.of(16);
    private final PivotArmSubsystem pivotArmSubsystem;
    
    public SpitCoralAutonCommand(PivotArmSubsystem subsystem){
        this.pivotArmSubsystem = subsystem;
        this.addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startingRotations = pivotArmSubsystem.grabberMotorRotations();
        pivotArmSubsystem.setGrabberPower(Percent.of(50));
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return true;
        }
        
        return pivotArmSubsystem.grabberMotorRotations().minus(startingRotations).gte(endRotations);
    }

    @Override
    public void end(boolean interrupted) {
        SharedState.get().setLoaded(false);
    }


}
