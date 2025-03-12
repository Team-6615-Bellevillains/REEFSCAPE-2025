package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Position;

public class GoToElevatorPositionCommand extends Command{
    ElevatorSubsystem elevator;
    PivotArmSubsystem pivot;
    ElevatorSubsystem.Position position;

    public GoToElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, PivotArmSubsystem pivotArmSubsystem, ElevatorSubsystem.Position position){
        this.addRequirements(elevatorSubsystem, pivotArmSubsystem);

        this.elevator = elevatorSubsystem;
        this.pivot = pivotArmSubsystem;
        this.position = position;
    }
    
    @Override
    public void initialize() {
        System.out.println("setting elevator");
        if(elevator.getPosition() != position){
            if (true/*elevator.getPosition() == Position.L4 || position == Position.L4*/){
                pivot.setArmPosition(true);
            }
            elevator.setPosition(position);
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            if (position != Position.L1){
                pivot.setArmPosition(true);
            } else pivot.setArmPosition(false);
        }
    }
}
