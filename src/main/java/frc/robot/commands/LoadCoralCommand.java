package frc.robot.commands;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SharedState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Position;

public class LoadCoralCommand extends Command{
    private final PivotArmSubsystem subsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private int sawTrigger;
    private double startingRotations;
    private LinearFilter coralMotorRpm_hpFilter;
    
    
    public LoadCoralCommand(PivotArmSubsystem subsystem, ElevatorSubsystem elevatorSubsystem){
        this.subsystem = subsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(subsystem, elevatorSubsystem);
        coralMotorRpm_hpFilter = LinearFilter.highPass(0.1, 0.02);
    }

    @Override
    public void initialize() {
        if (elevatorSubsystem.getPosition() != Position.L1) {
            this.end(true);
            return;
        }

        System.out.println("Initializing LoadCoral");
        sawTrigger = 0;
        subsystem.loadCoral();
        coralMotorRpm_hpFilter.reset();
        SharedState.get().setLoaded(false);
       // coralMotorRpm = subsystem.grabberMotorRpm();

        subsystem.setGrabberCurrentLimit(2);

    }
    
    @Override
    public void execute() {
        System.out.println(subsystem.grabberMotorCurrent());
        System.out.println(subsystem.grabberMotorRpm());
        
        SmartDashboard.putNumber("Grabber Motor Current:", subsystem.grabberMotorCurrent());
        SmartDashboard.putNumber("grabber motor rpm:", subsystem.grabberMotorRpm());
        double coralMotorRpm_Filter = coralMotorRpm_hpFilter.calculate(subsystem.grabberMotorRpm()); 
        System.out.println(coralMotorRpm_Filter);
        SmartDashboard.putNumber("high pass filtered rpm:", coralMotorRpm_Filter);
        SmartDashboard.putNumber("total rotations:", rotationsSinceTrigger());
        SmartDashboard.putNumber("starting rotations:", startingRotations);
    
        if (coralMotorRpm_Filter < -15) { 
            sawTrigger++;
            if(sawTrigger == 1)
                startingRotations = subsystem.grabberMotorRotations();
        } 
        // Coral probably just tapped the wheels, reset detection
        // else if (coralMotorRpm_Filter > 50) {
        //     sawTrigger = 0;
        // }
    }

    @Override
    public boolean isFinished() {
        return sawTrigger >1 && rotationsSinceTrigger()>4;
    }
            
    @Override
    public void end(boolean cancelled) {
        if (!cancelled) {
            SharedState.get().setLoaded(true);
        }
        subsystem.stopMotors();
        subsystem.setGrabberCurrentLimit(40);
    }

    private double rotationsSinceTrigger(){
        return subsystem.grabberMotorRotations()-startingRotations;
    }
}
