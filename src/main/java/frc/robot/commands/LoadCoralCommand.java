package frc.robot.commands;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotArmSubsystem;

public class LoadCoralCommand extends Command{
    private final PivotArmSubsystem subsystem;
    private int sawTrigger;
    private double startingRotations;
    private LinearFilter coralMotorRpm_hpFilter;
    
    
    public LoadCoralCommand(PivotArmSubsystem subsystem ){
        this.subsystem = subsystem;
        addRequirements(subsystem);
        coralMotorRpm_hpFilter = LinearFilter.highPass(0.1, 0.02);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing LoadCoral");
        sawTrigger = 0;
        subsystem.loadCoral();
        coralMotorRpm_hpFilter.reset();
       // coralMotorRpm = subsystem.grabberMotorRpm();
    
    
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
    }

    @Override
    public boolean isFinished() {
        return sawTrigger >1 && rotationsSinceTrigger()>5;
    }
            
    @Override
    public void end(boolean cancelled) {
        subsystem.stopMotors();
    }

    private double rotationsSinceTrigger(){
        return subsystem.grabberMotorRotations()-startingRotations;
    }
}
