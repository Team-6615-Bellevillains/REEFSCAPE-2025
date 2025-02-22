package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArmSubsystem extends SubsystemBase {
    
    private SparkMax armMotor = new SparkMax(35, MotorType.kBrushless);
    private SparkClosedLoopController armController = armMotor.getClosedLoopController();
    private SparkMax grabberMotor = new SparkMax(33, MotorType.kBrushless);
    private SparkMax conveyorMotor = new SparkMax(20, MotorType.kBrushless);

    public PivotArmSubsystem(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
        .p(2)
        .i(0)
        .d(0.1)
        .outputRange(-0.3, 0.1);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
       // System.out.println((armMotor.getEncoder().getPosition()/20)*360);
    }

    public void setArmPosition(int position){
        switch (position) {
            case 0:
                armController.setReference(0, ControlType.kPosition);
                break;
        
            case 1:
                armController.setReference(degreesToRotations(25), ControlType.kPosition);
                break;

            case 2:
                armController.setReference(degreesToRotations(45), ControlType.kPosition);
        }
    }

    private double degreesToRotations(double degrees){
        return -(degrees/360)*20;
    }

    public Command setArmPositionCommand(int position){
        return this.runOnce(()->setArmPosition(position));
    }

    public Command spitCoral(){
        return this.runEnd(() -> {
            grabberMotor.set(0.2);
            conveyorMotor.set(-0.2);
        }, () -> {
            grabberMotor.stopMotor();
            conveyorMotor.stopMotor();
        });
    }
}
