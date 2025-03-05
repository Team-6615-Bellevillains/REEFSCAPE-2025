package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotArmSubsystem extends SubsystemBase {
    
    private SparkMax armMotor = new SparkMax(35, MotorType.kBrushless);
    private SparkClosedLoopController armController = armMotor.getClosedLoopController();
    private SparkMax grabberMotor = new SparkMax(33, MotorType.kBrushless);
    private SparkFlex conveyorMotor = new SparkFlex(20, MotorType.kBrushless);
    private static final double GEAR_RATIO = 9;


    public PivotArmSubsystem(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
        .p(3)
        .i(0)
        .d(0)
        .outputRange(-0.80, 0.5);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor.getEncoder().setPosition(0);

        SparkFlexConfig config2 = new SparkFlexConfig();
        config2.idleMode(IdleMode.kBrake);
        conveyorMotor.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
     //System.out.println((armMotor.getEncoder().getPosition()/9)*360);
    }

    public void setArmPosition(int position){
        switch (position) {
            case 0:
                armController.setReference(degreesToRotations(-5), ControlType.kPosition);
                break;
        
            case 1:
                armController.setReference(degreesToRotations(25), ControlType.kPosition);
                break;

            case 2:
                armController.setReference(degreesToRotations(45), ControlType.kPosition);
                break;
        }
    }

    public double getPivotAngle(){
        return armMotor.getEncoder().getPosition()/(-GEAR_RATIO)*360;
    }

    private double degreesToRotations(double degrees){
        return -(degrees/360)*GEAR_RATIO;
    }

    public Command setArmPositionCommand(int position){
        return this.runOnce(()->setArmPosition(position));
    }
    
    public double grabberMotorCurrent(){
        return grabberMotor.getOutputCurrent();
    }
    public double grabberMotorRpm(){
        return grabberMotor.getEncoder().getVelocity();
    }
    public double grabberMotorRotations() {
        return grabberMotor.getEncoder().getPosition();
    }

    public void loadCoral(){
            grabberMotor.set(0.1);
            conveyorMotor.set(-0.1);
        }
    
    public void stopMotors(){
        grabberMotor.stopMotor();
        conveyorMotor.stopMotor();
    };
    public Command spitCoral(){
        return this.runEnd(() -> {
            grabberMotor.set(0.3);
            conveyorMotor.set(-0.2);
        }, () -> {
            grabberMotor.stopMotor();
            conveyorMotor.stopMotor();
        });
    }
    
    public Command reverseCoral(){
        return this.runEnd(() -> {
            grabberMotor.set(-0.2);
            conveyorMotor.set(0.2);
        }, () -> {
            grabberMotor.stopMotor();
            conveyorMotor.stopMotor();
        });
    }
   
}

