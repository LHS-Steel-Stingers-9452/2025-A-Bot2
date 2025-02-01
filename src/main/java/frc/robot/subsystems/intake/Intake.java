
package frc.robot.subsystems.intake;

import java.io.ObjectInputFilter.Config;
import java.lang.module.Configuration;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 
public class Intake extends SubsystemBase {

    
    private final TalonFX intakeKraken;
    
        public Intake() {

              intakeKraken = new TalonFX(0);

   // intakeKraken.getConfigurator().apply(null);
    
    }
    public void setIntakeKrakenSpeed(double intakeSpeed){
        intakeKraken.set(0.65);
    }
    public void stopIntake(){
        intakeKraken.set(0);
   }
    
    } 

 

    
