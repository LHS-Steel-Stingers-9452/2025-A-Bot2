 package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;

public class CommandManager {

    public static Command intakeCoral(Funnel funnel, Intake intake){

            //auto intake coral
            return funnel.runFunnel(0.4).alongWith(intake.runIntake(0.2))
                    .until(intake::hasCoral)
                    .andThen(new WaitCommand(0.15))
                    .andThen(funnel.runFunnel(0).alongWith(intake.runIntake(0)));
        }

    public static Command setPositions(Arm arm, Elevator elevator, double armPosition, double elevatorPosition){
             return arm.setPosition(0.25)
             .until(()-> arm.armPosition() < 0.27 && arm.armPosition() > 0.23)
             .andThen(
                elevator.setPosition(elevatorPosition)
             .alongWith(arm.setPosition(armPosition))
             );   
    }

public static Command intakePositions(Arm arm, Elevator elevator) {
   // return arm.setPosition(0.36).alongWith(elevator.runElevator(0));
   // return arm.runArm(0.1).alongWith(elevator.runElevator(0)).withTimeout(0.5);
    return arm.runArm(0.1).withTimeout(0.5).andThen(elevator.runElevator(0)).andThen(arm.runArm(0));
}

}
