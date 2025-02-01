package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.Intake;

public class CommandManager {

    public static Command intakePiece(Intake intake) {

            //auto intake game piece
            Command command =
                new InstantCommand(()-> intake.setIntakeKrakenSpeed(0), intake);
        
            return command;
        }





}