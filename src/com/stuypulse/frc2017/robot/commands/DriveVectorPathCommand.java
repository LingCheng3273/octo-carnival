package com.stuypulse.frc2017.robot.commands;

import com.stuypulse.frc2017.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class DriveVectorPathCommand extends CommandGroup {

    public DriveVectorPathCommand() {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.

        Command rotateToFirstVector = new RotateDegreesGyroCommand();
        Command rotateToSecondVector = new RotateDegreesGyroCommand();

        Command driveToFirstVector = new RotateDegreesGyroCommand();
        Command driveToSecondVector = new RotateDegreesGyroCommand();

        addSequential(new UpdateVectorCommandsCommand(rotateToFirstVector, driveToFirstVector, rotateToSecondVector, driveToSecondVector));
        addSequential(rotateToFirstVector);
        addSequential(driveToFirstVector);

        addSequential(rotateToSecondVector);
        addSequential(driveToSecondVector);
    }
}

class UpdateVectorCommandsCommand extends InstantCommand {

    private Command[] commands;

    public UpdateVectorCommandsCommand(Command rotateToFirstVector, Command driveToFirstVector, Command rotateToSecondVector, Command driveToSecondVector) {
        commands = new Command[] {rotateToFirstVector, driveToFirstVector, rotateToSecondVector, driveToSecondVector};
    }

    protected void initialize() {
        ((RotateDegreesGyroCommand) commands[0]).setDesiredAngle(Robot.cvVector[0].getDegrees());
        ((DriveForwardEncodersCommand) commands[1]).setInchesToMove(Robot.cvVector[0].getMagnitude());
        ((RotateDegreesGyroCommand) commands[2]).setDesiredAngle(Robot.cvVector[1].getDegrees());
        ((DriveForwardEncodersCommand) commands[3]).setInchesToMove(Robot.cvVector[1].getMagnitude());
    }
}