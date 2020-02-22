/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase
{
	private DriveTrain driveTrain;
	private Joystick leftStick;
	private Joystick rightStick;
	/**
	 * Creates a new DriveWithJoysticks.
	 */
	public DriveWithJoysticks(DriveTrain driveTrain, Joystick leftStick, Joystick rightStick)
	{
		this.driveTrain = driveTrain;
		this.leftStick = leftStick;
		this.rightStick = rightStick;

		addRequirements(driveTrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize()
	{
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute()
	{
		driveTrain.driveCaution(leftStick, rightStick);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		driveTrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}
}
