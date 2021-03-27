/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

/**
 * An example command that uses an example subsystem.
 */
public class RunHoodWithGameController extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Hood hood;
	private final XboxController controller;

	public RunHoodWithGameController(Hood hood, XboxController controller)
	{
		this.hood = hood;
		this.controller = controller;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(hood);
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
		hood.setHood(filter(controller.getY(Hand.kRight)) * -1.0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		hood.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}

	private double filter(double v)
	{
		return Math.abs(v) <= 0.5 ? 0.0 : v;
	}
}
