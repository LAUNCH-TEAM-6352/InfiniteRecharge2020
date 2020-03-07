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
import frc.robot.subsystems.Turret;

/**
 * An example command that uses an example subsystem.
 */
public class RunTurretWithGameController extends CommandBase
{
	@SuppressWarnings(
	{ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final Turret turret;
	private final XboxController controller;

	public RunTurretWithGameController(Turret turret, XboxController controller)
	{
		this.turret = turret;
		this.controller = controller;

		// Use addRequirements() here to declare subsystem dependencies.
		if (turret != null)
		{
			addRequirements(turret);
		}
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
		if (turret != null)
		{
			turret.setHood(controller.getY(Hand.kRight) * -1.0);
			turret.setAzimuth(controller.getX(Hand.kLeft));
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted)
	{
		turret.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished()
	{
		return false;
	}
}
