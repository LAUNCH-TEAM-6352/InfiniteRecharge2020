/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.TurretConstants;

/***
 * A subsystem that controls the turret.
 */
public class Turret extends SubsystemBase
{
	private enum AzimuthPosition
	{
		Unknown,
		LeftOfCenter,
		Center,
		RightOfCenter
	}

	private VictorSPX azimuthMotor = new VictorSPX(TurretConstants.azimuthMotorChannel);

	private DigitalInput leftAzimuthLimit = new DigitalInput(TurretConstants.leftAzimuthLimitChannel);
	private DigitalInput centerAzimuthLimit = new DigitalInput(TurretConstants.centerAzimuthLimitChannel);
	private DigitalInput rightAzimuthLimit = new DigitalInput(TurretConstants.rightAzimuthLimitChannel);

	private XboxController controller;

	// Where qwe think we are:
	private AzimuthPosition currentAzimuthPosition = AzimuthPosition.Unknown;

	/**
	 * Creates an instance.
	 */
	public Turret(XboxController controller)
	{
		this.controller = controller;
		azimuthMotor.setInverted(TurretConstants.isAzimuthMotorInverted);

		// If the azimuth is at a limit switch, this will set its current position:
		setCurrentAzimuthPosition();
	}

	/**
	 * Run azimuth motor at specified percantage.
	 * 
	 * @param percentage
	 */
	public void setAzimuth(double percentage)
	{
		// See if we are moving through the center position:
		if (!centerAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.Center;
		}
		
		boolean isRumble = false;
		if (percentage > 0)
		{
			// We are moving to the right.
			if (currentAzimuthPosition == AzimuthPosition.Center && centerAzimuthLimit.get())
			{
				// We were at center but not anymore:
				currentAzimuthPosition = AzimuthPosition.RightOfCenter;
			}
			else if (!rightAzimuthLimit.get())
			{
				// We are all the way to the right:
				currentAzimuthPosition = AzimuthPosition.RightOfCenter;
				percentage = 0;
				isRumble = true;
			}
		}
		else if (percentage < 0)
		{
			// We are movine to the left.
			if (currentAzimuthPosition == AzimuthPosition.Center && centerAzimuthLimit.get())
			{
				// We were at center but nut anymore:
				currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
			}
			else if (!leftAzimuthLimit.get())
			{
				// We are all the way to the left:
				currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
				percentage = 0;
				isRumble = true;
			}
		}

		controller.setRumble(RumbleType.kRightRumble, isRumble ? 1 : 0);

		SmartDashboard.putNumber(DashboardConstants.azimuthMotorKey, percentage);
		azimuthMotor.set(ControlMode.PercentOutput, percentage * TurretConstants.azimuthPercentageScaleFactor);
	}

	/**
	 * Moves the turret towards the center (front) position.
	 */
	public void moveToCenterPosition()
	{
		if (!centerAzimuthLimit.get())
		{
			// We are at center, do not move anymore:
			currentAzimuthPosition = AzimuthPosition.Center;
			setAzimuth(0);
		}
		else if (!leftAzimuthLimit.get() || currentAzimuthPosition == AzimuthPosition.LeftOfCenter)
		{
			currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
			setAzimuth(TurretConstants.moveToRightAutoPercentage);
		}
		else if (!rightAzimuthLimit.get() || currentAzimuthPosition == AzimuthPosition.RightOfCenter)
		{
			currentAzimuthPosition = AzimuthPosition.RightOfCenter;
			setAzimuth(TurretConstants.moveToLeftAutoPercentage);
		}
		else
		{
			// We do not know where we are at, move right until we
			// hit center or all the way right.
			setAzimuth(TurretConstants.moveToRightAutoPercentage);
		}
	}

	/***
	 * Convenience method to stop all turret motion.
	 */
	public void stop()
	{
		setAzimuth(0);
	}

	/**
	 * If possible, set the current azimuth position.
	 */
	public void setCurrentAzimuthPosition()
	{
		if (!leftAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.LeftOfCenter;
		}
		else if (!centerAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.Center;
		}
		else if (!rightAzimuthLimit.get())
		{
			currentAzimuthPosition = AzimuthPosition.RightOfCenter;
		}
	}

	/**
	 * Indicates if the azimuth is at the center (front) position.
	 */
	public boolean isAtCenterPosition()
	{
		return !centerAzimuthLimit.get();
	}

	@Override
	public void periodic()
	{

		SmartDashboard.putBoolean(DashboardConstants.leftAzimuthLimitKey, !leftAzimuthLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.frontAzimuthLimitKey, !centerAzimuthLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.rightAzimuthLimitKey, !rightAzimuthLimit.get());

		SmartDashboard.putString(DashboardConstants.turretPositionKey, currentAzimuthPosition.toString());
	}
}
