/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants;

public class Climber extends SubsystemBase
{
	private VictorSPX leftMotor = new VictorSPX(ClimberConstants.leftMotorChannel);
	private VictorSPX rightMotor = new VictorSPX(ClimberConstants.rightMotorChannel);

	private DigitalInput leftExtendedLimit = new DigitalInput(ClimberConstants.leftExtendedLimit);
	private DigitalInput leftRetractedLimit = new DigitalInput(ClimberConstants.leftRetractedLimit);
	private DigitalInput rightExtendedLimit = new DigitalInput(ClimberConstants.rightExtendedLimit);
	private DigitalInput rightRetractedLimit = new DigitalInput(ClimberConstants.rightRetractedLimit);

	private DoubleSolenoid solenoid = new DoubleSolenoid(ClimberConstants.outSolenoidChannel, ClimberConstants.inSolenoidChannel);

	private XboxController gameController;

	private boolean isActuatorsDeployed = false;

	/**
	 * Creates a new Climber.
	 */
	public Climber(XboxController gameController)
	{
		leftMotor.setInverted(ClimberConstants.isLeftMotorInverted);
		rightMotor.setInverted(ClimberConstants.isRightMotorInverted);

		this.gameController = gameController;
	}

	public void setSpeeds(double speedLeft, double speedRight)
	{
		// Assume we are going to rumble:
		boolean isLeftAtLimit = false;
		boolean isRightAtLimit = false;

		// Handle left leg:
		if (speedLeft > 0 && !leftExtendedLimit.get())
		{
			speedLeft = 0;
			isLeftAtLimit = true;
		}
		else if (speedLeft < 0 && !leftRetractedLimit.get())
		{
			speedLeft = 0;
			isLeftAtLimit = true;
		}
		leftMotor.set(ControlMode.PercentOutput, speedLeft);

		// Handle right leg:
		if (speedRight > 0 && !rightExtendedLimit.get())
		{
			speedRight = 0;
			isRightAtLimit = true;
		}
		else if (speedRight < 0 && !rightRetractedLimit.get())
		{
			speedRight = 0;
			isRightAtLimit = true;
		}
		rightMotor.set(ControlMode.PercentOutput, speedRight);

		gameController.setRumble(RumbleType.kLeftRumble, isLeftAtLimit && isRightAtLimit ? 1 : 0);
	}

	public void undeployActuators()
	{
		solenoid.set(DoubleSolenoid.Value.kReverse);
		isActuatorsDeployed = false;
	}

	public void deployActuators()
	{
		solenoid.set(DoubleSolenoid.Value.kForward);
		isActuatorsDeployed = true;
	}

	public void stop()
	{
		setSpeeds(0, 0);
	}

	public boolean isLeftExtended()
	{
		return !leftExtendedLimit.get();
	}

	public boolean isRightExtended()
	{
		return !rightExtendedLimit.get();
	}

	public boolean isActuatorsDeployed()
	{
		return isActuatorsDeployed;
	}

	@Override
	public void periodic()
	{
		SmartDashboard.putBoolean(DashboardConstants.leftClimberExtendedLimitKey, !leftExtendedLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.leftClimberRetractedLimitKey, !leftRetractedLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.rightClimberExtendedLimitKey, !rightExtendedLimit.get());
		SmartDashboard.putBoolean(DashboardConstants.rightClimberRetractedLimitKey, !rightRetractedLimit.get());
	}
}
