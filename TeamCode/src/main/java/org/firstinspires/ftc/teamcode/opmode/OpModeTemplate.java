package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

abstract public class OpModeTemplate extends CommandOpMode {
    protected SampleMecanumDrive mecanumDrive;
    protected Deposit deposit;
    protected Intake intake;
    protected GamepadEx driverGamepad;
    protected GamepadEx secondaryGamepad;

    protected void initHardware() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        deposit = new Deposit(hardwareMap, true);
        intake = new Intake(hardwareMap);

        register(intake, deposit);

        driverGamepad = new GamepadEx(gamepad1);
        secondaryGamepad = new GamepadEx(gamepad2);
    }
}
