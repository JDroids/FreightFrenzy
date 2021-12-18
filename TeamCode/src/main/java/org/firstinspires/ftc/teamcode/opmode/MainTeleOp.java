package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.DepositCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp(name="TeleOp")
public class MainTeleOp extends OpModeTemplate {
    @Override
    public void initialize() {
        initHardware(false);

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> deposit.goToHeight(25.0));
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(deposit::deploy);
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(deposit::retract);
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(() -> deposit.goToHeight(14.0));

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.Y).whenPressed(() -> Deposit.offset += 0.25);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.A).whenPressed(() -> Deposit.offset -= 0.25);
    }

    @Override
    public void run() {
        super.run();

        mecanumDrive.setDrivePower(
                new Pose2d(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x));

        mecanumDrive.updatePoseEstimate();

        intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        carousel.setPower(gamepad2.left_stick_y);
    }
}