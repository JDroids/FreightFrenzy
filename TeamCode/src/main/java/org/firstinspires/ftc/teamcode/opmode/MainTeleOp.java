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

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> deposit.goToHeight(28));
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(deposit::teleopDeploy);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.A).whenPressed(deposit::retract);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.Y).whenPressed(() -> deposit.goToHeight(14.0));
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.X).whenPressed(() -> deposit.goToHeight(7.0));
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.B).whenPressed(() -> deposit.goToHeight(7.0));

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(() -> Deposit.offset += 0.25);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> Deposit.offset -= 0.25);

        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_LEFT).whenPressed(turretCap::decreaseTurret);
        new GamepadButton(secondaryGamepad, GamepadKeys.Button.DPAD_RIGHT).whenPressed(turretCap::increaseTurret);

        new Trigger(() -> gamepad2.left_trigger > 0.3).whenActive(turretCap::decreaseTilt);
        new Trigger(() -> gamepad2.right_trigger > 0.3).whenActive(turretCap::increaseTilt);
    }

    @Override
    public void run() {
        super.run();

        mecanumDrive.setDrivePower(
                new Pose2d(-gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x));

        mecanumDrive.updatePoseEstimate();

        double rawIntakePower = -gamepad2.left_stick_y;
        intake.setPower(Math.signum(rawIntakePower) * rawIntakePower * rawIntakePower);

        carousel.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        turretCap.setExtendPower(-gamepad2.right_stick_y);
    }
}
