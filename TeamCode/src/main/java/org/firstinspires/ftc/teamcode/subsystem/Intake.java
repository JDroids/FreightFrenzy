package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private HardwareMap hardwareMap;
    private DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void register() {
        super.register();
    }

    @Override
    public void periodic() {

    }

    public void intake() {
        setPower(1.0);
    }

    public void outtake() {
        setPower(-1.0);
    }

    public void stop() {
        setPower(0.0);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }
}
