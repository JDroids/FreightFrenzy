package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private HardwareMap hardwareMap;
    private DcMotorEx intakeMotor;

    private double power;

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
        intakeMotor.setPower(power);
    }

    public void intake() {
        power = 1.0;
    }

    public void outtake() {
        power = 1.0;
    }

    public void stop() {
        power = 0.0;
    }

    public void setPower(double power) {
        this.power = power;
    }
}
