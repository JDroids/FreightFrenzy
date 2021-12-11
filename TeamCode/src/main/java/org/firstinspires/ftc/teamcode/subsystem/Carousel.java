package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel extends SubsystemBase {
    private HardwareMap hardwareMap;
    private DcMotorEx carouselMotor;

    private double power;

    public Carousel(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void register() {
        super.register();
    }

    @Override
    public void periodic() {
        carouselMotor.setPower(power);
    }

    public void setPower(double power) {
        this.power = power;
    }
}
