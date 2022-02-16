package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurretCap extends SubsystemBase {
    private Servo turretServo;
    private Servo tiltServo;
    private CRServo tapeMeasureExtendServo;

    private double turretServoPosition = 0.8;
    private double tiltServoPosition = 0.5;
    private double extendPower = 0.0;

    public TurretCap(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        tapeMeasureExtendServo =
                hardwareMap.get(CRServo.class, "extensionServo");
    }

    @Override
    public void periodic() {
        turretServo.setPosition(turretServoPosition);
        tiltServo.setPosition(tiltServoPosition);
        tapeMeasureExtendServo.setPower(extendPower);
    }

    public void increaseTilt() {
        tiltServoPosition = Range.clip(tiltServoPosition + 0.05, 0, 1);
    }

    public void decreaseTilt() {
        tiltServoPosition = Range.clip(tiltServoPosition - 0.05, 0, 1);
    }

    public void increaseTurret() {
        turretServoPosition = Range.clip(turretServoPosition + 0.05, 0, 1);
    }

    public void decreaseTurret() {
        turretServoPosition = Range.clip(turretServoPosition - 0.05, 0, 1);
    }

    public void setExtendPower(double extendPower) {
        this.extendPower = extendPower;
    }
}
