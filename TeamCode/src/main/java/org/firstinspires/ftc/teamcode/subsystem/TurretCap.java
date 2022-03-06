package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurretCap extends SubsystemBase {
    private Servo turretServo;
    private Servo tiltServo;
    private DcMotor tapeMeasureExtend;

    private double TILT_SERVO_LOWER = 0.46;
    private double TILT_SERVO_UPPER = 0.72;

    private double TURRET_SERVO_LOWER = 0.35;
    private double TURRET_SERVO_UPPER = 1.0;

    private double turretServoPosition = 1.0;
    private double tiltServoPosition = 0.68;
    private double extendPower = 0.0;

    public boolean isFastMode = true;

    public TurretCap(HardwareMap hardwareMap, boolean isAuto) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
        tapeMeasureExtend =
                hardwareMap.get(DcMotor.class, "tapeExtendMotor");

        tiltServo.setDirection(Servo.Direction.REVERSE);

        if (!isAuto) {
            turretServoPosition = 0.7;
            tiltServoPosition = 0.6;
        }
    }

    @Override
    public void periodic() {
        turretServo.setPosition(turretServoPosition);
        tiltServo.setPosition(tiltServoPosition);
        tapeMeasureExtend.setPower(extendPower);

        Log.d("turret cap positions",
                String.format(
                        "turretServo: %.3f, tiltServo: %.3f",
                        turretServoPosition,
                        tiltServoPosition));
    }

    public void increaseTilt() {
        tiltServoPosition = Range.clip(
                tiltServoPosition + 0.005,
                TILT_SERVO_LOWER,
                TILT_SERVO_UPPER);
    }

    public void decreaseTilt() {
        tiltServoPosition = Range.clip(
                tiltServoPosition - getIncrement(),
                TILT_SERVO_LOWER,
                TILT_SERVO_UPPER);
    }

    public void increaseTurret() {
        turretServoPosition = Range.clip(
                turretServoPosition + getIncrement(),
                TURRET_SERVO_LOWER,
                TURRET_SERVO_UPPER);
    }

    public void decreaseTurret() {
        turretServoPosition = Range.clip(
                turretServoPosition - getIncrement(),
                TURRET_SERVO_LOWER,
                TURRET_SERVO_UPPER);
    }

    private double getIncrement() {
        return isFastMode ? 0.025 : 0.005;
    }

    public void setExtendPower(double extendPower) {
        this.extendPower = extendPower;
    }
}
