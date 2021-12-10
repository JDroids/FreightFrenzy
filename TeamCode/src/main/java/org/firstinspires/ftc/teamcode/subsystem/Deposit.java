package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Deposit extends SubsystemBase {
    private final HardwareMap hardwareMap;
    private DcMotorEx extensionMotor;
    private Servo flipServo;

    private double targetHeight;
    private double power;

    private boolean shouldResetEncoder;

    private ElapsedTime timer = new ElapsedTime();

    public static double RETRACTED_HEIGHT = 0.3;
    public static double LEVEL_3_HEIGHT = 30;

    public static double RETRACTED_POSITION = 0.34;
    public static double MOVING_POSITION = 0.5;
    public static double DEPLOY_POSITION = 1.0;


    private enum State {
        RETRACTED,
        RETRACTING,
        GOING_TO_HEIGHT,
        DEPLOY
    }

    private State state = State.RETRACTED;

    public Deposit(HardwareMap hardwareMap, boolean shouldResetEncoder) {

        this.hardwareMap = hardwareMap;
        this.shouldResetEncoder = shouldResetEncoder;
    }

    private boolean hasBeenRegistered = false;

    @Override
    public void register() {
        super.register();

        extensionMotor = hardwareMap.get(DcMotorEx.class, "depositMotor");
        flipServo = hardwareMap.get(Servo.class, "depositServo");

        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (shouldResetEncoder) {
            extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        extensionMotor.setTargetPosition(inchesToTicks(RETRACTED_HEIGHT));
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flipServo.setPosition(RETRACTED_POSITION);

        hasBeenRegistered = true;
    }

    @Override
    public void periodic() {
        if (!hasBeenRegistered) {
            return;
        }
        
        extensionMotor.setTargetPosition(inchesToTicks(targetHeight));
        extensionMotor.setPower(1.0);

        switch (state) {
            case RETRACTED:
                flipServo.setPosition(RETRACTED_POSITION);
                break;
            case RETRACTING:
                flipServo.setPosition(MOVING_POSITION);

                if (!extensionMotor.isBusy()) {
                    state = State.RETRACTED;
                }
                break;
            case GOING_TO_HEIGHT:
                flipServo.setPosition(MOVING_POSITION);
                break;
            case DEPLOY:
                flipServo.setPosition(DEPLOY_POSITION);

                if (timer.seconds() > 0.5) {
                    targetHeight = RETRACTED_HEIGHT;
                    state = State.RETRACTING;
                }
        }
    }

    public void goToHeight(double targetHeight) {
        this.targetHeight = targetHeight;
        state = State.GOING_TO_HEIGHT;
    }

    public void goToLevel3() {
        goToHeight(LEVEL_3_HEIGHT);
    }

    public void retract() {
        targetHeight = RETRACTED_HEIGHT;
        state = State.RETRACTING;
    }

    public void deploy() {
        timer.reset();
        state = State.DEPLOY;
    }

    private static int inchesToTicks(double inches) {
        final double DIAMETER = 48 / 25.4;
        final double CIRCUMFERENCE = DIAMETER * Math.PI;

        final double TICKS_PER_ROTATION = 28 * 19.2;

        return (int) (inches / CIRCUMFERENCE * TICKS_PER_ROTATION);
    }
}
