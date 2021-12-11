package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class DepositCommand extends CommandBase {
    private Deposit deposit;
    private Runnable thingToRun;

    public DepositCommand(Deposit deposit, Runnable thingToRun) {
        this.deposit = deposit;
        this.thingToRun = thingToRun;
    }

    @Override
    public void initialize() {
        thingToRun.run();
    }

    @Override
    public boolean isFinished() {
        return !deposit.isBusy();
    }
}
