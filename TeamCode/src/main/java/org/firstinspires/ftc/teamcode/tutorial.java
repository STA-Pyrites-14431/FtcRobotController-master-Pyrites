package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "tutorial")
public class tutorial extends OpMode {

    DcMotor motor, motor2;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        telemetry.addData("Hardware: ", "Initialized");

    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.5) {
            motor.setPower(-0.5);
        } else if (gamepad1.left_trigger > 0.5) {
            motor.setPower(0.2);
        } else {
            motor.setPower(0);
        }
    }
}
