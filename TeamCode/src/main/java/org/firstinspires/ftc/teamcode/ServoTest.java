package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;


@TeleOp(name="ServoTest")
public class ServoTest extends OpMode {
//hardware initialization stuff
        Servo servo;
        double pos = 0.7;

        /**
         * User defined init method
         * <p>
         * This method will be called once when the INIT button is pressed.
         */
        @Override
        public void init() {
                servo= hardwareMap.get(Servo.class, "right_claw_servo");


        }

        /**
         * User defined loop method
         * <p>
         * This method will be called repeatedly in a loop while this op mode is running
         */
        @Override
        public void loop() {
                if(gamepad1.a){
                        pos -= 0.001;
                }
                else if(gamepad1.b){
                        pos += 0.001;
                }
                pos = Math.min(Math.max(pos, 0), 1);
                servo.setPosition(Math.min(Math.max(pos, 0), 1));
                telemetry.addData("servo pos",servo.getPosition());
                telemetry.addData("desired pos", pos);
                telemetry.update();
                sleep(15);

        }
}