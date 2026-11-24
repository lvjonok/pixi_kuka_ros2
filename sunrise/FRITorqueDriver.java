// An FRI torque-overlay app: client 172.31.1.148 on port 30200, send period 2 ms (the
// controllers' update_rate 500), joint impedance 1000/1000/1000/500/500/500/500 Nm/rad with
// damping 1.0. With it running, sunrise_joint_stiffness of fri_position_passthrough_controller
// must be the same seven numbers: FRI does not report the stiffness.
//
// Derived from RobotLocomotion/drake-iiwa-driver (BSD-3-Clause),
// kuka-driver/sunrise_1.14/DrakeFRITorqueDriver.java, itself based on KUKA's FRI sample code.
package lbr_fri_ros2;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

public class FRITorqueDriver extends RoboticsAPIApplication {
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private int _clientPort;

    @Override
    public void initialize() {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        _clientName = "172.31.1.148";
        _clientPort = 30200;
    }

    private void doFRISession(FRIConfiguration friConfiguration) {
    	getLogger().info("Creating FRI connection to " + friConfiguration.getHostName() +
    			":" + friConfiguration.getPortOnRemote());
    	getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        try {
            friSession.await(3600, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");
        FRIJointOverlay jointOverlay = new FRIJointOverlay(
            friSession, ClientCommandMode.TORQUE);

        JointImpedanceControlMode ctrMode =
            new JointImpedanceControlMode(1000, 1000, 1000, 500, 500, 500, 500);
        ctrMode.setDampingForAllJoints(1.0);
        PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);

        try {
            _lbr.move(posHold.addMotionOverlay(jointOverlay));
        } catch (final CommandInvalidException e) {
            getLogger().error(e.getLocalizedMessage());
        } finally {
            friSession.close();
        }
    }

    @Override
    public void run() {
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(2);
        friConfiguration.setPortOnRemote(_clientPort);
        while (true) {
          doFRISession(friConfiguration);
        }
    }

    public static void main(final String[] args) {
        final FRITorqueDriver app = new FRITorqueDriver();
        app.runApplication();
    }
}
