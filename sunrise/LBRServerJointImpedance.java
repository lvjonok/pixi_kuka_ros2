package lbr_fri_ros2;

// LBRServer with the control mode fixed to joint impedance and the client command mode to TORQUE,
// plus a stiffness and a damping preset. Installed beside the stock LBRServer: same package, same
// FRI dialogs. The first entry of each preset dialog is what this cabinet's LBRServer runs.
//
// It cannot be compiled outside Sunrise.Workbench (there is no Sunrise SDK on Linux), which
// compiles it when it is synchronised to the cabinet (docs/sunrise_setup.md).
//
// sunrise_joint_stiffness of fri_position_passthrough_controller must equal the stiffness chosen
// here: FRI does not report it, and the controller's gap clamp is computed from that parameter.

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.Arrays;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import com.kuka.connectivity.fastRobotInterface.*;

public class LBRServerJointImpedance extends RoboticsAPIApplication {
	private LBR lbr_;
	private Controller lbr_controller_;

	private String client_name_;
	private String[] client_names_ = {"172.31.1.148", "192.170.10.1"};
	private int send_period_;
	private String[] send_periods_ = {"1", "2", "5", "10"};

	// Nm/rad, A1..A7
	private String[] stiffness_names_ = {
		"200 all (today)",
		"500 all",
		"1000 A1-A3, 500 A4-A7 (Drake)"
	};
	private double[][] stiffness_presets_ = {
		{200, 200, 200, 200, 200, 200, 200},
		{500, 500, 500, 500, 500, 500, 500},
		{1000, 1000, 1000, 500, 500, 500, 500}
	};
	private double[] stiffness_;

	// Lehr's damping ratio, all joints
	private String[] damping_names_ = {
		"0.3 (today)",
		"0.7 (KUKA default)",
		"1.0 (Drake)"
	};
	private double[] damping_presets_ = {0.3, 0.7, 1.0};
	private double damping_;

	private FRIConfiguration fri_configuration_;
	private FRISession fri_session_;
	private FRIJointOverlay fri_overlay_;

	private JointImpedanceControlMode control_mode_;
	private ClientCommandMode command_mode_ = ClientCommandMode.TORQUE;

	public void request_user_config() {
		int id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Select the desired FRI send period [ms]:",
				send_periods_
		);
		send_period_ = Integer.valueOf(send_periods_[id]);
		getLogger().info("Send period set to: " + send_period_);

		id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Select your remote IP address:",
				client_names_
		);
		client_name_ = client_names_[id];
		getLogger().info("Remote address set to: " + client_name_);

		id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Select the joint impedance stiffness [Nm/rad]. "
				+ "sunrise_joint_stiffness in the ROS controller config must match it:",
				stiffness_names_
		);
		stiffness_ = stiffness_presets_[id];
		getLogger().info("Joint stiffness set to: " + stiffness_names_[id] + " = "
				+ Arrays.toString(stiffness_) + " Nm/rad");

		id = getApplicationUI().displayModalDialog(
				ApplicationDialogType.QUESTION,
				"Select the joint impedance damping (all joints):",
				damping_names_
		);
		damping_ = damping_presets_[id];
		getLogger().info("Joint damping set to: " + damping_names_[id] + " = " + damping_);

		control_mode_ = new JointImpedanceControlMode(
				stiffness_[0], stiffness_[1], stiffness_[2], stiffness_[3],
				stiffness_[4], stiffness_[5], stiffness_[6]
		);
		control_mode_.setDampingForAllJoints(damping_);
		getLogger().info("Control mode set to: JOINT_IMPEDANCE_CONTROL (fixed)");

		getLogger().info("Client command mode set to: " + command_mode_.name() + " (fixed)");
	}

	public void configure_fri() {
		fri_configuration_ = FRIConfiguration.createRemoteConfiguration(lbr_, client_name_);
		fri_configuration_.setSendPeriodMilliSec(send_period_);

		getLogger().info("Creating FRI connection to " + fri_configuration_.getHostName());
		getLogger().info(
			"SendPeriod: " + fri_configuration_.getSendPeriodMilliSec() + "ms |"
			+ " ReceiveMultiplier: " + fri_configuration_.getReceiveMultiplier()
		);

		fri_session_ = new FRISession(fri_configuration_);
		fri_overlay_ = new FRIJointOverlay(fri_session_, command_mode_);

		fri_session_.addFRISessionListener(new IFRISessionListener() {
			@Override
			public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
				getLogger().info("Session State change " + friChannelInformation.getFRISessionState().toString() );
			}

			@Override
			public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
				getLogger().info("Quality change signalled "+friChannelInformation.getQuality());
				getLogger().info("Jitter "+friChannelInformation.getJitter());
				getLogger().info("Latency "+friChannelInformation.getLatency());
			}
		});

		try {
			fri_session_.await(10, TimeUnit.SECONDS);
		} catch (final TimeoutException e) {
			getLogger().error(e.getLocalizedMessage());
			return;
		}

		getLogger().info("FRI connection established.");
	}

	@Override
	public void initialize() {
		lbr_controller_ = (Controller) getContext().getControllers().toArray()[0];
		lbr_ = (LBR) lbr_controller_.getDevices().toArray()[0];

		request_user_config();

		configure_fri();
	}

	@Override
	public void run() {
		getLogger().info("Holding position: stiffness " + Arrays.toString(stiffness_)
				+ " Nm/rad, damping " + damping_ + ", command mode " + command_mode_.name());
		lbr_.move(positionHold(control_mode_, -1, TimeUnit.SECONDS).addMotionOverlay(fri_overlay_));
		return;
	}

	@Override
	public void dispose() {
		getLogger().info("Disposing FRI session.");
		fri_session_.close();

		super.dispose();
	}

	public static void main(final String[] args) {
		LBRServerJointImpedance app = new LBRServerJointImpedance();
		app.runApplication();
	}
}
