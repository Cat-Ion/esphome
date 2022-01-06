#include <gtest/gtest.h>
#include "esphome/components/pid/pid_controller.h"
#include "mocks.h"

class PIDTest: public ::testing::Test {
	protected:
	esphome::pid::PIDController pid_;

	void SetUp() override {
		mocks_reset();
	}
};

TEST_F(PIDTest, P) {
	float error = 0;
	float setpoint = 0;
	
	auto &kp = pid_.kp;
	kp = 1;
	
	// process value == setpoint, expect 0
	error = 0;
	EXPECT_EQ(pid_.update(setpoint + error, setpoint), kp * error);
	
	// error = -1, expect -kp
	setpoint = 2;
	error = -1;
	EXPECT_EQ(pid_.update(setpoint + error, setpoint), kp * error);
	
	// error = 1, expect kp
	kp = -1;
	setpoint = -2;
	error = 1;
	EXPECT_EQ(pid_.update(setpoint + error, setpoint), kp * error);
}

TEST_F(PIDTest, I) {
	pid_.set_ki(1);

	double error = 1;
	double expected = 0;

	// This increments time and expected value according to ki and the used error
	auto pass_time = [&](uint32_t dt)->void {
		mock_values.millis += dt;
		expected += error * dt * pid_.ki / 1000;
	};
	
	// First time -> no time passed, expect 0
	mock_values.millis++;
	EXPECT_EQ(pid_.update(error, 0), expected);
	
	// 1 ms passed -> error * ki / 1000 increment
	pass_time(1);
	EXPECT_FLOAT_EQ(pid_.update(error, 0), expected);

	// 1 ms passed -> error * ki / 1000 increment
	pass_time(1);
	EXPECT_FLOAT_EQ(pid_.update(error, 0), expected);

	// 2 ms passed -> error * ki / 1000 *2 increment
	pass_time(2);
	EXPECT_FLOAT_EQ(pid_.update(error, 0), expected);

	// 2 ms passed -> error * ki / 1000 *2 increment
	pid_.set_ki(2);
	error = 2;
	pass_time(2);
	EXPECT_FLOAT_EQ(pid_.update(error, 0), expected);
}

TEST_F(PIDTest, D) {
	pid_.set_kd(2);
	
	// First time -> no time passed, expect 0
	mock_values.millis++;
	EXPECT_EQ(pid_.update(1, 0), 0);
	
	// 1 ms passed, change 1 -> expect kd*1000
	mock_values.millis++;
	EXPECT_FLOAT_EQ(pid_.update(1, 1), 2000);
	
	// 1 ms passed, no change to value -> 0
	mock_values.millis++;
	EXPECT_FLOAT_EQ(pid_.update(2, 1), 0);
	
	pid_.set_kd(-4);

	// 1 ms passed, no change to value -> 0
	mock_values.millis++;
	EXPECT_FLOAT_EQ(pid_.update(2, 1), 0);
	
	// 1 ms passed, change 2 -> expect kd*1000*2
	mock_values.millis++;
	EXPECT_FLOAT_EQ(pid_.update(1, 3), -8000);
}

TEST_F(PIDTest, Constraints) {
	float error = 1;
	float pv = 0;
	float &kp = pid_.kp;
	float &ki = pid_.ki;
	float &kd = pid_.kd;

	// Normal output that is within the constraints
	kp = 0.1;
	EXPECT_FLOAT_EQ(pid_.update(pv + error, pv), error * kp);

	pid_.reset_accumulated_integral();
	EXPECT_FLOAT_EQ(pid_.update(pv - error, pv), -error * kp);

	// Big outputs are clamped
	kp = 1000;
	pid_.min_output = -1;
	pid_.max_output = 1;
	
	pid_.reset_accumulated_integral();
	EXPECT_FLOAT_EQ(pid_.update(pv + error, pv), 1);
	
	pid_.reset_accumulated_integral();
	EXPECT_FLOAT_EQ(pid_.update(pv - error, pv), -1);
}
