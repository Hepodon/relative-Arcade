#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "mcl.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"

constexpr size_t PARTICLES = 3000;
ad::MCL<PARTICLES> mcl;
ad::Point last_odom_pos{0.0f, 0.0f};
bool mcl_initialized = false;

using namespace pros;

Controller userInput(E_CONTROLLER_MASTER);

MotorGroup aleft({0, 0, 0}, v5::MotorGears::blue);
MotorGroup aright({0, 0, 0}, v5::MotorGears::blue);

Motor arm_Score(0, v5::MotorGears::red);
Motor intake(0);

Distance Dleft(0);
Distance Dright(0);
Distance Dfront(0);
Distance DbackR(0);
Distance DbackL(0);

IMU inertialT(0);
IMU inertialB(0);

pros::Rotation leftVert(0);
pros::Rotation rightVert(0);

lemlib::TrackingWheel LEFT(&leftVert, lemlib::Omniwheel::NEW_2, 0, 1);
lemlib::TrackingWheel RIGHT(&rightVert, lemlib::Omniwheel::NEW_2, 0, 1);

lemlib::OdomSensors sensors(&LEFT, &RIGHT, nullptr, nullptr, &inertialT);

lemlib::Drivetrain DT(&aleft, &aright, 11, lemlib::Omniwheel::NEW_325, 450, 12);

lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );
lemlib::Chassis chassis(DT, lateral_controller, angular_controller, sensors);

void odom() {
  if (!mcl_initialized)
    return;

  auto pose = chassis.getPose();

  float odom_x = pose.x;
  float odom_y = pose.y;

  ad::Point current_odom{odom_x, odom_y};

  float dx = current_odom.x - last_odom_pos.x;
  float dy = current_odom.y - last_odom_pos.y;

  last_odom_pos = current_odom;

  float std_dev = std::hypot(dx, dy) / 4.0f;
  mcl.predict(dx, dy, std_dev);

  std::vector<ad::Reading> readings;

  float theta = inertialT.get_rotation() * M_PI / 180.0f;
  ad::Rotation heading = ad::rad(theta);

  ad::Position robot_pos{current_odom.x, current_odom.y, heading};

  auto add_sensor = [&](Distance &sensor, ad::Position offset) {
    auto v = sensor.get();
    if (!v)
      return;

    float d = v;

    float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
    constexpr float K = 3.0f;
    float std_dev = std::max(bound / K, 1e-6f);

    ad::Position rel = offset.rotate(robot_pos.theta);
    ad::Point ray_pt =
        rel.point() + ad::Point{rel.theta.cos(), rel.theta.sin()};

    readings.emplace_back(d, std_dev, rel.point(), ray_pt);
  };

  add_sensor(Dleft, ad::Position{-5.0f, 0.0f, ad::deg(180)});
  add_sensor(Dright, ad::Position{5.0f, 0.0f, ad::deg(0)});
  add_sensor(Dfront, ad::Position{0.0f, 5.0f, ad::deg(90)});
  add_sensor(DbackL, ad::Position{0.0f, -5.0f, ad::deg(-90)});

  mcl.update(readings);

  ad::Point est = mcl.estimate();

  chassis.setPose(est.x, est.y, 0);

  mcl.resample();
}

void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();

  // starting position (CHANGE THESE to your actual start)
  float start_x = 0.0f;
  float start_y = 0.0f;

  mcl.init(start_x, start_y, 2.5f);

  last_odom_pos = ad::Point{start_x, start_y};
  mcl_initialized = true;
}

Task odomTask([] {
  //   while (true) {
  //     odom();
  //     delay(10); // 100Hz
  //   }
});

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  float theta;
  float theta_deg;
  while (true) {
    // print robot location to the brain screen
    pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
    pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
    pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

    theta = std::atan2(userInput.get_analog(ANALOG_RIGHT_Y),
                       userInput.get_analog(ANALOG_RIGHT_X)) *
            (180.0f / M_PI); // returns angle in radians

    chassis.turnToHeading(theta, 100);

    // chassis.arcade(userInput.get_analog(ANALOG_LEFT_Y), 0, true, 0.65);

    // delay to save resources
    pros::delay(100);
  }
}