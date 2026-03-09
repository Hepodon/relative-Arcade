#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "mcl.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include <cmath>

constexpr size_t PARTICLES = 3000;
ad::MCL<PARTICLES> mcl;
ad::Point last_odom_pos{0.0f, 0.0f};
bool mcl_initialized = false;

using namespace pros;

pros::MotorGroup aleft({-10, -9, -8});
pros::MotorGroup aright({1, 2, 3});

pros::MotorGroup middle({7, -4});

pros::adi::Pneumatics match('a', false);
pros::adi::Pneumatics arm('h', false);
pros::adi::Pneumatics tripstate('b', false);
pros::adi::Pneumatics tripstate2('e', false);

pros::Rotation vertRot(5);

lemlib::TrackingWheel vert(&vertRot, lemlib::Omniwheel::NEW_2, 0.75);

pros::IMU inertialT(6);

pros::Distance Dleft(11);
pros::Distance Dright(20);
pros::Distance DfrontL(12);
pros::Distance DfrontR(18);
pros::Distance Dback(19);

lemlib::Drivetrain DT(&aleft, &aright, 11, lemlib::Omniwheel::NEW_325, 450, 6);

lemlib::OdomSensors sensors(&vert, nullptr, nullptr, nullptr, &inertialT);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(5,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       5.5, // derivative gain (kD)
                       3,   // anti windup
                       0.5, // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       1.5, // large error range, in inches
                       200, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(4.45, // proportional gain (kP)
                       0,    // integral gain (kI)
                       31.5, // derivative gain (kD)
                       3,    // anti windup
                       1,    // small error range, in degrees
                       100,  // small error range timeout, in milliseconds
                       3,    // large error range, in degrees
                       500,  // large error range timeout, in milliseconds
                       0     // maximum acceleration (slew)
    );

lemlib::Chassis chassis(DT, lateral_controller, angular_controller, sensors);

pros::Controller userInput(pros::E_CONTROLLER_MASTER);

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

  add_sensor(Dleft, ad::Position{-4.625f, 1.625f, ad::deg(-90)});
  add_sensor(Dright, ad::Position{4.625f, 1.625f, ad::deg(90)});
  add_sensor(DfrontR, ad::Position{5.0f, 5.625f, ad::deg(0)});
  add_sensor(DfrontL, ad::Position{-5.0f, -5.625f, ad::deg(0)});
  add_sensor(Dback, ad::Position{0.0f, -5.0f, ad::deg(180)});

  mcl.update(readings);

  ad::Point est = mcl.estimate();

  chassis.setPose(est.x, est.y, 0);

  mcl.resample();
}

bool in = false;
bool mid = false;
bool top = false;
bool out = false;

void scoreTop() {
  if (!top) {
    tripstate.extend();
    tripstate2.extend();
    middle.move(127);
    top = true;
    mid = false;
    out = false;
    in = false;
  } else {
    middle.brake();
    top = false;
    mid = false;
    out = false;
    in = false;
  }
}

void intake() {
  if (!in) {
    tripstate.extend();
    tripstate2.retract();
    middle.move(127);
    in = true;
    top = false;
    mid = false;
    out = false;
  } else {
    middle.brake();
    top = true;
    mid = false;
    out = false;
    in = false;
  }
}

void scoreMid() {
  if (!mid) {
    tripstate.retract();
    tripstate2.retract();
    middle.move(127);
    mid = true;
    top = false;
    out = false;
    in = false;
  } else {
    middle.brake();
    top = true;
    mid = false;
    out = false;
    in = false;
  }
}

void outtake() {
  out = true;
  top = false;
  mid = false;
  in = false;
  middle.move(-127);
  while (userInput.get_digital(DIGITAL_B)) {
    delay(5);
  }
  middle.brake();

  out = false;
  top = false;
  mid = false;
  in = false;
}

void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();

  // starting position (CHANGE THESE to your actual start)
  float start_x = 0.0f;
  float start_y = 0.0f;

  // mcl.init(start_x, start_y, 2.5f);

  last_odom_pos = ad::Point{start_x, start_y};
  mcl_initialized = true;
}

Task odomTask([] {
  while (true) {
    odom();
    delay(10); // 100Hz
  }
});

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  while (true) {
    chassis.arcade(userInput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),
                   userInput.get_analog(E_CONTROLLER_ANALOG_RIGHT_X), true,
                   0.4);

    if (userInput.get_digital_new_press(DIGITAL_L1)) {
      arm.toggle();
    }
    if (userInput.get_digital_new_press(DIGITAL_L2)) {
      match.toggle();
    }

    if (userInput.get_digital_new_press(DIGITAL_R2)) {
      scoreTop();
    } else if (userInput.get_digital_new_press(DIGITAL_R1)) {
      intake();
    } else if (userInput.get_digital_new_press(DIGITAL_A)) {
      scoreMid();
    } else if (userInput.get_digital(DIGITAL_B)) {
      outtake();
    } else if (userInput.get_digital_new_press(DIGITAL_DOWN)) {
      tripstate.toggle();
    } else if (userInput.get_digital_new_press(DIGITAL_UP)) {
      tripstate2.toggle();
    }

    delay(5);
  }
}
