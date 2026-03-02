#include "pros/apix.h"
#include "pros/distance.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

// Official VEX Competition Field size in inches
const float FIELD_SIZE = 140.42f;
const float HALF_SIZE = 0.5f * FIELD_SIZE;
const float FIELD_MIN = -HALF_SIZE;
const float FIELD_MAX = HALF_SIZE;

struct Rotation {
private:
  float rad_;

public:
  constexpr Rotation() : rad_(0.0f) {}
  constexpr explicit Rotation(float radians) : rad_(radians) {}

  static Rotation deg(float degrees) {
    return Rotation(degrees * (std::numbers::pi_v<float> / 180.0f));
  }

  static Rotation rad(float radians) { return Rotation(radians); }

  float as_deg() const { return rad_ * (180.0f / std::numbers::pi_v<float>); }

  float as_rad() const { return rad_; }

  Rotation normalize_with_cap(Rotation cap) const {
    float theta = rad_;
    float two_cap = 2.0f * cap.rad_;
    theta = std::fmod(std::fmod(theta + cap.rad_, two_cap) + two_cap, two_cap);
    theta -= cap.rad_;
    return Rotation(theta);
  }

  Rotation normalize() const {
    return normalize_with_cap(Rotation::deg(180.0f));
  }

  Rotation round(Rotation increment) const {
    float deg_inc = increment.as_deg();
    float rounded = std::round(as_deg() / deg_inc) * deg_inc;
    return Rotation::deg(rounded);
  }

  Rotation abs() const { return Rotation(std::fabs(rad_)); }

  float sin() const { return std::sin(rad_); }
  float cos() const { return std::cos(rad_); }

  float sinc() const {
    if (std::fabs(rad_) < 1e-4f)
      return 1.0f;
    return std::sin(rad_) / rad_;
  }

  Rotation operator+(Rotation other) const {
    return Rotation(rad_ + other.rad_);
  }
  Rotation operator-(Rotation other) const {
    return Rotation(rad_ - other.rad_);
  }
  Rotation operator*(float scalar) const { return Rotation(rad_ * scalar); }
  Rotation operator/(float scalar) const { return Rotation(rad_ / scalar); }
  Rotation operator-() const { return Rotation(-rad_); }

  Rotation &operator+=(Rotation other) {
    rad_ += other.rad_;
    return *this;
  }
  Rotation &operator-=(Rotation other) {
    rad_ -= other.rad_;
    return *this;
  }
  Rotation &operator*=(float scalar) {
    rad_ *= scalar;
    return *this;
  }
  Rotation &operator/=(float scalar) {
    rad_ /= scalar;
    return *this;
  }

  bool operator==(Rotation other) const { return rad_ == other.rad_; }
  auto operator<=>(Rotation other) const { return rad_ <=> other.rad_; }

  explicit operator float() const { return rad_; }
};

inline Rotation operator*(float scalar, Rotation rot) { return rot * scalar; }

constexpr Rotation deg(float degrees) {
  return Rotation(degrees * (std::numbers::pi_v<float> / 180.0f));
}

constexpr Rotation rad(float radians) { return Rotation(radians); }
struct Position;

struct Point {
  float x = 0.0f;
  float y = 0.0f;

  Point() = default;
  Point(float x, float y) : x(x), y(y) {}
  explicit Point(const Position &pos);

  float hypot() const { return std::hypot(x, y); }
  float dot(const Point &other) const { return x * other.x + y * other.y; }
  float cross(const Point &other) const { return x * other.y - y * other.x; }

  Point rotate(Rotation angle) const {
    float c = angle.cos(), s = angle.sin();
    return {x * c - y * s, x * s + y * c};
  }

  float dist(const Point &other) const { return (*this - other).hypot(); }

  Rotation angle(const Point &other) const {
    return Rotation::rad(std::atan2(other.y - y, other.x - x));
  }

  Point operator+(Point other) const { return {x + other.x, y + other.y}; }
  Point operator-(Point other) const { return {x - other.x, y - other.y}; }
  Point operator*(float s) const { return {x * s, y * s}; }
  Point operator/(float s) const { return {x / s, y / s}; }

  Point &operator+=(Point other) {
    x += other.x;
    y += other.y;
    return *this;
  }
  Point &operator-=(Point other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }
  Point &operator*=(float s) {
    x *= s;
    y *= s;
    return *this;
  }
  Point &operator/=(float s) {
    x /= s;
    y /= s;
    return *this;
  }

  bool operator==(const Point &) const = default;
};

inline Point operator*(float s, Point p) { return p * s; }
struct Position {
  float x = 0.0f;
  float y = 0.0f;
  Rotation theta;

  Position() = default;
  Position(float x, float y, Rotation theta) : x(x), y(y), theta(theta) {}

  static Position origin() { return {0.0f, 0.0f, Rotation::deg(0.0f)}; }

  void set(float x_, float y_, Rotation theta_) {
    x = x_;
    y = y_;
    theta = theta_;
  }

  Point point() const { return {x, y}; }

  Position rotate(Rotation angle) const {
    Point p = point().rotate(angle);
    return {p.x, p.y, theta + angle};
  }

  Position operator+(Point other) const {
    return {x + other.x, y + other.y, theta};
  }
  Position operator-(Point other) const {
    return {x - other.x, y - other.y, theta};
  }

  bool operator==(const Position &) const = default;
};

inline Point::Point(const Position &pos) : x(pos.x), y(pos.y) {}
struct Line {
  Point start;
  Point end;

  inline std::optional<float> square_intersect_distance(float center_x,
                                                        float center_y,
                                                        float width,
                                                        float height) const {
    float half_width = width * 0.5f;
    float half_height = height * 0.5f;

    float rel_start_x = start.x - center_x;
    float rel_start_y = start.y - center_y;

    float dx = end.x - start.x;
    float dy = end.y - start.y;

    float best_t = std::numeric_limits<float>::infinity();

    if (std::abs(dx) > 1e-6f) {
      float inv_dx = 1.0f / dx;
      float target_x = dx > 0.0f ? half_width : -half_width;
      float t = (target_x - rel_start_x) * inv_dx;

      if (t >= 0.0f) {
        float y = rel_start_y + t * dy;
        if (std::abs(y) <= half_height) {
          best_t = t;
        }
      }
    }

    if (std::abs(dy) > 1e-6f) {
      float inv_dy = 1.0f / dy;
      float target_y = dy > 0.0f ? half_height : -half_height;
      float t = (target_y - rel_start_y) * inv_dy;

      if (t >= 0.0f && t < best_t) {
        float x = rel_start_x + t * dx;
        if (std::abs(x) <= half_width) {
          best_t = t;
        }
      }
    }

    if (best_t < std::numeric_limits<float>::infinity()) {
      return best_t * std::hypot(dx, dy);
    }
    return std::nullopt;
  }
};
struct XorShift32 {
  uint32_t state;

  inline XorShift32(uint32_t seed = pros::micros())
      : state(seed == 0 ? 0x12345678 : seed) {}

  inline uint32_t next_u32() {
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
  }

  inline float next_f32() { return (next_u32() >> 8) * (1.0f / (1u << 24)); }

  inline float range_f32(float min, float max) {
    return min + (max - min) * next_f32();
  }

  inline float gaussian(float std_dev) {
    float u1 = std::max(next_f32(), 1e-12f);
    float u2 = next_f32();
    float r = std::sqrt(-2.0f * std::log(u1));
    float theta = 2.0f * M_PI * u2;
    return r * std::cos(theta) * std_dev;
  }
};
struct Reading {
  float recorded;
  float inv_var;
  Point relative_pos;
  Point proj_relative;

  Reading(float recorded, float std_dev, Point relative_pos,
          Point proj_relative)
      : recorded(recorded), inv_var(-0.5f / (std_dev * std_dev)),
        relative_pos(relative_pos), proj_relative(proj_relative) {}

  inline std::optional<float> predict(Point particle_pos) const {
    return Line{relative_pos + particle_pos, proj_relative + particle_pos}
        .square_intersect_distance(0.0f, 0.0f, FIELD_SIZE, FIELD_SIZE);
  }
};
template <size_t N> struct MCL {
  float particle_x[N];
  float particle_y[N];
  float particle_weights[N];

  float temp_x[N];
  float temp_y[N];
  float temp_weights[N];

  float presample_x[N];
  float presample_y[N];
  float presample_weights[N];

  XorShift32 rng;

  MCL();
  void init(float x, float y, float spread);
  void predict(float dx, float dy, float std_dev);
  void update(const std::vector<Reading> &readings);
  Point estimate() const;
  void resample();
};