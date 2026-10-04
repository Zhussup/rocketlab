#pragma once

#include <cmath>
#include <cstddef>

namespace rocketlab::core {

/// Minimal double-precision 3-vector.
///
/// The core deliberately has no third-party math dependency: it is headless,
/// testable in isolation, and every UI adapter (FTXUI canvas, ImGui, raylib)
/// converts at its own boundary instead. All lengths are metres, all times
/// seconds, all velocities metres per second — SI everywhere, no exceptions.
struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vec3() = default;
  constexpr Vec3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

  [[nodiscard]] constexpr double& operator[](std::size_t i) noexcept {
    return i == 0 ? x : (i == 1 ? y : z);
  }
  [[nodiscard]] constexpr double operator[](std::size_t i) const noexcept {
    return i == 0 ? x : (i == 1 ? y : z);
  }

  constexpr Vec3& operator+=(const Vec3& o) noexcept {
    x += o.x; y += o.y; z += o.z;
    return *this;
  }
  constexpr Vec3& operator-=(const Vec3& o) noexcept {
    x -= o.x; y -= o.y; z -= o.z;
    return *this;
  }
  constexpr Vec3& operator*=(double s) noexcept {
    x *= s; y *= s; z *= s;
    return *this;
  }
  constexpr Vec3& operator/=(double s) noexcept {
    x /= s; y /= s; z /= s;
    return *this;
  }
};

[[nodiscard]] constexpr Vec3 operator+(Vec3 a, const Vec3& b) noexcept { return a += b; }
[[nodiscard]] constexpr Vec3 operator-(Vec3 a, const Vec3& b) noexcept { return a -= b; }
[[nodiscard]] constexpr Vec3 operator-(const Vec3& a) noexcept { return {-a.x, -a.y, -a.z}; }
[[nodiscard]] constexpr Vec3 operator*(Vec3 a, double s) noexcept { return a *= s; }
[[nodiscard]] constexpr Vec3 operator*(double s, Vec3 a) noexcept { return a *= s; }
[[nodiscard]] constexpr Vec3 operator/(Vec3 a, double s) noexcept { return a /= s; }

[[nodiscard]] constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

[[nodiscard]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

[[nodiscard]] constexpr double norm_squared(const Vec3& a) noexcept { return dot(a, a); }
[[nodiscard]] inline double norm(const Vec3& a) noexcept { return std::sqrt(dot(a, a)); }

/// Returns the unit vector, or the zero vector if `a` is degenerate.
[[nodiscard]] inline Vec3 normalized(const Vec3& a) noexcept {
  const double n = norm(a);
  return n > 0.0 ? a / n : Vec3{};
}

[[nodiscard]] constexpr bool is_finite(const Vec3& a) noexcept {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

}  // namespace rocketlab::core
