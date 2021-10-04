#pragma once

namespace mocap_sub {

int main();

inline uint32_t index;

inline std::string object_name;
inline float position[3];
inline float velocity[3];
inline float orientation[4];
inline float orientation_euler[3];
inline float angular_velocity[3];

inline float latency;
inline bool new_data = false;
inline int matched = false;

} // namespace mocap_sub