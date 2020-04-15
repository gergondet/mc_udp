/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_udp/data/RobotSensors.h>

#include <cstring>

namespace mc_udp
{

namespace
{

void sensor(std::vector<ForceSensor> & sensors, const std::string & name, const double data[6])
{
  for(size_t i = 0; i < sensors.size(); ++i)
  {
    auto & fs = sensors[i];
    if(fs.name == name)
    {
      std::memcpy(fs.reading, data, 6 * sizeof(double));
      return;
    }
  }
  sensors.emplace_back();
  sensors.back().name = name;
  std::memcpy(sensors.back().reading, data, 6 * sizeof(double));
}

}

void RobotSensors::fsensor(const std::string & name, double data[6])
{
  sensor(fsensors, name, data);
}

void RobotSensors::extrasensor(const std::string & name, double data[6])
{
  sensor(extrasensors, name, data);
}

size_t RobotSensors::size() const
{
  return
      // Size of id
      sizeof(uint64_t) +
      // Size of encoders buffer length + data
      sizeof(uint64_t) + encoders.size() * sizeof(double) +
      // Size of encoder velocities buffer length + data
      sizeof(uint64_t) + encoderVelocities.size() * sizeof(double) +
      // Size of torques buffer length + data
      sizeof(uint64_t) + torques.size() * sizeof(double) +
      // Size of fsensors
      fsensorsSize(fsensors) +
      // Size of orientation + angularVelocity + angularAcceleration
      9 * sizeof(double) +
      // Size of position (pIn)
      3 * sizeof(double) +
      // Size of floating base position
      3 * sizeof(double) +
      // Size of floating base orientation
      3 * sizeof(double) +
      // Size of floating base velocity
      6 * sizeof(double) +
      // Size of floating base linear acceleration
      6 * sizeof(double) +
      // Size of extra sensors
      fsensorsSize(extrasensors);
}

size_t RobotSensors::fsensorsSize(const std::vector<ForceSensor> & sensors) const
{
  size_t ret = sizeof(uint64_t); // Lenght of fSensors
  for(size_t i = 0; i < sensors.size(); ++i)
  {
    const auto & sensor = sensors[i];
    ret += sizeof(uint64_t) + sensor.name.size() * sizeof(char) + 6 * sizeof(double);
  }
  return ret;
}

namespace
{

void memcpy_advance(uint8_t * dest, const void * src, size_t n, size_t & offset)
{
  std::memcpy(dest + offset, src, n);
  offset += n;
}

} // namespace

void RobotSensors::toBuffer(uint8_t * buffer) const
{
  size_t offset = 0;
  uint64_t tmp = 0; // temporary to store size

  memcpy_advance(buffer, &id, sizeof(uint64_t), offset);

  tmp = encoders.size();
  memcpy_advance(buffer, &tmp, sizeof(uint64_t), offset);
  memcpy_advance(buffer, encoders.data(), encoders.size() * sizeof(double), offset);

  tmp = encoderVelocities.size();
  memcpy_advance(buffer, &tmp, sizeof(uint64_t), offset);
  memcpy_advance(buffer, encoderVelocities.data(), encoderVelocities.size() * sizeof(double), offset);

  tmp = torques.size();
  memcpy_advance(buffer, &tmp, sizeof(uint64_t), offset);
  memcpy_advance(buffer, torques.data(), torques.size() * sizeof(double), offset);

  auto fsToBuffer = [&](const std::vector<ForceSensor> & sensors)
  {
    tmp = sensors.size();
    memcpy_advance(buffer, &tmp, sizeof(uint64_t), offset);
    for(size_t i = 0; i < sensors.size(); ++i)
    {
      auto & fs = sensors[i];
      tmp = fs.name.size();
      memcpy_advance(buffer, &tmp, sizeof(uint64_t), offset);
      memcpy_advance(buffer, fs.name.c_str(), fs.name.size() * sizeof(char), offset);
      memcpy_advance(buffer, fs.reading, 6 * sizeof(double), offset);
    }
  };
  fsToBuffer(fsensors);
  memcpy_advance(buffer, orientation, 3 * sizeof(double), offset);
  memcpy_advance(buffer, angularVelocity, 3 * sizeof(double), offset);
  memcpy_advance(buffer, angularAcceleration, 3 * sizeof(double), offset);
  memcpy_advance(buffer, position, 3 * sizeof(double), offset);
  memcpy_advance(buffer, floatingBasePos, 3 * sizeof(double), offset);
  memcpy_advance(buffer, floatingBaseRPY, 3 * sizeof(double), offset);
  memcpy_advance(buffer, floatingBaseVel, 6 * sizeof(double), offset);
  memcpy_advance(buffer, floatingBaseAcc, 6 * sizeof(double), offset);
  fsToBuffer(extrasensors);
}

namespace
{

void memcpy_advance(void * dest, const uint8_t * src, size_t n, size_t & offset)
{
  std::memcpy(dest, src + offset, n);
  offset += n;
}

} // namespace

void RobotSensors::fromBuffer(uint8_t * buffer)
{
  size_t offset = 0;
  uint64_t tmp = 0; // temporary to store size

  memcpy_advance(&id, buffer, sizeof(uint64_t), offset);

  memcpy_advance(&tmp, buffer, sizeof(uint64_t), offset);
  encoders.resize(tmp);
  memcpy_advance(encoders.data(), buffer, tmp * sizeof(double), offset);

  memcpy_advance(&tmp, buffer, sizeof(uint64_t), offset);
  encoderVelocities.resize(tmp);
  memcpy_advance(encoderVelocities.data(), buffer, tmp * sizeof(double), offset);

  memcpy_advance(&tmp, buffer, sizeof(uint64_t), offset);
  torques.resize(tmp);
  memcpy_advance(torques.data(), buffer, tmp * sizeof(double), offset);

  auto fsFromBuffer = [&](std::vector<ForceSensor> & sensors)
  {
    memcpy_advance(&tmp, buffer, sizeof(uint64_t), offset);
    sensors.resize(tmp);
    for(size_t i = 0; i < sensors.size(); ++i)
    {
      auto & fs = sensors[i];
      tmp = fs.name.size();
      memcpy_advance(&tmp, buffer, sizeof(uint64_t), offset);
      fs.name.assign(reinterpret_cast<char *>(&buffer[offset]), tmp * sizeof(char));
      offset += tmp * sizeof(char);
      memcpy_advance(fs.reading, buffer, 6 * sizeof(double), offset);
    }
  };
  fsFromBuffer(fsensors);
  memcpy_advance(orientation, buffer, 3 * sizeof(double), offset);
  memcpy_advance(angularVelocity, buffer, 3 * sizeof(double), offset);
  memcpy_advance(angularAcceleration, buffer, 3 * sizeof(double), offset);
  memcpy_advance(position, buffer, 3 * sizeof(double), offset);
  memcpy_advance(floatingBasePos, buffer, 3 * sizeof(double), offset);
  memcpy_advance(floatingBaseRPY, buffer, 3 * sizeof(double), offset);
  memcpy_advance(floatingBaseVel, buffer, 6 * sizeof(double), offset);
  memcpy_advance(floatingBaseAcc, buffer, 6 * sizeof(double), offset);
  fsFromBuffer(extrasensors);
}

} // namespace mc_udp
