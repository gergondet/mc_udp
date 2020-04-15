/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_udp/client/Client.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <signal.h>

static bool running = true;

static const std::string extra = "Choreonoid::ExtraSensor";

void handler(int)
{
  running = false;
}

void cli(mc_control::MCGlobalController & ctl)
{
  while(running)
  {
    std::string ui;
    std::getline(std::cin, ui);
    std::stringstream ss;
    ss << ui;
    std::string token;
    ss >> token;
    if(token == "stop")
    {
      LOG_INFO("Stopping connection")
      running = false;
    }
    else if(token == "hs" || token == "GoToHalfSitPose" || token == "half_sitting")
    {
      ctl.GoToHalfSitPose_service();
    }
    else
    {
      std::cerr << "Unkwown command " << token << std::endl;
    }
  }
}

template<typename T>
void updateConfig(mc_rtc::Configuration & config, po::variables_map & vm, const char * name, T & param)
{
  if(!vm.count(name))
  {
    if(config.has(name))
    {
      param = static_cast<T>(config(name));
    }
    else
    {
      config.add(name, param);
    }
  }
  else
  {
    config.add(name, param);
  }
}

int main(int argc, char * argv[])
{
  std::string conf_file = "";
  std::string host = "localhost";
  int port = 4444;

  /** Whether encoder desired velocities should be sent */
  bool withEncoderVelocity = false;

  /** Whether to use one or two client(s) */
  bool singleClient = false;

  po::options_description desc("MCUDPControl options");
  // clang-format off
  desc.add_options()
    ("help", "Display help message")
    ("encoderVelocity", "Send/receive encoder velocities")
    ("host,h", po::value<std::string>(&host), "Connection host")
    ("port,p", po::value<int>(&port), "Connection port")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file")
    ("single,s", "Use a single client");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  if(vm.count("encoderVelocity"))
  {
    LOG_INFO("[UDP] Sending/Receiving encoder velocities");
    withEncoderVelocity = true;
  }

  if(vm.count("single"))
  {
    singleClient = true;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file, nullptr);
  auto config = gconfig.config;
  if(!config.has("UDP"))
  {
    config.add("UDP");
  }
  config = config("UDP");
  updateConfig(config, vm, "host", host);
  updateConfig(config, vm, "port", port);
  mc_control::MCGlobalController controller(gconfig);
  if(singleClient)
  {
    LOG_INFO("Connect UDP client to " << host << ":" << port)
  }
  else
  {
    LOG_INFO("Connecting UDP sensors client to " << host << ":" << port)
  }
  mc_udp::Client sensorsClient(host, port);
  mc_udp::Client * controlClientPtr = &sensorsClient;
  if(!singleClient)
  {
    LOG_INFO("Connecting UDP control client to " << host << ":" << port + 1)
    controlClientPtr = new mc_udp::Client(host, port + 1);
  }
  mc_udp::Client & controlClient = *controlClientPtr;
  bool init = false;
  // RTC port to robot force sensors
  std::unordered_map<std::string, std::string> fsensors;
  fsensors["rfsensor"] = "RightFootForceSensor";
  fsensors["lfsensor"] = "LeftFootForceSensor";
  fsensors["rhsensor"] = "RightHandForceSensor";
  fsensors["lhsensor"] = "LeftHandForceSensor";
  std::map<std::string, sva::ForceVecd> wrenches;
  auto refIndex = [&controller](const std::string & jName) {
    const auto & rjo = controller.robot().refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      if(rjo[i] == jName)
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  };
  auto gripperIndex = [&](const std::vector<std::string> & joints) {
    std::vector<int> gIndex;
    for(const auto & j : joints)
    {
      gIndex.push_back(refIndex(j));
    }
    return gIndex;
  };
  std::map<std::string, std::vector<int>> grippers;
  std::map<std::string, std::vector<int>> grippersAll;
  std::map<std::string, std::vector<double>> gripperState;
  for(const auto & g : controller.gripperActiveJoints())
  {
    grippers[g.first] = gripperIndex(g.second);
    gripperState[g.first].resize(g.second.size());
  }
  for(const auto & g : controller.gripperJoints())
  {
    grippersAll[g.first] = gripperIndex(g.second);
  }
  auto updateGripperState = [&](const std::vector<double> & qIn) {
    for(auto & g : gripperState)
    {
      for(size_t i = 0; i < g.second.size(); ++i)
      {
        if(qIn[grippers[g.first][i]] != -1)
        {
          g.second[i] = qIn[grippers[g.first][i]];
        }
      }
    }
  };
  std::map<size_t, double> ignoredJoints;
  std::map<size_t, double> ignoredVelocities;
  if(config.has("IgnoredJoints"))
  {
    const auto & c = config("IgnoredJoints");
    std::vector<std::string> joints = c("joints", std::vector<std::string>{});
    std::map<std::string, double> ignoredValues = c("values", std::map<std::string, double>{});
    std::map<std::string, double> ignoredVelocityValues = c("velocities", std::map<std::string, double>{});

    for(const auto & jN : joints)
    {
      if(controller.robot().hasJoint(jN))
      {
        const auto & rjo = controller.robot().refJointOrder();
        const auto idx = std::distance(rjo.begin(), std::find(rjo.begin(), rjo.end(), jN));
        double qInit = 0;
        double alphaInit = 0;
        // Ignored joint values
        if(ignoredValues.count(jN) > 0)
        { // Use user-provided value for the ignored joint
          qInit = ignoredValues[jN];
        }
        else
        {
          // Use halfsitting configuration
          qInit = controller.robot().stance().at(jN)[0];
        }
        ignoredJoints[idx] = qInit;

        // Ignored velocity values
        if(ignoredVelocityValues.count(jN) > 0)
        { // Use user-provided value for the ignored joint
          alphaInit = ignoredVelocityValues[jN];
        }
        ignoredVelocities[idx] = alphaInit;
        LOG_WARNING("[UDP] Joint " << jN << " is ignored, value = " << qInit << ", velocity=" << alphaInit);
      }
      else
      {
        LOG_WARNING("[UDP] Ignored joint " << jN << " is not present in robot " << controller.robot().name());
      }
    }
  }
  uint64_t prev_id = 0;
  using duration_ms = std::chrono::duration<double, std::milli>;
  duration_ms udp_run_dt{0};
  controller.controller().logger().addLogEntry("perf_UDP", [&udp_run_dt]() { return udp_run_dt.count(); });
  signal(SIGINT, handler);
  std::thread cli_thread([&controller]() { cli(controller); });
  std::vector<double> qIn;
  std::vector<double> alphaIn;
  while(running)
  {
    using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                            std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;
    if(sensorsClient.recv())
    {
      auto start = clock::now();
      auto & sc = sensorsClient.sensors();
      qIn = sc.encoders;
      alphaIn = sc.encoderVelocities;

      // Ignore encoder value for ignored joints
      for(const auto & j : ignoredJoints)
      {
        qIn[j.first] = j.second;
      }
      if(withEncoderVelocity)
      {
        for(const auto & j : ignoredVelocities)
        {
          alphaIn[j.first] = j.second;
        }
      }

      controller.setEncoderValues(qIn);
      controller.setEncoderVelocities(alphaIn);

      controller.setJointTorques(sc.torques);
      Eigen::Vector3d rpy;
      rpy << sensorsClient.sensors().orientation[0], sensorsClient.sensors().orientation[1],
          sensorsClient.sensors().orientation[2];
      controller.setSensorOrientation(Eigen::Quaterniond(mc_rbdyn::rpyToMat(rpy)));
      Eigen::Vector3d pos;
      pos << sc.position[0], sc.position[1], sc.position[2];
      controller.setSensorPosition(pos);
      Eigen::Vector3d vel;
      vel << sensorsClient.sensors().angularVelocity[0], sensorsClient.sensors().angularVelocity[1],
          sensorsClient.sensors().angularVelocity[2];
      controller.setSensorAngularVelocity(vel);
      Eigen::Vector3d acc;
      acc << sensorsClient.sensors().angularAcceleration[0], sensorsClient.sensors().angularAcceleration[1],
          sensorsClient.sensors().angularAcceleration[2];
      controller.setSensorAcceleration(acc);

      // Floating base sensor
      if(controller.robot().hasBodySensor("FloatingBase"))
      {
        controller.setSensorPositions(
            controller.robot(),
            {{"FloatingBase", {sc.floatingBasePos[0], sc.floatingBasePos[1], sc.floatingBasePos[2]}}});
        Eigen::Vector3d fbRPY;
        controller.setSensorOrientations(
            controller.robot(),
            {{"FloatingBase", Eigen::Quaterniond(mc_rbdyn::rpyToMat(
                                  {sc.floatingBaseRPY[0], sc.floatingBaseRPY[1], sc.floatingBaseRPY[2]}))}});
        controller.setSensorAngularVelocities(
            controller.robot(),
            {{"FloatingBase", {sc.floatingBaseVel[0], sc.floatingBaseVel[1], sc.floatingBaseVel[2]}}});
        controller.setSensorLinearVelocities(
            controller.robot(),
            {{"FloatingBase", {sc.floatingBaseVel[3], sc.floatingBaseVel[4], sc.floatingBaseVel[5]}}});
        controller.setSensorAccelerations(
            controller.robot(),
            {{"FloatingBase", {sc.floatingBaseAcc[0], sc.floatingBaseAcc[1], sc.floatingBaseAcc[2]}}});
      }

      for(const auto & fs : sc.fsensors)
      {
        Eigen::Vector6d reading;
        reading << fs.reading[3], fs.reading[4], fs.reading[5], fs.reading[0], fs.reading[1], fs.reading[2];
        wrenches[fsensors.at(fs.name)] = sva::ForceVecd(reading);
      }
      for(const auto & fs : sc.extrasensors)
      {
        Eigen::Vector6d reading;
        reading << fs.reading[3], fs.reading[4], fs.reading[5], fs.reading[0], fs.reading[1], fs.reading[2];
        auto & datastore = controller.controller().datastore();
        if(!datastore.has(extra))
        {
          datastore.make<sva::ForceVecd>(extra, reading);
        }
        else
        {
          datastore.assign(extra, sva::ForceVecd(reading));
        }
      }
      controller.setWrenches(wrenches);
      updateGripperState(sensorsClient.sensors().encoders);
      controller.setActualGripperQ(gripperState);
      if(!init)
      {
        auto init_start = clock::now();
        if(!controller.controller().datastore().has(extra))
        {
          controller.controller().datastore().make<sva::ForceVecd>(extra, sva::ForceVecd::Zero());
        }
        controller.init(qIn);
        controller.setGripperCurrentQ(gripperState);
        for(const auto & g : gripperState)
        {
          controller.setGripperTargetQ(g.first, g.second);
        }
        controller.running = true;
        init = true;
        auto init_end = clock::now();
        duration_ms init_dt = init_end - init_start;
        const auto & rjo = controller.ref_joint_order();
        auto & qOut = controlClient.control().encoders;
        auto & alphaOut = controlClient.control().encoderVelocities;
        if(qOut.size() != rjo.size())
        {
          qOut.resize(rjo.size());
        }
        if(withEncoderVelocity && alphaOut.size() != rjo.size())
        {
          alphaOut.resize(rjo.size());
        }
        LOG_INFO("[MCUDPControl] Init duration " << init_dt.count())
        sensorsClient.init();
        if(!singleClient)
        {
          controlClient.init();
        }
      }
      else
      {
        if(prev_id + 1 != sc.id)
        {
          LOG_WARNING("[MCUDPControl] Missed one or more sensors reading (previous id: "
                      << prev_id << ", current id: " << sensorsClient.sensors().id << ")")
        }
        if(controller.run())
        {
          const auto & robot = controller.robot();
          const auto & mbc = robot.mbc();
          const auto & rjo = controller.ref_joint_order();
          auto & qOut = controlClient.control().encoders;
          auto & alphaOut = controlClient.control().encoderVelocities;
          for(size_t i = 0; i < rjo.size(); ++i)
          {
            const auto & jN = rjo[i];
            if(robot.hasJoint(jN))
            {
              auto jIndex = robot.jointIndexByName(jN);
              if(mbc.q[jIndex].size() == 1)
              {
                auto jIdx = robot.jointIndexByName(jN);
                qOut[i] = mbc.q[jIdx][0];
                if(withEncoderVelocity)
                {
                  alphaOut[i] = mbc.alpha[jIdx][0];
                }
              }
            }
          }

          // Ignore QP output for ignored joints
          for(const auto & j : ignoredJoints)
          {
            qOut[j.first] = j.second;
          }
          if(withEncoderVelocity)
          {
            for(const auto & j : ignoredVelocities)
            {
              alphaOut[j.first] = j.second;
            }
          }

          auto gripperQOut = controller.gripperQ();
          for(const auto & g : grippersAll)
          {
            for(size_t i = 0; i < g.second.size(); ++i)
            {
              if(g.second[i] != -1)
              {
                qOut[g.second[i]] = gripperQOut[g.first][i];
              }
            }
          }
          controlClient.control().id = sensorsClient.sensors().id;
          controlClient.send();
        }
      }
      prev_id = sc.id;
      udp_run_dt = clock::now() - start;
    }
  }
  return 0;
}
