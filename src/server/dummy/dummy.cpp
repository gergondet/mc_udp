#include <mc_nng/server/Server.h>
#include <mc_nng/logging.h>

#include <unistd.h>

int main(int argc, char * argv[])
{
  std::string uri = "tcp://*:4444";
  int timeout = 4;
  if(argc > 1)
  {
    uri = argv[1];
  }
  if(argc > 2)
  {
    timeout = std::atoi(argv[2]);
  }
  mc_nng::Server server(uri, timeout);
  auto & sensors = server.sensors();
  sensors.encoders = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  sensors.torques = {100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110};
  sensors.fsensors =
  {
    {"rfsensor", {1,2,3,4,5,6}},
    {"lfsensor", {10,20,30,40,50,60}},
    {"rhsensor", {100,200,300,400,500,600}},
    {"lhsensor", {1000,2000,3000,4000,5000,6000}}
  };
  sensors.orientation[0] = 1;
  sensors.orientation[1] = 2;
  sensors.orientation[2] = 3;
  sensors.angularVelocity[0] = 4;
  sensors.angularVelocity[1] = 5;
  sensors.angularVelocity[2] = 6;
  sensors.angularAcceleration[0] = 7;
  sensors.angularAcceleration[1] = 8;
  sensors.angularAcceleration[2] = 9;
  server.sensors().id = 0;
  while(1)
  {
    server.send();
    if(server.recv())
    {
      if(server.control().id != server.sensors().id)
      {
        MC_NNG_WARNING("[dummy] Server control id " << server.control().id << " does not match sensors id " << server.sensors().id)
      }
      std::cout << "control id: " << server.control().id << " command size: " << server.control().encoders.size() << "\n";
    }
    server.sensors().id += 1;
  }
  return 0;
}