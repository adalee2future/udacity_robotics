#include <gazebo/gazebo.hh>

namespace gazebo
{
    class MyWorld: public WorldPlugin
    {
      public: MyWorld(): WorldPlugin()
      {
        printf("Welcome to Ada's World!\n");
      }

      public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
      {
      }
    };

    GZ_REGISTER_WORLD_PLUGIN(MyWorld)
}


