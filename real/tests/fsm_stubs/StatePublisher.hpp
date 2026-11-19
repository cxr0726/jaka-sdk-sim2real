#pragma once
#include <array>
#include <stdexcept>
#include <string>
namespace RLController
{
    class StatePublisher
    {
    public:
        static inline bool fail_construct=false;
        static inline int calls=0;
        static inline bool bind_ok=true;
        static inline std::array<double,27> last_joints{};
        static inline std::array<double,4> last_quat{};
        explicit StatePublisher(const std::string&)
        {if(fail_construct)throw std::runtime_error("injected publisher construction failure");}
        bool ok() const { return bind_ok; }
        void publish(const std::array<double,27>& joints,const std::array<double,4>& quat)
        {++calls;last_joints=joints;last_quat=quat;}
    };
}
