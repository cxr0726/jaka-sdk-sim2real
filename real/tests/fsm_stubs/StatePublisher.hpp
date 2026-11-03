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
        explicit StatePublisher(const std::string&)
        {if(fail_construct)throw std::runtime_error("injected publisher construction failure");}
        void publish(const std::array<double,27>&,const std::array<double,4>&){++calls;}
    };
}
