#pragma once

#include <iostream>

namespace DYNO {
namespace Exceptions {

class InvalidConfigurationValue : public std::exception {
   public:
    const char* what() {
        return "This configuration value can not be parsed correctly.";
    }
};

class NotImplemented : public std::exception {
   public:
    const char* what() { return "This feature is not implemented."; }
};

class NoSensorReferenceFrame : public std::exception {
   public:
    const char* what() {
        return "No default sensor reference frame specified for this system!";
    }
};

class NoSensorConfiguration : public std::exception {
   public:
    const char* what() {
        return "No default sensor configuration specified for this system!";
    }
};

class NoAerodynamicParameters : public std::exception {
   public:
    const char* what() {
        return "No aerodynamic parameters defined for this system!";
    }
};

class NoControllerPreset : public std::exception {
   public:
    const char* what() {
        return "No controller preset defined for this system!";
    }
};

class NoSensorSuite : public std::exception {
   public:
    const char* what() { return "No sensor suite defined for this system!"; }
};

}  // namespace Exceptions
}  // namespace DYNO
