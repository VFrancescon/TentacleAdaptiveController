#include "ControllerPrototype.hpp"
#include "ExtraComputationFuncs.hpp"

// ver1.
// we initialise:
// 1. initial field
// 2. baseline error
//
// we feed:
// 1. error
// 2. change of error

// we output:
//  1. next state of field

// the program tells us what to d

int main(int argc, char** argv) {
    Vector3d field;
    field << 1, 1, 1;
    int baselineError = 400;

    int error;
    int d_error;  // error - previous error

    if (argc == 3) {
        error = std::stoi(argv[argc - 2]);
        d_error = std::stoi(argv[argc - 1]);
    } else {
        error = baselineError;
        d_error = 0;
    }

    std::cout << "---------------------\n";
    std::cout << "inputted error: " << error << "\n";
    std::cout << "inputted d_error " << d_error << "\n";
    std::cout << "---------------------\n";

    double error_wrt_baseline = (double)error / (double)baselineError;
    std::cout << "Calculated % error " << error_wrt_baseline << "\n";

    int signflag = std::signbit(d_error);
    signflag = (signflag == 0) ? 1 : -1;
    std::cout << "calculated signFlag " << signflag << "\n";
    std::cout << "---------------------\n";

    double Kp, Kd;
    Kp = 1 - error_wrt_baseline;

    switch (abs(d_error)) {
        case 0 ... 15:
            Kd = 1.40;
            break;

        case 16 ... 40:
            Kd = 1.20;
            break;

        case 41 ... 60:
            Kd = 1.10;
            break;

        default:
            Kd = 1;
            break;
    }

    std::cout << "calculated Kp " << Kp << "\n";
    std::cout << "calculated Kd " << Kd << "\n";
    std::cout << "---------------------\n";

    if(Kp == 0) Kp = 1;

    Vector3d newField =
        field + (Kp * error_wrt_baseline + Kd) * signflag * field;

    std::cout << "Final op: (" << Kp << " * " << error_wrt_baseline << " + "
              << Kd << ") * " << signflag << " = "
              << (Kp * error_wrt_baseline + Kd) * signflag << "\n";
    std::cout << "Field transitions from: \n";
    std::cout << "x: " << field(0) << "-> " << newField(0) << "\n";
    std::cout << "y: " << field(1) << "-> " << newField(1) << "\n";
    std::cout << "z: " << field(2) << "-> " << newField(2) << "\n";
    std::cout << "---------------------\n";

    return 0;
}