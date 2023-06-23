#include "PD_Insertion.hpp"


int main(int argc, char* argv[]){

    std::string filename = "/home/vittorio/TentacleAdaptiveController/Uopt2_Nav2_OPT2RE.csv";
    if(argc == 2) {
        filename = argv[1];
        }
    
    std::ifstream file(filename, std::ios::in);
    std::string line, word;
    
    getline(file,line);
    std::vector<float> bx, by, bz;
    while(std::getline(file, line)){
        int counter = 0;
        std::stringstream sstr(line);
        // std::cout << line << "\n";
        while(std::getline(sstr, word, ',')){

            if(counter == 6) {
                bx.push_back(std::stof(word)* -3);
                bx.push_back(std::stof(word)* -3);
                bx.push_back(std::stof(word)* -3);
            }
            if(counter == 7) {
                by.push_back(std::stof(word) * -4);
                by.push_back(std::stof(word) * -4);
                by.push_back(std::stof(word) * -4);}
            
            if(counter == 8) {
                bz.push_back(std::stof(word) * -3);
                bz.push_back(std::stof(word) * -3);
                bz.push_back(std::stof(word) * -3);
            }
            counter++;
        }
    }
    std::cout << "Size of input: " << bx.size() << "\n";
    
    std::vector<float> bxINV = bx, byINV = by, bzINV = bz;
    std::reverse(bxINV.begin(), bxINV.end());
    std::reverse(byINV.begin(), byINV.end());
    std::reverse(bzINV.begin(), bzINV.end());
    int OP_MODE = 1;
    MiddlewareLayer mid(OP_MODE);
    mid.set3DField(0,0,0);
    std::cout << "Press enter to begin\n";
    std::cin.get();
    int stepper_count  = 0;
    // mid.set3DField(2,0,-2);
    for(int i = 0; i < 15; i++){
        mid.retractIntroducer();
        usleep(5e5);
        stepper_count++;
    }
    mid.set3DField(5, 5, -10);
    for (int i = 0; i < 15; i++){
        mid.retractIntroducer();
        usleep(5e5);
        stepper_count++;

    }

    for(int i = 0; i < 20; i++){
        mid.retractIntroducer();
        usleep(5e5);
        stepper_count++;
    }

    
    for(int i = 0; i < 5; i++){
        mid.retractIntroducer();
        usleep(5e5);
        stepper_count++;
    }
    // mid.set3DVectorOUT(bx, by, bz);
    // mid.set3DVectorIN(bxINV, byINV, bzINV);
    for(int i = 0; i < stepper_count+20; i++){
        mid.stepIntroducer();
        usleep(5e5);
        stepper_count--;
    }
    mid.set3DField(-10, 5, -10);

    // mid.set3DField(-10, 5, -10);
    for(int i = 0; i < 40; i++){
        mid.retractIntroducer();
        usleep(5e5);
        stepper_count++;
    }
    for(int i = 0; i < stepper_count; i++){
        mid.stepIntroducer();
        usleep(5e5);
    }
    return 0;
}