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
    MiddlewareLayer mid;
    mid.set3DField(0,0,0);
    std::cout << "Press enter to begin\n";
    std::cin.get();

    mid.set3DVectorOUT(bx, by, bz);
    mid.set3DVectorIN(bxINV, byINV, bzINV);

    mid.retractIntroducer(mid.stepper_count);
    return 0;
}