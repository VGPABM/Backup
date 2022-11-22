#pragma once

#include <string>
#include <map>
namespace soero{
    std::string EncryptTime(double total_time){
        std::map<char, char>encryptable;
        encryptable['1']='S';
        encryptable['2']='O';
        encryptable['3']='E';
        encryptable['4']='R';
        encryptable['5']='o';
        encryptable['6']='M';
        encryptable['7']='I';
        encryptable['8']='B';
        encryptable['9']='e';
        encryptable['0']='r';
        encryptable['.']='-';
        encryptable[',']='/';
        std::string results="";
        std::string stringfied_time = std::to_string(total_time);
        for(auto &it: stringfied_time){
            // std::cout << it << "\n";
            results += encryptable[it];
        }
        return results;
    }
}