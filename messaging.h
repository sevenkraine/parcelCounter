#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <plc.h>


namespace bfv {

extern const char *infmt;
extern const char *outfmt;
extern const long PLC_IDLE_INTERVAL;


struct PlcMsg {
    int PLC_MessageID = -1;
    unsigned int PLC_TriggerID = 0;
    unsigned int PLC_Timestamp = 0;
    unsigned int PLC_Belt_Speed = 0;
    int PLC_Status_Code = 0;
    bool PLC_Flow_Request = false;
    bool PLC_Vision_Calibration_Request = false;
    bool PLC_Vision_Status_Request = false;
    bool PAD_1 = false;
};

struct BfvMsg {
    int BFV_MessageID = -1;
    unsigned int BFV_Status_Timestamp = 0;
    int BFV_Report_FlowID = 0;
    int BFV_CalibrationID = 0;
    int BFV_StatusID = 0;
    unsigned int BFV_Length_in_Flow_Direction = 0;
    unsigned int BFV_Total_Length = 0;
    unsigned int BFV_Total_Width = 0;
    short int BFV_Package_Count = 0;
    short int BFV_Percent_Area_Right = 0;
    short int BFV_Percent_Area_Left = 0;
    short int BFV_Status_Code = 0;
};

class Link {
public:
    Link() = default;
    Link(std::string &mod, std::string &enipd, int in, int out):
        module(mod), daemon(enipd), inid(in), outid(out) {   };
    void up();
    void down();
    void recv(PlcMsg &msg);
    void send(BfvMsg &msg);
    bool valid();
    BfvMsg retrv();
    std::mutex msg_mutex;

private:
    void fork();
    PLC *plc = NULL;
    std::string module = "cipioslv";
    std::string daemon = "127.0.0.1:315";
    int inid = 100;
    int outid = 101;
    std::shared_ptr<PlcMsg> in = std::make_shared<PlcMsg>(PlcMsg());
    std::shared_ptr<BfvMsg> out = std::make_shared<BfvMsg>(BfvMsg());
    int timelag = 5;
    bool is_up = false;
};

}

bool operator==(const bfv::BfvMsg &lhs, const bfv::BfvMsg &rhs);
bool operator!=(const bfv::BfvMsg &lhs, const bfv::BfvMsg &rhs);
bool operator==(const bfv::PlcMsg &lhs, const bfv::PlcMsg &rhs);
bool operator!=(const bfv::PlcMsg &lhs, const bfv::PlcMsg &rhs);
std::ostream &operator<<(std::ostream &os, const bfv::BfvMsg &msg);
std::ostream &operator<<(std::ostream &os, const bfv::PlcMsg &msg);
